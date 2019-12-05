use super::pbrt::{Float, Spectrum};
use super::geometry::{Point2i, Point2f, Bounds2i, Bounds2f, Bounding2, Vector2f};
use super::filter::Filter;
use super::profiler::Profiler;
use super::stats_accumulator::StatsAccumulator;
use super::parallel::AtomicFloat;
use super::spectrum::{xyz_to_rgb_slice, xyz_to_rgb};
use super::imageio::write_image;
use smallvec::SmallVec;
use std::sync::{Arc, Mutex};
use std::convert::TryInto;
use std::mem::size_of;
use owning_ref::MutexGuardRefMut;

const FILTER_TABLE_WIDTH: usize = 16;

#[derive(Debug, Default, Clone, Copy)]
pub struct FilmTilePixel {
    contrib_sum: Spectrum,
    filter_weight_sum: Float
}

pub struct Film {
    pub full_resolution: Point2i,
    pub diagonal: Float,
    pub filter: Arc<dyn Filter>,
    pub filename: String,
    pub cropped_pixel_bounds: Bounds2i,

    pixels: Arc<Mutex<Vec<Pixel>>>,
    filter_table: [Float; FILTER_TABLE_WIDTH * FILTER_TABLE_WIDTH],
    scale: Float,
    max_sample_luminance: Float
}

impl Film {
    pub fn new(
        resolution: Point2i,
        crop_window: Bounds2f,
        filter: Arc<dyn Filter>,
        diagonal: Float,
        filename: String,
        scale: Float,
        max_sample_luminance: Option<Float>
    ) -> Film {
        // Compute film image bounds
        let cropped_pixel_bounds = Bounds2i::new(
            Point2i::new(
                (resolution.x as Float * crop_window.min.x).ceil().into(),
                (resolution.y as Float * crop_window.min.y).ceil().into()
            ),
            Point2i::new(
                (resolution.x as Float * crop_window.max.x).ceil().into(),
                (resolution.y as Float * crop_window.max.y).ceil().into()
            )
        );
        info!("Created film with full resolution {}. Crop window of {} -> croppedPixelBounds {}",
            resolution, crop_window, cropped_pixel_bounds);

        // Allocate film image storage
        let pixel_size = cropped_pixel_bounds.area() as usize;
        let pixels: Arc<Mutex<Vec<Pixel>>> =
            Arc::new(Mutex::new(Vec::with_capacity(pixel_size)));
        pixels.lock().unwrap().resize_with(pixel_size, Default::default);

        StatsAccumulator::instance().report_memory_counter(
            String::from("Memory/Film pixels"), (size_of::<Pixel>() * pixel_size) as i64);

        // Precompute filter weight table
        let mut offset: usize = 0;
        let filter_table = [0.0 as Float; FILTER_TABLE_WIDTH * FILTER_TABLE_WIDTH];
        for y in 0..FILTER_TABLE_WIDTH {
            for x in 0..FILTER_TABLE_WIDTH {
                let p = Point2f {
                    x: (x as Float + 0.5) * filter.radius.x / FILTER_TABLE_WIDTH as Float,
                    y: (y as Float + 0.5) * filter.radius.y / FILTER_TABLE_WIDTH as Float
                };
                filter_table[offset] = filter.evaluate(&p);
                offset += 1;
            }
        }

        Film{
            full_resolution: resolution,
            diagonal: diagonal * 0.001,
            filter: filter.clone(),
            filename,
            scale,
            cropped_pixel_bounds,
            pixels,
            filter_table,
            max_sample_luminance: max_sample_luminance.unwrap_or_default(),
        }
    }

    pub fn get_sample_bounds(&self) -> Bounds2i {
        Bounds2i::new(
            (self.cropped_pixel_bounds.min as Float + Point2f::new(0.5, 0.5) - self.filter.radius).floor() as i32,
            (self.cropped_pixel_bounds.max as Float - Point2f::new(0.5, 0.5) + self.filter.radius).ceil() as i32
        )
    }

    pub fn get_physical_extent(&self) -> Bounds2f {
        let aspect = self.full_resolution.y as Float / self.full_resolution.x as Float;
        let x = (self.diagonal * self.diagonal / (1.0 + aspect * aspect)).sqrt();
        let y = aspect * x;
        Bounds2f::new(
            Point2f::new(-x / 2.0, -y / 2.0),
            Point2f::new(x / 2.0, y / 2.0)
        )
    }

    pub fn get_film_tile(&self, sample_bounds: &Bounds2i) -> FilmTile {
        // Bound image pixels that samples in _sampleBounds_ contribute to
        let half_pixel = Vector2f::new(0.5, 0.5);
        let float_bounds = Bounds2f{
            min: Point2f{ x: sample_bounds.min.x as Float, y: sample_bounds.min.y as Float},
            max: Point2f{ x: sample_bounds.max.x as Float, y: sample_bounds.max.y as Float},
        };
        let p0 = (float_bounds.min - half_pixel - self.filter.radius).ceil().cast();
        let p1 = ((float_bounds.max - half_pixel + self.filter.radius).floor() + Point2f::new(1.0, 1.0)).cast();

        let tile_pixel_bounds = Bounds2i::new(p0, p1).intersect(&self.cropped_pixel_bounds);
        FilmTile::new(tile_pixel_bounds, self.filter.radius, &self.filter_table, self.max_sample_luminance)
    }

    pub fn merge_film_tile(&mut self, tile: &FilmTile) {
        let _profile = Profiler::instance().profile("Film::merge_file_tile()");
        vlog!(1, "Merging film tile {}", tile.pixel_bounds);
        for pixel in &tile.pixel_bounds {
            // Merge _pixel_ into _Film::pixels_
            let tile_pixel = tile.pixels[tile.get_pixel_index(&pixel)];
            let mut merge_pixel = self.get_pixel(&pixel);
            let mut xyz = [0.0 as Float; 3];
            tile_pixel.contrib_sum.to_xyz(&mut xyz);
            for i in 0..3 {
                merge_pixel.xyz[i] += xyz[i];
            }
            merge_pixel.filter_weight_sum += tile_pixel.filter_weight_sum;
        }
    }

    pub fn set_image(&mut self, img: &[Spectrum]) {
        let n_pixels = self.cropped_pixel_bounds.area() as usize;
        let mut pixels = self.pixels.lock().unwrap();
        for i in 0..n_pixels {
            let mut p = &mut pixels[i];
            img[i].to_xyz(&mut p.xyz);
            p.filter_weight_sum = 1.0;
            p.splat_xyz[0].store(0.0);
            p.splat_xyz[1].store(0.0);
            p.splat_xyz[2].store(0.0);
        }
    }

    pub fn add_splat(&mut self, p: &Point2f, mut v: Spectrum) {
        let _profile = Profiler::instance().profile("File::add_splat()");

        if v.has_nans() {
            error!("Ignoring splatted spectrum with NaN values at ({}, {})", p.x, p.y);
            return;
        } else if v.y() < 0.0 {
            error!("Ignoring splatted spectrum with negative luminance {} at ({}, {})", v.y(), p.x, p.y);
            return;
        } else if v.y().is_infinite() {
            error!("Ignoring splatted spectrum with infinite luminance at ({}, {})", p.x, p.y);
            return;
        }

        let pi: Point2i = p.floor().cast();
        if self.cropped_pixel_bounds.inside_exclusive(&pi) {
            return;
        }
        if v.y() > self.max_sample_luminance {
            v *= self.max_sample_luminance / v.y();
        }
        let mut xyz = [0.0 as Float; 3];
        v.to_xyz(&mut xyz);
        let mut pixel = self.get_pixel(&pi);
        for i in 0..3 {
            pixel.splat_xyz[i].add(xyz[i]);
        }
    }

    pub fn write_image(&mut self, splat_scale: Float) {
        // Convert image to RGB and compute final pixel values
        info!("Converting image to RGB and computing final weighted pixel values");
        let mut rgb: Vec<Float> = vec![0.0; 3 * self.cropped_pixel_bounds.area() as usize];
        let mut offset = 0_usize;
        let pixels = self.pixels.lock().unwrap();
        for p in &self.cropped_pixel_bounds {
            // Convert pixel XYZ to RGB
            let pixel = &pixels[self.get_pixel_index(&p)];
            let start = 3 * offset;
            xyz_to_rgb_slice(&pixel.xyz, &mut rgb[start..start + 3]);

            // Normalize pixel with weight sum
            let filter_weight_sum = pixel.filter_weight_sum;
            if filter_weight_sum != 0.0 {
                let inv_wt = 1.0 / filter_weight_sum;
                rgb[start] = (rgb[start] * inv_wt).max(0.0);
                rgb[start + 1] = (rgb[start + 1] * inv_wt).max(0.0);
                rgb[start + 2] = (rgb[start + 2] * inv_wt).max(0.0);
            }

            // Add splat value at pixel
            let mut splat_rgb = [0.0 as Float; 3];
            let splat_xyz = [pixel.splat_xyz[0].load(), pixel.splat_xyz[1].load(), pixel.splat_xyz[2].load()];
            xyz_to_rgb(&splat_xyz, &mut splat_rgb);
            rgb[start] += splat_scale * splat_rgb[0];
            rgb[start + 1] += splat_scale * splat_rgb[1];
            rgb[start + 2] += splat_scale * splat_rgb[2];

            // Scale pixel value by _scale_
            rgb[start] *= self.scale;
            rgb[start+ 1] *= self.scale;
            rgb[start+ 2] *= self.scale;
            offset += 1;
        }

        // Write RGB image
        info!("Writing image {} with bounds {}", self.filename, self.cropped_pixel_bounds);
        write_image(self.filename, &rgb, self.cropped_pixel_bounds, self.full_resolution);
    }

    pub fn clear(&mut self) {
        for p in &self.cropped_pixel_bounds {
            let mut pixel = self.get_pixel(&p);
            pixel.splat_xyz[0].store(0.0);
            pixel.splat_xyz[1].store(0.0);
            pixel.splat_xyz[2].store(0.0);
            pixel.xyz[0] = 0.0;
            pixel.xyz[1] = 0.0;
            pixel.xyz[2] = 0.0;
            pixel.filter_weight_sum = 0.0;
        }
    }

    fn get_pixel(&self, p: &Point2i) -> MutexGuardRefMut<Vec<Pixel>, Pixel> {
        assert!(self.cropped_pixel_bounds.inside_exclusive(p));
        let width = self.cropped_pixel_bounds.max.x - self.cropped_pixel_bounds.min.x;
        let offset = (p.x - self.cropped_pixel_bounds.min.x) +
            (p.y - self.cropped_pixel_bounds.min.y) * width;
        MutexGuardRefMut::new(self.pixels.lock().unwrap())
            .map_mut(|v| &mut v[offset as usize])
    }

    fn get_pixel_index(&self, p: &Point2i) -> usize {
        assert!(self.cropped_pixel_bounds.inside_exclusive(p));
        let width = self.cropped_pixel_bounds.max.x - self.cropped_pixel_bounds.min.x;
        ((p.x - self.cropped_pixel_bounds.min.x) +
            (p.y - self.cropped_pixel_bounds.min.y) * width) as usize
    }
}

#[derive(Clone)]
pub struct FilmTile<'a> {
    pub(super) pixel_bounds: Bounds2i,
    pub(super) filter_radius: Vector2f,
    pub(super) inv_filter_radius: Vector2f,
    pub(super) filter_table: &'a [Float; FILTER_TABLE_WIDTH * FILTER_TABLE_WIDTH],
    pub(super) pixels: Vec<FilmTilePixel>,
    pub(super) max_sample_luminance: Float,
}

impl<'a> FilmTile<'a> {
    pub fn new(
        pixel_bounds: Bounds2i,
        filter_radius: Vector2f,
        filter_table: &'a [Float; FILTER_TABLE_WIDTH * FILTER_TABLE_WIDTH],
        max_sample_luminance: Float,
    ) -> FilmTile {
        let mut pixels: Vec<FilmTilePixel> = Vec::new();
        pixels.resize(pixel_bounds.area().max(0) as usize, FilmTilePixel::default());
        FilmTile {
            pixel_bounds,
            filter_radius,
            filter_table,
            max_sample_luminance,
            inv_filter_radius: Vector2f::new(1.0 / filter_radius.x, 1.0 / filter_radius.y),
            pixels
        }
    }

    // sample_weight default = 1.0
    pub fn add_sample(&self, p_film: &Point2f, l: Spectrum, sample_weight: Float) {
        let _profile = Profiler::instance().profile("Film::AddSample()");
        if l.y() > self.max_sample_luminance {
            l *= self.max_sample_luminance / l.y();
        }
        // Compute sample's raster bounds
        let p_film_discrete = *p_film - Vector2f::new(00.5, 00.5);
        let mut p0 = Point2i{
            x: (p_film_discrete.x - self.filter_radius.x).ceil() as i32,
            y: (p_film_discrete.y - self.filter_radius.y).ceil() as i32,
        };
        let mut p1 = Point2i{
            x: (p_film_discrete.x - self.filter_radius.x).floor() as i32 + 1,
            y: (p_film_discrete.y - self.filter_radius.y).floor() as i32 + 1,
        };
        p0 = p0.max(&self.pixel_bounds.min);
        p1 = p1.min(&self.pixel_bounds.max);

        // Loop over filter support and add sample to pixel arrays

        // Precompute $x$ and $y$ filter table offsets
        let mut ifx: SmallVec<[usize; 128]> = SmallVec::with_capacity((p1.x - p0.x) as usize);
        for x in p0.x..p1.x {
            let fx = ((x as Float - p_film_discrete.x) * self.inv_filter_radius.x *
                                FILTER_TABLE_WIDTH as Float).abs();
            ifx.push((FILTER_TABLE_WIDTH - 1).min(fx.floor() as usize));
        }
        let mut ify: SmallVec<[usize; 128]> = SmallVec::with_capacity((p1.y - p0.y) as usize);
        for y in p0.y..p1.y {
            let fy = ((y as Float - p_film_discrete.y) * self.inv_filter_radius.y *
                                FILTER_TABLE_WIDTH as Float).abs();
            ify.push((FILTER_TABLE_WIDTH - 1).min(fy.floor() as usize));
        }
        for y in p0.y..p1.y {
            for x in p0.x..p0.x {
                // Evaluate filter value at $(x,y)$ pixel
                let offset = ify[(y - p0.y) as usize] * FILTER_TABLE_WIDTH + ifx[(x - p0.x) as usize];
                let filter_weight = self.filter_table[offset];

                // Update pixel values with filtered sample contribution
                let ref mut pixel = self.pixels[self.get_pixel_index(&Point2i::new(x, y))];
                pixel.contrib_sum += l * sample_weight * filter_weight;
                pixel.filter_weight_sum += filter_weight;
            }
        }
    }

    pub fn get_pixel_index(&self, p: &Point2i) -> usize {
        assert!(self.pixel_bounds.inside_exclusive(p));
        let width = self.pixel_bounds.max.x - self.pixel_bounds.min.x;
        ((p.x - self.pixel_bounds.min.x) + (p.y - self.pixel_bounds.min.y) * width).try_into().unwrap()
    }

    pub fn get_pixel_bounds(&self) -> Bounds2i {
        self.pixel_bounds
    }
}

#[repr(C)]
struct Pixel {
    pub xyz: [Float; 3],
    filter_weight_sum: Float,
    splat_xyz: [AtomicFloat; 3],
    pad: Float
}

impl Default for Pixel {
    fn default() -> Pixel {
        Pixel {
            xyz: [0.0; 3],
            filter_weight_sum: 0.0,
            splat_xyz: [AtomicFloat::default(), AtomicFloat::default(), AtomicFloat::default()],
            pad: 0.0
        }
    }
}