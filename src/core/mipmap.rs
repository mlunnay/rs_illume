use super::pbrt::{Float, is_power_of_2, round_up_pow2_i32, mod_t, log2_int_i32};
use super::stats_accumulator::StatsAccumulator;
use super::profiler::Profiler;
use super::geometry::{Point2i, Point2f, Vector2f};
use super::spectrum::{RGBSpectrum, SampledSpectrum};
use super::memory::BlockedArray;
use super::texture::lanczos;
use std::mem::size_of;
use std::ops::*;
use parking_lot::Once;
use rayon::prelude::*;
use num::Float as NumFloat;

#[derive(Debug, Copy, Clone, PartialOrd, PartialEq)]
pub enum ImageWrap {
    Repeat,
    Black,
    Clamp
}

#[derive(Debug, Default, Copy, Clone)]
pub struct ResampleWeight {
    pub first_texel: u32,
    pub weight: [Float; 4]
}

const WEIGHT_LUT_SIZE: usize = 128;
static mut WEIGHT_LUT: [Float; WEIGHT_LUT_SIZE] = [0.0; WEIGHT_LUT_SIZE];

pub struct MIPMap<T> {
    do_trilinear: bool,
    max_anisotropy: Float,
    wrap_mode: ImageWrap,
    resolution: Point2i,
    pyramid: Vec<BlockedArray<T>>,
}

impl<T> MIPMap<T>
where
T: Default +
    Clone +
    Copy +
    Default +
    AddAssign +
    Clamp +
    Lerp +
    Mul<Float, Output=T> +
    Div<Float, Output=T> +
    Add<Output=T> + 
    Send +
    Sync +
    num::Zero
{
    pub fn new(
        mut resolution: Point2i,
        img: &[T],
        do_trilinear: bool,
        max_anisotropy: Float,
        wrap_mode: ImageWrap
    ) -> MIPMap<T> {
        let _profile = Profiler::instance().profile("MIP map generation");
        let mut mipmap = MIPMap::<T>{
            do_trilinear,
            max_anisotropy,
            wrap_mode,
            resolution,
            pyramid: Vec::new()
        };

        let mut resampled_image: Vec<T> = Vec::new();
        if !is_power_of_2(resolution.x) || !is_power_of_2(resolution.y) {
            // Resample image to power-of-two resolution
            let res_pow2 = Point2i::new(round_up_pow2_i32(resolution.x), round_up_pow2_i32(resolution.y));
            info!("Resampling MIPMap from {} to {}. Ration= {}", resolution, res_pow2,
                ((res_pow2.x * res_pow2.y) as Float / (resolution.x * resolution.y) as Float));
            // Resample image in $s$ direction
            let s_weights = MIPMap::resample_weights(resolution.x, res_pow2.x);
             resampled_image.resize((res_pow2.x * res_pow2.y) as usize, Default::default());

            // Apply _sWeights_ to zoom in $s$ direction
            (0..resolution.y as usize).into_par_iter()//.chunks(16)
                .for_each(|t| {
                    for s in 0..res_pow2.x as usize {
                        // Compute texel $(s,t)$ in $s$-zoomed image
                        // resampled_image is already defaulted
                        for j in 0..4 {
                            let mut orig_s = s_weights[s].first_texel + j as u32;
                            if wrap_mode == ImageWrap::Repeat {
                                orig_s = mod_t(orig_s, resolution.x as u32);
                            } else if wrap_mode == ImageWrap::Clamp {
                                orig_s = num::clamp(orig_s, 0, resolution.x as u32 - 1);
                            }
                            if orig_s >= 0 && orig_s < resolution.x as u32 {
                                resampled_image[t * res_pow2.x as usize + s] += 
                                    img[t * resolution.x as usize + orig_s as usize] *
                                    s_weights[s].weight[j];
                            }
                        }
                    }
                });

            // Resample image in $t$ direction
            let t_weights = MIPMap::resample_weights(resolution.y, res_pow2.y);
            (0..res_pow2.x as usize).into_par_iter().for_each(|s| {
                let mut work_data: Vec<T> = vec![Default::default(); res_pow2.y as usize];
                for t in 0..res_pow2.y as usize {
                    for j in 0..4 {
                        let offset = t_weights[t].first_texel as usize + j;
                        if wrap_mode == ImageWrap::Repeat {
                            offset = mod_t(offset, resolution.y as usize);
                        } else if wrap_mode == ImageWrap::Clamp {
                            offset = num::clamp(offset, 0, resolution.y as usize - 1);
                        }
                        if offset >= 0 && offset < resolution.y as usize {
                            work_data[t] += resampled_image[offset * res_pow2.x as usize + s] *
                                t_weights[t].weight[j];
                        }
                    }
                }
                for t in 0..res_pow2.y as usize {
                    resampled_image[t * res_pow2.x as usize + s] = Clamp::clamp(work_data[t], 0.0, Float::infinity());
                }
            });
            resolution = res_pow2;
        }
        // Initialize levels of MIPMap from image
        let n_levels = 1 + log2_int_i32(resolution.x.max(resolution.y)) as usize;
        mipmap.pyramid.reserve(n_levels);

        // Initialize most detailed level of MIPMap
        mipmap.pyramid.push(BlockedArray::new_from(
            resolution.x as usize,
            resolution.y as usize,
            if !resampled_image.is_empty() { &resampled_image } else { img }
        ));
        for i in 1..n_levels {
            // Initialize $i$th MIPMap level from $i-1$st level
            let s_res = 1.max(mipmap.pyramid[i - 1].u_size() / 2);
            let t_res = 1.max(mipmap.pyramid[i - 1].v_size() / 2);
            let mut ba = BlockedArray::<T>::new(s_res, t_res);

            // Filter four texels from finer level of pyramid
            (0..t_res).into_par_iter().for_each(|t| {
                for s in 0..s_res {
                    ba[(s,t)] = (*mipmap.texel(i - 1, 2 * s, 2 * t) +
                        *mipmap.texel(i - 1, 2 * s + 1, 2 * t) +
                        *mipmap.texel(i - 1, 2 * s, 2 * t + 1) +
                        *mipmap.texel(i - 1, 2 * s + 1, 2 * t + 1)) * 0.25 as Float
                }
            });
            mipmap.pyramid.push(ba);
        }

        // Initialize EWA filter weights if needed
        static ONCE: Once = Once::new();
        unsafe{
            ONCE.call_once(|| {
                for i in 0..WEIGHT_LUT_SIZE {
                    let alpha: Float = 2.0;
                    let r2 = i as Float / (WEIGHT_LUT_SIZE - 1) as Float;
                    WEIGHT_LUT[i] = (-alpha * r2).exp() - (-alpha).exp();
                }
            });
        }

        StatsAccumulator::instance().report_memory_counter(String::from("Memory/Texture MIP maps"), ((4 * mipmap.resolution[0] * mipmap.resolution[1] * size_of::<T>() as i32) / 3) as i64);
        mipmap
    }

    pub fn texel(&self, level: usize, mut s: usize, mut t: usize) -> &T {
        assert!(level < self.pyramid.len());
        let l = &self.pyramid[level];
        // Compute texel $(s,t)$ accounting for boundary conditions
        match self.wrap_mode {
            ImageWrap::Repeat => {
                s = mod_t(s, l.u_size());
                t = mod_t(t, l.v_size());
            }
            ImageWrap::Clamp => {
                s = num::clamp(s, 0, l.u_size() - 1);
                t = num::clamp(t, 0, l.v_size() - 1);
            }
            ImageWrap::Black => {
                if s < 0 || s >= l.u_size() || t < 0 || t >= l.v_size() {
                    return &num::Zero::zero();
                }
            }
        }
        &l[(s,t)]
    }

    fn resample_weights(old_res: i32, new_res: i32) -> Vec<ResampleWeight> {
        assert!(new_res >= old_res);
        let mut wt: Vec<ResampleWeight> = Vec::with_capacity(new_res as usize);
        let filter_width: Float = 2.0;
        for i in 0..new_res as usize {
            // Compute image resampling weights for _i_th texel
            let center = (i as Float + 0.5) * old_res as Float / new_res as Float;
            wt[i].first_texel = (center - filter_width + 0.5).floor() as u32;
            for j in 0..4 {
                let pos = wt[i].first_texel as Float + j as Float + 0.5;
                wt[i].weight[j] = lanczos((pos - center) / filter_width, 2.0);
            }

            // Normalize filter weights for texel resampling
            let inv_sum_wts = 1.0 / (wt[i].weight[0] + wt[i].weight[1] +
                wt[i].weight[2] + wt[i].weight[3]) as Float;
            for j in 0..4 {
                wt[i].weight[j] *= inv_sum_wts;
            }
        }
        wt
    }

    pub fn width(&self) ->i32 {
        self.resolution.x
    }

    pub fn height(&self) -> i32 {
        self.resolution.y
    }

    pub fn levels(&self) -> usize {
        self.pyramid.len()
    }

    /// default width is 0.0
    pub fn lookup(&self, st: &Point2f, width: Float) -> T {
        StatsAccumulator::instance().report_counter(String::from("Texture/Trilinear lookups"), 1);
        let _p = Profiler::instance().profile("MIPMap::lookup() (trilinear)");
        // Compute MIPMap level for trilinear filtering
        let level = self.levels() as Float - 1.0 + width.max(1e-8).log2();

        if level < 0.0 {
            self.triangle(0, st)
        } else if level >= self.levels() as Float - 1.0 {
            *self.texel(self.levels() - 1, 0, 0)
        } else {
            let i_level = level.floor() as usize;
            let delta = level - i_level as Float;
            self.triangle(i_level, st).lerp(&self.triangle(i_level + 1, st), delta)
        }
    }

    pub fn lookup_vec(&self, st: &Point2f, mut dstdx: Vector2f, mut dstdy: Vector2f) -> T {
        if self.do_trilinear {
            let width = dstdx.x.abs().max(dstdx.y.abs())
                .max(dstdy.y.abs().max(dstdy.y.abs()));
            return self.lookup(st, 2.0 * width);
        }
        StatsAccumulator::instance().report_counter(String::from("Texture/EWA lookups"), 1);
        let _p = Profiler::instance().profile("MIPMap::lookup() (EWA)");
        // Compute ellipse minor and major axes
        if dstdx.length_squared() < dstdy.length_squared() {
            std::mem::swap(&mut dstdx, &mut dstdy);
        }
        let major_length = dstdx.length();
        let mut minor_length = dstdy.length();

        // Clamp ellipse eccentricity if too large
        if minor_length * self.max_anisotropy < major_length && minor_length > 0.0 {
            let scale = major_length / (minor_length * self.max_anisotropy);
            dstdy *= scale;
            minor_length *= scale;
        }
        if minor_length == 0.0 {
            return self.triangle(0, st);
        }

        // Choose level of detail for EWA lookup and perform EWA filtering
        let lod = (self.levels() as Float - 1.0 + minor_length.log2()).max(0.0);
        let ilod = lod.floor() as usize;
        self.ewa(ilod, *st, dstdx, dstdy).lerp(&self.ewa(ilod + 1, *st, dstdx, dstdy), lod - ilod as Float)
    }

    fn triangle(&self, mut level: usize, st: &Point2f) -> T {
        level = num::clamp(level, 0, self.levels() - 1);
        let s = st.x * self.pyramid[level].u_size() as Float - 0.5;
        let t = st.y * self.pyramid[level].v_size() as Float - 0.5;
        let s0 = s.floor();
        let t0 = t.floor();
        let ds = s - s0;
        let dt = t - t0;
        *self.texel(level, s0 as usize, t0 as usize) * (1.0 - ds) * (1.0 - dt) +
            *self.texel(level, s0 as usize, t0 as usize + 1) * (1.0 - ds) * dt +
            *self.texel(level, s0 as usize + 1, t0 as usize) * ds * (1.0 - dt) +
            *self.texel(level, s0 as usize + 1, t0 as usize + 1) * ds * dt
    }

    fn ewa(
        &self,
        level: usize,
        mut st: Point2f,
        mut dst0: Vector2f,
        mut dst1: Vector2f
    ) -> T {
        if level >= self.levels() {
            return *self.texel(self.levels() - 1, 0, 0);
        }
        // Convert EWA coordinates to appropriate scale for level
        st.x = st.x * self.pyramid[level].u_size() as Float - 0.5;
        st.y = st.y * self.pyramid[level].v_size() as Float - 0.5;
        dst0.x *= self.pyramid[level].u_size() as Float;
        dst0.y *= self.pyramid[level].v_size() as Float;
        dst1.x *= self.pyramid[level].u_size() as Float;
        dst1.y *= self.pyramid[level].v_size() as Float;

        // Compute ellipse coefficients to bound EWA filter region
        let mut a = dst0.y * dst0.y + dst1.y * dst1.y + 1.0;
        let mut b = -2.0 * (dst0.x * dst0.y + dst1.x * dst1.y);
        let mut c = dst0.x * dst0.x + dst1.x * dst1.x + 1.0;
        let inv_f = 1.0 / (a * c - b * b * 0.25);
        a *= inv_f;
        b *= inv_f;
        c *= inv_f;

        // Compute the ellipse's $(s,t)$ bounding box in texture space
        let det = -b * b + 4.0 * a * c;
        let inv_det = 1.0 / det;
        let u_sqrt = (det * c).sqrt();
        let v_sqrt = (a * det).sqrt();
        let s0 = (st.x - 2.0 * inv_det * u_sqrt).ceil() as usize;
        let s1 = (st.x + 2.0 * inv_det * u_sqrt).floor() as usize;
        let t0 = (st.y - 2.0 * inv_det * v_sqrt).ceil() as usize;
        let t1 = (st.y + 2.0 * inv_det * v_sqrt).floor() as usize;

        // Scan over ellipse bound and compute quadratic equation
        let mut sum: T = num::Zero::zero();
        let mut sum_wts: Float = 0.0;
        for it in t0..=t1 {
            let tt = it as Float - st.y;
            for is in s0..=s1 {
                let ss = is as Float - st.x;
                // Compute squared radius and filter texel if inside ellipse
                let r2 = a * ss * ss + b * ss * tt + c * tt * tt;
                if r2 < 1.0 {
                    let index = ((r2 * WEIGHT_LUT_SIZE as Float) as usize).min(WEIGHT_LUT_SIZE - 1);
                    let weight = WEIGHT_LUT[index];
                    sum += *self.texel(level, is, it);
                    sum_wts += weight;
                }
            }
        }
        sum / sum_wts
    }
}

pub trait Clamp {
    fn clamp(self, min: Float, max: Float) -> Self;
}

impl Clamp for Float {
    fn clamp(self, min: Float, max: Float) -> Self {
        num::clamp(self, min, max)
    }
}

impl Clamp for SampledSpectrum {
    fn clamp(self, min: Float, max: Float) -> Self {
        self.clamp(min, max)
    }
}

impl Clamp for RGBSpectrum {
    fn clamp(self, min: Float, max: Float) -> Self {
        self.clamp(min, max)
    }
}

pub trait Lerp {
    fn lerp(&self, o: &Self, t: Float) -> Self;
}

impl Lerp for Float {
    fn lerp(&self, o: &Self, t: Float) -> Self {
        super::pbrt::lerp(t, *self, *o)
    }
}

impl Lerp for SampledSpectrum {
    fn lerp(&self, o: &Self, t: Float) -> Self {
        self.lerp(o, t)
    }
}

impl Lerp for RGBSpectrum {
    fn lerp(&self, o: &Self, t: Float) -> Self {
        self.lerp(o, t)
    }
}