use super::pbrt::Float;
use super::geometry::{Point2i, Point2f};
use super::camera::CameraSample;
use super::rng::Rng;
use super::profiler::Profiler;

pub trait Sampler {
    fn get_samples_per_pixel(&self) -> u64;

    fn start_pixel(&self, p: &Point2i);

    fn get_1d(&self) -> Float;

    fn get_2d(&self) -> Point2i;

    fn request_1d_array(&self, n: usize);

    fn request_2d_array(&self, n: usize);

    fn get_camera_sample(&self, p_raster: &Point2i) -> CameraSample;

    fn clone_with_seed(&self, seed: u64) -> Box<Self>;

    fn start_next_sample(&self) -> bool;

    fn set_sample_number(&self, sample_num: u64) -> bool;

    fn current_sample_number(&self) -> usize;
}

pub trait SamplerChild {
    fn get_1d(&mut self) -> Float;
    fn get_2d(&mut self) -> Point2f;
    fn round_count(&self, n: usize) -> usize { n }
}

pub struct SamplerBase {
    pub samples_per_pixel: u64,
    pub current_pixel: Point2i,
    pub current_pixel_sample_index: usize,
    pub samples_1d_array_sizes: Vec<u32>,
    pub samples_2d_array_sizes: Vec<u32>,
    pub sample_array_1d: Vec<Vec<Float>>,
    pub sample_array_2d: Vec<Vec<Point2f>>,
    array_1d_offset: usize,
    array_2d_offset: usize
}

impl SamplerBase {
    pub fn new(samples_per_pixel: u64) -> SamplerBase {
        SamplerBase{
            samples_per_pixel,
            current_pixel: Point2i::default(),
            current_pixel_sample_index: 0,
            samples_1d_array_sizes: Vec::new(),
            samples_2d_array_sizes: Vec::new(),
            sample_array_1d: Vec::new(),
            sample_array_2d: Vec::new(),
            array_1d_offset: 0,
            array_2d_offset: 0
        }
    }

    pub fn start_pixel(&mut self, p: &Point2i) {
        self.current_pixel = *p;
        self.current_pixel_sample_index = 0;
        // Reset array offsets for next pixel sample
        self.array_1d_offset = 0;
        self.array_2d_offset = 0;
    }

    pub fn get_camera_sample(&self, child: &impl SamplerChild, p_raster: &Point2i) -> CameraSample {
        CameraSample {
            p_film: child.get_2d() + p_raster.into(),
            time: child.get_1d(),
            p_lens: child.get_2d()
        }
    }

    pub fn request_1d_array(&self, child: &impl SamplerChild, n: usize) {
        assert_eq!(child.round_count(n), n);
        self.samples_1d_array_sizes.push(n as u32);
        let size = n * self.samples_per_pixel as usize;
        self.sample_array_1d.push(vec![0.0; size]);
    }

    pub fn request_2d_array(&self, child: &impl SamplerChild, n: usize) {
        assert_eq!(child.round_count(n), n);
        self.samples_2d_array_sizes.push(n as u32);
        let size = n * self.samples_per_pixel as usize;
        self.sample_array_2d.push(vec![Point2f::default(); size]);
    }

    pub fn get_1d_array(&mut self, n: usize) -> Option<&[Float]> {
        if self.array_1d_offset == self.sample_array_1d.len() {
            return None;
        }
        assert_eq!(self.samples_1d_array_sizes[self.array_1d_offset] as usize, n);
        assert!(self.current_pixel_sample_index < self.samples_per_pixel as usize);
        self.array_1d_offset += 1;
        let start = self.current_pixel_sample_index * n;
        Some(&self.sample_array_1d[self.array_1d_offset - 1][start..start + n])
    }

    pub fn get_2d_array(&mut self, n: usize) -> Option<&[Point2f]> {
        if self.array_2d_offset == self.sample_array_2d.len() {
            return None;
        }
        assert_eq!(self.samples_2d_array_sizes[self.array_2d_offset] as usize, n);
        assert!(self.current_pixel_sample_index < self.samples_per_pixel as usize);
        self.array_2d_offset += 1;
        let start = self.current_pixel_sample_index * n;
        Some(&self.sample_array_2d[self.array_2d_offset - 1][start..start + n])
    }

    pub fn start_next_sample(&mut self) -> bool {
        // Reset array offsets for next pixel sample
        self.array_1d_offset = 0;
        self.array_2d_offset = 0;
        self.current_pixel_sample_index += 1;
        self.current_pixel_sample_index < self.samples_per_pixel as usize
    }

    pub fn set_sample_number(&mut self, sample_num: usize) -> bool {
        // Reset array offsets for next pixel sample
        self.array_1d_offset = 0;
        self.array_2d_offset = 0;
        self.current_pixel_sample_index = sample_num;
        self.current_pixel_sample_index < self.samples_per_pixel as usize
    }
}

pub struct PixelSampler {
    sampler: SamplerBase,
    samples_1d: Vec<Vec<Float>>,
    samples_2d: Vec<Vec<Point2f>>,
    current_1d_dimension: u32,
    current_2d_dimension: u32,
    rng: Rng
}

impl PixelSampler {
    pub fn new(samples_per_pixel: u64, n_sampled_dimensions: u32) -> Self {
        let mut samples_1d: Vec<Vec<Float>> = Vec::new();
        let mut samples_2d: Vec<Vec<Point2f>> = Vec::new();
        for i in 0..n_sampled_dimensions as usize {
            samples_1d.push(vec![0.0; samples_per_pixel as usize]);
            samples_2d.push(vec![Point2f::default(); samples_per_pixel as usize]);
        }
        PixelSampler {
            sampler: SamplerBase::new(samples_per_pixel),
            samples_1d,
            samples_2d,
            current_1d_dimension: 0,
            current_2d_dimension: 0,
            rng: Rng::default()
        }
    }

    pub fn samples_per_pixel(&self) -> u64 {
        self.sampler.samples_per_pixel
    }

    pub fn start_pixel(&mut self, p: &Point2i) {
        self.sampler.start_pixel(p);
    }

    pub fn get_camera_sample(&self, p_raster: &Point2i) -> CameraSample {
        self.sampler.get_camera_sample(self, p_raster)
    }

    pub fn request_1d_array(&self, child: &impl SamplerChild, n: usize) {
        self.sampler.request_1d_array(self, n);
    }

    pub fn request_2d_array(&self, child: &impl SamplerChild, n: usize) {
        self.sampler.request_2d_array(self, n);
    }

    pub fn get_1d_array(&mut self, n: usize) -> Option<&[Float]> {
        self.sampler.get_1d_array(n)
    }

    pub fn get_2d_array(&mut self, n: usize) -> Option<&[Point2f]> {
        self.sampler.get_2d_array(n)
    }

    pub fn start_next_sample(&mut self) -> bool {
        self.current_1d_dimension = 0;
        self.current_2d_dimension = 0;
        self.sampler.start_next_sample()
    }

    pub fn set_sample_number(&mut self, sample_num: usize) -> bool {
        self.current_1d_dimension = 0;
        self.current_2d_dimension = 0;
        self.sampler.set_sample_number(sample_num)
    }
}

impl SamplerChild for PixelSampler {
    fn get_1d(&mut self) -> Float {
        let _profile = Profiler::instance().profile("Sampler::GetSample[12]D()");
        assert!(self.sampler.current_pixel_sample_index < self.sampler.samples_per_pixel as usize);
        if (self.current_1d_dimension as usize) < self.samples_1d.len() {
            self.current_1d_dimension += 1;
            self.samples_1d[self.current_1d_dimension as usize - 1][self.sampler.current_pixel_sample_index]
        } else {
            self.rng.uniform_float()
        }
    }

    fn get_2d(&mut self) -> Point2f {
        let _profile = Profiler::instance().profile("Sampler::GetSample[12]D()");
        assert!(self.sampler.current_pixel_sample_index < self.sampler.samples_per_pixel as usize);
        if (self.current_2d_dimension as usize) < self.samples_2d.len() {
            self.current_2d_dimension += 1;
            self.samples_2d[self.current_2d_dimension as usize - 1][self.sampler.current_pixel_sample_index]
        } else {
            Point2f::new(self.rng.uniform_float(), self.rng.uniform_float())
        }
    }
}



pub trait GlobalSaplerChild {
    fn get_index_for_sample(&self, sample_num: usize) -> usize;
    fn sample_dimension(&self, index: usize, dimension: u32) -> Float;
}

pub struct GlobalSampler {
    pub sampler: SamplerBase,
    pub dimension: u32,
    pub interval_sample_index: usize,
    pub array_end_dim: u32
}

impl GlobalSampler {
    const array_start_dim: usize = 5;

    pub fn new(samples_per_pixel: u64) -> GlobalSampler {
        GlobalSampler {
            sampler: SamplerBase::new(samples_per_pixel),
            dimension: 0,
            interval_sample_index: 0,
            array_end_dim: 0
        }
    }

    pub fn samples_per_pixel(&self) -> u64 {
        self.sampler.samples_per_pixel
    }

    pub fn start_pixel(&mut self, child: &impl GlobalSaplerChild, p: &Point2i) {
        let _profile = Profiler::instance().profile("Sampler::StartPixelSample()");
        self.sampler.start_pixel(p);
        self.dimension = 0;
        self.interval_sample_index = child.get_index_for_sample(0);
        // Compute _arrayEndDim_ for dimensions used for array samples
        self.array_end_dim = (GlobalSampler::array_start_dim + self.sampler.sample_array_1d.len() + 2 * self.sampler.sample_array_2d.len()) as u32;

        // Compute 1D array samples for _GlobalSampler_
        for i in 0..self.sampler.samples_1d_array_sizes.len() {
            let n_samples = self.sampler.samples_1d_array_sizes[i] as usize * self.sampler.samples_per_pixel as usize;
            for j in 0..n_samples {
                let index = child.get_index_for_sample(j);
                self.sampler.sample_array_1d[i][j] = child.sample_dimension(index, (GlobalSampler::array_start_dim + i) as u32);
            }
        }

        // Compute 2D array samples for _GlobalSampler_
        let mut dim = GlobalSampler::array_start_dim + self.sampler.samples_1d_array_sizes.len();
        for i in 0..self.sampler.samples_2d_array_sizes.len() {
            let n_samples = self.sampler.samples_2d_array_sizes[i] as usize * self.sampler.samples_per_pixel as usize;
            for j in 0.. n_samples {
                let idx = child.get_index_for_sample(j);
                self.sampler.sample_array_2d[i][j].x = child.sample_dimension(idx, dim as u32);
                self.sampler.sample_array_2d[i][j].y = child.sample_dimension(idx, dim as u32 + 1);
            }
            dim += 2;
        }
        assert_eq!(self.array_end_dim, dim as u32);
    }

    pub fn start_next_sample(&mut self, child: &impl GlobalSaplerChild) -> bool {
        self.dimension = 0;
        self.interval_sample_index = child.get_index_for_sample(self.sampler.current_pixel_sample_index + 1);
        self.sampler.start_next_sample()
    }

    pub fn set_sample_number(&mut self, child: &impl GlobalSaplerChild, sample_num: usize) -> bool {
        self.dimension = 0;
        self.interval_sample_index = child.get_index_for_sample(sample_num);
        self.sampler.set_sample_number(sample_num)
    }
}

/// implements the SamplerChild trait for a given struct T where GlobalSampler instance is at the given field.
macro_rules! globalsampler_samplerchild_impl {
    ($T: ident, $field: ident) => {
        impl SamplerChild for $T {
            fn get_1d(&mut self) -> Float {
                let _profile = Profiler::instance().profile("Sampler::StartPixelSample()");
                if self.$field.dimension >= GlobalSampler::array_start_dim as u32 && self.$field.dimension < self.$field.array_end_dim {
                    self.$field.dimension = self.$field.array_end_dim;
                }
                self.sample_dimension(self.$field)
            }
    
            fn get_2d(&mut self) -> Point2f {
                let _profile = Profiler::instance().profile("Sampler::StartPixelSample()");
                if self.$field.dimension >= GlobalSampler::array_start_dim as u32 && self.$field.dimension < self.$field.array_end_dim {
                    self.$field.dimension = self.$field.array_end_dim;
                }
                let p = Point2f::new(self.sample_dimension(self.$field.interval_sample_index, self.$field.dimension),
                    self.sample_dimension(self.$field.interval_sample_index, self.$field.dimension + 1));
                self.$field.dimension += 2;
                p
            }
        }
    };
}
