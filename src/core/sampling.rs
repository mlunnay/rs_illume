//! 1D and 2D Sampling structures and functions.

use super::rng::{Rng, ONE_MINUS_EPSILON};
use super::pbrt::{Float, find_interval, lerp, consts::{PI, INV_PI, INV_2_PI, INV_4_PI, FRAC_PI_4, FRAC_PI_2}};
use super::geometry::{Point2f, Vector3f, Vector2f};
use std::sync::Arc;

#[derive(Debug, Clone)]
pub struct Distribution1D  {
    /// Probability distribution function.
    pub func: Vec<Float>,
    /// Cumulative distribution function.
    pub cdf: Vec<Float>,
    // The integral of the probability distribution function.
    pub func_int: Float
}

impl Distribution1D {
    /// Create a new Distribution1D by copying the passed Vec, calculating the cumulative distribution function for the values and store the inegral of the function.
    pub fn new(f: &Vec<Float>) -> Distribution1D {
        let n = f.len();
        let mut cdf: Vec<Float> = Vec::new();
        // Compute integral of step function at $x_i$
        cdf.push(0.0);
        for i in 1..n+1 {
            cdf.push(cdf[i - 1] + f[i - 1] / n as Float);
        }

        // Transform step function integral into CDF
        let func_int = cdf[n];
        if func_int == 0.0 {
            for i in 1..n+1 { cdf[i] = i as Float / n as Float };
        } else {
            for i in 1..n+1 { cdf[i] /= func_int };
        }

        Distribution1D{
            func: f.clone(),
            cdf,
            func_int
        }
    }

    pub fn count(&self) -> usize {
        self.func.len()
    }

    /// Samples the distribution for the given random u and returns the value in the range [0,1).
    /// If pdf is not None it is set to the value of the pdf.
    /// if off is not None, it returns the offset into the array of function values of the largest index where the CDF was less than or equal to u. 
    pub fn sample_continuous(&self, u: Float, pdf: Option<&mut Float>, off: Option<&mut usize>) -> Float {
        // Find surrounding CDF segments and _offset_
        let offset = find_interval(self.cdf.len(), |i| self.cdf[i] <= u);
        if let Some(off) = off {
            *off = offset;
        }
        // Compute offset along CDF segment
        let du = u - self.cdf[offset];
        if self.cdf[offset + 1] - self.cdf[offset] > 0.0 {
            assert!(self.cdf[offset + 1] > self.cdf[offset]);
            du /= self.cdf[offset + 1] - self.cdf[offset];
        }
        assert!(!du.is_nan());

        // Compute PDF for sampled offset
        if let Some(pdf) = pdf {
            *pdf = if self.func_int > 0.0 { self.func[offset] / self.func_int } else { 0.0 };
        }

        // Return $x\in{}[0,1)$ corresponding to sample
        (offset as Float + du) / self.count() as Float
    }

    /// Samples the CDF that surrounds the value u.
    pub fn sample_descrete(&self, u: Float, pdf: Option<&mut Float>, u_remapped: Option<&mut Float>) -> usize {
        // Find surrounding CDF segments and _offset_
        let offset = find_interval(self.cdf.len(), |i| self.cdf[i] <= u);
        if let Some(pdf) = pdf {
            *pdf = if self.func_int > 0.0 { self.func[offset] / (self.func_int * self.count() as Float) } else { 0.0 };
        }
        if let Some(u_remapped) = u_remapped {
            *u_remapped = (u - self.cdf[offset]) / (self.cdf[offset + 1] - self.cdf[offset]);
            assert!(*u_remapped >= 0.0 && *u_remapped <= 1.0);
        }
        offset
    }

    /// Compute the PDF for sampling a given value from the discrete PDF.
    pub fn descrete_pdf(&self, index: usize) -> Float {
        assert!(index < self.func.len());
        self.func[index] / (self.func_int * self.func.len() as Float)
    }
}

#[derive(Debug, Clone)]
pub struct Distribution2D {
    pub p_conditional_v: Vec<Arc<Distribution1D>>,
    pub p_marginal: Arc<Distribution1D>,
}

impl Distribution2D {
    pub fn new(func: Vec<Float>, nu: i32, nv: i32) -> Self {
        let mut p_conditional_v: Vec<Arc<Distribution1D>> = Vec::new();
        for v in 0..nv {
            // compute conditional sampling distribution for $\tilde{v}$
            let f: Vec<Float> = func[(v * nu) as usize..((v + 1) * nu) as usize].to_vec();
            p_conditional_v.push(Arc::new(Distribution1D::new(&f)));
        }
        // compute marginal sampling distribution $p[\tilde{v}]$
        let mut marginal_func: Vec<Float> = Vec::with_capacity(nv as usize);
        for v in 0..nv {
            marginal_func.push(p_conditional_v[v as usize].func_int);
        }
        let p_marginal: Arc<Distribution1D> = Arc::new(Distribution1D::new(&marginal_func));
        Distribution2D {
            p_conditional_v,
            p_marginal,
        }
    }
    pub fn sample_continuous(&self, u: &Point2f, pdf: &mut Float) -> Point2f {
        let mut pdfs: [Float; 2] = [0.0 as Float; 2];
        let mut v: usize = 0_usize;
        let d1: Float = self
            .p_marginal
            .sample_continuous(u[1], Some(&mut (pdfs[1])), Some(&mut v));
        let d0: Float = self.p_conditional_v[v].sample_continuous(u[0], Some(&mut (pdfs[0])), None);
        *pdf = pdfs[0] * pdfs[1];
        Point2f { x: d0, y: d1 }
    }
    pub fn pdf(&self, p: &Point2f) -> Float {
        let iu: usize = num::clamp(
            (p[0] * self.p_conditional_v[0].count() as Float) as usize,
            0,
            self.p_conditional_v[0].count() - 1_usize,
        );
        let iv: usize = num::clamp(
            (p[1] * self.p_marginal.count() as Float) as usize,
            0,
            self.p_marginal.count() - 1_usize,
        );
        self.p_conditional_v[iv].func[iu] / self.p_marginal.func_int
    }
}

/// randomly permutes an array of sample values, each of which has n_dimensions dimensions.
pub fn shuffle<T>(samp: &[T], count: u32, n_dimensions: u32, rng: &Rng) {
    for i in 0..count {
        let other = i + rng.uniform_uint32_bounded(samp.len() as u32 - i);
        for j in 0..n_dimensions {
            samp.swap((n_dimensions * i + j) as usize, (n_dimensions * other + j) as usize);
        }
    }
}

/// Return a random point in the unit circle using rejection.
pub fn rejection_sample_disk(rng: &Rng) -> Point2f {
    let p: Point2f;
    loop {
        p.x = 1.0 - 2.0 * rng.uniform_float();
        p.y = 1.0 - 2.0 * rng.uniform_float();
        if p.x * p.x + p.y * p.y <= 1.0 {
            return p;
        }
    }
}

/// Sample a direction on the hemisphere uniformly with respect to solid angle.
pub fn uniform_sample_hemisphere(u: &Point2f) -> Vector3f {
    let z = u.x;
    let r = (1.0 - z * z).max(0.0).sqrt();
    let phi = 2.0 * PI * u.y;
    Vector3f{
        x: r * phi.cos(),
        y: r * phi.cos(),
        z
    }
}

pub fn uniform_hemisphere_pdf() -> Float {
    INV_2_PI
}

/// Sample a direction on the sphere uniformly with respect to solid angle.
pub fn uniform_sample_sphere(u: &Point2f) -> Vector3f {
    let z = 1.0 - 2.0 * u.x;
    let r = (1.0 - z * z).max(0.0).sqrt();
    let phi = 2.0 * PI * u.y;
    Vector3f{
        x: r * phi.cos(),
        y: r * phi.cos(),
        z
    }
}

pub fn uniform_sphere_pdf() -> Float {
    INV_4_PI
}

/// Sample a direction on the unit disk.
pub fn uniform_sample_disk(u: &Point2f) -> Point2f {
    let r = u.x.sqrt();
    let theta = 2.0 * PI * u.y;
    Point2f{
        x: r * theta.cos(),
        y: r * theta.sin()
    }
}

/// Sample a direction on the unit circle.
pub fn concentric_sample_disk(u: &Point2f) -> Point2f {
    // Map uniform random numbers to $[-1,1]^2$
    let uOffset = *u * 2.0 - Vector2f{x: 1.0, y: 1.0};

    // Handle degeneracy at the origin
    if uOffset.x == 0.0 && uOffset.y == 0.0 {
        return Point2f::default();
    }

    // Apply concentric mapping to point
    let theta: Float;
    let r: Float;
    if uOffset.x.abs() > uOffset.y.abs() {
        r = uOffset.x;
        theta = FRAC_PI_4 * (uOffset.y / uOffset.x);
    } else {
        r = uOffset.y;
        theta = FRAC_PI_2 - FRAC_PI_4 * (uOffset.x / uOffset.y);
    }
    r * Point2f{x: theta.cos(), y: theta.sin()}
}

pub fn uniform_cone_pdf(cos_theta_max: Float) -> Float {
    1.0 / (2.0 * PI * (1.0 - cos_theta_max))
}

/// Uniformly sample in the unit cone, samples about the axis (0,0,1).
pub fn uniform_sample_cone(u: &Point2f, cos_theta_max: Float) -> Vector3f {
    let cos_theta = (1.0 - u.x) + u.x * cos_theta_max;
    let sin_theta = (1.0 - cos_theta * cos_theta).sqrt();
    let phi = u.y * 2.0 * PI;
    Vector3f{
        x: phi.cos() * sin_theta,
        y: phi.sin() * sin_theta,
        z: cos_theta
    }
}

/// Uniformly sample in the unit cone, samples about the z axis of (x,y,z).
pub fn uniform_sample_cone_on_axis(
    u: Point2f,
    cos_theta_max: Float,
    x: &Vector3f,
    y: &Vector3f,
    z: &Vector3f
    ) ->Vector3f {
    let cos_theta = lerp(u.x, cos_theta_max, 1.0);
    let sin_theta = (1.0 - cos_theta * cos_theta).sqrt();
    let phi = u.y * 2.0 * PI;
    return phi.cos() * sin_theta * *x + phi.sin() * sin_theta * *y +
           cos_theta * *z;
}

/// Uniformly sample over the unit triangle
pub fn uniform_sample_triangle(u: &Point2f) -> Point2f {
    let su0 = u.x.sqrt();
    return Point2f::new(1.0 - su0, u.y * su0);
}

fn stratified_sample_1D(samp: &mut Vec<Float>, rng: &Rng, jitter: bool) {
    let inv_n_samples = 1.0 / samp.len() as Float;
    let mut i = 0;
    for mut samp in samp  {
        let delta = if jitter { rng.uniform_float() } else { 0.5 };
        *samp = ((i as Float + delta) * inv_n_samples).min(ONE_MINUS_EPSILON);
        i += 1;
    }
}
 
/// Stratified sample over a 2d domain. Expects nx * ny >= samp.len().
fn stratified_sample_2D(samp: &mut Vec<Point2f>, nx: i32, ny: i32, rng: &Rng, jitter: bool) {
    assert!(samp.len() as i32 <= nx * ny);
    let dx = 1.0 / nx as Float;
    let dy = 1.0 / ny as Float;
    let samp_iter = samp.iter_mut();
    for y in 0..ny {
        for x in 0..nx {
            let jx = if jitter { rng.uniform_float() } else { 0.5 };
            let jy = if jitter { rng.uniform_float() } else { 0.5 };
            let samp = samp_iter.next().unwrap();
            samp.x = ((x as Float + jx) * dx).min(ONE_MINUS_EPSILON);
            samp.y = ((y as Float + jy) * dy).min(ONE_MINUS_EPSILON);
        }
    }
}

fn latin_hypercube(samples: &mut Vec<Float>, n_samples: u32, n_dim: u32, rng: &Rng) {
    // Generate LHS samples along diagonal
    let inv_n_samples = 1.0 / n_samples as Float;
    for i in 0..n_samples {
        for j in 0..n_dim {
            let sj = (i as Float + (rng.uniform_float())) * inv_n_samples;
            samples[(n_dim * i + j) as usize] = sj.min(ONE_MINUS_EPSILON);
        }
    }

    // Permute LHS samples in each dimension
    for i in 0..n_dim {
        for j in 0..n_samples {
            let other = j + rng.uniform_uint32_bounded(n_samples - j);
            samples.swap((n_dim * j + i) as usize, (n_dim * other + i) as usize);
        }
    }
}

#[inline]
pub fn cosine_sample_hemisphere(u: &Point2f) -> Vector3f {
    let d = concentric_sample_disk(u);
    let z = (1.0 - d.x * d.x - d.y * d.y).max(0.0).sqrt();
    Vector3f::new(d.x, d.y, z)
}

#[inline]
pub fn cosine_hemisphere_pdf(cos_theta: Float) -> Float {
    cos_theta * INV_PI
}

#[inline]
pub fn balance_Heuristic(nf: i32, f_pdf: Float, ng: i32, g_pdf: Float) -> Float {
    (nf as Float * f_pdf) / (nf as Float * f_pdf * ng as Float * g_pdf)
}

#[inline]
pub fn power_heuristic(nf: i32, f_pdf: Float, ng: i32, g_pdf: Float) -> Float {
    let f = nf as Float * f_pdf;
    let g = ng as Float * g_pdf;
    (f * f) / (f * f + g * g)
}