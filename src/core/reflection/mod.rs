use super::pbrt::{Float, Spectrum, lerp, consts::{PI, INV_PI}};
use super::geometry::{Vector3f, Normal3f, Point2f, dot_vec_normal};
use super::interaction::SurfaceInteraction;
use super::profiler::Profiler;
use super::rng::ONE_MINUS_EPSILON;
use super::sampling::{uniform_sample_hemisphere, uniform_hemisphere_pdf, cosine_sample_hemisphere};
use std::fmt;
use num::clamp;

pub mod scaled_bxdf;
pub use scaled_bxdf::*;
pub mod fresnel;
pub use fresnel::*;
pub mod specular_reflection;
pub use specular_reflection::*;
pub mod specular_transmission;
pub use specular_transmission::*;
pub mod fresnel_specular;
pub use fresnel_specular::*;
pub mod lambertian_reflection;
pub use lambertian_reflection::*;
pub mod lambertian_transmission;
pub use lambertian_transmission::*;
pub mod oren_nayar;
pub use oren_nayar::*;
pub mod microfacet_reflection;
pub use microfacet_reflection::*;
pub mod microfacet_transmission;
pub use microfacet_transmission::*;
pub mod fresnel_blend;
pub use fresnel_blend::*;
pub mod fourier_bsdf;
pub use fourier_bsdf::*;

pub struct BxDFType {}

impl BxDFType {
    pub const BSDF_REFLECTION: u8 = 1 << 0;
    pub const BSDF_TRANSMISSION: u8 = 1 << 1;
    pub const BSDF_DIFFUSE: u8 = 1 << 2;
    pub const BSDF_GLOSSY: u8 = 1 << 3;
    pub const BSDF_SPECULAR: u8 = 1 << 4;
    pub const BSDF_ALL: u8 = BxDFType::BSDF_DIFFUSE as u8 | BxDFType::BSDF_GLOSSY as u8 |
            BxDFType::BSDF_SPECULAR as u8 | BxDFType::BSDF_REFLECTION as u8 |
            BxDFType::BSDF_TRANSMISSION as u8;
}

const MAX_BX_DFS: usize = 8;

pub struct BSDF {
    pub eta: Float,
    ns: Normal3f,
    ng: Normal3f,
    ss: Vector3f,
    ts: Vector3f,
    n_bxdfs: u8,
    bxdfs: [Box<dyn BxDF>; MAX_BX_DFS]
}

impl BSDF {
    pub fn new(si: &SurfaceInteraction, eta: Float) -> BSDF {
        BSDF {
            eta,
            ns: si.shading.n,
            ng: si.n,
            ss: si.shading.dpdu.normalize(),
            ts: si.shading.dpdu.normalize().cross(&Vector3f::from(si.shading.n)),
            n_bxdfs: 0,
            bxdfs: [
                Box::new(DefaultBxDF{}),
                Box::new(DefaultBxDF{}),
                Box::new(DefaultBxDF{}),
                Box::new(DefaultBxDF{}),
                Box::new(DefaultBxDF{}),
                Box::new(DefaultBxDF{}),
                Box::new(DefaultBxDF{}),
                Box::new(DefaultBxDF{})
            ]
        }
    }

    pub fn add(&self, b: Box<dyn BxDF>) {
        assert!((self.n_bxdfs as usize) < MAX_BX_DFS);
        self.bxdfs[self.n_bxdfs as usize] = b;
        self.n_bxdfs += 1;
    }

    #[inline]
    pub fn num_components(&self, flags: u8) -> usize {
        let mut num = 0_usize;
        for i in 0..self.n_bxdfs as usize {
            if self.bxdfs[i].matches_flags(flags) {
                num += 1;
            }
        }
        num
    }

    pub fn world_to_local(&self, v: &Vector3f) -> Vector3f {
        Vector3f::new(v.dot(&self.ss), v.dot(&self.ts), dot_vec_normal(v, &self.ns))
    }

    pub fn local_to_world(&self, v: &Vector3f) -> Vector3f {
        Vector3f::new(self.ss.x * v.x + self.ts.x * v.y + self.ns.x * v.z,
            self.ss.y * v.x + self.ts.y * v.y + self.ns.y * v.z,
            self.ss.z * v.x + self.ts.z * v.y + self.ns.z * v.z)
    }

    pub fn f(
        &self,
        wo_w: &Vector3f,
        wi_w: &Vector3f,
        flags: u8
    ) -> Spectrum {
        let _profile = Profiler::instance().profile("BSDF Evaluation");
        let wi = self.world_to_local(&wi_w);
        let wo = self.world_to_local(&wo_w);
        if wo.z == 0.0 {
            return Spectrum::new(0.0);
        }
        let reflect = dot_vec_normal(wi_w, &self.ng) * dot_vec_normal(wo_w, &self.ng) > 0.0;
        let mut f = Spectrum::new(0.0);
        for i in 0..self.n_bxdfs as usize {
            if self.bxdfs[i].matches_flags(flags) &&
                ((reflect && self.bxdfs[i].get_type() & BxDFType::BSDF_REFLECTION != 0) ||
                (!reflect && self.bxdfs[i].get_type() & BxDFType::BSDF_TRANSMISSION != 0)) {
                f += self.bxdfs[i].f(&wo, &wi);
            }
        }
        f
    }

    pub fn rho(
        &self,
        samples1: &[Point2f],
        samples2: &[Point2f],
        flags: u8
    ) -> Spectrum {
        let ret = Spectrum::new(0.0);
        for i in 0..self.n_bxdfs as usize {
            if self.bxdfs[i].matches_flags(flags) {
                ret += self.bxdfs[i].rho(samples1, samples2);
            }
        }
        ret
    }

    pub fn rho_vec(
        &self,
        wo: &Vector3f,
        samples: &[Point2f],
        flags: u8
    ) -> Spectrum {
        let wo = self.world_to_local(wo);
        let ret = Spectrum::new(0.0);
        for i in 0..self.n_bxdfs as usize {
            if self.bxdfs[i].matches_flags(flags) {
                ret += self.bxdfs[i].rho_vec(&wo, samples);
            }
        }
        ret
    }

    pub fn sample_f(
        &self,
        wo: &Vector3f,
        wi: &mut Vector3f,
        u: &Point2f,
        pdf: &mut Float,
        bxdf_type: u8,
        sampled_type: &mut u8
    ) -> Spectrum {
        let _profile = Profiler::instance().profile("BSDF Sampling");
        // Choose which _BxDF_ to sample
        let matching_comps = self.num_components(bxdf_type);
        if matching_comps == 0 {
            *pdf = 0.0;
            *sampled_type = 0;
            return Spectrum::new(0.0);
        }
        let comp = ((u[0] * matching_comps as Float).floor() as usize).min(matching_comps - 1);

        // Get _BxDF_ pointer for chosen component
        let bxdf: Option<&Box<dyn BxDF>> = None;
        let mut count = comp;
        let mut bxdf_index = 0;
        for i in 0..self.n_bxdfs as usize {
            if self.bxdfs[i].matches_flags(bxdf_type) {
                count -= 1;
                bxdf = Some(&self.bxdfs[i]);
                bxdf_index = i;
                break;
            }
        }
        let bxdf = if let Some(b) = bxdf { b } else { panic!("BxDF not found.") };
        vlog!(2, "BSDF::Sample_f chose comp = {} / matching = {}, bxdf: {}", comp, matching_comps, bxdf.to_string());

        // Remap _BxDF_ sample _u_ to $[0,1)^2$
        let u_remapped = Point2f::new((u[0] * (matching_comps - comp) as Float).min(ONE_MINUS_EPSILON), u[1]);

        // Sample chosen _BxDF_
        let mut wi_local = Vector3f::default();
        let wo_local = self.world_to_local(&wo);
        if wo_local.z == 0.0 {
            return Spectrum::new(0.0);
        }
        *pdf = 0.0;
        *sampled_type = bxdf.get_type();
        let mut f = bxdf.sample_f(&wo, &mut wi, &u_remapped, pdf, sampled_type);
        vlog!(2, "For wo = {:?}, sampled f = {}, pdf = {}, ratio = {}, wi = {:?}",
            wo_local, f, *pdf, if *pdf > 0.0 { f / *pdf } else { Spectrum::new(0.0) }, wi_local);
        if *pdf == 0.0 {
            *sampled_type = 0;
            return Spectrum::new(0.0);
        }
        *wi = self.local_to_world(&wi_local);

        // Compute overall PDF with all matching _BxDF_s
        if bxdf.get_type() & BxDFType::BSDF_SPECULAR == 0 && matching_comps > 1 {
            for i in 0..self.n_bxdfs as usize {
                if i != bxdf_index && self.bxdfs[i].matches_flags(bxdf_type) {
                    *pdf += self.bxdfs[i].pdf(&wo_local, &wi_local);
                }
            }
        }
        if matching_comps > 1 {
            *pdf /= matching_comps as Float;
        }

        // Compute value of BSDF for sampled direction
        if bxdf.get_type() & BxDFType::BSDF_SPECULAR == 0 {
            let reflect = dot_vec_normal(wi, &self.ng) * dot_vec_normal(wo, &self.ng) > 0.0;
            f = Spectrum::new(0.0);
            for i in 0..self.n_bxdfs as usize {
                if self.bxdfs[i].matches_flags(bxdf_type) &&
                    ((reflect && self.bxdfs[i].get_type() & BxDFType::BSDF_REFLECTION != 0) ||
                    (!reflect && self.bxdfs[i].get_type() & BxDFType::BSDF_TRANSMISSION != 0)) {
                    f += self.bxdfs[i].f(&wo_local, &wi_local);
                }
            }
        }
        vlog!(2, "Overall f = {}, pdf = {}, ratio = {}", f, *pdf, if *pdf > 0.0 { f / *pdf } else { Spectrum::new(0.0) });
        f
    }

    pub fn pdf(
        &self,
        wo: &Vector3f,
        wi: &Vector3f,
        flags: u8
    ) -> Float {
        let _profile = Profiler::instance().profile("BSDF::pdf");
        if self.n_bxdfs == 0 {
            return 0.0;
        }
        let wo_local = self.world_to_local(wo);
        let wi_local = self.world_to_local(wi);
        if wo_local.z == 0.0 {
            return 0.0;
        }
        let mut pdf: Float = 0.0;
        let mut mathing_comps: u8 = 0;
        for i in 0..self.n_bxdfs as usize {
            if self.bxdfs[i].matches_flags(flags) {
                mathing_comps += 1;
                pdf += self.bxdfs[i].pdf(&wo_local, &wi_local);
            }
        }
        if mathing_comps > 0 { 
            pdf / mathing_comps as Float
        } else { 
            0.0
        }
    }
}

impl fmt::Display for BSDF {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "[ BSDF eta: {} nBXDFs: {}", self.eta, self.n_bxdfs)?;
        for i in 0..self.n_bxdfs as usize {
            write!(f, "\n  bxdfs[{}]: {}", i, self.bxdfs[i])?;
        }
        write!(f, " ]")
    }
}

pub trait BxDF: fmt::Display {
    fn get_type(&self) -> u8;

    fn matches_flags(&self, t: u8) -> bool {
        self.get_type() & t == self.get_type()
    }

    fn f(
        &self,
        wo: &Vector3f,
        wi: &Vector3f
    ) -> Spectrum;

    fn sample_f(
        &self,
        wo: &Vector3f,
        wi: &mut Vector3f,
        sample: &Point2f,
        pdf: &mut Float,
        sampled_type: &mut u8
    ) -> Spectrum {
        // Cosine-sample the hemisphere, flipping the direction if necessary
        *wi = cosine_sample_hemisphere(sample);
        if wo.z < 0.0 {
            wi.z *= -1.0;
        }
        *pdf = self.pdf(wo, wi);
        self.f(wo, wi)
    }

    fn rho_vec(
        &self,
        wo: &Vector3f,
        samples: &[Point2f]
    ) -> Spectrum {
        let mut r = Spectrum::new(0.0);
        for sample in samples {
            // Estimate one term of $\rho_\roman{hd}$
            let mut wi = Vector3f::default();
            let mut pdf: Float = 0.0;
            let mut st: u8 = 0;
            let f = self.sample_f(wo, &mut wi, sample, &mut pdf, &mut st);
            if pdf > 0.0 {
                r += f * abs_cos_theta(&wi) / pdf;
            }
        }
        r / samples.len() as Float
    }

    fn rho(
        &self,
        samples1: &[Point2f],
        samples2: &[Point2f]
    ) -> Spectrum {
        let mut r = Spectrum::new(0.0);
        for i in 0..samples1.len() {
            // Estimate one term of $\rho_\roman{hd}$
            let mut wi = Vector3f::default();
            let wo = uniform_sample_hemisphere(&samples1[i]);
            let pdfo = uniform_hemisphere_pdf();
            let mut pdfi: Float = 0.0;
            let mut st: u8 = 0;
            let f = self.sample_f(&wo, &mut wi, &samples2[i], &mut pdfi, &mut st);
            if pdfi > 0.0 {
                r += f * abs_cos_theta(&wi) * abs_cos_theta(&wo) / (pdfo * pdfi);
            }
        }
        r / (PI * samples1.len() as Float)
    }

    fn pdf(&self, wo: &Vector3f, wi: &Vector3f) -> Float {
        if same_hemisphere(wo, wi) { abs_cos_theta(wi) * INV_PI } else { 0.0 }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct DefaultBxDF {}

impl BxDF for DefaultBxDF {
    fn get_type(&self) -> u8 {
        panic!("DefaultBxdf called should be an implementation of BxDF.");
    }

    fn matches_flags(&self, t: u8) -> bool {
        panic!("DefaultBxdf called should be an implementation of BxDF.");
    }

    fn f(
        &self,
        wo: &Vector3f,
        wi: Vector3f
    ) -> Spectrum {
        panic!("DefaultBxdf called should be an implementation of BxDF.");
    }

    fn sample_f(
        &self,
        wo: &Vector3f,
        wi: &mut Vector3f,
        sample: &Point2f,
        pdf: &mut Float,
        sampled_type: &mut u8
    ) -> Spectrum {
        panic!("DefaultBxdf called should be an implementation of BxDF.");
    }

    fn rho_vec(
        &self,
        wo: &Vector3f,
        samples: &[Point2f]
    ) -> Spectrum {
        panic!("DefaultBxdf called should be an implementation of BxDF.");
    }

    fn rho(
        &self,
        samples1: &[Point2f],
        samples2: &[Point2f]
    ) -> Spectrum {
        panic!("DefaultBxdf called should be an implementation of BxDF.");
    }

    fn pdf(&self, wo: &Vector3f, wi: &Vector3f) -> Float {
        panic!("DefaultBxdf called should be an implementation of BxDF.");
    }
}

impl fmt::Display for DefaultBxDF {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "DefaultBxDF{{}}")
    }
}

pub fn fr_dielectric(cos_theta_i: Float, eta_i: Float, eta_t: Float) -> Float {
    let mut cos_theta_i = clamp(cos_theta_i, -1.0, 1.0);
    // Potentially swap indices of refraction
    let mut eta_i = eta_i;
    let mut eta_t = eta_t;
    let entering = cos_theta_i > 0.0;
    if !entering {
        std::mem::swap(&mut eta_i, &mut eta_t);
        cos_theta_i = cos_theta_i.abs();
    }

    // Compute _cosThetaT_ using Snell's law
    let sin_theta_i = (1.0 - cos_theta_i * cos_theta_i).max(0.0).sqrt();
    let sin_theta_t = eta_i / eta_t * sin_theta_i;

    // Handle total internal reflection
    if sin_theta_t >= 1.0 {
        return 1.0;
    }
    let cos_theta_t = (1.0 - sin_theta_t * sin_theta_t).max(0.0).sqrt();
    let rparl = ((eta_t * cos_theta_i) - (eta_i * cos_theta_t)) /
        ((eta_t * cos_theta_i) + (eta_i * cos_theta_t));
    let rperp = ((eta_i * cos_theta_i) - (eta_t * cos_theta_t)) /
        ((eta_i * cos_theta_i) + (eta_t * cos_theta_t));
    (rparl * rparl + rperp * rperp) / 2.0
}

// https://seblagarde.wordpress.com/2013/04/29/memo-on-fresnel-equations/
pub fn fr_conductor(
    mut cos_theta_i: Float,
    etai: &Spectrum,
    etat: &Spectrum,
    k: &Spectrum
) -> Spectrum {
    cos_theta_i = clamp(cos_theta_i, -1.0, 1.0);
    let eta = *etat / *etai;
    let etak = *k / *etai;

    let cos_theta_i2 = cos_theta_i * cos_theta_i;
    let sin_theta_i2 = 1.0 - cos_theta_i2;
    let eta2 = eta * eta;
    let etak2 = etak * etak;

    let t0 = eta2 - etak2 - Spectrum::new(sin_theta_i2);
    let asplusb2 = (t0 * t0 + 4.0 * eta2 * etak2).sqrt();
    let t1 = asplusb2 + Spectrum::new(cos_theta_i2);
    let a = (0.5 * (asplusb2 + t0)).sqrt();
    let t2 = 2.0 * cos_theta_i * a;
    let rs = (t1 - t2) / (t1 + t2);

    let t3 = cos_theta_i2 * asplusb2 + Spectrum::new(sin_theta_i2 * sin_theta_i2);
    let t4 = t2 * sin_theta_i2;
    let rp = rs * (t2 - t4) / (t3 + t4);

    0.5 * (rp + rs)
}

// BSDF Inline Functions
#[inline]
pub fn cos_theta(w: &Vector3f) -> Float {
    w.z
}

#[inline]
pub fn cos2_theta(w: &Vector3f) -> Float {
    w.z * w.z
}

#[inline]
pub fn abs_cos_theta(w: &Vector3f) -> Float {
    w.z.abs()
}

#[inline]
pub fn sin2_theta(w: &Vector3f) -> Float {
    (1.0 - cos2_theta(w)).max(0.0)
}

#[inline]
pub fn sin_theta(w: &Vector3f) -> Float {
    sin2_theta(w).sqrt()
}

#[inline]
pub fn tan_theta(w: &Vector3f) -> Float {
    sin_theta(w) / cos_theta(w)
}

#[inline]
pub fn tan2_theta(w: &Vector3f) -> Float {
    sin2_theta(w) / cos2_theta(w)
}

#[inline]
pub fn cos_phi(w: &Vector3f) -> Float {
    let sin_theta = sin_theta(w);
    if sin_theta == 0.0 { 1.0 } else { num::clamp(w.x / sin_theta, -1.0, 1.0) }
}

#[inline]
pub fn sin_phi(w: &Vector3f) -> Float {
    let sin_theta = sin_theta(w);
    if sin_theta == 0.0 { 1.0 } else { num::clamp(w.y / sin_theta, -1.0, 1.0) }
}

#[inline]
pub fn cos2_phi(w: &Vector3f) -> Float {
    cos_phi(w) * cos_phi(w)
}

#[inline]
pub fn sin2_phi(w: &Vector3f) -> Float {
    sin_phi(w) * sin_phi(w)
}

#[inline]
pub fn cos_d_phi(wa: &Vector3f, wb: &Vector3f) -> Float {
    num::clamp((wa.x * wb.x + wa.y * wb.y) /
        ((wa.x * wa.x + wa.y * wa.y) / (wb.x * wb.x + wb.y * wb.y)).sqrt(), -1.0, 1.0)
}

#[inline]
pub fn reflect(wo: &Vector3f, n: &Vector3f) -> Vector3f {
    -*wo + 2.0 * wo.dot(n) * *n
}

#[inline]
pub fn refract(wi: &Vector3f, n: &Normal3f, eta: Float, wt: &mut Vector3f) -> bool {
    // Compute $\cos \theta_\roman{t}$ using Snell's law
    let cos_theta_i = dot_vec_normal(wi, n);
    let sin2_theta_i = (1.0 - cos_theta_i * cos_theta_i).max(0.0);
    let sin2_theta_t = eta * eta * sin2_theta_i;

    // Handle total internal reflection for transmission
    if sin2_theta_t >= 1.0 {
        return false;
    }
    let cos_theta_t = (1.0 - sin2_theta_t).sqrt();
    *wt = eta * -*wi + (eta * cos_theta_i - cos_theta_t) * Vector3f::from(*n);
    return true;
}

#[inline]
pub fn same_hemisphere(w: &Vector3f, wp: &Vector3f) -> bool {
    w.z * wp.z > 0.0
}

#[inline]
pub fn same_hemisphere_normal(w: &Vector3f, wp: &Normal3f) -> bool {
    w.z * wp.z > 0.0
}

// https://seblagarde.wordpress.com/2013/04/29/memo-on-fresnel-equations/
//
// The Schlick Fresnel approximation is:
//
// R = R(0) + (1 - R(0)) (1 - cos theta)^5,
//
// where R(0) is the reflectance at normal indicence.
#[inline]
pub fn schlick_weight(cos_theta: Float) -> Float {
    let m = clamp(1.0 - cos_theta, 0.0, 1.0);
    (m * m) * (m * m) * m
}

#[inline]
pub fn fr_schlick(r0: Float, cos_theta: Float) -> Float {
    lerp(schlick_weight(cos_theta), r0, 1.0)
}

#[inline]
pub fn fr_schlick_spectrum(r0: &Spectrum, cos_theta: Float) -> Spectrum {
    r0.lerp(&Spectrum::new(1.0), schlick_weight(cos_theta))
}