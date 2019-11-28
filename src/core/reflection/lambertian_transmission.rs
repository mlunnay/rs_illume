use super::{BxDF, BxDFType, same_hemisphere, abs_cos_theta};
use crate::core::pbrt::{Float, Spectrum, consts::INV_PI};
use crate::core::geometry::{Vector3f, Point2f};
use crate::core::sampling::{cosine_sample_hemisphere};
use std::fmt;

#[derive(Copy, Clone)]
pub struct LambertianTransmission {
    t: Spectrum
}

impl LambertianTransmission {
    pub fn new(t: Spectrum) -> LambertianTransmission {
        LambertianTransmission { t }
    }
}

impl BxDF for LambertianTransmission {
    fn get_type(&self) -> u8 {
        BxDFType::BSDF_TRANSMISSION | BxDFType::BSDF_DIFFUSE
    }

    fn f(
        &self,
        wo: &Vector3f,
        wi: &Vector3f
    ) -> Spectrum {
        self.t * INV_PI
    }

    fn rho(
        &self,
        samples1: &[Point2f],
        samples2: &[Point2f]
    ) -> Spectrum {
        self.t
    }

    fn rho_vec(
        &self,
        wo: &Vector3f,
        samples: &[Point2f]
    ) -> Spectrum {
        self.t
    }

    fn sample_f(
        &self,
        wo: &Vector3f,
        wi: &mut Vector3f,
        sample: &Point2f,
        pdf: &mut Float,
        sampled_type: &mut u8
    ) -> Spectrum {
        *wi = cosine_sample_hemisphere(sample);
        if wo.z > 0.0 {
            wi.z *= -1.0;
        }
        *pdf = self.pdf(wo, wi);
        self.f(wo, wi)
    }

    fn pdf(&self, wo: &Vector3f, wi: &Vector3f) -> Float {
        if !same_hemisphere(wo, wi) { abs_cos_theta(wi) * INV_PI } else { 0.0 }
    }
}

impl fmt::Display for LambertianTransmission {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "[ LambertianTransmission t: {} ]", self.t)
    }
}