use super::{BxDF};
use crate::core::pbrt::{Float, Spectrum};
use crate::core::geometry::{Vector3f, Point2f};
use std::fmt;
use std::sync::Arc;

#[derive(Clone)]
pub struct ScaledBxDF {
    bxdf: Arc<dyn BxDF + Send + Sync>,
    scale: Spectrum
}

impl ScaledBxDF {
    pub fn new(bxdf: Arc<dyn BxDF + Send + Sync>, scale: Spectrum) -> Self {
        ScaledBxDF {
            bxdf,
            scale
        }
    }
}

impl BxDF for ScaledBxDF {
    fn get_type(&self) -> u8 {
        self.bxdf.get_type()
    }

    fn f(
        &self,
        wo: &Vector3f,
        wi: &Vector3f
    ) -> Spectrum {
        self.scale * self.bxdf.f(wo, wi)
    }

    fn sample_f(
        &self,
        wo: &Vector3f,
        wi: &mut Vector3f,
        sample: &Point2f,
        pdf: &mut Float,
        sampled_type: &mut u8
    ) -> Spectrum {
        self.scale * self.bxdf.sample_f(wo, wi, sample, pdf, sampled_type)
    }

    fn rho_vec(
        &self,
        wo: &Vector3f,
        samples: &[Point2f]
    ) -> Spectrum {
        self.scale * self.bxdf.rho_vec(wo, samples)
    }

    fn rho(
        &self,
        samples1: &[Point2f],
        samples2: &[Point2f]
    ) -> Spectrum {
        self.scale * self.bxdf.rho(samples1, samples2)
    }

    fn pdf(&self, wo: &Vector3f, wi: &Vector3f) -> Float {
        self.bxdf.pdf(wo, wi)
    }
}

impl fmt::Display for ScaledBxDF {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "[ ScaledBxDF bxdf: {} scale: {} ]", self.bxdf, self.scale)
    }
}