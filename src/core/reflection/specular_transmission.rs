use super::{BxDF, Fresnel, FresnelDielectric, BxDFType, cos_theta, abs_cos_theta};
use crate::core::pbrt::{Float, Spectrum};
use crate::core::geometry::{Vector3f, Point2f, Normal3f};
use crate::core::material::TransportMode;
use crate::core::reflection::refract;
use std::fmt;

#[derive(Copy, Clone)]
pub struct SpecularTransmission {
    t: Spectrum,
    eta_a: Float,
    eta_b: Float,
    fresnel: FresnelDielectric,
    mode: TransportMode
}

impl SpecularTransmission {
    pub fn new (t: Spectrum, eta_a: Float, eta_b: Float, mode: TransportMode) -> Self {
        SpecularTransmission {
            t,
            eta_a,
            eta_b,
            fresnel: FresnelDielectric::new(eta_a, eta_b),
            mode
        }
    }
}

impl BxDF for SpecularTransmission {
    fn get_type(&self) -> u8 {
        BxDFType::BSDF_TRANSMISSION | BxDFType::BSDF_SPECULAR
    }

    fn f(
        &self,
        wo: &Vector3f,
        wi: &Vector3f
    ) -> Spectrum {
        Spectrum::new(0.0)
    }

    fn sample_f(
        &self,
        wo: &Vector3f,
        wi: &mut Vector3f,
        sample: &Point2f,
        pdf: &mut Float,
        sampled_type: &mut u8
    ) -> Spectrum {
        // Figure out which $\eta$ is incident and which is transmitted
        let entering = cos_theta(wo) > 0.0;
        let eta_i = if entering { self.eta_a } else { self.eta_b };
        let eta_t = if entering { self.eta_b } else { self.eta_a };

        // Compute ray direction for specular transmission
        if refract(wo, &Normal3f::new(0.0, 0.0, 1.0).face_forward(&Normal3f::from(*wo)), eta_i / eta_t, wi) {
            return Spectrum::new(0.0);
        }
        *pdf = 1.0;
        let mut ft = self.t * (Spectrum::new(1.0) - self.fresnel.evaluate(cos_theta(wi)));
        // Account for non-symmetry with transmission to different medium
        if self.mode == TransportMode::Radiance {
            ft *= (eta_i * eta_i) / (eta_t * eta_t);
        }
        ft / abs_cos_theta(wi)
    }

    fn pdf(&self, wo: &Vector3f, wi: &Vector3f) -> Float {
        0.0
    }
}

impl fmt::Display for SpecularTransmission {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "[ SpecularTransmission t: {} eta_a: {} eta_b: {} fresnel: {} mode: {} ]",
            self.t, self.eta_a, self.eta_b, self.fresnel,
            if self.mode == TransportMode::Radiance { "RADIANCE" } else { "IMPORTANCE" })
    }
}