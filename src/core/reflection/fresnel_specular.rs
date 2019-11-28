use super::{BxDF, BxDFType, cos_theta, abs_cos_theta, fr_dielectric};
use crate::core::pbrt::{Float, Spectrum};
use crate::core::geometry::{Vector3f, Point2f, Normal3f};
use crate::core::material::TransportMode;
use crate::core::reflection::refract;
use std::fmt;

#[derive(Copy, Clone)]
pub struct FresnelSpecular {
    r: Spectrum,
    t: Spectrum,
    eta_a: Float,
    eta_b: Float,
    mode: TransportMode
}

impl FresnelSpecular {
    pub fn new(
        r: Spectrum,
        t: Spectrum,
        eta_a: Float,
        eta_b: Float,
        mode: TransportMode
    ) -> FresnelSpecular {
        FresnelSpecular { r, t, eta_a, eta_b, mode }
    }
}

impl BxDF for FresnelSpecular {
    fn get_type(&self) -> u8 {
        BxDFType::BSDF_REFLECTION | BxDFType::BSDF_TRANSMISSION | BxDFType::BSDF_SPECULAR
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
        let f = fr_dielectric(cos_theta(wo), self.eta_a, self.eta_b);
        if sample[0] < f {
            // Compute specular reflection for _FresnelSpecular_

            // Compute perfect specular reflection direction
            *wi = Vector3f::new(-wo.x, -wo.y, wo.z);
            *sampled_type = BxDFType::BSDF_SPECULAR | BxDFType::BSDF_REFLECTION;
            *pdf = f;
            return f * self.r / abs_cos_theta(wi);
        } else {
            // Compute specular transmission for _FresnelSpecular_

            // Figure out which $\eta$ is incident and which is transmitted
            let entering = cos_theta(wo) > 0.0;
            let eta_i = if entering { self.eta_a } else { self.eta_b };
            let eta_t = if entering { self.eta_b } else { self.eta_a };

            // Compute ray direction for specular transmission
            if refract(wo, &Normal3f::new(0.0, 0.0, 1.0).face_forward(&Normal3f::from(*wo)), eta_i / eta_t, wi) {
                return Spectrum::new(0.0);
            }
            let ft = self.t * (1.0 - f);

            // Account for non-symmetry with transmission to different medium
            if self.mode == TransportMode::Radiance {
                ft *= (eta_i * eta_i) / (eta_t * eta_t);
            }
            *sampled_type = BxDFType::BSDF_SPECULAR | BxDFType::BSDF_TRANSMISSION;
            *pdf = 1.0 - f;
            ft / abs_cos_theta(wi)
        }
    }

    fn pdf(&self, wo: &Vector3f, wi: &Vector3f) -> Float {
        0.0
    }
}

impl fmt::Display for FresnelSpecular {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "[ FresnelSpecular r: {} t: {} eta_a: {} eta_b: {} mode: {} ]",
            self.r, self.t, self.eta_a, self.eta_b,
            if self.mode == TransportMode::Radiance { "RADIANCE" } else { "IMPORTANCE" })
    }
}