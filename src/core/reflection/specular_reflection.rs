use super::{BxDF, Fresnel, BxDFType, cos_theta, abs_cos_theta};
use crate::core::pbrt::{Float, Spectrum};
use crate::core::geometry::{Vector3f, Point2f};
use std::fmt;

#[derive(Copy, Clone)]
pub struct SpecularReflection {
    r: Spectrum,
    fresnel: Box<dyn Fresnel>
}

impl SpecularReflection {
    pub fn new(r: Spectrum, fresnel: Box<dyn Fresnel>) -> SpecularReflection {
        SpecularReflection{ r, fresnel }
    }
}

impl BxDF for SpecularReflection {
    fn get_type(&self) -> u8 {
        BxDFType::BSDF_REFLECTION | BxDFType::BSDF_SPECULAR
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
        // Compute perfect specular reflection direction
        *wi = Vector3f::new(-wo.x, -wo.y, wo.z);
        *pdf = 1.0;
        self.fresnel.evaluate(cos_theta(wi)) * self.r / abs_cos_theta(wi)
    }

    fn pdf(&self, wo: &Vector3f, wi: &Vector3f) -> Float {
        0.0
    }
}

impl fmt::Display for SpecularReflection {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "[ SpecularReflection r: {} fresnel: {} ]", self.r, self.fresnel)
    }
}