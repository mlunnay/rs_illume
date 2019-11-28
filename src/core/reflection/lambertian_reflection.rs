use super::{BxDF, BxDFType};
use crate::core::pbrt::{Spectrum, consts::INV_PI};
use crate::core::geometry::{Vector3f, Point2f};
use std::fmt;

#[derive(Copy, Clone)]
pub struct LambertianReflection {
    r: Spectrum
}

impl LambertianReflection {
    pub fn new(r: Spectrum) -> LambertianReflection {
        LambertianReflection { r }
    }
}

impl BxDF for LambertianReflection {
    fn get_type(&self) -> u8 {
        BxDFType::BSDF_REFLECTION | BxDFType::BSDF_DIFFUSE
    }

    fn f(
        &self,
        wo: &Vector3f,
        wi: &Vector3f
    ) -> Spectrum {
        self.r * INV_PI
    }

    fn rho(
        &self,
        samples1: &[Point2f],
        samples2: &[Point2f]
    ) -> Spectrum {
        self.r
    }

    fn rho_vec(
        &self,
        wo: &Vector3f,
        samples: &[Point2f]
    ) -> Spectrum {
        self.r
    }
}

impl fmt::Display for LambertianReflection {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "[ LambertianReflection r: {} ]", self.r)
    }
}