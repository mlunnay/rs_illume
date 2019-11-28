use super::{BxDF, BxDFType, abs_cos_theta, sin_theta, sin_phi, cos_phi};
use crate::core::pbrt::{Float, Spectrum, consts::INV_PI, radians};
use crate::core::geometry::{Vector3f};
use std::fmt;

#[derive(Copy, Clone)]
pub struct OrenNayar {
    r: Spectrum,
    a: Float,
    b: Float
}

impl OrenNayar {
    pub fn new(r: Spectrum, sigma: Float) -> OrenNayar {
        let sigma = radians(sigma);
        let sigma2 = sigma * sigma;
        let a = 1.0 - (sigma2 / (2.0 * (sigma2 + 0.33)));
        let b = 0.45 * sigma2 / (sigma2 + 0.09);
        OrenNayar{ r, a, b }
    }
}

impl BxDF for OrenNayar {
    fn get_type(&self) -> u8 {
        BxDFType::BSDF_REFLECTION | BxDFType::BSDF_DIFFUSE
    }

    fn f(
        &self,
        wo: &Vector3f,
        wi: &Vector3f
    ) -> Spectrum {
        let sin_theta_i = sin_theta(wi);
        let sin_theta_o = sin_theta(wo);
        // Compute cosine term of Oren-Nayar model
        let mut max_cos: Float = 0.0;
        if sin_theta_i > 1e-4 && sin_theta_o > 1e-4 {
            let sin_phi_i = sin_phi(wi);
            let sin_phi_o = sin_phi(wo);
            let cos_phi_i = cos_phi(wi);
            let cos_phi_o = cos_phi(wo);
            let d_cos = cos_phi_i * cos_phi_o + sin_phi_i * sin_phi_o;
            max_cos = d_cos.max(0.0);
        }

        // Compute sine and tangent terms of Oren-Nayar model
        let sin_alpha: Float;
        let tan_beta: Float;
        if abs_cos_theta(wi) > abs_cos_theta(wo) {
            sin_alpha = sin_theta_o;
            tan_beta = sin_theta_i / abs_cos_theta(wi);
        } else {
            sin_alpha = sin_theta_i;
            tan_beta = sin_theta_o / abs_cos_theta(wo);
        }
        self.r * INV_PI * (self.a + self.b * max_cos * sin_alpha * tan_beta)
    }
}

impl fmt::Display for OrenNayar {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "[ OrenNayar r: {} a: {} b: {} ]", self.r, self.a, self.b)
    }
}