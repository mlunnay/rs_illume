use super::{MicrofacetDistribution, TrowbridgeReitzDistribution};
use crate::core::pbrt::Float;
use crate::core::geometry::{Vector3f, Point2f};
use std::fmt;

#[derive(Debug, Default, Copy, Clone)]
pub struct DisneyMicrofacetDistribution {
    inner: TrowbridgeReitzDistribution
}

impl DisneyMicrofacetDistribution {
    pub fn new(alphax: Float, alphay: Float) -> DisneyMicrofacetDistribution {
        DisneyMicrofacetDistribution {
            inner: TrowbridgeReitzDistribution::new(alphax, alphay, true)
        }
    }
}

impl MicrofacetDistribution for DisneyMicrofacetDistribution {
    fn get_sample_visible_area(&self) -> bool {
        self.inner.get_sample_visible_area()
    }

    fn d(&self, wh: &Vector3f) -> Float {
        self.inner.d(wh)
    }

    fn lambda(&self, w: &Vector3f) -> Float {
        self.inner.lambda(w)
    }

    fn sample_wh(&self, wo: &Vector3f, u: &Point2f) -> Vector3f {
        self.inner.sample_wh(wo, u)
    }

    fn g(&self, wo: &Vector3f, wi: &Vector3f) -> Float {
        // Disney uses the separable masking-shadowing model.
        self.g1(wo) * self.g1(wi)
    }
}

impl fmt::Display for DisneyMicrofacetDistribution {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "[ DisneyMicrofacetDistribution alpha_x: {} alpha_y: {} ]", self.inner.get_alphax(), self.inner.get_alphay())
    }
}