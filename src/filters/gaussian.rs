use crate::core::filter::Filter;
use crate::core::pbrt::Float;
use crate::core::geometry::{Vector2f, Point2f};

/// This [Filter](crate::core::filter::Filter) applies a Gaussian bump that is centered at the pixel and radially symmetric around it.
pub struct GaussianFilter {
    // Filter common 
    pub radius: Vector2f,
    pub inv_radius: Vector2f,

    alpha: Float,
    exp_x: Float,
    exp_y: Float,
}

impl GaussianFilter {
    pub fn new(radius: Vector2f, alpha: Float) -> GaussianFilter {
        GaussianFilter {
            radius: radius,
            inv_radius: Vector2f { x: 1.0 / radius.x, y: 1.0 / radius.y },
            alpha: alpha,
            exp_x: (-alpha * radius.x * radius.x).exp(),
            exp_y: (-alpha * radius.y * radius.y).exp()
        }
    }

    fn gaussian(&self, d: Float, expv: Float) -> Float {
        ((-self.alpha * d * d).exp() - expv).max(0.0)
    }
}

impl Filter for GaussianFilter {
    fn get_radius(&self) -> Vector2f {
        self.radius
    }

    fn get_inv_radius(&self) -> Vector2f {
        self.inv_radius
    }

    fn evaluate(&self, p: &Point2f) -> Float {
        self.gaussian(p.x, self.exp_x) * self.gaussian(p.y, self.exp_y)
    }
}