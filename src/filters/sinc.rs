use crate::core::filter::Filter;
use crate::core::pbrt::{Float, consts::PI};
use crate::core::geometry::{Vector2f, Point2f};

/// This implements a [Filter] based on the sinc function.
pub struct LanczosSincFilter {
    // Filter common 
    pub radius: Vector2f,
    pub inv_radius: Vector2f,

    tau: Float
}

impl LanczosSincFilter {
    pub fn new(radius: Vector2f, tau: Float) -> LanczosSincFilter {
        LanczosSincFilter {
            radius,
            inv_radius: Vector2f { x: 1.0 / radius.x, y: 1.0 / radius.y },
            tau
        }
    }

    pub fn sinc(&self, mut x: Float) -> Float {
        x = x.abs();
        if x < 1e-5 {
            1.0
        } else {
            (PI * x).sin() / (PI * x)
        }
    }

    pub fn windowed_sinc(&self, mut x: Float, radius: Float) -> Float {
        x = x.abs();
        if x > radius {
            return 0.0;
        }
        let lanczos = self.sinc(x / self.tau);
        self.sinc(x) * lanczos
    }
}

impl Filter for LanczosSincFilter {
    fn get_radius(&self) -> Vector2f {
        self.radius
    }

    fn get_inv_radius(&self) -> Vector2f {
        self.inv_radius
    }

    fn evaluate(&self, p: &Point2f) -> Float {
        self.windowed_sinc(p.x, self.radius.x) * self.windowed_sinc(p.y, self.radius.y)
    }
}