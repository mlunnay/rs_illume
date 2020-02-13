use crate::core::filter::Filter;
use crate::core::pbrt::Float;
use crate::core::geometry::{Vector2f, Point2f};

/// A [Filter] that tends to do a good job of trading off between ringing (phantom edges next to actual edges in the image) and blurring (excessively blurred results).
pub struct MitchellFilter {
    // Filter common 
    pub radius: Vector2f,
    pub inv_radius: Vector2f,

    b: Float,
    c: Float
}

impl MitchellFilter {
    pub fn new(radius: Vector2f, b: Float, c: Float) -> MitchellFilter {
        MitchellFilter {
            radius,
            inv_radius: Vector2f { x: 1.0 / radius.x, y: 1.0 / radius.y },
            b,
            c
        }
    }

    pub fn mitchell1d(&self, mut x: Float) -> Float {
        x = (2.0 * x).abs();
        if x > 1.0 {
            ((-self.b - 6.0 * self.c) * x * x * x + (6.0 * self.b + 30.0 * self.c) * x * x +
                (-12.0 * self.b - 48.0 * self.c) * x + (8.0 * self.b + 24.0 * self.c)) *
                (1.0 / 6.0)
        } else {
            ((12.0 - 9.0 * self.b - 6.0 * self.c) * x * x * x +
                (-18.0 + 12.0 * self.b + 6.0 * self.c) * x * x + (6.0 - 2.0 * self.b)) *
                (1.0 / 6.0)
        }
    }
}

impl Filter for MitchellFilter {
    fn get_radius(&self) -> Vector2f {
        self.radius
    }

    fn get_inv_radius(&self) -> Vector2f {
        self.inv_radius
    }

    fn evaluate(&self, p: &Point2f) -> Float {
        self.mitchell1d(p.x * self.inv_radius.x) * self.mitchell1d(p.y * self.inv_radius.y)
    }
}