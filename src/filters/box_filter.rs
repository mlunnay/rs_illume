use crate::core::filter::Filter;
use crate::core::pbrt::Float;
use crate::core::geometry::{Vector2f, Point2f};

/// This [Filter](crate::core::filter::Filter) equally weights all samples within a square region of the image.
pub struct BoxFilter {
    // Filter common 
    pub radius: Vector2f,
    pub inv_radius: Vector2f
}

impl BoxFilter {
    pub fn new(radius: Vector2f) -> BoxFilter {
        BoxFilter {
            radius,
            inv_radius: Vector2f { x: 1.0 / radius.x, y: 1.0 / radius.y }
        }
    }
}

impl Filter for BoxFilter {
    fn get_radius(&self) -> Vector2f {
        self.radius
    }

    fn get_inv_radius(&self) -> Vector2f {
        self.inv_radius
    }

    fn evaluate(&self, p: &Point2f) -> Float {
        1.0
    }
}