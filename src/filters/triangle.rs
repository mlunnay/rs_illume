use crate::core::filter::Filter;
use crate::core::pbrt::Float;
use crate::core::geometry::{Vector2f, Point2f};

/// A filter where the weight falls off linearly from the filter center over the square extent of the filter.
pub struct TriangleFilter {
    // Filter common 
    pub radius: Vector2f,
    pub inv_radius: Vector2f
}

impl TriangleFilter {
    pub fn new(radius: Vector2f) -> TriangleFilter {
        TriangleFilter {
            radius,
            inv_radius: Vector2f { x: 1.0 / radius.x, y: 1.0 / radius.y }
        }
    }
}

impl Filter for TriangleFilter {
    fn get_radius(&self) -> Vector2f {
        self.radius
    }

    fn get_inv_radius(&self) -> Vector2f {
        self.inv_radius
    }

    fn evaluate(&self, p: &Point2f) -> Float {
        (self.radius.x - p.x.abs()).max(0.0) * (self.radius.y - p.y.abs()).max(0.0)
    }
}