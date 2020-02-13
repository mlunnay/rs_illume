use super::pbrt::Float;
use super::geometry::{Vector2f, Point2f};

/// An interface for the filtering used in image reconstruction. 
pub trait Filter {
    fn get_radius(&self) -> Vector2f;
    fn get_inv_radius(&self) -> Vector2f;

    fn evaluate(&self, p: &Point2f) -> Float;
}