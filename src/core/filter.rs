use super::pbrt::Float;
use super::geometry::{Vector2f, Point2f};

pub trait Filter {
    fn get_radius(&self) -> Vector2f;
    fn get_inv_radius(&self) -> Vector2f;

    fn evaluate(p: &Point2f) -> Float;
}