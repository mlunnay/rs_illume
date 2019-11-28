use super::pbrt::Float;
use super::geometry::{Vector3f, Point2f};
use super::reflection::abs_cos_theta;
use std::fmt;

pub mod beckmann_distribution;
pub use beckmann_distribution::*;
pub mod trowbridge_reitz_distribution;
pub use trowbridge_reitz_distribution::*;
pub mod disney_distribution;
pub use disney_distribution::*;

pub trait MicrofacetDistribution: fmt::Display {
    fn get_sample_visible_area(&self) -> bool;

    fn d(&self, wh: &Vector3f) -> Float;

    fn lambda(&self, w: &Vector3f) -> Float;

    fn sample_wh(&self, wo: &Vector3f, u: &Point2f) -> Vector3f;

    fn g1(&self, w: &Vector3f) -> Float {
        1.0 / (1.0 + self.lambda(w))
    }

    fn g(&self, wo: &Vector3f, wi: &Vector3f) -> Float {
        1.0 / (1.0 + self.lambda(wo) + self.lambda(wi))
    }

    fn pdf(&self, wo: &Vector3f, wh: &Vector3f) -> Float {
        if self.get_sample_visible_area() {
            self.d(wh) * self.g1(wo) * wo.dot(wo).abs() / abs_cos_theta(wo)
        } else {
            self.d(wh) * abs_cos_theta(wh)
        }
    }
}