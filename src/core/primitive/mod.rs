use super::pbrt::Float;
use super::geometry::{Bounding3, Ray};
use super::interaction::SurfaceInteraction;
use super::material::{Material, TransportMode};
use super::light::AreaLight;
use std::sync::Arc;
use obstack::Obstack;

pub trait Primitive: Send + Sync {
    fn world_bound(&self) -> Box<dyn Bounding3<Float>>;
    fn intersect(&self, ray: &Ray) -> Option<SurfaceInteraction>;
    fn intersect_p(&self, ray: &Ray) -> bool;
    fn get_area_light(&self) -> Option<Arc<dyn AreaLight + Send + Sync>>;
    fn get_material(&self) -> Option<Arc<dyn Material + Send + Sync>>;
    fn compute_scattering_function(&self, arena: &Obstack, isect: &mut SurfaceInteraction, mode: TransportMode, allow_multiple_lobes: bool) {
        if let Some(material) = self.get_material() {
            material.compute_scattering_function(arena, isect, mode, allow_multiple_lobes);
        }
    }
}

mod geometric_primitive;
pub use geometric_primitive::*;
mod transformed_primitive;
pub use transformed_primitive::*;
mod aggregate;
pub use aggregate::*;