use obstack::Obstack;
use super::surface_interaction::SurfaceInteraction;

pub enum TransportMode {
    Radiance,
    Importance
}

pub trait Material {
    compute_scattering_function(&self, arena: &Obstack, isect: &mut SurfaceInteraction, mode: TransportMode, allow_multiple_lobes: bool);
}