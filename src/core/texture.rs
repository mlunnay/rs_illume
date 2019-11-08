use super::surface_interaction::SurfaceInteraction;

pub trait Texture<T> {
    fn evaluate(&self, si: &SurfaceInteraction) -> T;
}