use super::pbrt::Spectrum;
use super::primitive::Primitive;
use super::geometry::{Bounds3f, Ray, Vector3f};
use super::light::{Light, LightFlags};
use super::sampler::Sampler;
use super::interaction::{Interaction, SurfaceInteraction};
use super::stats_accumulator::StatsAccumulator;
use std::sync::Arc;

pub struct Scene {
    aggregate: Arc<dyn Primitive + Send + Sync>,
    _world_bound: Bounds3f,
    pub lights: Vec<Arc<dyn Light + Send + Sync>>,
    // Store infinite light sources separately for cases where we only want
    // to loop over them.
    pub infinite_lights: Vec<Arc<dyn Light + Send + Sync>>
}

impl Scene {
    pub fn new(
        aggregate: Arc<dyn Primitive + Send + Sync>,
        lights: Vec<Arc<dyn Light + Send + Sync>>
    ) -> Scene {
        let mut scene = Scene {
            aggregate,
            _world_bound: aggregate.world_bound().aabb(),
            lights: lights,
            infinite_lights: Vec::new()
        };
        for light in scene.lights {
            light.preprocess(&scene);
            if light.get_flags() & LightFlags::Infinite != 0 {
                scene.infinite_lights.push(light.clone());
            }
        }
        scene
    }

    pub fn world_bound(&self) -> Bounds3f {
        self._world_bound
    }

    pub fn intersect(&self, ray: &Ray) -> Option<SurfaceInteraction> {
        StatsAccumulator::instance().report_counter(String::from("Intersections/Regular ray intersection tests"), 1);
        debug_assert!(ray.d != Vector3f::default());
        self.aggregate.intersect(ray)
    }

    pub fn intersect_p(&self, ray: &Ray) -> bool {
        StatsAccumulator::instance().report_counter(String::from("Intersections/Shadow ray intersection tests"), 1);
        debug_assert!(ray.d != Vector3f::default());
        self.aggregate.intersect_p(ray)
    }

    /// A generalization of Scene::Intersect() that returns the first intersection with a light-scattering surface along the given ray as well as the beam transmittance up to that point.
    pub fn intersect_tr(&self, mut ray: &Ray, sampler: Box<dyn Sampler>) -> (Option<SurfaceInteraction>, Spectrum) {
        let mut tr = Spectrum::new(1.0);
        loop {
            // Accumulate beam transmittance for ray segment
            if let Some(medium) = ray.medium {
                tr *= medium.tr(ray, sampler);
            }
            if let Some(hit_surface) = self.intersect(ray) {
                // Initialize next ray segment or terminate transmittance computation
                if let Some(primitive) = hit_surface.primitive {
                    if primitive.get_material().is_none() {
                        return (Some(hit_surface), tr);
                    }
                }
                ray = &hit_surface.spawn_ray(&ray.d);
            }
            else {
                return (None, tr);
            }
        }
    }
}