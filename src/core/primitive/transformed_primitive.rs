use super::Primitive;
use crate::core::pbrt::Float;
use crate::core::surface_interaction::SurfaceInteraction;
use crate::core::material::{Material, TransportMode};
use crate::core::light::Light;
use crate::core::geometry::{Ray, Bounding3};
use crate::core::stats_accumulator::StatsAccumulator;
use crate::core::animated_transform::AnimatedTransform;
use crate::core::transform::Transform;
use std::sync::Arc;
use std::mem;
use obstack::Obstack;

pub struct TransformedPrimitive {
    primitive: Arc<dyn Primitive>,
    primitive_to_world: AnimatedTransform
}

impl TransformedPrimitive {
    pub fn new(
        primitive: Arc<dyn Primitive>,
        primitive_to_world: AnimatedTransform
    ) -> TransformedPrimitive {
        StatsAccumulator::instance().report_memory_counter(String::from("Memory/Primitive"), mem::size_of::<TransformedPrimitive>() as i64);
        TransformedPrimitive {
            primitive,
            primitive_to_world
        }
    }
}

impl Primitive for TransformedPrimitive {
    fn world_bound(&self) -> Box<dyn Bounding3<Float>>{
        self.primitive_to_world.motion_bounds(self.primitive.world_bound())
    }

    fn intersect(&self, r: &Ray) -> Option<SurfaceInteraction>{
        // Compute _ray_ after transformation by _PrimitiveToWorld_
        let interpolated_prim_to_world = Transform::default();
        self.primitive_to_world.interpolate(r.time, &mut interpolated_prim_to_world);
        let ray = interpolated_prim_to_world.inverse().transform_ray(r);
        if let Some(mut isect) = self.primitive.intersect(&ray) {
            ray.t_max = r.t_max;
            // Transform instance's intersection data to world space
            if !interpolated_prim_to_world.is_identity() {
                isect = interpolated_prim_to_world.tranform_surface_interaction(&isect);
                assert!(isect.n.dot(&isect.shading.n) >= 0.0);
                return Some(isect);
            }
        }
        return None;
    }

    fn intersect_p(&self, ray: &Ray) -> bool{
        let interpolated_prim_to_world = Transform::default();
        self.primitive_to_world.interpolate(ray.time, &mut interpolated_prim_to_world);
        self.primitive.intersect_p(&interpolated_prim_to_world.inverse().transform_ray(ray))
    }

    fn get_area_light(&self) -> Option<Arc<dyn Light>>{
        None
    }

    fn get_material(&self) -> Option<Arc<dyn Material>>{
        None
    }

    fn compute_scattering_function(&self, arena: &Obstack, isect: &mut SurfaceInteraction, mode: TransportMode, allow_multiple_lobes: bool) {
        panic!("TransformedPrimitive::compute_scattering_functions() shouldn't be called.");
    }
}