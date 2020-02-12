use super::Primitive;
use crate::core::pbrt::Float;
use crate::core::interaction::SurfaceInteraction;
use crate::core::material::{Material};
use crate::core::light::Light;
use crate::core::shape::Shape;
use crate::core::medium::MediumInterface;
use crate::core::geometry::{Ray, Bounding3};
use crate::core::stats_accumulator::StatsAccumulator;
use std::sync::Arc;
use std::mem;

pub struct GeometricPrimitive {
    shape: Arc<dyn Shape>,
    material: Option<Arc<dyn Material>>,
    area_light: Option<Arc<dyn Light>>,
    medium_interface: Option<Arc<MediumInterface>>
}

impl GeometricPrimitive {
    pub fn new(
        shape: Arc<dyn Shape>,
        material: Option<Arc<dyn Material>>,
        area_light: Option<Arc<dyn Light>>,
        medium_interface: Option<Arc<MediumInterface>>
    ) -> GeometricPrimitive {
        StatsAccumulator::instance().report_memory_counter(String::from("Memory/Primitive"), mem::size_of::<GeometricPrimitive>() as i64);
        GeometricPrimitive {
            shape,
            material,
            area_light,
            medium_interface
        }
    }
}

impl Primitive for GeometricPrimitive {
    fn world_bound(&self) -> Box<dyn Bounding3<Float>> {
        self.shape.world_bound()
    }

    fn intersect(&self, ray: &Ray) -> Option<SurfaceInteraction> {
        if let Some((mut isect, t_hit)) = self.shape.intersect(ray, true) {
            ray.t_max = t_hit;
            isect.primitive = Some(self);
            assert!(isect.n.dot(&isect.shading.n) >= 0.0);
            // Initialize _SurfaceInteraction::mediumInterface_ after _Shape_
            // intersection
            if let Some(medium_interface) = self.medium_interface {
                if medium_interface.is_medium_transition() {
                    isect.medium_interface = Some(medium_interface);
                } else {
                    if let Some(mi) = ray.medium {
                        isect.medium_interface = Some(Arc::new(MediumInterface {
                            inside: Some(mi.clone()),
                            outside: Some(mi.clone())
                        }));
                    }
                }
            }
            Some(isect)
        } else {
            None
        }
    }

    fn intersect_p(&self, ray: &Ray) -> bool {
        self.shape.intersect_p(ray, true)
    }

    fn get_area_light(&self) -> Option<Arc<dyn Light>> {
        if let Some(light) = self.area_light {
            Some(light.clone())
        } else {
            None
        }
    }

    fn get_material(&self) -> Option<Arc<dyn Material>> {
        if let Some(material) = self.material {
            Some(material.clone())
        } else {
            None
        }
    }

}