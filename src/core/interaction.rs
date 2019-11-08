use super::pbrt::{Float, consts::SHADOW_EPSILON};
use super::geometry::{Vector3f, Normal3f, Point3f, Ray, offset_ray_origin};
use super::medium::{Medium, MediumInterface};
use std::sync::Arc;

pub trait Interaction {
    fn get_p(&self) -> Point3f;
    fn get_time(&self) -> Float;
    fn get_p_error(&self) -> Vector3f;
    fn get_wo(&self) -> Vector3f;
    fn get_n(&self) -> Normal3f;
    fn get_medium_interface(&self) -> Option<Arc<MediumInterface>> {
        None
    }
    
    fn is_surface_interaction(&self) -> bool {
        self.get_n() != Normal3f::default()
    }

    fn is_medium_interaction(&self) -> bool {
        !self.is_surface_interaction()
    }

    /// Spawn a ray from this Interaction going in direction d.
    fn spawn_ray(&self, d: &Vector3f) -> Ray {
        let o = offset_ray_origin(&self.get_p(), &self.get_p_error(), &self.get_n(), d);
        Ray{
            o,
            d: *d,
            t_max: num::Float::infinity(),
            time: self.get_time(),
            medium: self.get_medium(d),
            differential: None
        }
    }

    /// Spawn a ray from this Interaction going towards p.
    fn spawn_ray_to_point(&self, p: &Point3f) -> Ray {
        let d = *p - self.get_p();
        let o = offset_ray_origin(&self.get_p(), &self.get_p_error(), &self.get_n(), &d);
        Ray{
            o,
            d,
            t_max: 1.0 - SHADOW_EPSILON,
            time: self.get_time(),
            medium: self.get_medium(&d),
            differential: None
        }
    }

    /// Spawn a ray going from this Interaction towards another Interaction.
    fn spawn_ray_to(&self, i: &dyn Interaction) -> Ray {
        let o = offset_ray_origin(&self.get_p(), &self.get_p_error(), &self.get_n(), &(i.get_p() - self.get_p()));
        let target = offset_ray_origin(&i.get_p(), &i.get_p_error(), &i.get_n(), &(o - i.get_p()));
        let d = target - o;
        Ray{
            o,
            d,
            t_max: 1.0 - SHADOW_EPSILON,
            time: self.get_time(),
            medium: self.get_medium(&d),
            differential: None
        }
    }

    fn get_medium(&self, w: &Vector3f) -> Option<Arc<dyn Medium>> {
        match self.get_medium_interface() {
            Some(ref medium) => {
                if w.dot(&self.get_n().into()) > 0.0 {
                    medium.inside.clone()
                }
                else {
                    medium.outside.clone()
                }
            }
            None => None
        }
    }
}

/// A Simple implementation of Interaction.
#[derive(Debug, Default, Copy, Clone)]
pub struct SimpleInteraction {
    pub p: Point3f,
    pub time: Float,
    pub p_error: Vector3f,
    pub wo: Vector3f,
    pub n: Normal3f
}

impl SimpleInteraction {
    pub fn new(
        p: Point3f,
        time: Float,
        p_error: Vector3f,
        wo: Vector3f,
        n: Normal3f
    ) -> SimpleInteraction {
        SimpleInteraction{
            p,
            time,
            p_error,
            wo,
            n
        }
    }
}

impl Interaction for SimpleInteraction {
    fn get_p(&self) -> Point3f {
        self.p
    }

    fn get_time(&self) -> Float {
        self.time
    }

    fn get_p_error(&self) -> Vector3f {
        self.p_error
    }

    fn get_wo(&self) -> Vector3f {
        self.wo
    }

    fn get_n(&self) -> Normal3f {
        self.n
    }

}