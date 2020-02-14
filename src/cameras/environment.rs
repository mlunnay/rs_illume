use crate::core::pbrt::{Float, lerp, consts::PI};
use crate::core::geometry::{Vector3f, Point3f, Ray};
use crate::core::camera::{Camera, CameraSample};
use crate::core::animated_transform::AnimatedTransform;
use crate::core::film::Film;
use crate::core::medium::Medium;
use crate::core::profiler::Profiler;
use std::sync::Arc;

pub struct EnvironmentCamera {
    // camera common
    pub camera_to_world: AnimatedTransform,
    pub shutter_open: Float,
    pub shutter_close: Float,
    pub film: Arc<Film>,
    pub medium: Option<Arc<dyn Medium + Send + Sync>>,
}

impl EnvironmentCamera {
    pub fn new(
        camera_to_world: AnimatedTransform,
        shutter_open: Float,
        shutter_close: Float,
        film: Arc<Film>,
        medium: Option<Arc<dyn Medium + Send + Sync>>
    ) -> EnvironmentCamera {
        EnvironmentCamera {
            camera_to_world,
            shutter_open,
            shutter_close,
            film,
            medium
        }
    }
}

impl Camera for EnvironmentCamera {
    fn get_camera_to_world(&self) -> AnimatedTransform {
        self.camera_to_world
    }

    fn get_shutter_open(&self) -> Float {
        self.shutter_open
    }

    fn get_shutter_close(&self) -> Float {
        self.shutter_close
    }

    fn get_film(&self) -> Arc<Film> {
        self.film
    }

    fn get_medium(&self) -> Option<Arc<dyn Medium + Send + Sync>> {
        self.medium
    }

    fn generate_ray(&self, sample: &CameraSample, ray: &mut Ray) -> Float {
        let _guard = Profiler::instance().profile("Camera::generate_ray()");
        // Compute environment camera ray direction
        let theta = PI * sample.p_film.y / self.film.full_resolution.y as Float;
        let phi = 2.0 * PI * sample.p_film.x / self.film.full_resolution.x as Float;
        let dir = Vector3f::new(theta.sin() * phi.cos(), theta.cos(),
            theta.sin() * phi.sin());
        *ray = Ray::new(Point3f::default(), dir);
        ray.time = lerp(sample.time, self.shutter_open, self.shutter_close);
        *ray = self.camera_to_world.transform_ray(&*ray);
        0.0
    }
}