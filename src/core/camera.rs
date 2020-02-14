use super::pbrt::{Float, Spectrum};
use super::geometry::{Point2f, Ray, RayDifferential, Vector3f};
use super::animated_transform::AnimatedTransform;
use super::medium::Medium;
use super::interaction::Interaction;
use super::light::VisibilityTester;
use super::film::Film;
use std::fmt;
use std::sync::Arc;

pub trait Camera: Send + Sync {
    fn get_camera_to_world(&self) -> AnimatedTransform;

    fn get_shutter_open(&self) -> Float;

    fn get_shutter_close(&self) -> Float;

    fn get_film(&self) -> Arc<Film>;

    fn get_medium(&self) -> Option<Arc<dyn Medium + Send + Sync>> {
        None
    }

    fn generate_ray(&self, sample: &CameraSample, ray: &mut Ray) -> Float;

    fn generate_ray_differential(&self, sample: &CameraSample, ray: &mut Ray) -> Float {
        let wt = self.generate_ray(sample, ray);
        if wt == 0.0 {
            return 0.0;
        }

        // Find camera ray after shifting a fraction of a pixel in the $x$ direction
        let mut rd = RayDifferential::default();
        let wtx: Float = 0.0;
        let offsets = [0.05 as Float, -0.05 as Float];
        for eps in offsets.iter() {
            let sshift = sample;
            sshift.p_film.x += eps;
            let rx = Ray::default();
            wtx = self.generate_ray(sshift, &mut rx);
            rd.rx_origin = ray.o + (rx.o - ray.o) / *eps;
            rd.rx_direction = ray.d + (rx.d - ray.d) / *eps;
            if wtx != 0.0 {
                break;
            }
        }
        if wtx == 0.0 {
            return 0.0;
        }

        // Find camera ray after shifting a fraction of a pixel in the $y$ direction
        let wty: Float = 0.0;
        let offsets = [0.05 as Float, -0.05 as Float];
        for eps in offsets.iter() {
            let sshift = sample;
            sshift.p_film.y += eps;
            let ry = Ray::default();
            wty = self.generate_ray(sshift, &mut ry);
            rd.ry_origin = ray.o + (ry.o - ray.o) / *eps;
            rd.ry_direction = ray.d + (ry.d - ray.d) / *eps;
            if wty != 0.0 {
                break;
            }
        }
        if wty == 0.0 {
            return 0.0;
        }

        ray.differential = Some(rd);
        wt
    }

    fn we(&self, ray: &Ray, p_raster2: Option<&mut Point2f>) -> Spectrum {
        error!("Camera::we() is not implemented!");
        Spectrum::new(0.0)
    }

    fn pdf_we(&self, ray: &Ray) -> (Float, Float) {
        error!("Camera::pdf_we() is not implemented!");
        (0.0, 0.0)
    }

    fn sample_wi(
        &self,
        iref: Box<dyn Interaction>,
        u: &Point2f,
        wi: &mut Vector3f,
        pdf: &mut Float,
        p_raster: &mut Point2f,
        vis: &mut VisibilityTester
    ) -> Spectrum {
        error!("Camera::sample_we() is not implemented!");
        Spectrum::new(0.0)
    }
}

#[derive(Default, Debug, Copy, Clone)]
pub struct CameraSample {
    pub p_film: Point2f,
    pub p_lens: Point2f,
    pub time: Float
}

impl fmt::Display for CameraSample {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "[ p_film: {}, p_lens: {}, time {} ]", self.p_film, self.p_lens, self.time)
    }
}