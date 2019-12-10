use super::pbrt::{Float, Spectrum};
use super::geometry::{Point2f, Vector3f, Ray, Normal3f};
use super::medium::MediumInterface;
use super::transform::Transform;
use super::interaction::{Interaction, SimpleInteraction};
use super::scene::Scene;
use super::sampler::Sampler;
use super::stats_accumulator::StatsAccumulator;
use std::sync::Arc;
use std::borrow::Borrow;

pub struct LightFlags {}

impl LightFlags {
    pub const DeltaPosition: u8 = 1;
    pub const DeltaDirection: u8 = 2;
    pub const Area: u8 = 4;
    pub const Infinite: u8 = 8;
}

pub trait Light: Send + Sync {
    fn get_flags(&self) -> u8;
    fn get_n_samples(&self) -> usize;
    fn get_medium_interface(&self) -> Arc<MediumInterface>;

    fn get_light_to_world(&self) -> Transform;
    fn get_world_to_light(&self) -> Transform;

    fn sample_li(
        &self,
        iref: Box<dyn Interaction>,
        u: &Point2f,
        wi: &mut Vector3f,
        pdf: &mut Float,
        vis: &mut VisibilityTester
    ) -> Spectrum;

    fn power(&self) -> Spectrum;

    fn preprocess(&self, scene: &Scene) {}

    fn le(&self, r: &Ray) -> Spectrum {
        Spectrum::new(0.0)
    }

    fn pdf_li(&self, iref: Box<dyn Interaction>, wi: &Vector3f) -> Float;

    fn sample_le(
        &self,
        u1: &Point2f,
        u2: &Point2f,
        time: Float,
        ray: &mut Ray,
        n_light: &mut Normal3f,
        pdf_pos: &mut Float,
        pdf_dir: &mut Float
    ) -> Spectrum;

    fn pdf_le(
        &self,
        ray: &Ray,
        n_light: &Normal3f,
        pdf_pos: &mut Float,
        pdf_dir: &mut Float
    );
}

pub trait AreaLight: Light {
    fn l(&self, intr: &dyn Interaction, w: &Vector3f) -> Spectrum;
}

pub struct VisibilityTester {
    _p0: Box<dyn Interaction>,
    _p1: Box<dyn Interaction>
}

impl VisibilityTester {
    pub fn new(p0: Box<dyn Interaction>, p1: Box<dyn Interaction>) -> VisibilityTester {
        VisibilityTester { _p0: p0, _p1: p1 }
    }

    pub fn p0(&self) -> &dyn Interaction {
        self._p0.borrow()
    }

    pub fn p1(&self) -> &dyn Interaction {
        self._p1.borrow()
    }

    pub fn unoccluded(&self, scene: &Scene) -> bool {
        !scene.intersect_p(&self._p0.spawn_ray_to(&*self._p1))
    }

    pub fn tr(&self, scence: &Scene, sampler: Box<dyn Sampler>) -> Spectrum {
        let mut ray = self._p0.spawn_ray_to(&*self._p1);
        let tr = Spectrum::new(1.0);
        loop {
            // Update transmittance for current ray segment
            if let Some(medium) = ray.medium {
                tr *= medium.tr(&ray, sampler);
            }

            if let Some(isect) = scence.intersect(&ray) {
                // Handle opaque surface along ray's path
                if let Some(primitive) = isect.primitive {
                    if primitive.get_material().is_none() {
                        return Spectrum::new(0.0);
                    }
                }

                // Generate next ray segment or return final transmittance
                ray = isect.spawn_ray_to(&*self._p1);
            } else {
                break;
            }
        }
        tr
    }
}

impl Default for VisibilityTester {
    fn default() -> VisibilityTester {
        VisibilityTester {
            _p0: Box::new(SimpleInteraction::default()),
            _p1: Box::new(SimpleInteraction::default())
        }
    }
}

#[inline]
pub fn is_delta_light(flags: u8) -> bool {
    flags & LightFlags::DeltaPosition != 0 ||
        flags & LightFlags::DeltaDirection != 0
}

/// Call in the new method of Lights to increment the Lights stat
#[inline]
pub fn increment_light_count() {
    StatsAccumulator::instance().report_counter(String::from("Scene/Lights"), 1);
}

/// Call in the new method of AreaLights to increment the AreaLights stat. Note increment_light_count needs to also be called.
#[inline]
pub fn increment_area_light_count() {
    StatsAccumulator::instance().report_counter(String::from("Scene/AreaLights"), 1);
}