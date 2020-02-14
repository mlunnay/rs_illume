use crate::core::{
    integrator::{Integrator, SamplerIntegrator, SamplerIntegratorBase},
    geometry::{Bounds2i, Ray, Vector3f, Normal3},
    camera::Camera,
    sampler::{Sampler},
    profiler::Profiler,
    scene::Scene,
    pbrt::{Spectrum, Float},
    interaction::{Interaction, SurfaceInteraction},
    material::TransportMode,
    sampling::{cosine_sample_hemisphere, cosine_hemisphere_pdf, uniform_sample_hemisphere, uniform_hemisphere_pdf}
};
use std::sync::Arc;
use obstack::Obstack;

pub struct AOIntegrator {
    base: SamplerIntegratorBase,
    cos_sample: bool,
    n_samples: u32
}

impl AOIntegrator {
    pub fn new(
        cos_sample: bool,
        ns: usize,
        camera: Arc<dyn Camera + Send + Sync>,
        sampler: Arc<dyn Sampler + Send + Sync>,
        pixel_bounds: Bounds2i
    ) -> AOIntegrator
    {
        let n_samples = sampler.round_count(ns);
        if n_samples != n_samples {
            warn!("Taking {} samples, not {} as specified", n_samples, ns);
        }
        sampler.request_2d_array(n_samples);
        AOIntegrator {
            base: SamplerIntegratorBase::new(camera, sampler, pixel_bounds),
            cos_sample,
            n_samples: n_samples as u32
        }
    }
}

impl SamplerIntegrator for AOIntegrator {
    fn li(&self, ray: &Ray, scene: &Scene, sampler: Box<dyn Sampler>, areana: &mut Obstack, depth: i32) -> Spectrum {
        let p = Profiler::instance().profile("SamplerIntegrator::li()");
        let mut l = Spectrum::new(0.0);
        // Intersect _ray_ with scene and store intersection in _isect_
        let mut isect = SurfaceInteraction::default();
        let mut ray = ray.clone();

        loop {
            if let Some(si) = scene.intersect(&ray) {
                isect = si;
                isect.compute_scattering_functions(&ray, areana, true, TransportMode::Radiance);
                if isect.get_bsdf().is_none() {
                    vlog!(2, "Skipping intersection due to None bsdf");
                    ray = isect.spawn_ray(&ray.d);
                    continue;
                }

                // Compute coordinate frame based on true geometry, not shading
                // geometry.
                let n = isect.n.face_forward(&Normal3{ x: -ray.d.x, y: -ray.d.y, z: -ray.d.z  });
                let s = isect.dpdu.normalize();
                let t = Vector3f::cross(&isect.n.into(), &s);

                if let Some(u) = sampler.get_2d_array(self.n_samples as usize) {
                    for i in 0..self.n_samples as usize {
                        let (mut wi, pdf) = if self.cos_sample {
                            let wi = cosine_sample_hemisphere(&u[i]);
                            (wi, cosine_hemisphere_pdf(wi.z.abs()))
                        } else {
                            let wi = uniform_sample_hemisphere(&u[i]);
                            (wi, uniform_hemisphere_pdf())
                        };

                        // Transform wi from local frame to world space.
                        wi = Vector3f::new(
                            s.x * wi.x + t.x * wi.y + n.x * wi.z,
                            s.y * wi.x + t.y * wi.y + n.y * wi.z,
                            s.z * wi.x + t.z * wi.y + n.z * wi.z,
                        );

                        if !scene.intersect_p(&isect.spawn_ray(&wi)) {
                            l += Spectrum::new(wi.dot(&n.into()) / (pdf * self.n_samples as Float));
                        }
                    }
                }
            }
            return l;
        }
    }
}

impl Integrator for AOIntegrator {
    fn render(&self, scene: &Scene) {
        self.base.render(self, scene);
    }

    fn specular_reflect(
        &self,
        ray: &Ray,
        isect: &SurfaceInteraction,
        scene: &Scene,
        sampler: Box<dyn Sampler>,
        arena: &mut Obstack,
        depth: i32
    ) -> Spectrum {
        self.base.specular_reflect(self, ray, isect, scene, sampler, arena, depth)
    }
}