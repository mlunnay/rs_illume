use super::pbrt::{Float, Spectrum};
use super::geometry::{Point2f, Point2i, Bounds2i, Bounding2, Ray, RayDifferential, Vector3f, dot_vec_normal};
use super::scene::Scene;
use super::interaction::{Interaction, SurfaceInteraction};
use super::sampler::Sampler;
use super::sampling::{Distribution1D, power_heuristic};
use super::light::{Light, VisibilityTester, is_delta_light};
use super::camera::Camera;
use super::profiler::Profiler;
use super::stats_accumulator::StatsAccumulator;
use super::reflection::BxDFType;
use super::progress_reporter::ProgressReporter;
use obstack::Obstack;
use std::sync::Arc;
use rayon::prelude::*;

pub trait Integrator {
    fn render(&self, scene: &Scene);

    fn specular_reflect(
        &self,
        ray: &Ray,
        isect: &SurfaceInteraction,
        scene: &Scene,
        sampler: Box<dyn Sampler>,
        arena: &mut Obstack,
        depth: i32
    ) -> Spectrum;
}

pub trait SamplerIntegrator: Send + Sync {
    fn preprocess(&self, scene: &Scene, sampler: Arc<dyn Sampler + Send + Sync>) {}

    fn li(&self, ray: &Ray, scene: &Scene, sampler: Box<dyn Sampler>, areana: &mut Obstack, depth: i32) -> Spectrum;
}

pub struct SamplerIntegratorBase {
    pub camera: Arc<dyn Camera + Send + Sync>,
    sampler: Arc<dyn Sampler + Send + Sync>,
    pixel_bounds: Bounds2i
}

impl SamplerIntegratorBase {
    pub fn new(camera: Arc<dyn Camera + Send + Sync>, sampler: Arc<dyn Sampler + Send + Sync>, pixel_bounds: Bounds2i) -> SamplerIntegratorBase {
        SamplerIntegratorBase { camera, sampler, pixel_bounds }
    }

    pub fn render(&self, child: &impl SamplerIntegrator, scene: &Scene) {
        child.preprocess(scene, self.sampler);
        // Render image tiles in parallel

        // Compute number of tiles, _nTiles_, to use for parallel rendering
        let sample_bounds = self.camera.get_film().get_sample_bounds();
        let sample_extent = sample_bounds.diagonal();
        const tile_size: i32 = 16;
        let n_tiles = Point2i::new((sample_extent.x + tile_size - 1) / tile_size,
            (sample_extent.y + tile_size - 1) / tile_size);
        let reporter = ProgressReporter::new((n_tiles.x * n_tiles.y) as u64, "Rendering");
        Bounds2i::new(Point2i::default(), n_tiles).into_iter().par_bridge().for_each(|tile| {
            // Render section of image corresponding to _tile_

            // Allocate _MemoryArena_ for tile
            let arena = Obstack::new();

            // Get sampler instance for tile
            let seed = tile.y * n_tiles.x + tile.x;
            let tile_sampler = self.sampler.clone_with_seed(seed as u64);

            // Compute sample bounds for tile
            let x0 = sample_bounds.min.x + tile.x * tile_size;
            let x1 = (x0 + tile_size).min(sample_bounds.max.x);
            let y0 = sample_bounds.min.y + tile.y * tile_size;
            let y1 = (y0 + tile_size).min(sample_bounds.max.y);
            let tile_bounds = Bounds2i::new(Point2i::new(x0, y0), Point2i::new(x1, y1));
            info!("Starting image tile {}", tile_bounds);

            // Get _FilmTile_ for tile
            let film_tile = self.camera.get_film().get_film_tile(&tile_bounds);

            // Loop over pixels in tile to render them
            for pixel in &tile_bounds {
                {
                    let __pp = Profiler::instance().profile("Sampler::StartPixelSample()");
                    tile_sampler.start_pixel(&pixel);
                }

                // Do this check after the StartPixel() call; this keeps
                // the usage of RNG values from (most) Samplers that use
                // RNGs consistent, which improves reproducability /
                // debugging.
                if !self.pixel_bounds.inside_exclusive(&pixel) {
                    continue;
                }

                let mut done = false;
                while !done {
                    // Initialize _CameraSample_ for current sample
                    let camera_sample = tile_sampler.get_camera_sample(&pixel);

                    // Generate camera ray for current sample
                    let mut ray = Ray::default();
                    let ray_weight = self.camera.generate_ray_differential(&camera_sample, &mut ray);
                    ray.scale_differentials(1.0 / (tile_sampler.get_samples_per_pixel() as Float).sqrt());
                    StatsAccumulator::instance().report_counter(String::from("Integrator/Camera rays traced"), 1);

                    // Evaluate radiance along camera ray
                    let mut l = Spectrum::new(0.0);
                    if ray_weight > 0.0 {
                        l = child.li(&ray, scene, tile_sampler, &mut arena, 0);
                    }

                    // Issue warning if unexpected radiance value returned
                    if l.has_nans() {
                        error!("Not-a-number radiance value returned for pixel ({}, {}), sample {}. Setting to black.", pixel.x, pixel.y, tile_sampler.current_sample_number());
                        l = Spectrum::new(0.0);
                    } else if l.y() < -1e-5 {
                        error!("Negative luminance value, {}, returned for pixel ({}, {}), sample {}. Setting to black.", l.y(), pixel.x, pixel.y, tile_sampler.current_sample_number());
                        l = Spectrum::new(0.0);
                    } else if l.y().is_infinite() {
                        error!("Infinite luminance value returned for pixel ({}, {}), sample {}. Setting to black.", pixel.x, pixel.y, tile_sampler.current_sample_number());
                        l = Spectrum::new(0.0);
                    }
                    vlog!(1, "Camera sample: {} -> ray: {} -> L = {}", camera_sample, ray, l);

                    // Add camera ray's contribution to image
                    film_tile.add_sample(&camera_sample.p_film, &l, ray_weight);

                    done = !tile_sampler.start_next_sample();
                    
                    // arena will fall out of scope and be dropped.
                }
            }
            info!("Finished image tile {}", tile_bounds);

            // Merge image tile into _Film_
            self.camera.get_film().merge_film_tile(&film_tile);
            reporter.update(1);
        });
        reporter.done();

        info!("Rendering finished");

        // Save final image after rendering
        self.camera.get_film().write_image(1.0);
    }

    pub fn specular_reflect(
        &self,
        child: &impl SamplerIntegrator,
        ray: &Ray,
        isect: &SurfaceInteraction,
        scene: &Scene,
        sampler: Box<dyn Sampler>,
        arena: &mut Obstack,
        depth: i32
    ) -> Spectrum {
        // Compute specular reflection direction _wi_ and BSDF value
        let wo = isect.wo;
        let mut wi = Vector3f::default();
        let mut pdf: Float = 0.0;
        let bxdf_type = BxDFType::BSDF_REFLECTION | BxDFType::BSDF_SPECULAR;
        let mut _sampled_type: u8 = 0;
        let f = isect.bsdf.unwrap().sample_f(&wo, &mut wi, &self.sampler.get_2d().cast(), &mut pdf, bxdf_type, &mut _sampled_type);

        // Return contribution of specular reflection
        let ns = isect.shading.n;
        if pdf > 0.0 && !f.is_black() && dot_vec_normal(&wi, &ns).abs() != 0.0 {
            // Compute ray differential _rd_ for specular reflection
            let mut rd = isect.spawn_ray(&wi);
            if let Some(rdiff) = ray.differential {
                let rx_origin = isect.p + *isect.dpdx.read().unwrap();
                let ry_origin = isect.p + *isect.dpdy.read().unwrap();
                // Compute differential reflected directions
                let dndx = isect.shading.dndu * *isect.dudx.read().unwrap() +
                    isect.shading.dndv * *isect.dvdx.read().unwrap();
                let dndy = isect.shading.dndu * *isect.dudy.read().unwrap() +
                    isect.shading.dndv * *isect.dvdy.read().unwrap();
                let dwodx = -rdiff.rx_direction - wo;
                let dwody = -rdiff.ry_direction - wo;
                let d_dndx = dot_vec_normal(&dwodx, &ns) + dot_vec_normal(&wo, &dndx);
                let d_dndy = dot_vec_normal(&dwody, &ns) + dot_vec_normal(&wo, &dndy);
                let rx_direction = wi - dwodx + 2.0 * Vector3f::from(dot_vec_normal(&wo, &ns) * Vector3f::from(dndx + d_dndx * ns));
                let ry_direction = wi - dwody + 2.0 * Vector3f::from(dot_vec_normal(&wo, &ns) * Vector3f::from(dndy + d_dndy * ns));
                rd.differential = Some(RayDifferential {
                    rx_origin,
                    ry_origin,
                    rx_direction,
                    ry_direction
                });
            }
            f * child.li(&rd, scene, sampler, arena, depth + 1) * dot_vec_normal(&wi, &ns).abs() / pdf
        } else {
            Spectrum::new(0.0)
        }
    }

    pub fn specular_transmit(
        &self,
        child: &impl SamplerIntegrator,
        ray: &Ray,
        isect: &SurfaceInteraction,
        scene: &Scene,
        sampler: Box<dyn Sampler>,
        arena: &mut Obstack,
        depth: i32
    ) -> Spectrum {
        let wo = isect.wo;
        let mut wi = Vector3f::default();
        let mut pdf: Float = 0.0;
        let bxdf_type = BxDFType::BSDF_TRANSMISSION | BxDFType::BSDF_SPECULAR;
        let mut _sampled_type: u8 = 0;
        let bsdf = isect.bsdf.unwrap();
        let f = bsdf.sample_f(&wo, &mut wi, &self.sampler.get_2d().cast(), &mut pdf, bxdf_type, &mut _sampled_type);
        let mut l = Spectrum::new(0.0);
        let mut ns = isect.shading.n;
        if pdf > 0.0 && !f.is_black() && dot_vec_normal(&wi, &ns).abs() != 0.0 {
            // Compute ray differential _rd_ for specular reflection
            let mut rd = isect.spawn_ray(&wi);
            if let Some(rdiff) = ray.differential {
                let rx_origin = isect.p + *isect.dpdx.read().unwrap();
                let ry_origin = isect.p + *isect.dpdy.read().unwrap();
                // Compute differential reflected directions
                let mut dndx = isect.shading.dndu * *isect.dudx.read().unwrap() +
                    isect.shading.dndv * *isect.dvdx.read().unwrap();
                let mut dndy = isect.shading.dndu * *isect.dudy.read().unwrap() +
                    isect.shading.dndv * *isect.dvdy.read().unwrap();

                // The BSDF stores the IOR of the interior of the object being
                // intersected.  Compute the relative IOR by first out by
                // assuming that the ray is entering the object.
                let mut eta = 1.0 / bsdf.eta;
                if dot_vec_normal(&wo, &ns) < 0.0 {
                    // If the ray isn't entering, then we need to invert the
                    // relative IOR and negate the normal and its derivatives.
                    eta = 1.0 / eta;
                    ns = -ns;
                    dndx = -dndx;
                    dndy = -dndy;
                }

                /*
                Notes on the derivation:
                - pbrt computes the refracted ray as: \wi = -\eta \omega_o + [ \eta (\wo \cdot \N) - \cos \theta_t ] \N
                    It flips the normal to lie in the same hemisphere as \wo, and then \eta is the relative IOR from
                    \wo's medium to \wi's medium.
                - If we denote the term in brackets by \mu, then we have: \wi = -\eta \omega_o + \mu \N
                - Now let's take the partial derivative. (We'll use "d" for \partial in the following for brevity.)
                    We get: -\eta d\omega_o / dx + \mu dN/dx + d\mu/dx N.
                - We have the values of all of these except for d\mu/dx (using bits from the derivation of specularly
                    reflected ray deifferentials).
                - The first term of d\mu/dx is easy: \eta d(\wo \cdot N)/dx. We already have d(\wo \cdot N)/dx.
                - The second term takes a little more work. We have:
                    \cos \theta_i = \sqrt{1 - \eta^2 (1 - (\wo \cdot N)^2)}.
                    Starting from (\wo \cdot N)^2 and reading outward, we have \cos^2 \theta_o, then \sin^2 \theta_o,
                    then \sin^2 \theta_i (via Snell's law), then \cos^2 \theta_i and then \cos \theta_i.
                - Let's take the partial derivative of the sqrt expression. We get:
                    1 / 2 * 1 / \cos \theta_i * d/dx (1 - \eta^2 (1 - (\wo \cdot N)^2)).
                - That partial derivatve is equal to:
                    d/dx \eta^2 (\wo \cdot N)^2 = 2 \eta^2 (\wo \cdot N) d/dx (\wo \cdot N).
                - Plugging it in, we have d\mu/dx =
                    \eta d(\wo \cdot N)/dx - (\eta^2 (\wo \cdot N) d/dx (\wo \cdot N))/(-\wi \cdot N).
                */
                let dwodx = -rdiff.rx_direction - wo;
                let dwody = -rdiff.ry_direction - wo;
                let d_dndx = dot_vec_normal(&dwodx, &ns) + dot_vec_normal(&wo, &dndx);
                let d_dndy = dot_vec_normal(&dwody, &ns) + dot_vec_normal(&wo, &dndy);
                let mu = eta * dot_vec_normal(&wo, &ns) - dot_vec_normal(&wi, &ns).abs();
                let dmudx = (eta - (eta * eta * dot_vec_normal(&wo, &ns)) / dot_vec_normal(&wi, &ns).abs()) * d_dndx;
                let dmudy = (eta - (eta * eta * dot_vec_normal(&wo, &ns)) / dot_vec_normal(&wi, &ns).abs()) * d_dndy;
                let rx_direction = wi - eta * dwodx + Vector3f::from(mu * dndx + dmudx * ns);
                let ry_direction = wi - eta * dwody + Vector3f::from(mu * dndy + dmudy * ns);
                rd.differential = Some(RayDifferential {
                    rx_origin,
                    ry_origin,
                    rx_direction,
                    ry_direction
                });
            }
            l = f * child.li(&rd, scene, sampler, arena, depth + 1) * dot_vec_normal(&wi, &ns).abs() / pdf
        }
        l
    }
}

// default for handle_media is false.
pub fn uniform_sample_all_lights(
    it: Box<dyn Interaction>,
    scene: &Scene,
    arena: &mut Obstack,
    sampler: Box<dyn Sampler>,
    n_light_samples: &Vec<u32>,
    handle_media: bool
) -> Spectrum {
    let _profile = Profiler::instance().profile("Direct Lighting");
    let mut l = Spectrum::new(0.0);
    for j in 0..scene.lights.len() {
        // Accumulate contribution of _j_th light to _L_
        let light = scene.lights[j];
        let n_samples = n_light_samples[j];
        let u_light_array = sampler.get_2d_array(n_samples as usize);
        let u_scattering_array = sampler.get_2d_array(n_samples as usize);
        if u_light_array.is_none() || u_scattering_array.is_none() {
            // Use a single sample for illumination from _light_
            let u_light: Point2f = sampler.get_2d().cast();
            let u_scattering: Point2f = sampler.get_2d().cast();
            l += estimate_direct(it, &u_scattering, light, &u_light, scene, sampler, arena, handle_media, false);
        } else {
            // Estimate direct lighting using sample arrays
            let mut ld = Spectrum::new(0.0);
            for k in 0..n_samples as usize {
                ld += estimate_direct(it, &u_scattering_array.unwrap()[k], light, &u_light_array.unwrap()[k], scene, sampler, arena, handle_media, false);
            }
            l += ld / n_samples as Float;
        }
    }

    l
}

// default for handle_media is false, and light_distrib is None
pub fn uniform_sample_one_light(
    it: Box<dyn Interaction>,
    scene: &Scene,
    arena: &mut Obstack,
    sampler: Box<dyn Sampler>,
    handle_media: bool,
    light_distrib: Option<&mut Distribution1D>
) -> Spectrum {
    let _profile = Profiler::instance().profile("Direct Lighting");
    // Randomly choose a single light to sample, _light_
    let n_lights = scene.lights.len();
    if n_lights == 0 {
        return Spectrum::new(0.0);
    }
    let light_num: usize;
    let mut light_pdf: Float = 0.0;
    if let Some(light_distrib) = light_distrib {
        light_num = light_distrib.sample_discrete(sampler.get_1d(), Some(&mut light_pdf), None);
        if light_pdf == 0.0 {
            return Spectrum::new(0.0);
        }
    } else {
        light_num = ((sampler.get_1d() * n_lights as Float) as usize).min(n_lights - 1);
        light_pdf = 1.0 / n_lights as Float;
    }
    let light = scene.lights[light_num];
    let u_light: Point2f = sampler.get_2d().cast();
    let u_scattering: Point2f = sampler.get_2d().cast();
    estimate_direct(it, &u_scattering, light, &u_light, scene, sampler, arena, handle_media, false) / light_pdf
}

// default for handle_media is false, and specular is false.
pub fn estimate_direct(
    it: Box<dyn Interaction>,
    u_shading: &Point2f,
    light: Arc<dyn Light + Send + Sync>,
    u_light: &Point2f,
    scene: &Scene,
    sampler: Box<dyn Sampler>,
    arena: &mut Obstack,
    handle_media: bool,
    specular: bool
) -> Spectrum {
    let bsdf_flags = if specular { BxDFType::BSDF_ALL } else { BxDFType::BSDF_ALL & !BxDFType::BSDF_SPECULAR };
    let mut ld = Spectrum::new(0.0);
    // Sample light source with multiple importance sampling
    let mut wi = Vector3f::default();
    let mut light_pdf: Float = 0.0;
    let mut scattering_pdf: Float = 0.0;
    let mut visibility = VisibilityTester::default();
    let mut li = light.sample_li(it, u_light, &mut wi, &mut light_pdf, &mut visibility);
    vlog!(2, "EstimateDirect u_light: {} -> li: {}, wi: {}, pdf: {}", u_light, li, wi, light_pdf);
    if light_pdf > 0.0 && !li.is_black() {
        // Compute BSDF or phase function's value for light sample
        let mut f = Spectrum::default();
        if it.is_surface_interaction() {
            // Evaluate BSDF for light sampling strategy
            if let Some(bsdf) = it.get_bsdf() {
                if let Some(shading) = it.get_shading() {
                    f = bsdf.f(&it.get_wo(), &wi, bsdf_flags) * dot_vec_normal(&wi, &shading.n).abs();
                    scattering_pdf = bsdf.pdf(&it.get_wo(), &wi, bsdf_flags);
                    vlog!(2, "  surf f*dot : {}, scattering_pdf: {}", f, scattering_pdf);
                }
            }
        } else {
            // Evaluate phase function for light sampling strategy
            if let Some(phase) = it.get_phase() {
                let p = phase.p(&it.get_wo(), &wi);
                f = Spectrum::new(p);
                scattering_pdf = p;
                vlog!(2, "  medium p: {}", p);
            }
        }
        if !f.is_black() {
            // Compute effect of visibility for light source sample
            if handle_media {
                li *= visibility.tr(scene, sampler);
                vlog!(2, "  after Tr, LI: {}", li);
            } else {
                if !visibility.unoccluded(scene) {
                    vlog!(2, "  shadow ray blocked");
                    li = Spectrum::new(0.0);
                } else {
                    vlog!(2, "  shadow ray unoccluded");
                }
            }

            // Add light's contribution to reflected radiance
            if !li.is_black() {
                if is_delta_light(light.get_flags()) {
                    ld += f * li / light_pdf;
                } else {
                    let weight = power_heuristic(1, light_pdf, 1, scattering_pdf);
                    ld += f * li * weight / light_pdf;
                }
            }
        }
    }

    // Sample BSDF with multiple importance sampling
    if !is_delta_light(light.get_flags()) {
        let mut f = Spectrum::new(0.0);
        let mut sampled_specular = false;
        if it.is_surface_interaction() {
            // Sample scattered direction for surface interactions
            let mut sampled_type: u8 = 0;
            if let Some(bsdf) = it.get_bsdf() {
                if let Some(shading) = it.get_shading() {
                    f = bsdf.sample_f(&it.get_wo(), &mut wi, &u_shading, &mut scattering_pdf, bsdf_flags, &mut sampled_type);
                    f *= dot_vec_normal(&wi, &shading.n).abs();
                    sampled_specular = sampled_type & BxDFType::BSDF_SPECULAR != 0;
                }
            }
        } else {
            // Sample scattered direction for medium interactions
            if let Some(phase) = it.get_phase() {
                let p = phase.sample_p(&it.get_wo(), &mut wi, u_shading);
                f = Spectrum::new(p);
                scattering_pdf = p;
            }
        }
        vlog!(2, "  BSDF / phase sampling f: {}, scattering_pdf: {}", f, scattering_pdf);
        if !f.is_black() && scattering_pdf > 0.0 {
            // Account for light contributions along sampled direction _wi_
            let mut weight: Float = 1.0;
            if !sampled_specular {
                light_pdf = light.pdf_li(it, &wi);
                if light_pdf == 0.0 {
                    return ld;
                }
                weight = power_heuristic(1, scattering_pdf, 1, light_pdf);
            }

            // Find intersection and compute transmittance
            let mut light_isect = SurfaceInteraction::default();
            let ray = it.spawn_ray(&wi);
            let mut tr = Spectrum::new(1.0);
            let found_surface_interaction = if handle_media {
                let (isect, tr_) = scene.intersect_tr(&ray, sampler);
                tr = tr_;
                if let Some(is) = isect {
                    light_isect = is;
                    true
                } else {
                    false
                }
            } else {
                if let Some(isect) = scene.intersect(&ray) {
                    light_isect = isect;
                    true
                } else {
                    false
                }
            };

            // Add light contribution from material sampling
            let mut li = Spectrum::new(0.0);
            if found_surface_interaction {
                if let Some(primitive) = light_isect.primitive {
                    if let Some(area_light) = primitive.get_area_light() {
                        let al = &*area_light as *const _ as *const usize;
                        let l = &*light as *const _ as *const usize;
                        if al == l {
                            li = light_isect.le(&-wi);
                        }
                    }
                }
            } else {
                li = light.le(&ray);
            }
        }
    }
    ld
}

pub fn compute_light_power_distribution(scene: &Scene) -> Option<Arc<Distribution1D>> {
    if scene.lights.is_empty() {
        return None;
    }
    let light_power: Vec<Float> = Vec::new();
    for light in &scene.lights {
        light_power.push(light.power().y());
    }
    Some(Arc::new(Distribution1D::new(&light_power)))
}