use crate::core::pbrt::{Float, lerp, quadratic, consts::PI};
use crate::core::geometry::{Vector3f, Point3f, Point2f, Bounds2f, Bounding2, Ray, Normal3f, Bounds2i, Point2i};
use crate::core::camera::{Camera, CameraSample};
use crate::core::transform::Transform;
use crate::core::animated_transform::AnimatedTransform;
use crate::core::film::Film;
use crate::core::medium::Medium;
use crate::core::profiler::Profiler;
use crate::core::stats_accumulator::StatsAccumulator;
use crate::core::sampling::concentric_sample_disk;
use crate::core::reflection::refract;
use crate::core::low_discrepancy::radical_inverse;
use crate::core::image_io::write_image;
use crate::core::rng::Rng;
use std::sync::Arc;
use rayon::prelude::*;

#[derive(Debug, Clone)]
pub struct RealisticCamera {
    // camera common
    pub camera_to_world: AnimatedTransform,
    pub shutter_open: Float,
    pub shutter_close: Float,
    pub film: Arc<Film>,
    pub medium: Option<Arc<dyn Medium>>,
    // RealisticCamera Private Data
    simple_weighting: bool,
    element_interfaces: Vec<LensElementInterface>,
    exit_pupil_bounds: Vec<Bounds2f>
}

#[derive(Debug, Default, Clone, Copy)]
pub struct LensElementInterface {
    pub curvature_radius: Float,
    pub thickness: Float,
    pub eta: Float,
    pub aperture_radius: Float
}

impl RealisticCamera {
    pub fn new(
        camera_to_world: AnimatedTransform,
        shutter_open: Float,
        shutter_close: Float,
        aperture_diameter: Float,
        focus_distance: Float,
        simple_weighting: bool,
        lens_data: &Vec<Float>,
        film: Arc<Film>,
        medium: Option<Arc<dyn Medium>>
    ) -> RealisticCamera {
        let element_interfaces: Vec<LensElementInterface> = Vec::new();
        for i in (0..lens_data.len()).step_by(4) {
            let diameter: Float = lens_data[i + 3];
            if lens_data[i] == 0.0 {
                if aperture_diameter > diameter {
                    warn!("Specified aperture diameter {} is greater than maximum possible {}. Clamping it.",
                        aperture_diameter, diameter);
                } else {
                    diameter = aperture_diameter;
                }
            }
            element_interfaces.push(LensElementInterface {
                curvature_radius: lens_data[i] * 0.001,
                thickness: lens_data[i + 1] * 0.001,
                eta: lens_data[i + 2],
                aperture_radius: diameter * 0.001 / 2.0
            });
        }

        let mut camera = RealisticCamera {
            camera_to_world,
            shutter_open,
            shutter_close,
            film,
            medium,
            simple_weighting,
            element_interfaces,
            exit_pupil_bounds: Vec::new()
        };
        // Compute lens--film distance for given focus distance
        let fb = camera.focus_binary_search(focus_distance);
        info!("Binary search focus: {} -> {}", fb, camera.focus_distance(fb));
        let last = element_interfaces.last_mut().unwrap();
        last.thickness = camera.focus_thick_lens(focus_distance);
        info!("Thick lens focus: {} -> {}",
            last.thickness, camera.focus_distance(last.thickness));

        // Compute exit pupil bounds at sampled points on the film
        let n_samples = 64_usize;
        camera.exit_pupil_bounds = (0..n_samples).into_par_iter().map(|i| {
            let r0 = i as Float / n_samples as Float * film.diagonal / 2.0;
            let r1 = (i + 1) as Float / n_samples as Float * film.diagonal / 2.0;
            camera.bound_exit_pupil(r0, r1)
        }).collect();

        if simple_weighting {
            warn!("\"simpleweighting\" option with RealisticCamera no longer necessarily matches regular camera images. Further, pixel values will vary a bit depending on the aperture size. See this discussion for details: https://github.com/mmp/pbrt-v3/issues/162#issuecomment-348625837");
        }
        camera
    }

    fn lens_rear_z(&self) -> Float {
        self.element_interfaces.last().unwrap().thickness
    }

    fn lens_front_z(&self) -> Float {
        self.element_interfaces.iter().fold(0.0, |acc, e| acc + e.thickness)
    }

    fn rear_element_radius(&self) -> Float {
        self.element_interfaces.last().unwrap().aperture_radius
    }

    fn trace_lenses_from_film(&self, r_camera: &Ray, r_out: Option<&mut Ray>) -> bool {
        let mut element_z: Float = 0.0;
        // Transform _rCamera_ from camera to lens system space
        static camera_to_lens: Transform = Transform::scale(1.0, 1.0, -1.0);
        let r_lens = camera_to_lens.transform_ray(r_camera);
        for i in (0..self.element_interfaces.len()).rev() {
            let element = self.element_interfaces[i];
            // Update ray from film accounting for interaction with _element_
            element_z -= element.thickness;

            // Compute intersection of ray with lens element
            let mut t: Float = 0.0;
            let mut n = Normal3f::default();
            let is_stop = element.curvature_radius == 0.0;
            if is_stop {
                // The refracted ray computed in the previous lens element
                // interface may be pointed towards film plane(+z) in some
                // extreme situations; in such cases, 't' becomes negative.
                if r_lens.d.z >= 0.0 {
                    return false;
                }
                t = (element_z - r_lens.o.z) / r_lens.d.z;
            } else {
                let radius = element.curvature_radius;
                let z_center = element_z + element.curvature_radius;
                if !self.intersect_spherical_element(radius, z_center, &r_lens, &mut t, &mut n) {
                    return false;
                }
            }
            assert!(t >= 0.0);

            // Test intersection point against element aperture
            let p_hit = r_lens.point_at_time(t);
            let r2 = p_hit.x * p_hit.x + p_hit.y * p_hit.y;
            if r2 > element.aperture_radius * element.aperture_radius {
                return false;
            }
            r_lens.o = p_hit;

            // Update ray path for element interface interaction
            if !is_stop {
                let mut w = Vector3f::default();
                let eta_i = element.eta;
                let eta_t = if i > 0 && self.element_interfaces[i - 1].eta != 0.0 {
                    self.element_interfaces[i - 1].eta
                } else { 1.0 };
                if !refract(&(-r_lens.d).normalize(), &n, eta_i / eta_t, &mut w) {
                    return false;
                }
                r_lens.d = w;
            }
        }
        // Transform _rLens_ from lens system space back to camera space
        if let Some(r_out) = r_out {
            static lens_to_camera: Transform = Transform::scale(1.0, 1.0, -1.0);
            *r_out = lens_to_camera.transform_ray(&r_lens);
        }
        true
    }

    fn intersect_spherical_element(
        &self,
        radius: Float,
        z_center: Float,
        ray: &Ray,
        t: &mut Float,
        n: &mut Normal3f
    ) -> bool {
        // Compute _t0_ and _t1_ for ray--element intersection
        let o = ray.o - Vector3f::new(0.0, 0.0, z_center);
        let a = ray.d.x * ray.d.x + ray.d.y * ray.d.y + ray.d.z * ray.d.z;
        let b = 2.0 * (ray.d.x * o.x + ray.d.y * o.y + ray.d.z * o.z);
        let c = o.x * o.x + o.y * o.y + o.z * o.z - radius * radius;
        let mut t0: Float = 0.0;
        let mut t1: Float = 0.0;
        if !quadratic(a, b, c, &mut t0, &mut t1) {
            return false;
        }

        // Select intersection $t$ based on ray direction and element curvature
        let ip = o + *t * ray.d;
        *n = Normal3f::new(ip.x, ip.y, ip.z).normalize()
            .face_forward(&Normal3f::new(-ray.d.x, -ray.d.y, -ray.d.z));
        true
    }

    fn trace_lense_from_scene(
        &self,
        r_camera: &Ray,
        r_out: Option<&mut Ray>
    ) -> bool {
        let mut element_z = -self.lens_front_z();
        // Transform _rCamera_ from camera to lens system space
        static camera_to_lens: Transform = Transform::scale(1.0, 1.0, -1.0);
        let mut r_lens = camera_to_lens.transform_ray(r_camera);
        for i in 0..self.element_interfaces.len() {
            let element = self.element_interfaces[i];
            // Compute intersection of ray with lens element
            let mut t: Float = 0.0;
            let mut n = Normal3f::default();
            let is_stop = element.curvature_radius == 0.0;
            if is_stop {
                t = (element_z - r_lens.o.z) / r_lens.d.z;
            } else {
                let radius = element.curvature_radius;
                let z_center = element_z + element.curvature_radius;
                if !self.intersect_spherical_element(radius, z_center, &r_lens, &mut t, &mut n) {
                    return false;
                }
            }
            assert!(t >= 0.0);

            // Test intersection point against element aperture
            let p_hit = r_lens.point_at_time(t);
            let r2 = p_hit.x * p_hit.x + p_hit.y * p_hit.y;
            if r2 > element.aperture_radius * element.aperture_radius {
                return false;
            }
            r_lens.o = p_hit;
        
            // Update ray path for element interface interaction
            if !is_stop {
                let mut w = Vector3f::default();
                let eta_i = if i == 0 || self.element_interfaces[i - 1].eta == 0.0
                    { 1.0 } else { self.element_interfaces[i - 1].eta } ;
                let eta_t = if element.eta != 0.0 { element.eta } else { 1.0 };
                if !refract(&(-r_lens.d).normalize(), &n, eta_i / eta_t, &mut w) {
                    return false;
                }
                r_lens.d = w;
            }
            element_z += element.thickness;
        }
        // Transform _rLens_ from lens system space back to camera space
        if let Some(r_out) = r_out {
            static lens_to_camera: Transform = Transform::scale(1.0, 1.0, -1.0);
            *r_out = lens_to_camera.transform_ray(&r_lens);
        }
        true
    }

    fn draw_lens_system(&self) {
        let sumz = -self.lens_front_z();
        let mut z = sumz;
        for i in 0..self.element_interfaces.len() {
            let element = self.element_interfaces[i];
            let r = element.curvature_radius;
            if r == 0.0 {
                // stop
                print!("{{Thick, Line[{{{{{}, {}}}, {{{}, {}}}}}], Line[{{{{{}, {}}}, {{{}, {}}}}}]}}, ",
                    z, element.aperture_radius, z, 2.0 * element.aperture_radius,
                    z, -element.aperture_radius, z, -2.0 * element.aperture_radius);
            } else {
                let theta = (element.aperture_radius / r).asin().abs();
                if r > 0.0 {
                    // convex as seen from front of lens
                    let t0 = PI - theta;
                    let t1 = PI + theta;
                    print!("Circle[{{{}, 0}}, {}, {{{}, {}}}], ", z + r, r, t0, t1);
                } else {
                    // concave as seen from front of lens
                    let t0 = -theta;
                    let t1 = theta;
                    print!("Circle[{{{}, 0}}, {}, {{{}, {}}}], ", z + r, -r, t0, t1);
                }
                if element.eta != 0.0 && element.eta != 1.0 {
                    // connect top/bottom to next element
                    assert!(i + 1 < self.element_interfaces.len());
                    let next_aperture_radius = self.element_interfaces[i + 1].aperture_radius;
                    let h = element.aperture_radius.max(next_aperture_radius);
                    let hlow = element.aperture_radius.min(next_aperture_radius);

                    let zp0 = if r > 0.0 {
                        z + element.curvature_radius - element.aperture_radius / theta.tan()
                    } else {
                        z + element.curvature_radius + element.aperture_radius / theta.tan()
                    };

                    let next_curvature_radius = self.element_interfaces[i + 1].curvature_radius;
                    let next_theta = (next_aperture_radius / next_curvature_radius).asin().abs();
                    let zp1 = if next_curvature_radius > 0.0 {
                        z + element.thickness + next_curvature_radius - next_aperture_radius / next_theta.tan()
                    } else {
                        z + element.thickness + next_curvature_radius + next_aperture_radius / next_theta.tan()
                    };

                    // Connect tops
                    print!("Line[{{{{{}, {}}}, {{{}, {}}}}}], ", zp0, h, zp1, h);
                    print!("Line[{{{{{}, {}}}, {{{}, {}}}}}], ", zp0, -h, zp1, -h);

                    // vertical lines when needed to close up the element profile
                    if element.aperture_radius < next_aperture_radius {
                        print!("Line[{{{{{}, {}}}, {{{}, {}}}}}], ", zp0, h, zp0, hlow);
                        print!("Line[{{{{{}, {}}}, {{{}, {}}}}}], ", zp0, -h, zp0, -hlow);
                    } else {
                        print!("Line[{{{{{}, {}}}, {{{}, {}}}}}], ", zp1, h, zp1, hlow);
                        print!("Line[{{{{{}, {}}}, {{{}, {}}}}}], ", zp1, -h, zp1, -hlow);
                    }
                }
            }
            z += element.thickness;
        }
        
        // 24mm height for 35mm film
        print!("Line[{{{{0, -.012}}, {{0, .012}}}}], ");
        // optical axis
        print!("Line[{{{{0, 0}}, {{{}, 0}}}}] ", 1.2 * sumz);
    }

    fn draw_ray_path_from_film(
        &self,
        r: &Ray,
        arrow: bool,
        to_optical_intercept: bool
    ) {
        let mut element_z: Float = 0.0;
        // Transform _ray_ from camera to lens system space
        static camera_to_lens: Transform = Transform::scale(1.0, 1.0, -1.0);
        let mut ray = camera_to_lens.transform_ray(r);
        print!("{{ ");
        if !self.trace_lenses_from_film(r, None) {
            print!("Dashed, ");
        }
        for i in (0..self.element_interfaces.len()).rev() {
            let element = self.element_interfaces[i];
            element_z -= element.thickness;
            let is_stop = element.curvature_radius == 0.0;
            // Compute intersection of ray with lens element
            let mut t: Float = 0.0;
            let mut n = Normal3f::default();
            if is_stop {
                t = -(ray.o.z - element_z) / ray.d.z;
            } else {
                let radius = element.curvature_radius;
                let z_center = element_z + element.curvature_radius;
                if !self.intersect_spherical_element(radius, z_center, &ray, &mut t, &mut n) {
                    print!("}}");
                    return;
                }
            }
            assert!(t >= 0.0);

            print!("Line[{{{{{}, {}}}, {{{}, {}}}}}], ", ray.o.z, ray.o.z,
                ray.point_at_time(t).z,ray.point_at_time(t).x );
            
            // Test intersection point against element aperture
            let p_hit = ray.point_at_time(t);
            let r2 = p_hit.x * p_hit.x + p_hit.y * p_hit.y;
            if r2 > element.aperture_radius * element.aperture_radius {
                print!("}}");
                return;
            }
            ray.o = p_hit;

            // Update ray path for element interface interaction
            if !is_stop {
                let mut w = Vector3f::default();
                let eta_i = element.eta;
                let eta_t = if i > 0 && self.element_interfaces[i - 1].eta != 0.0 {
                    self.element_interfaces[i - 1].eta
                } else { 1.0 };
                if !refract(&(-ray.d).normalize(), &n, eta_i / eta_t, &mut w) {
                    print!("}}");
                    return;
                }
                ray.d = w;
            }
        }

        ray.d = ray.d.normalize();
        let mut ta = (element_z / 4.0).abs();
        if to_optical_intercept {
            ta = -ray.o.x / ray.d.x;
            print!("Point[{{{}, {}}}], ", ray.point_at_time(ta).z, ray.point_at_time(ta).x);
        }
        print!("{}[{{{{{}, {}}}, {{{}, {}}}}}]", if arrow { "Arrow" } else { "Line" },
            ray.o.z, ray.o.x, ray.point_at_time(ta).z, ray.point_at_time(ta).x);

        // overdraw the optical axis if needed ...
        if to_optical_intercept {
            print!(", Line[{{{{{}, 0}}, {{{}, 0}}}}]", ray.o.z, ray.point_at_time(ta).z * 1.05);
        }
        print!("}}");
    }

    fn draw_ray_path_from_scene(
        &self,
        r: &Ray,
        arrow: bool,
        to_optical_intercept: bool
    ) {
        let mut element_z = -self.lens_front_z();

        // Transform _rCamera_ from camera to lens system space
        static camera_to_lens: Transform = Transform::scale(1.0, 1.0, -1.0);
        let mut ray = camera_to_lens.transform_ray(r);
        for i in 0..self.element_interfaces.len() {
            let element = self.element_interfaces[i];
            // Compute intersection of ray with lens element
            let mut t: Float = 0.0;
            let mut n = Normal3f::default();
            let is_stop = element.curvature_radius == 0.0;
            if is_stop {
                t = -(ray.o.z - element_z) / ray.d.z;
            } else {
                let radius = element.curvature_radius;
                let z_center = element_z + element.curvature_radius;
                if !self.intersect_spherical_element(radius, z_center, &ray, &mut t, &mut n) {
                    return;
                }
            }
            assert!(t >= 0.0);

            print!("Line[{{{{{}, {}}}, {{{}, {}}}}}], ", ray.o.z, ray.o.x,
                ray.point_at_time(t).z, ray.point_at_time(t).x);

            // Test intersection point against element aperture
            let p_hit = ray.point_at_time(t);
            let r2 = p_hit.x * p_hit.x + p_hit.y * p_hit.y;
            if r2 > element.aperture_radius * element.aperture_radius {
                return;
            }
            ray.o = p_hit;
        
            // Update ray path for element interface interaction
            if !is_stop {
                let mut w = Vector3f::default();
                let eta_i = if i == 0 || self.element_interfaces[i - 1].eta == 0.0
                    { 1.0 } else { self.element_interfaces[i - 1].eta } ;
                let eta_t = if element.eta != 0.0 { element.eta } else { 1.0 };
                if !refract(&(-ray.d).normalize(), &n, eta_i / eta_t, &mut w) {
                    return;
                }
                ray.d = w;
            }
            element_z += element.thickness;
        }

        let mut ta = -ray.o.z / ray.d.z;
        if to_optical_intercept {
            ta = -ray.o.x / ray.d.x;
            print!("Point[{{{}, {}}}], ", ray.point_at_time(ta).z, ray.point_at_time(ta).x);
        }
        print!("{}[{{{{{}, {}}}, {{{}, {}}}}}]", if arrow { "Arrow" } else { "Line" },
            ray.o.z, ray.o.x, ray.point_at_time(ta).z, ray.point_at_time(ta).x);
    }

    fn compute_cardinal_points (
        r_in: &Ray,
        r_out: &Ray,
        pz: &mut Float,
        fz: &mut Float
    ) {
        let tf = -r_out.o.x / r_out.d.x;
        *fz = -r_out.point_at_time(tf).z;
        let tp = (r_in.o.x - r_out.o.x) / r_out.d.x;
        *pz = -r_out.point_at_time(tp).z;
    }

    fn compute_thick_lens_approximation(
        &self,
        pz: &mut [Float; 2],
        fz: &mut [Float; 2]
    ) {
        // Find height $x$ from optical axis for parallel rays
        let x = 0.001 * self.film.diagonal;

        // Compute cardinal points for film side of lens system
        let mut r_scene = Ray::new(Point3f::new(x, 0.0, self.lens_front_z() + 1.0),
            Vector3f::new(0.0, 0.0, -1.0));
        let mut r_film = Ray::default();
        assert!(self.trace_lense_from_scene(&r_scene, Some(&mut r_film)),
            "Unable to trace ray from scene to film for thick lens approximation. Is aperture stop extremely small?");
        RealisticCamera::compute_cardinal_points(&r_scene, &r_film, &mut pz[0], &mut fz[0]);

        // Compute cardinal points for scene side of lens system
        r_film = Ray::new(Point3f::new(x, 0.0, self.lens_rear_z() - 1.0),
            Vector3f::new(0.0, 0.0, 1.0));
        assert!(self.trace_lenses_from_film(&r_film, Some(&mut r_scene)),
            "Unable to trace ray from film to scene for thick lens approximation. Is aperture stop extremely small?");
        RealisticCamera::compute_cardinal_points(&r_film, &r_scene, &mut pz[1], &mut fz[1]);
    }

    fn focus_thick_lens(&self, focus_distance: Float) -> Float {
        let mut pz: [Float; 2] = [0.0; 2];
        let mut fz: [Float; 2] = [0.0; 2];
        self.compute_thick_lens_approximation(&mut pz, &mut fz);
        info!("Cardinal points: p' = {} f' = {}, p = {} f = {}.",
            pz[0], fz[0], pz[1], fz[1]);
        info!("Effective focal length {}", fz[0] - pz[0]);
        // Compute translation of lens, _delta_, to focus at _focusDistance_
        let f = fz[0] - pz[0];
        let z = -focus_distance;
        let c = (pz[1] - z - pz[0]) * (pz[1] - z - 4.0 * f - pz[0]);
        assert!(c > 0.0, "Coefficient must be positive. It looks focusDistance: {} is too short for a given lenses configuration");
        let delta = 0.5 * (pz[1] - z + pz[0] - c.sqrt());
        self.element_interfaces.last().unwrap().thickness + delta
    }

    fn focus_binary_search(&self, focus_distance: Float) -> Float {
        // Find _filmDistanceLower_, _filmDistanceUpper_ that bound focus distance
        let mut film_distance_lower = self.focus_thick_lens(focus_distance);
        let mut film_distance_upper = film_distance_lower;
        while self.focus_distance(film_distance_lower) > focus_distance {
            film_distance_lower *= 1.005;
        }
        while self.focus_distance(film_distance_upper) < focus_distance {
            film_distance_upper /= 1.005;
        }

        // Do binary search on film distances to focus
        for i in 0..20 {
            let fmid = 0.5 * (film_distance_lower + film_distance_upper);
            let mid_focus = self.focus_distance(fmid);
            if mid_focus < focus_distance {
                film_distance_lower = fmid;
            } else {
                film_distance_upper = fmid;
            }
        }
        0.5 * (film_distance_lower + film_distance_upper)
    }

    fn focus_distance(&self, film_distance: Float) -> Float {
        // Find offset ray from film center through lens
        let bounds = self.bound_exit_pupil(0.0, 0.001 * self.film.diagonal);

        let scale_factors: [Float; 3] = [0.1, 0.01, 0.001];
        let mut lu: Float = 0.0;
        let mut ray = Ray::default();

        // Try some different and decreasing scaling factor to find focus ray
        // more quickly when `aperturediameter` is too small.
        // (e.g. 2 [mm] for `aperturediameter` with wide.22mm.dat),
        let mut found_focus_ray = false;
        for scale in scale_factors.iter() {
            lu = scale * bounds.max.x;
            if self.trace_lenses_from_film(
                &Ray::new(Point3f::new(0.0, 0.0, self.lens_rear_z()),
                    Vector3f::new(lu, 0.0, film_distance)),
                Some(&mut ray)) {
                found_focus_ray = true;
                break;
            }
        }

        if !found_focus_ray {
            error!("Focus ray at lens pos({},0) didn't make it through the lenses with film distance {}?!??", lu, film_distance);
            return num::Float::infinity();
        }

        // Compute distance _zFocus_ where ray intersects the principal axis
        let t_focus = -ray.o.x / ray.d.x;
        let mut z_focus = ray.point_at_time(t_focus).z;
        if z_focus < 0.0 { num::Float::infinity() } else { z_focus }
    }

    fn bound_exit_pupil(
        &self,
        p_film_x0: Float,
        p_film_x1: Float
    ) -> Bounds2f {
        let mut pupil_bounds = Bounds2f::default();
        // Sample a collection of points on the rear lens to find exit pupil
        const n_samples: usize = 1024 * 1024;
        let mut n_exiting_rays = 0_u32;

        // Compute bounding box of projection of rear element on sampling plane
        let rear_radius = self.rear_element_radius();
        let proj_rear_bounds = Bounds2f::new(Point2f::new(-1.5 * rear_radius, -1.5 * rear_radius),
            Point2f::new(1.5 * rear_radius, 1.5 * rear_radius));
        for i in 0..n_samples {
            // Find location of sample points on $x$ segment and rear lens element
            let p_film = Point3f::new(lerp((i as Float + 0.5) / n_samples as Float, p_film_x0, p_film_x1), 0.0, 0.0);
            let u: [Float; 2] = [radical_inverse(0, i as u64), radical_inverse(1, i as u64)];
            let p_rear = Point3f::new(lerp(u[0], proj_rear_bounds.min.x, proj_rear_bounds.max.x),
                lerp(u[1], proj_rear_bounds.min.y, proj_rear_bounds.max.y),
                self.lens_rear_z());
            
            // Expand pupil bounds if ray makes it through the lens system
            if pupil_bounds.inside(&Point2f::new(p_rear.x, p_rear.y)) ||
                self.trace_lenses_from_film(&Ray::new(p_film, p_rear - p_film), None) {
                pupil_bounds = pupil_bounds.union(&Bounds2f::from_point(Point2f::new(p_rear.x, p_rear.y)));
                n_exiting_rays += 1;
            }
        }

        // Return entire element bounds if no rays made it through the lens system
        if n_exiting_rays == 0 {
            info!("Unable to find exit pupil in x = [{},{}] on film.", p_film_x0, p_film_x1);
            return proj_rear_bounds;
        }

        // Expand bounds to account for sample spacing
        pupil_bounds.expand(2.0 * proj_rear_bounds.diagonal().length() /
            (n_samples as Float).sqrt())
    }

    fn render_exit_pupil(
        &self,
        sx: Float,
        sy: Float,
        filename: &str
    ) {
        let p_film = Point3f::new(sx, sy, 0.0);
        let rear_element_radius = self.rear_element_radius();

        const n_samples: usize = 2048;
        let mut image: Vec<Float> = vec![0.0 as Float; 3 * n_samples * n_samples];

        for y in 0..n_samples {
            let fy = y as Float / (n_samples - 1) as Float;
            let ly = lerp(fy, -rear_element_radius, rear_element_radius);
            for x in 0..n_samples {
                let offset = (x + y * n_samples) * 3;
                let fx = x as Float / (n_samples - 1) as Float;
                let lx = lerp(fx, -rear_element_radius, rear_element_radius);

                let p_rear = Point3f::new(lx, ly, self.lens_rear_z());

                if lx * lx + ly * ly > rear_element_radius * rear_element_radius {
                    image[offset] = 1.0;
                    image[offset + 1] = 1.0;
                    image[offset + 2] = 1.0;
                } else if self.trace_lenses_from_film(&Ray::new(p_film, p_rear - p_film), None) {
                    image[offset] = 0.5;
                    image[offset + 1] = 0.5;
                    image[offset + 2] = 0.5;
                }
                else {
                    image[offset] = 0.0;
                    image[offset + 1] = 0.0;
                    image[offset + 2] = 0.0;
                }
            }
        }

        write_image(filename, image,
            Bounds2i::new(Point2i::default(), Point2i::new(n_samples as i32, n_samples as i32)),
            Point2i::new(n_samples as i32, n_samples as i32));
    }

    fn sample_exit_pupil(
        &self,
        p_film: &Point2f,
        lens_sample: &Point2f,
        sample_bounds_area: &mut Float
    ) -> Point3f {
        // Find exit pupil bound for sample distance from film center
        let r_film = (p_film.x * p_film.x + p_film.y * p_film.y).sqrt();
        let mut r_index = (r_film / (self.film.diagonal() / 2.0) * self.exit_pupil_bounds.len() as Float) as usize;
        r_index = r_index.min(self.exit_pupil_bounds.len() - 1);
        let pupil_bounds = self.exit_pupil_bounds[r_index];
        *sample_bounds_area = pupil_bounds.area();

        // Generate sample point inside exit pupil bound
        let p_lens = pupil_bounds.lerp(lens_sample);

        // Return sample point rotated by angle of _pFilm_ with $+x$ axis
        let sin_theta = if r_film != 0.0 { p_film.y / r_film } else { 0.0 };
        let cos_theta = if r_film != 0.0 { p_film.x / r_film } else { 1.0 };
        Point3f::new(cos_theta * p_lens.x - sin_theta * p_lens.y,
            sin_theta * p_lens.x + cos_theta * p_lens.y, self.lens_rear_z())
    }

    fn test_exit_pupil_bounds(&self) {
        let film_diagonal = self.film.diagonal;

        static rng: Rng = Rng::default();

        let u = rng.uniform_float();
        let p_film = Point3f::new(u * film_diagonal / 2.0, 0.0, 0.0);

        let r = p_film.x / (film_diagonal / 2.0);
        let pupil_index = ((r * (self.exit_pupil_bounds.len() - 1) as Float).floor() as usize)
            .min(self.exit_pupil_bounds.len() - 1);
        let mut pupil_bounds = self.exit_pupil_bounds[pupil_index];
        if pupil_index + 1 < self.exit_pupil_bounds.len() {
            pupil_bounds = pupil_bounds.union(&self.exit_pupil_bounds[pupil_index + 1]);
        }

        // Now, randomly pick points on the aperture and see if any are outside
        // of pupil bounds...
        for i in 0..1000 {
            let u2 = Point2f::new(rng.uniform_float(), rng.uniform_float());
            let mut pd = concentric_sample_disk(&u2);
            pd *= self.rear_element_radius();

            let test_ray = Ray::new(p_film, Point3f::new(pd.x, pd.y, 0.0) - p_film);
            let mut test_out = Ray::default();
            if ! self.trace_lenses_from_film(&test_ray, Some(&mut test_out)) {
                continue;
            }

            if !pupil_bounds.inside(&pd) {
                eprintln!("Aha! ({},{}) went through, but outside bounds ({},{}) - ({},{})",
                    pd.x, pd.y, pupil_bounds.min.x, pupil_bounds.min.y,
                    pupil_bounds.max.x, pupil_bounds.max.y);
                self.render_exit_pupil(pupil_index as Float / self.exit_pupil_bounds.len() as Float *
                    film_diagonal / 2.0, 0.0, "low.err");
                self.render_exit_pupil((pupil_index + 1) as Float / self.exit_pupil_bounds.len() as Float *
                    film_diagonal / 2.0, 0.0, "high.err");
                self.render_exit_pupil(p_film.x, 0.0, "mid.err");
                std::process::exit(0);
            }
        }
        eprint!(".");
    }
}

impl Camera for RealisticCamera {
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

    fn get_medium(&self) -> Option<Arc<dyn Medium>> {
        self.medium
    }

    fn generate_ray(&self, sample: &CameraSample, ray: &mut Ray) -> Float {
        let _guard = Profiler::instance().profile("Camera::generate_ray()");
        // Find point on film, _pFilm_, corresponding to _sample.pFilm_
        let s  = Point2f::new(sample.p_film.x / self.film.full_resolution.x,
            sample.p_film.y / self.film.full_resolution.y);
        let p_film2 = self.film.get_physical_etent().lerp(s);
        let p_film = Point3f::new(-p_film2.x, p_film2.y, 0.0);

        // Trace ray from _pFilm_ through lens system
        let mut exit_pupil_bounds_area: Float = 0.0;
        let p_rear = self.sample_exit_pupil(&Point2f::new(p_film.x, p_film.y),
            &sample.p_lens, &mut exit_pupil_bounds_area);
        let mut r_film = Ray::new(p_film, p_rear - p_film);
        r_film.time = lerp(sample.time, self.shutter_open, self.shutter_close);
        if !self.trace_lenses_from_film(&r_film, Some(ray)) {
            StatsAccumulator::instance().report_percentage(String::from("Camera/Rays vignetted by lens system"), 1, 1);
            return 0.0;
        }

        // Finish initialization of _RealisticCamera_ ray
        *ray = self.camera_to_world.transform_ray(&ray);
        ray.d = ray.d.normalize();
        ray.medium = self.medium;

        // Return weighting for _RealisticCamera_ ray
        let cos_theta = r_film.d.normalize().z;
        let cos4_theta = cos_theta * cos_theta * cos_theta * cos_theta;
        if self.simple_weighting {
            cos4_theta * exit_pupil_bounds_area / self.exit_pupil_bounds[0].area()
        } else {
            (self.shutter_close - self.shutter_open) *
                (cos4_theta * exit_pupil_bounds_area) / (self.lens_rear_z() * self.lens_rear_z())
        }
    }
}