use crate::core::pbrt::{Float, lerp, consts::PI, Spectrum};
use crate::core::geometry::{Vector3f, Point2f, Point3f, Bounds2f, Ray, RayDifferential, Normal3f, dot_normal_vec};
use crate::core::camera::{Camera, CameraSample};
use crate::core::transform::Transform;
use crate::core::animated_transform::AnimatedTransform;
use crate::core::film::Film;
use crate::core::medium::{Medium, MediumInterface};
use crate::core::interaction::{Interaction, SimpleInteraction};
use crate::core::light::VisibilityTester;
use crate::core::profiler::Profiler;
use crate::core::sampling::concentric_sample_disk;
use std::sync::Arc;

pub struct PerspectiveCamera {
    // camera common
    pub camera_to_world: AnimatedTransform,
    pub shutter_open: Float,
    pub shutter_close: Float,
    pub film: Arc<Film>,
    pub medium: Option<Arc<dyn Medium + Send + Sync>>,
    // projective camera common
    camera_to_screen: Transform,
    raster_to_camera: Transform,
    screen_to_raster: Transform,
    raster_to_screen: Transform,
    lens_radius: Float,
    focal_distance: Float,
    // OrthographicCamera private
    dx_camera: Vector3f,
    dy_camera: Vector3f,
    a: Float
}

impl PerspectiveCamera {
    pub fn new(
        camera_to_world: AnimatedTransform,
        screen_window: Bounds2f,
        shutter_open: Float,
        shutter_close: Float,
        lens_radius: Float,
        focal_distance: Float,
        fov: Float,
        film: Arc<Film>,
        medium: Option<Arc<dyn Medium + Send + Sync>>
    ) -> PerspectiveCamera {
        // Compute projective camera screen transformations
        let screen_to_raster = Transform::scale(film.full_resolution.x as Float, film.full_resolution.y as Float, 1.0) *
            Transform::scale(1.0 / (screen_window.max.x - screen_window.min.x),
                1.0 / (screen_window.min.y - screen_window.max.y), 1.0) *
            Transform::translate(&Vector3f::new(-screen_window.min.x, -screen_window.max.y, 0.0));
        let raster_to_screen = screen_to_raster.inverse();
        let camera_to_screen = Transform::perspective(fov, 1e-2, 1000.0);
        let raster_to_camera = camera_to_screen.inverse() * raster_to_screen;

        // Compute image plane bounds at $z=1$ for _PerspectiveCamera_
        let res = film.full_resolution;
        let mut min = raster_to_camera.transform_point(&Point3f::default());
        let mut max = raster_to_camera.transform_point(&Point3f::new(res.x as Float, res.y as Float, 0.0));
        min /= min.z;
        max /= max.z;
        let a = ((max.x - min.x) * (max.y - min.y)).abs();

        PerspectiveCamera {
            camera_to_world,
            shutter_open,
            shutter_close,
            film,
            medium,
            camera_to_screen,
            raster_to_camera,
            screen_to_raster,
            raster_to_screen,
            lens_radius,
            focal_distance,
            dx_camera: raster_to_camera.transform_vector(&Vector3f::new(1.0, 0.0, 0.0)),
            dy_camera: raster_to_camera.transform_vector(&Vector3f::new(0.0, 1.0, 0.0)),
            a
        }
    }
}

impl Camera for PerspectiveCamera {
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
        // Compute raster and camera sample positions
        let p_film = Point3f::new(sample.p_film.x, sample.p_film.y, 0.0);
        let p_camera = self.raster_to_camera.transform_point(&p_film);
        *ray = Ray::new(Point3f::default(), Vector3f::from(p_camera).normalize());
        // Modify ray for depth of field
        if self.lens_radius > 0.0 {
            // Sample point on lens
            let p_lens = self.lens_radius * concentric_sample_disk(&sample.p_lens);

            // Compute point on plane of focus
            let ft = self.focal_distance / ray.d.z;
            let p_focus = ray.point_at_time(ft);

            // Update ray for effect of lens
            ray.o = Point3f::new(p_lens.x, p_lens.y, 0.0);
            ray.d = (p_focus - ray.o).normalize();
        }
        ray.time = lerp(sample.time, self.shutter_open, self.shutter_close);
        ray.medium = self.medium;
        *ray = self.camera_to_world.transform_ray(ray);
        1.0
    }

    fn generate_ray_differential(&self, sample: &CameraSample, ray: &mut Ray) -> Float {
        let _guard = Profiler::instance().profile("Camera::generate_ray_differential()");
        // Compute main perspective viewing ray

        // Compute raster and camera sample positions
        let p_film = Point3f::new(sample.p_film.x, sample.p_film.y, 0.0);
        let p_camera = self.raster_to_camera.transform_point(&p_film);
        *ray = Ray::new(p_camera, Vector3f::new(0.0, 0.0, 1.0));

        // Modify ray for depth of field
        if self.lens_radius > 0.0 {
            // Sample point on lens
            let p_lens = self.lens_radius * concentric_sample_disk(&sample.p_lens);

            // Compute point on plane of focus
            let ft = self.focal_distance / ray.d.z;
            let p_focus = ray.point_at_time(ft);

            // Update ray for effect of lens
            ray.o = Point3f::new(p_lens.x, p_lens.y, 0.0);
            ray.d = (p_focus - ray.o).normalize();
        }

        // Compute ray differentials for _OrthographicCamera_
        let mut rd: RayDifferential = RayDifferential::default();
        if self.lens_radius > 0.0 {
            // Compute _PerspectiveCamera_ ray differentials accounting for lens

            // Sample point on lens
            let p_lens = self.lens_radius * concentric_sample_disk(&sample.p_lens);
            let dx = Vector3f::from(p_camera + self.dx_camera).normalize();
            let ft = self.focal_distance / ray.d.z;
            let p_focus = Point3f::default() + (ft * dx);
            rd.rx_origin = Point3f::new(p_lens.x, p_lens.y, 0.0);
            rd.rx_direction = (p_focus - rd.rx_origin).normalize();

            let dy = Vector3f::from(p_camera + self.dy_camera).normalize();
            p_focus = Point3f::default() + (ft * dy);
            rd.ry_origin = Point3f::new(p_lens.x, p_lens.y, 0.0);
            rd.ry_direction = (p_focus - rd.ry_origin).normalize();
        } else {
            rd.rx_origin = ray.o;
            rd.ry_origin = ray.o;
            rd.rx_direction = Vector3f::from(p_camera) + self.dx_camera;
            rd.ry_direction = Vector3f::from(p_camera) + self.dy_camera;
        }
        ray.differential = Some(rd);
        ray.time = lerp(sample.time, self.shutter_open, self.shutter_close);
        ray.medium = self.medium;
        *ray = self.camera_to_world.transform_ray(ray);
        1.0
    }

    fn we(&self, ray: &Ray, p_raster2: Option<&mut Point2f>) -> Spectrum {
        // Interpolate camera matrix and check if $\w{}$ is forward-facing
        let c2w = Transform::default();
        self.camera_to_world.interpolate(ray.time, &mut c2w);
        let cos_theta = ray.d.dot(&c2w.transform_vector(&Vector3f::new(0.0, 0.0, 0.1)));
        if cos_theta <= 0.0 {
            return Spectrum::new(0.0);
        }

        // Map ray $(\p{}, \w{})$ onto the raster grid
        let p_focus = ray.point_at_time(if self.lens_radius > 0.0 { self.focal_distance } else { 1.0 });
        let p_raster = self.raster_to_camera.inverse()
            .transform_point(&c2w.inverse().transform_point(&p_focus));
        
        // Return raster position if requested
        if let Some(p) = p_raster2 {
            *p = Point2f::new(p_raster.x, p_raster.y);
        }

        // Return zero importance for out of bounds points
        let sample_bounds = self.film.get_sample_bounds();
        if p_raster.x < sample_bounds.min.x as Float || p_raster.x >= sample_bounds.max.x as Float ||
            p_raster.y < sample_bounds.min.y as Float || p_raster.y >= sample_bounds.max.y as Float {
            return Spectrum::new(0.0);
        }

        // Compute lens area of perspective camera
        let lens_area = if self.lens_radius != 0.0 { PI * self.lens_radius * self.lens_radius } else { 1.0 };

        // Return importance for point on image plane
        let cos2_theta = cos_theta * cos_theta;
        Spectrum::new(1.0 / (self.a * lens_area * cos2_theta * cos2_theta))
    }

    fn pdf_we(&self, ray: &Ray) -> (Float, Float) {
        // Interpolate camera matrix and fail if $\w{}$ is not forward-facing
        let c2w = Transform::default();
        self.camera_to_world.interpolate(ray.time, &mut c2w);
        let cos_theta = ray.d.dot(&c2w.transform_vector(&Vector3f::new(0.0, 0.0, 0.1)));
        if cos_theta <= 0.0 {
            return (0.0, 0.0);
        }

        // Map ray $(\p{}, \w{})$ onto the raster grid
        let p_focus = ray.point_at_time(if self.lens_radius > 0.0 { self.focal_distance } else { 1.0 });
        let p_raster = self.raster_to_camera.inverse()
            .transform_point(&c2w.inverse().transform_point(&p_focus));
        
        // Return zero probability for out of bounds points
        let sample_bounds = self.film.get_sample_bounds();
        if p_raster.x < sample_bounds.min.x as Float || p_raster.x >= sample_bounds.max.x as Float ||
            p_raster.y < sample_bounds.min.y as Float || p_raster.y >= sample_bounds.max.y as Float {
            return (0.0, 0.0);
        }

        // Compute lens area of perspective camera
        let lens_area = if self.lens_radius != 0.0 { PI * self.lens_radius * self.lens_radius } else { 1.0 };
        (1.0 / lens_area, 1.0 / (self.a * cos_theta * cos_theta * cos_theta))
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
        // Uniformly sample a lens interaction _lensIntr_
        let p_lens = self.lens_radius * concentric_sample_disk(u);
        let p_lens_world = self.camera_to_world.transform_point(iref.get_time(), &Point3f::new(p_lens.x, p_lens.y, 0.0));
        let lens_intr: Box<SimpleInteraction> = Box::new(SimpleInteraction::default());
        lens_intr.p = p_lens_world;
        lens_intr.time = iref.get_time();
        if let Some(medium) = self.medium {
            lens_intr.medium_interface = Some(Arc::new(MediumInterface::from(medium)));
        }
        lens_intr.n = Normal3f::from(self.camera_to_world.transform_vector(iref.get_time(), &Vector3f::new(0.0, 0.0, 1.0)));

        // Populate arguments and compute the importance value
        *vis = VisibilityTester::new(iref, lens_intr);
        let dist = wi.length();
        *wi = (lens_intr.p - iref.get_p()) / dist;
        
        // Compute PDF for importance arriving at _ref_

        // Compute lens area of perspective camera
        let lens_area = if self.lens_radius != 0.0 { PI * self.lens_radius * self.lens_radius } else { 1.0 };
        *pdf = (dist * dist) / (dot_normal_vec(&lens_intr.n, &*wi).abs() * lens_area);
        self.we(&lens_intr.spawn_ray(&-*wi), Some(p_raster))
    }
}