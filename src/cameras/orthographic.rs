use crate::core::pbrt::{Float, lerp};
use crate::core::geometry::{Vector3f, Point3f, Bounds2f, Ray, RayDifferential};
use crate::core::camera::{Camera, CameraSample};
use crate::core::transform::Transform;
use crate::core::animated_transform::AnimatedTransform;
use crate::core::film::Film;
use crate::core::medium::Medium;
use crate::core::profiler::Profiler;
use crate::core::sampling::concentric_sample_disk;
use std::sync::Arc;

pub struct OrthographicCamera {
    // camera common
    pub camera_to_world: AnimatedTransform,
    pub shutter_open: Float,
    pub shutter_close: Float,
    pub film: Arc<Film>,
    pub medium: Option<Arc<dyn Medium>>,
    // projective camera common
    camera_to_screen: Transform,
    raster_to_camera: Transform,
    screen_to_raster: Transform,
    raster_to_screen: Transform,
    lens_radius: Float,
    focal_distance: Float,
    // OrthographicCamera private
    dx_camera: Vector3f,
    dy_camera: Vector3f
}

impl OrthographicCamera {
    pub fn new(
        camera_to_world: AnimatedTransform,
        screen_window: Bounds2f,
        shutter_open: Float,
        shutter_close: Float,
        lens_radius: Float,
        focal_distance: Float,
        film: Arc<Film>,
        medium: Option<Arc<dyn Medium>>
    ) -> OrthographicCamera {
        // Compute projective camera screen transformations
        let screen_to_raster = Transform::scale(film.full_resolution.x, film.full_resolution.y, 1.0) *
            Transform::scale(1.0 / (screen_window.max.x - screen_window.min.x),
                1.0 / (screen_window.min.y - screen_window.max.y), 1.0) *
            Transform::translate(&Vector3f::new(-screen_window.min.x, -screen_window.max.y, 0.0));
        let raster_to_screen = screen_to_raster.inverse();
        let camera_to_screen = Transform::orthographic(0.0, 1.0);
        let raster_to_camera = camera_to_screen.inverse() * raster_to_screen;
        
        OrthographicCamera {
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
            dy_camera: raster_to_camera.transform_vector(&Vector3f::new(0.0, 1.0, 0.0))
        }
    }
}

impl Camera for OrthographicCamera {
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
        ray.time = lerp(sample.time, self.shutter_open, self.shutter_close);
        ray.medium = self.medium;
        *ray = self.camera_to_world.transform_ray(ray);
        1.0
    }

    fn generate_ray_differential(&self, sample: &CameraSample, ray: &mut Ray) -> Float {
        let _guard = Profiler::instance().profile("Camera::generate_ray_differential()");
        // Compute main orthographic viewing ray

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
            // Compute _OrthographicCamera_ ray differentials accounting for lens

            // Sample point on lens
            let p_lens = self.lens_radius * concentric_sample_disk(&sample.p_lens);
            let ft = self.focal_distance / ray.d.z;
            let p_focus = p_camera + self.dx_camera + (ft * Vector3f::new(0.0, 0.0, 1.0));
            rd.rx_origin = Point3f::new(p_lens.x, p_lens.y, 0.0);
            rd.rx_direction = (p_focus - rd.rx_origin).normalize();

            p_focus = p_camera + self.dy_camera + (ft * Vector3f::new(0.0, 0.0, 1.0));
            rd.ry_origin = Point3f::new(p_lens.x, p_lens.y, 0.0);
            rd.ry_direction = (p_focus - rd.ry_origin).normalize();
        } else {
            rd.rx_origin = ray.o + self.dx_camera;
            rd.ry_origin = ray.o + self.dy_camera;
            rd.rx_direction = ray.d;
            rd.ry_direction = ray.d;
        }
        ray.differential = Some(rd);
        ray.time = lerp(sample.time, self.shutter_open, self.shutter_close);
        ray.medium = self.medium;
        *ray = self.camera_to_world.transform_ray(ray);
        1.0
    }
}