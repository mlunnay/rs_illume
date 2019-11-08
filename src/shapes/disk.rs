use std::sync::Arc;
use crate::core::pbrt::{Float, consts::PI, radians};
use crate::core::shape::Shape;
use crate::core::geometry::{Bounding3, Bounds3f, Ray, Point2f, Point3f, Normal3f, Vector3f};
use crate::core::transform::Transform;
use crate::core::material::Material;
use crate::core::interaction::*;
use crate::core::sampling::concentric_sample_disk;
use crate::core::surface_interaction::SurfaceInteraction;
use crate::core::profiler::Profiler;
use num::clamp;

pub struct Disk {
    height: Float,
    inner_radius: Float,
    radius: Float,
    phi_max: Float,
    // inherited from class Shape (see shape.h)
    object_to_world: Transform,
    world_to_object: Transform,
    reverse_orientation: bool,
    transform_swaps_handedness: bool,
    pub material: Option<Arc<dyn Material + Send + Sync>>,
}

impl Disk {
    pub fn new(
        object_to_world: Transform,
        world_to_object: Transform,
        reverse_orientation: bool,
        height: Float,
        inner_radius: Float,
        radius: Float,
        phi_max: Float
    ) -> Disk {
        Disk{
            height,
            inner_radius,
            radius,
            phi_max: radians(clamp(phi_max, 0.0, 360.0)),
            object_to_world,
            world_to_object,
            reverse_orientation,
            transform_swaps_handedness: object_to_world.swaps_handness(),
            material: None
        }
    }
}

impl Default for Disk {
    fn default() -> Disk {
        let object_to_world = Transform::default();
        Disk{
            height: 1.0,
            inner_radius: 0.0,
            radius: 1.0,
            phi_max: PI * 2.0,
            object_to_world,
            world_to_object: Transform::default(),
            reverse_orientation: false,
            transform_swaps_handedness: object_to_world.swaps_handness(),
            material: None
        }
    }
}

impl Shape for Disk {
    fn object_bound(&self) -> Box<dyn Bounding3<Float>> {
        Box::new(Bounds3f{
            min: Point3f {
                x: -self.radius,
                y: -self.radius,
                z: self.height
            },
            max: Point3f {
                x: self.radius,
                y: self.radius,
                z: self.height
            }
        })
    }

    fn world_bound(&self) -> Box<dyn Bounding3<Float>> {
        self.object_to_world.transform_bounds(self.object_bound())
    }

    fn get_reverse_orientation(&self) -> bool {
        self.reverse_orientation
    }

    fn get_transform_swaps_handedness(&self) -> bool {
        self.transform_swaps_handedness
    }

    fn area(&self) -> Float {
        self.phi_max * 0.5 * (self.radius * self.radius - self.inner_radius * self.inner_radius)
    }

    fn sample(&self, u: &Point2f, pdf: &mut Float) -> Box<dyn Interaction> {
        let pd = concentric_sample_disk(u);
        let p_obj = Point3f::new(pd.x * self.radius, pd.y * self.radius, self.height);
        let mut it = SimpleInteraction::default();
        it.n = self.object_to_world.transform_normal(&Normal3f::new(0.0, 0.0, 1.0)).normalize();
        if self.reverse_orientation { it.n *= -1.0 };
        it.p = self.object_to_world.transform_point_with_abs_error(&p_obj, &Vector3f::default(), &mut it.p_error);
        *pdf = 1.0 / self.area();
        Box::new(it)
    }

    fn intersect(&self, ray: &Ray, test_alpha_texture: bool) -> Option<(SurfaceInteraction, Float)> {
        let _p = Profiler::instance().profile("Disk::intersect");
        // Transform _Ray_ to object space
        let o_err = Vector3f::default();
        let d_err = Vector3f::default();
        let ray = self.world_to_object.transform_ray_with_error(&ray, &mut o_err, &mut d_err);

        // Compute plane intersection for disk

        // Reject disk intersections for rays parallel to the disk's plane
        if ray.d.z == 0.0 {
            return None;
        }
        let t_shape_hit = (self.height - ray.o.z) / ray.d.z;
        if t_shape_hit <= 0.0 || t_shape_hit >= ray.t_max {
            return None;
        }

        // See if hit point is inside disk radii and $\phimax$
        let p_hit = ray.point_at_time(t_shape_hit);
        let dist2 = p_hit.x * p_hit.x + p_hit.y * p_hit.y;
        if dist2 > self.radius * self.radius
            || dist2 < self.inner_radius * self.inner_radius {
            return None;
        }

        // Test disk $\phi$ value against $\phimax$
        let phi = p_hit.y.atan2(p_hit.x);
        if phi < 0.0 {
            phi += 2.0 * PI;
        }
        if phi > self.phi_max {
            return None;
        }

        // Find parametric representation of disk hit
        let u = phi / self.phi_max;
        let r_hit = dist2.sqrt();
        let v = (self.radius - r_hit) / (self.radius - self.inner_radius);
        let dpdu = Vector3f::new(-self.phi_max * p_hit.y, self.phi_max * p_hit.x, 0.0);
        let dpdv = Vector3f::new(p_hit.x, p_hit.y, 0.0) * (self.inner_radius - self.radius) / r_hit;
        let dndu = Normal3f::default();
        let dndv = Normal3f::default();

        // Refine disk intersection point
        p_hit.z = self.height;

        // Compute error bounds for disk intersection
        let p_error = Vector3f::default();

        // Initialize _SurfaceInteraction_ from parametric information
        let isect = self.object_to_world.tranform_surface_interaction(&SurfaceInteraction::new(&p_hit, &p_error, &Point2f::new(u, v),
                                                    &-ray.d, &dpdu, &dpdv, &dndu, &dndv,
                                                    ray.time, Some(self), 0));

        Some((isect, t_shape_hit as Float))
    }

    fn intersect_p(&self, ray: &Ray, test_alpha_texture: bool) -> bool {
        let _p = Profiler::instance().profile("Disk::intersect_p");
        // Transform _Ray_ to object space
        let o_err = Vector3f::default();
        let d_err = Vector3f::default();
        let ray = self.world_to_object.transform_ray_with_error(&ray, &mut o_err, &mut d_err);

        // Compute plane intersection for disk

        // Reject disk intersections for rays parallel to the disk's plane
        if ray.d.z == 0.0 {
            return false;
        }
        let t_shape_hit = (self.height - ray.o.z) / ray.d.z;
        if t_shape_hit <= 0.0 || t_shape_hit >= ray.t_max {
            return false;
        }

        // See if hit point is inside disk radii and $\phimax$
        let p_hit = ray.point_at_time(t_shape_hit);
        let dist2 = p_hit.x * p_hit.x + p_hit.y * p_hit.y;
        if dist2 > self.radius * self.radius
            || dist2 < self.inner_radius * self.inner_radius {
            return false;
        }

        // Test disk $\phi$ value against $\phimax$
        let phi = p_hit.y.atan2(p_hit.x);
        if phi < 0.0 {
            phi += 2.0 * PI;
        }
        if phi > self.phi_max {
            return false;
        }
        true
    }
}