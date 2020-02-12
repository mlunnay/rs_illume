use std::sync::Arc;
use crate::core::pbrt::{Float, consts::PI, gamma, lerp};
use crate::core::shape::Shape;
use crate::core::geometry::{Bounding3, Bounds3f, Ray, Point2f, Point3f, Normal3f, Vector3f};
use crate::core::transform::Transform;
use crate::core::material::Material;
use crate::core::interaction::*;
use crate::core::interaction::SurfaceInteraction;
use crate::core::efloat::{EFloat, quadratic};
use crate::core::profiler::Profiler;

pub struct Cylinder {
    radius: Float,
    z_min: Float,
    z_max: Float,
    phi_max: Float,
    // inherited from class Shape (see shape.h)
    object_to_world: Transform,
    world_to_object: Transform,
    reverse_orientation: bool,
    transform_swaps_handedness: bool,
    pub material: Option<Arc<dyn Material + Send + Sync>>,
}

impl Cylinder {
    pub fn new(
        object_to_world: Transform,
        world_to_object: Transform,
        reverse_orientation: bool,
        radius: Float,
        z_min: Float,
        z_max: Float,
        phi_max: Float
    ) -> Cylinder {
        Cylinder {
            object_to_world,
            world_to_object,
            radius,
            z_min,
            z_max,
            phi_max,
            reverse_orientation,
            transform_swaps_handedness: object_to_world.swaps_handness(),
            material: None
        }
    }
}

impl Shape for Cylinder {
    fn object_bound(&self) -> Box<dyn Bounding3<Float>> {
        Box::new(Bounds3f{
            min: Point3f {
                x: -self.radius,
                y: -self.radius,
                z: self.z_min
            },
            max: Point3f {
                x: self.radius,
                y: self.radius,
                z: self.z_max
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

    fn intersect(&self, ray: &Ray, test_alpha_texture: bool) -> Option<(SurfaceInteraction, Float)> {
        let _p = Profiler::instance().profile("Cylinder::intersect");
        let mut phi: Float = 0.0;
        let mut p_hit: Point3f;
        // Transform _Ray_ to object space
        let mut o_err = Vector3f::default();
        let mut d_err = Vector3f::default();
        let ray = self.world_to_object.transform_ray_with_error(&ray, &mut o_err, &mut d_err);

        // Compute quadratic sphere coefficients

        // Initialize _efloat_ ray coordinate values
        let ox = EFloat::new(ray.o.x, o_err.x);
        let oy = EFloat::new(ray.o.y, o_err.y);
        let dx = EFloat::new(ray.d.x, d_err.x);
        let dy = EFloat::new(ray.d.y, d_err.y);
        let a = dx * dx + dy * dy;
        let b = (dx * ox + dy * oy) * 2.0;
        let c = ox * ox + oy * oy - EFloat::new(self.radius, 0.0) * EFloat::new(self.radius, 0.0);

        // Solve quadratic equation for _t_ values
        let mut t0 = EFloat::default();
        let mut t1 = EFloat::default();
        if !quadratic(a, b, c, &mut t0, &mut t1) {
            return None;
        }

        // Check quadric shape _t0_ and _t1_ for nearest intersection
        if t0.upper_bound() > ray.t_max || t1.lower_bound() <= 0.0 {
            return None;
        }
        let mut t_shape_hit = t0;
        if t_shape_hit.lower_bound() <= 0.0 {
            t_shape_hit = t1;
            if t_shape_hit.upper_bound() > ray.t_max {
                return None;
            }
        }

        // Check quadric shape _t0_ and _t1_ for nearest intersection
        if t0.upper_bound() > ray.t_max || t1.lower_bound() <= 0.0 {
            return None;
        }
        let t_shape_hit = t0;
        if t_shape_hit.lower_bound() <= 0.0 {
            t_shape_hit = t1;
            if t_shape_hit.upper_bound() > ray.t_max {
                return None;
            }
        }

        // Compute cylinder hit point and $\phi$
        p_hit = ray.point_at_time(t_shape_hit.v);

        // Refine cylinder intersection point
        let hit_rad = (p_hit.x * p_hit.x + p_hit.y * p_hit.y).sqrt();
        p_hit.x *= self.radius / hit_rad;
        p_hit.y *= self.radius / hit_rad;
        phi = p_hit.y.atan2(p_hit.x);
        if phi < 0.0 {
            phi += 2.0 * PI;
        }

        // Test cylinder intersection against clipping parameters
        if p_hit.z < self.z_min || p_hit.z > self.z_max || phi > self.phi_max {
            if t_shape_hit == t1 {
                return None;
            }
            t_shape_hit = t1;
            if t1.upper_bound() > ray.t_max {
                return None;
            }
            // Compute cylinder hit point and $\phi$
            p_hit = ray.point_at_time(t_shape_hit.v);

            // Refine cylinder intersection point
            let hit_rad = (p_hit.x * p_hit.x + p_hit.y * p_hit.y).sqrt();
            p_hit.x *= self.radius / hit_rad;
            p_hit.y *= self.radius / hit_rad;
            phi = p_hit.y.atan2(p_hit.x);
            if phi < 0.0 {
                phi += 2.0 * PI;
            }
            if p_hit.z < self.z_min || p_hit.z > self.z_max || phi > self.phi_max {
                return None;
            }
        }

        // find parametric representation of cylinder hit
        let u = phi / self.phi_max;
        let v = (p_hit.z - self.z_min) / (self.z_max - self.z_min);

        // Compute cylinder $\dpdu$ and $\dpdv$
        let dpdu = Vector3f::new(-self.phi_max * p_hit.y, self.phi_max * p_hit.x, 0.0);
        let dpdv = Vector3f::new(0.0, 0.0, self.z_max - self.z_min);

        // Compute cylinder $\dndu$ and $\dndv$
        let d2_pduu = -self.phi_max * self.phi_max * Vector3f::new(p_hit.x, p_hit.y, 0.0);
        let d2_pduv = Vector3f::default();
        let d2_pdvv = Vector3f::default();

        // Compute coefficients for fundamental forms
        let e = dpdu.dot(&dpdu);
        let f = dpdu.dot(&dpdv);
        let g = dpdv.dot(&dpdv);
        let n = dpdu.cross(&dpdv).normalize();
        let e = n.dot(&d2_pduu);
        let f = n.dot(&d2_pduv);
        let g = n.dot(&d2_pdvv);

        // Compute $\dndu$ and $\dndv$ from fundamental form coefficients
        let inv_e_g_f2 = 1.0 / (e * g - f * f);
        let dndu = Normal3f::from((f * f - e * g) * inv_e_g_f2 * dpdu +
                                (e * f - f * e) * inv_e_g_f2 * dpdv);
        let dndv = Normal3f::from((g * f - f * g) * inv_e_g_f2 * dpdu +
                                (f * f - g * e) * inv_e_g_f2 * dpdv);

        // Compute error bounds for cylinder intersection
        let p_error = gamma(3) * Vector3f::new(p_hit.x, p_hit.y, 0.0).abs();

        // Initialize _SurfaceInteraction_ from parametric information
        let isect = self.object_to_world.tranform_surface_interaction(&SurfaceInteraction::new(p_hit, p_error, Point2f::new(u, v),
                                                    -ray.d, dpdu, dpdv, dndu, dndv,
                                                    ray.time, Some(self), 0));

        Some((isect, t_shape_hit.v as Float))
    }

    fn intersect_p(&self, ray: &Ray, test_alpha_texture: bool) -> bool {
        let _p = Profiler::instance().profile("Cylinder::intersect_p");
        let mut phi: Float = 0.0;
        let mut p_hit: Point3f;
        // Transform _Ray_ to object space
        let mut o_err = Vector3f::default();
        let mut d_err = Vector3f::default();
        let ray = self.world_to_object.transform_ray_with_error(&ray, &mut o_err, &mut d_err);

        // Compute quadratic cyliner coefficients

        // Initialize _efloat_ ray coordinate values
        let ox = EFloat::new(ray.o.x, o_err.x);
        let oy = EFloat::new(ray.o.y, o_err.y);
        let dx = EFloat::new(ray.d.x, d_err.x);
        let dy = EFloat::new(ray.d.y, d_err.y);
        let a = dx * dx + dy * dy;
        let b = (dx * ox + dy * oy) * 2.0;
        let c = ox * ox + oy * oy - EFloat::new(self.radius, 0.0) * EFloat::new(self.radius, 0.0);

        // Solve quadratic equation for _t_ values
        let mut t0 = EFloat::default();
        let mut t1 = EFloat::default();
        if !quadratic(a, b, c, &mut t0, &mut t1) {
            return false;
        }

        // Check quadric shape _t0_ and _t1_ for nearest intersection
        if t0.upper_bound() > ray.t_max || t1.lower_bound() <= 0.0 {
            return false;
        }
        let mut t_shape_hit = t0;
        if t_shape_hit.lower_bound() <= 0.0 {
            t_shape_hit = t1;
            if t_shape_hit.upper_bound() > ray.t_max {
                return false;
            }
        }

        // Check quadric shape _t0_ and _t1_ for nearest intersection
        if t0.upper_bound() > ray.t_max || t1.lower_bound() <= 0.0 {
            return false;
        }
        let t_shape_hit = t0;
        if t_shape_hit.lower_bound() <= 0.0 {
            t_shape_hit = t1;
            if t_shape_hit.upper_bound() > ray.t_max {
                return false;
            }
        }

        // Compute cylinder hit point and $\phi$
        p_hit = ray.point_at_time(t_shape_hit.v);

        // Refine cylinder intersection point
        let hit_rad = (p_hit.x * p_hit.x + p_hit.y * p_hit.y).sqrt();
        p_hit.x *= self.radius / hit_rad;
        p_hit.y *= self.radius / hit_rad;
        phi = p_hit.y.atan2(p_hit.x);
        if phi < 0.0 {
            phi += 2.0 * PI;
        }

        // Test cylinder intersection against clipping parameters
        if p_hit.z < self.z_min || p_hit.z > self.z_max || phi > self.phi_max {
            if t_shape_hit == t1 {
                return false;
            }
            t_shape_hit = t1;
            if t1.upper_bound() > ray.t_max {
                return false;
            }
            // Compute cylinder hit point and $\phi$
            p_hit = ray.point_at_time(t_shape_hit.v);

            // Refine cylinder intersection point
            let hit_rad = (p_hit.x * p_hit.x + p_hit.y * p_hit.y).sqrt();
            p_hit.x *= self.radius / hit_rad;
            p_hit.y *= self.radius / hit_rad;
            phi = p_hit.y.atan2(p_hit.x);
            if phi < 0.0 {
                phi += 2.0 * PI;
            }
            if p_hit.z < self.z_min || p_hit.z > self.z_max || phi > self.phi_max {
                return false;
            }
        }
        true
    }

    fn area(&self) -> Float {
        self.phi_max * self.radius * (self.z_max - self.z_min)
    }

    fn sample(&self, u: &Point2f, pdf: &mut Float) -> Box<dyn Interaction> {
        let z = lerp(u.x, self.z_min, self.z_max);
        let phi = u.y * self.phi_max;
        let p_obj = Point3f::new(self.radius * phi.cos(), self.radius * phi.sin(), z);
        let it = SimpleInteraction::default();
        it.n = self.object_to_world.transform_normal(&Normal3f::new(p_obj.x, p_obj.y, 0.0)).normalize();
        if self.reverse_orientation {
            it.n *= -1.0;
        }
        // Reproject _p_obj_ to cylinder surface and compute _p_objError_
        let hit_rad = (p_obj.x * p_obj.x + p_obj.y * p_obj.y).sqrt();
        p_obj.x *= self.radius / hit_rad;
        p_obj.y *= self.radius / hit_rad;
        let p_obj_error = gamma(3) * Vector3f::new(p_obj.x, p_obj.y, 0.0).abs();
        it.p = self.object_to_world.transform_point_with_abs_error(&p_obj, &p_obj_error, &mut it.p_error);
        *pdf = 1.0 / self.area();
        Box::new(it)
    }
}

impl Default for Cylinder {
    fn default() -> Cylinder {
        let object_to_world = Transform::default();
        Cylinder{
            object_to_world,
            world_to_object: Transform::default(),
            radius: 1.0,
            z_min: 0.0,
            z_max: 1.0,
            phi_max: PI * 2.0,
            reverse_orientation: false,
            transform_swaps_handedness: object_to_world.swaps_handness(),
            material: None
        }
    }
}