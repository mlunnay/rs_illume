use std::sync::Arc;
use crate::core::pbrt::{Float, consts::PI, radians};
use crate::core::shape::Shape;
use crate::core::geometry::{Bounding3, Bounds3f, Ray, Point2f, Point3f, Normal3f, Vector3f};
use crate::core::transform::Transform;
use crate::core::material::Material;
use crate::core::interaction::*;
use crate::core::surface_interaction::SurfaceInteraction;
use crate::core::efloat::{EFloat, quadratic};
use crate::core::profiler::Profiler;
use num::clamp;

pub struct Cone {
    radius: Float,
    height: Float,
    phi_max: Float,
    // inherited from class Shape (see shape.h)
    object_to_world: Transform,
    world_to_object: Transform,
    reverse_orientation: bool,
    transform_swaps_handedness: bool,
    pub material: Option<Arc<dyn Material + Send + Sync>>,
}

impl Cone {
    pub fn new(
        object_to_world: Transform,
        world_to_object: Transform,
        reverse_orientation: bool,
        radius: Float,
        height: Float,
        phi_max: Float
    ) -> Cone {
        Cone {
            object_to_world,
            world_to_object,
            radius,
            height,
            phi_max: radians(clamp(phi_max, 0.0, 360.0)),
            reverse_orientation,
            transform_swaps_handedness: object_to_world.swaps_handness(),
            material: None
        }
    }
}

impl Default for Cone {
    fn default() -> Cone {
        let object_to_world = Transform::default();
        Cone {
            object_to_world,
            world_to_object: Transform::default(),
            radius: 1.0,
            height: 1.0,
            phi_max: PI * 2.0,
            reverse_orientation: false,
            transform_swaps_handedness: object_to_world.swaps_handness(),
            material: None
        }
    }
}

impl Shape for Cone {
    fn object_bound(&self) -> Box<dyn Bounding3<Float>> {
        Box::new(Bounds3f{
            min: Point3f {
                x: -self.radius,
                y: -self.radius,
                z: 0.0
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

    fn intersect(&self, ray: &Ray, test_alpha_texture: bool) -> Option<(SurfaceInteraction, Float)> {
        let _p = Profiler::instance().profile("Cone::intersect");
        let mut phi: Float = 0.0;
        let mut p_hit: Point3f;
        // Transform _Ray_ to object space
        let mut o_err = Vector3f::default();
        let mut d_err = Vector3f::default();
        let ray = self.world_to_object.transform_ray_with_error(&ray, &mut o_err, &mut d_err);

        // Compute quadratic cone coefficients

        // Initialize _efloat_ ray coordinate values
        let ox = EFloat::new(ray.o.x, o_err.x);
        let oy = EFloat::new(ray.o.y, o_err.y);
        let oz = EFloat::new(ray.o.z, o_err.z);
        let dx = EFloat::new(ray.d.x, d_err.x);
        let dy = EFloat::new(ray.d.y, d_err.y);
        let dz = EFloat::new(ray.d.z, d_err.z);
        let mut k = EFloat::new(self.radius, 0.0) / EFloat::new(self.height, 0.0);
        k = k * k;
        let a = dx * dx + dy * dy - k * dz * dz;
        let b = dx * ox + dy * oy - k * dz * (oz - self.height) * 2.0;
        let c = ox * ox + oy * oy - k * (oz - self.height) * (oz - self.height);

        // Solve quadratic equation for _t_ values
        let t0 = EFloat::default();
        let t1 = EFloat::default();
        if !quadratic(a, b, c, &mut t0, &mut t1) {
            return None;
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

        // Compute cone inverse mapping
        p_hit = ray.point_at_time(t_shape_hit.v);
        phi = p_hit.y.atan2(p_hit.x);
        if phi < 0.0 {
            phi += 2.0 * PI;
        }

        // Test cone intersection against clipping parameters
        if p_hit.z < 0.0 || p_hit.z > self.height || phi > self.phi_max {
            if t_shape_hit == t1 {
                return None;
            }
            t_shape_hit = t1;
            if t1.upper_bound() > ray.t_max {
                return None;
            }
            // Compute cone inverse mapping
            p_hit = ray.point_at_time(t_shape_hit.v);
            phi = p_hit.y.atan2(p_hit.x);
            if phi < 0.0 {
                phi += 2.0 * PI;
            }
            if p_hit.z < 0.0 || p_hit.z > self.height || phi > self.phi_max {
                return None;
            }
        }

        // find parametric representation of cone hit
        let u = phi / self.phi_max;
        let v = p_hit.z / self.height;

        // Compute cone $\dpdu$ and $\dpdv$
        let dpdu = Vector3f::new(-self.phi_max * p_hit.y, self.phi_max * p_hit.x, 0.0);
        let dpdv = Vector3f::new(-p_hit.x / (1.0 - v), -p_hit.y / (1.0 - v), self.height);

        // Compute cone $\dndu$ and $\dndv$
        let d2_pduu = -self.phi_max * self.phi_max * Vector3f::new(p_hit.x, p_hit.y, 0.0);
        let d2_pduv = self.phi_max / (1.0 - v) * Vector3f::new(p_hit.y, -p_hit.x, 0.0);
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

        // Compute error bounds for cone intersection

        // Compute error bounds for intersection computed with ray equation
        let px = ox + t_shape_hit * dx;
        let py = oy + t_shape_hit * dy;
        let pz = oz + t_shape_hit * dz;
        let p_error = Vector3f::new(px.get_absolute_error(), py.get_absolute_error(),
                                pz.get_absolute_error());

        // Initialize _SurfaceInteraction_ from parametric information
        let isect = self.object_to_world.tranform_surface_interaction(&SurfaceInteraction::new(&p_hit, &p_error, &Point2f::new(u, v),
                                                    &-ray.d, &dpdu, &dpdv, &dndu, &dndv,
                                                    ray.time, Some(self), 0));
        Some((isect, t_shape_hit.v as Float))
    }

    fn intersect_p(&self, ray: &Ray, test_alpha_texture: bool) -> bool {
        let _p = Profiler::instance().profile("Cone::intersect_p");
        let mut phi: Float = 0.0;
        let mut p_hit: Point3f;
        // Transform _Ray_ to object space
        let mut o_err = Vector3f::default();
        let mut d_err = Vector3f::default();
        let ray = self.world_to_object.transform_ray_with_error(&ray, &mut o_err, &mut d_err);

        // Compute quadratic cone coefficients

        // Initialize _efloat_ ray coordinate values
        let ox = EFloat::new(ray.o.x, o_err.x);
        let oy = EFloat::new(ray.o.y, o_err.y);
        let oz = EFloat::new(ray.o.z, o_err.z);
        let dx = EFloat::new(ray.d.x, d_err.x);
        let dy = EFloat::new(ray.d.y, d_err.y);
        let dz = EFloat::new(ray.d.z, d_err.z);
        let mut k = EFloat::new(self.radius, 0.0) / EFloat::new(self.height, 0.0);
        k = k * k;
        let a = dx * dx + dy * dy - k * dz * dz;
        let b = dx * ox + dy * oy - k * dz * (oz - self.height) * 2.0;
        let c = ox * ox + oy * oy - k * (oz - self.height) * (oz - self.height);

        // Solve quadratic equation for _t_ values
        let t0 = EFloat::default();
        let t1 = EFloat::default();
        if !quadratic(a, b, c, &mut t0, &mut t1) {
            return false;
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

        // Compute cone inverse mapping
        p_hit = ray.point_at_time(t_shape_hit.v);
        phi = p_hit.y.atan2(p_hit.x);
        if phi < 0.0 {
            phi += 2.0 * PI;
        }

        // Test cone intersection against clipping parameters
        if p_hit.z < 0.0 || p_hit.z > self.height || phi > self.phi_max {
            if t_shape_hit == t1 {
                return false;
            }
            t_shape_hit = t1;
            if t1.upper_bound() > ray.t_max {
                return false;
            }
            // Compute cone inverse mapping
            p_hit = ray.point_at_time(t_shape_hit.v);
            phi = p_hit.y.atan2(p_hit.x);
            if phi < 0.0 {
                phi += 2.0 * PI;
            }
            if p_hit.z < 0.0 || p_hit.z > self.height || phi > self.phi_max {
                return false;
            }
        }
        true
    }

    fn area(&self) -> Float {
        self.radius * (self.height * self.height + self.radius * self.radius).sqrt() * self.phi_max / 2.0
    }

    fn sample(&self, u: &Point2f, pdf: &mut Float) -> Box<dyn Interaction> {
        error!("Cone does not implement sample");
        Box::new(SimpleInteraction::default())
    }
}