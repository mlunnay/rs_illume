use std::sync::Arc;
use crate::core::pbrt::{Float, consts::PI, radians};
use crate::core::shape::Shape;
use crate::core::geometry::{Bounding3, Bounds3f, Ray, Point2f, Point3f, Normal3f, Vector3f};
use crate::core::transform::Transform;
use crate::core::material::Material;
use crate::core::interaction::*;
use crate::core::interaction::SurfaceInteraction;
use crate::core::efloat::{EFloat, quadratic};
use crate::core::profiler::Profiler;
use num::clamp;

pub struct Hyperboloid {
    p1: Point3f,
    p2: Point3f,
    z_min: Float,
    z_max: Float,
    phi_max: Float,
    r_max: Float,
    ah: Float,
    ch: Float,
    // inherited from class Shape (see shape.h)
    object_to_world: Transform,
    world_to_object: Transform,
    reverse_orientation: bool,
    transform_swaps_handedness: bool,
    pub material: Option<Arc<dyn Material + Send + Sync>>,
}

impl Hyperboloid {
    pub fn new(
        object_to_world: Transform,
        world_to_object: Transform,
        reverse_orientation: bool,
        p1: &Point3f,
        p2: &Point3f,
        tm: Float
    ) -> Hyperboloid {
        let radius1 = (p1.x * p1.x + p1.y * p1.y).sqrt();
        let radius2 = (p2.x * p2.x + p2.y * p2.y).sqrt();
        // Compute implicit function coefficients for hyperboloid
        let (p1, p2) = if p2.z == 0.0 {
            (p1, p2)
        }
        else {
            (p2, p1)
        };
        let mut pp = p1.clone();
        let mut xy1 = 0.0;
        let mut xy2 = 0.0;
        let mut ah: Float = num::Float::infinity();
        let mut ch: Float = num::Float::infinity();

        while ah.is_infinite() || ch.is_infinite() {
            pp = pp + 2.0 * (*p2 - *p1);
            xy1 = pp.x * pp.x + pp.y * pp.y;
            xy2 = p2.x * p2.x + p2.y * p2.y;
            ah = (1.0 / xy1 - (pp.z * pp.z) / (xy1 * p2.z * p2.z)) /
                (1.0 - (xy2 * pp.z * pp.z) / (xy1 * p2.z * p2.z));
            ch = (ah * xy2 - 1.0) / (p2.z * p2.z);
        }

        Hyperboloid {
            object_to_world,
            world_to_object,
            reverse_orientation,
            transform_swaps_handedness: object_to_world.swaps_handness(),
            material: None,
            phi_max: radians(clamp(tm, 0.0, 360.0)),
            r_max: radius1.max(radius2),
            z_min: p1.z.min(p2.z),
            z_max: p1.z.max(p2.z),
            p1: p1.clone(),
            p2: p2.clone(),
            ah,
            ch
        }
    }
}

impl Default for Hyperboloid {
    fn default() -> Hyperboloid {
        let object_to_world = Transform::default();
        Hyperboloid::new(
            Transform::default(),
            Transform::default(),
            false,
            &Point3f::new(0.0, 0.0, 0.0),
            &Point3f::new(1.0, 1.0, 1.0),
            360.0
        )
    }
}

macro_rules! sqr {
    ($a: expr) => {
        $a * $a
    };
}
macro_rules! quad {
    ($a: expr) => {
        $a * $a * $a * $a
    };
}

impl Shape for Hyperboloid {
    fn object_bound(&self) -> Box<dyn Bounding3<Float>> {
        Box::new(Bounds3f{
            min: Point3f {
                x: -self.r_max,
                y: -self.r_max,
                z: self.z_min
            },
            max: Point3f {
                x: self.r_max,
                y: self.r_max,
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
        let _p = Profiler::instance().profile("Hyperboloid::intersect");
        let mut phi: Float = 0.0;
        let mut p_hit: Point3f;
        // Transform _Ray_ to object space
        let mut o_err = Vector3f::default();
        let mut d_err = Vector3f::default();
        let ray = self.world_to_object.transform_ray_with_error(&ray, &mut o_err, &mut d_err);

        // Compute quadratic hyperboloid coefficients

        // Initialize _efloat_ ray coordinate values
        let ox = EFloat::new(ray.o.x, o_err.x);
        let oy = EFloat::new(ray.o.y, o_err.y);
        let oz = EFloat::new(ray.o.z, o_err.z);
        let dx = EFloat::new(ray.d.x, d_err.x);
        let dy = EFloat::new(ray.d.y, d_err.y);
        let dz = EFloat::new(ray.d.z, d_err.z);
        let a = dx * dx * self.ah + dy * dy * self.ah - dz * dz * self.ch;
        let b = 2.0 * (dx * ox * self.ah + dy * oy * self.ah - dz * oz * self.ch);
        let c = ox * ox * self.ah + oy * oy * self.ah - oz * oz - 1.0 * self.ch;

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

        // Compute hyperboloid inverse mapping
        p_hit = ray.point_at_time(t_shape_hit.v);
        let v = (p_hit.z - self.p1.z) / (self.p2.z - self.p1.z);
        let pr = self.p1 * (1.0 - v) + self.p2 * v;
        phi = pr.x * p_hit.y - p_hit.x * pr.y.atan2(p_hit.x * pr.x + p_hit.y * pr.y);
        if phi < 0.0 { phi += 2.0 * PI };

        // Test hyperboloid intersection against clipping parameters
        if p_hit.z < self.z_min || p_hit.z > self.z_max || phi > self.phi_max {
            if t_shape_hit == t1 { return None };
            t_shape_hit = t1;
            if t1.upper_bound() > ray.t_max { return None };
            // Compute hyperboloid inverse mapping
            p_hit = ray.point_at_time(t_shape_hit.v);
            v = (p_hit.z - self.p1.z) / (self.p2.z - self.p1.z);
            let pr = (1.0 - v) * self.p1 + v * self.p2;
            phi = (pr.x * p_hit.y - p_hit.x * pr.y).atan2(p_hit.x * pr.x + p_hit.y * pr.y);
            if phi < 0.0 {phi += 2.0 * PI};
            if p_hit.z < self.z_min || p_hit.z > self.z_max || phi > self.phi_max { return None; }
        }

        // Compute parametric representation of hyperboloid hit
        let u = phi / self.phi_max;

        // Compute hyperboloid $\dpdu$ and $\dpdv$
        let cos_phi = phi.cos();
        let sin_phi = phi.sin();
        let dpdu = Vector3f::new(-self.phi_max * p_hit.y, self.phi_max * p_hit.x, 0.0);
        let dpdv = Vector3f::new((self.p2.x - self.p1.x) * cos_phi - (self.p2.y - self.p1.y) * sin_phi,
                    (self.p2.x - self.p1.x) * sin_phi + (self.p2.y - self.p1.y) * cos_phi, self.p2.z - self.p1.z);

        // Compute hyperboloid $\dndu$ and $\dndv$
        let d2_pduu = -self.phi_max * self.phi_max * Vector3f::new(p_hit.x, p_hit.y, 0.0);
        let d2_pduv = self.phi_max * Vector3f::new(-dpdv.y, dpdv.x, 0.0);
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

        // Compute error bounds for intersection computed with ray equation
        let px = ox + t_shape_hit * dx;
        let py = oy + t_shape_hit * dy;
        let pz = oz + t_shape_hit * dz;
        let p_error = Vector3f::new(px.get_absolute_error(), py.get_absolute_error(),
                                pz.get_absolute_error());

        // Initialize _SurfaceInteraction_ from parametric information
        let isect = self.object_to_world.tranform_surface_interaction(&SurfaceInteraction::new(p_hit, p_error, Point2f::new(u, v),
                                                    -ray.d, dpdu, dpdv, dndu, dndv,
                                                    ray.time, Some(self), 0));
        Some((isect, t_shape_hit.v as Float))
    }

    fn intersect_p(&self, ray: &Ray, test_alpha_texture: bool) -> bool {
        let _p = Profiler::instance().profile("Hyperboloid::intersect_p");
        let mut phi: Float = 0.0;
        let mut p_hit: Point3f;
        // Transform _Ray_ to object space
        let mut o_err = Vector3f::default();
        let mut d_err = Vector3f::default();
        let ray = self.world_to_object.transform_ray_with_error(&ray, &mut o_err, &mut d_err);

        // Compute quadratic hyperboloid coefficients

        // Initialize _efloat_ ray coordinate values
        let ox = EFloat::new(ray.o.x, o_err.x);
        let oy = EFloat::new(ray.o.y, o_err.y);
        let oz = EFloat::new(ray.o.z, o_err.z);
        let dx = EFloat::new(ray.d.x, d_err.x);
        let dy = EFloat::new(ray.d.y, d_err.y);
        let dz = EFloat::new(ray.d.z, d_err.z);
        let a = dx * dx * self.ah + dy * dy * self.ah - dz * dz * self.ch;
        let b = 2.0 * (dx * ox * self.ah + dy * oy * self.ah - dz * oz * self.ch);
        let c = ox * ox * self.ah + oy * oy * self.ah - oz * oz - 1.0 * self.ch;

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

        // Compute hyperboloid inverse mapping
        p_hit = ray.point_at_time(t_shape_hit.v);
        let v = (p_hit.z - self.p1.z) / (self.p2.z - self.p1.z);
        let pr = self.p1 * (1.0 - v) + self.p2 * v;
        phi = pr.x * p_hit.y - p_hit.x * pr.y.atan2(p_hit.x * pr.x + p_hit.y * pr.y);
        if phi < 0.0 { phi += 2.0 * PI };

        // Test hyperboloid intersection against clipping parameters
        if p_hit.z < self.z_min || p_hit.z > self.z_max || phi > self.phi_max {
            if t_shape_hit == t1 { return false };
            t_shape_hit = t1;
            if t1.upper_bound() > ray.t_max { return false };
            // Compute hyperboloid inverse mapping
            p_hit = ray.point_at_time(t_shape_hit.v);
            v = (p_hit.z - self.p1.z) / (self.p2.z - self.p1.z);
            let pr = (1.0 - v) * self.p1 + v * self.p2;
            phi = (pr.x * p_hit.y - p_hit.x * pr.y).atan2(p_hit.x * pr.x + p_hit.y * pr.y);
            if phi < 0.0 {phi += 2.0 * PI};
            if p_hit.z < self.z_min || p_hit.z > self.z_max || phi > self.phi_max { return false; }
        }
        true
    }

    fn area(&self) -> Float {
        self.phi_max / 6.0 *
           (2.0 * quad!(self.p1.x) - 2.0 * self.p1.x * self.p1.x * self.p1.x * self.p2.x + 2.0 * quad!(self.p2.x) +
            2.0 * (self.p1.y * self.p1.y + self.p1.y * self.p2.y + self.p2.y * self.p2.y) *
                (sqr!(self.p1.y - self.p2.y) + sqr!(self.p1.z - self.p2.z)) +
            self.p2.x * self.p2.x * (5.0 * self.p1.y * self.p1.y + 2.0 * self.p1.y * self.p2.y - 4.0 * self.p2.y * self.p2.y +
                           2.0 * sqr!(self.p1.z - self.p2.z)) +
            self.p1.x * self.p1.x * (-4.0 * self.p1.y * self.p1.y + 2.0 * self.p1.y * self.p2.y +
                           5.0 * self.p2.y * self.p2.y + 2.0 * sqr!(self.p1.z - self.p2.z)) -
            2.0 * self.p1.x * self.p2.x *
                (self.p2.x * self.p2.x - self.p1.y * self.p1.y + 5.0 * self.p1.y * self.p2.y - self.p2.y * self.p2.y -
                 self.p1.z * self.p1.z + 2.0 * self.p1.z * self.p2.z - self.p2.z * self.p2.z))
    }

    fn sample(&self, u: &Point2f, pdf: &mut Float) -> Box<dyn Interaction> {
        error!("Hyperboloid does not implement sample");
        Box::new(SimpleInteraction::default())
    }
}