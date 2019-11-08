use std::sync::Arc;
use crate::core::pbrt::{Float, consts::PI, gamma, radians};
use crate::core::shape::Shape;
use crate::core::geometry::{Bounding3, Bounds3f, Ray, Point2f, Point3f, Normal3f, Vector3f, offset_ray_origin, dot_normal_vec, coordinate_system, spherical_direction_vec3};
use crate::core::transform::Transform;
use crate::core::material::Material;
use crate::core::sampling::{uniform_sample_sphere, uniform_cone_pdf};
use crate::core::interaction::*;
use crate::core::surface_interaction::SurfaceInteraction;
use num::clamp;
use crate::core::efloat::{EFloat, quadratic};
use crate::core::profiler::Profiler;

#[derive(Clone)]
pub struct Sphere {
    pub radius: Float,
    pub z_min: Float,
    pub z_max: Float,
    pub theta_min: Float,
    pub theta_max: Float,
    pub phi_max: Float,
    // inherited from class Shape (see shape.h)
    object_to_world: Transform,
    world_to_object: Transform,
    reverse_orientation: bool,
    transform_swaps_handedness: bool,
    pub material: Option<Arc<dyn Material + Send + Sync>>,
}

impl Default for Sphere{
    fn default() -> Sphere {
        let object_to_world = Transform::default();
        Sphere {
            object_to_world,
            world_to_object: Transform::default(),
            radius: 1.0,
            z_min: -1.0,
            z_max: 1.0,
            theta_min: (-1.0 as Float).acos(),
            theta_max: (1.0 as Float).acos(),
            phi_max: PI * 2.0,
            reverse_orientation: false,
            transform_swaps_handedness: object_to_world.swaps_handness(),
            material: None
        }
    }
}

impl Sphere {
    fn new(
        object_to_world: Transform,
        world_to_object: Transform,
        reverse_orientation: bool,
        radius: Float,
        z_min: Float,
        z_max: Float,
        phi_max: Float
    ) -> Sphere {
        let min = z_min.min(z_max);
        let max = z_min.max(z_max);
        Sphere{
            object_to_world,
            world_to_object,
            reverse_orientation,
            radius,
            z_min: clamp(min, -radius, radius),
            z_max: clamp(max, -radius, radius),
            theta_min: clamp(min / radius, -1.0, 1.0).acos(),
            theta_max: clamp(max / radius, -1.0, 1.0).acos(),
            phi_max: radians(clamp(phi_max, 0.0, 360.0)),
            transform_swaps_handedness: object_to_world.swaps_handness(),
            material: None
        }
    }
}

impl Shape for Sphere {
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
        let _p = Profiler::instance().profile("Sphere::intersect");
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
        let oz = EFloat::new(ray.o.z, o_err.z);
        let dx = EFloat::new(ray.d.x, d_err.x);
        let dy = EFloat::new(ray.d.y, d_err.y);
        let dz = EFloat::new(ray.d.z, d_err.z);
        let a = dx * dx + dy * dy + dz * dz;
        let b = (dx * ox + dy * oy + dz * oz) * 2.0;
        let c = ox * ox + oy * oy + oz * oz - EFloat::new(self.radius, 0.0) * EFloat::new(self.radius, 0.0);

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

        // Compute sphere hit position and $\phi$
        p_hit = ray.point_at_time(t_shape_hit.v);

        // Refine sphere intersection point
        p_hit *= self.radius / p_hit.distance(&Point3f::new(0.0, 0.0, 0.0));
        if p_hit.x == 0.0 && p_hit.y == 0.0 {
            p_hit.x = 1e-5 * self.radius;
        }
        phi = p_hit.y.atan2(p_hit.x);
        if phi < 0.0 {
            phi += 2.0 * PI;
        }

        // Test sphere intersection against clipping parameters
        if (self.z_min > -self.radius && p_hit.z < self.z_min)
            || (self.z_max < self.radius && p_hit.z > self.z_max)
            || phi > self.phi_max
        {
            if t_shape_hit == t1 {
                return None;
            }
            if t1.upper_bound() > ray.t_max {
                return None;
            }
            t_shape_hit = t1;
            // Compute sphere hit position and $\phi$
            p_hit = ray.point_at_time(t_shape_hit.v);

            // Refine sphere intersection point
            p_hit *= self.radius / p_hit.distance(&Point3f::default());
            if p_hit.x == 0.0 && p_hit.y == 0.0 {
                p_hit.x = 1e-5 * self.radius;
            }
            phi = p_hit.y.atan2(p_hit.x);
            if phi < 0.0 {
                phi += 2.0 * PI;
            }
            if self.z_min > -self.radius && p_hit.z < self.z_min
            || (self.z_max < self.radius && p_hit.z > self.z_max)
            || phi > self.phi_max {
                return None;
            }
        }

        // find parametric representation of sphere hit
        let u = phi / self.phi_max;
        let theta = clamp(p_hit.z / self.radius, -1.0, 1.0).acos();
        let v = (theta - self.theta_min) / (self.theta_max - self.theta_min);

        // Compute sphere $\dpdu$ and $\dpdv$
        let z_radius = (p_hit.x * p_hit.x + p_hit.y * p_hit.y).sqrt();
        let inv_z_radius = 1.0 / z_radius;
        let cos_phi = p_hit.x * inv_z_radius;
        let sin_phi = p_hit.y * inv_z_radius;
        let dpdu = Vector3f::new(-self.phi_max * p_hit.y, self.phi_max * p_hit.x, 0.0);
        let dpdv = (self.theta_max - self.theta_min) *
            Vector3f::new(p_hit.z * cos_phi, p_hit.z * sin_phi, -self.radius * theta.sin());

        // Compute sphere $\dndu$ and $\dndv$
        let d2_pduu = -self.phi_max * self.phi_max * Vector3f::new(p_hit.x, p_hit.y, 0.0);
        let d2_pduv = (self.theta_max - self.theta_min) * p_hit.z * self.phi_max * Vector3f::new(-sin_phi, cos_phi, 0.);
        let d2_pdvv = -(self.theta_max - self.theta_min) * (self.theta_max - self.theta_min) *
                    Vector3f::new(p_hit.x, p_hit.y, p_hit.z);

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

        // Compute error bounds for sphere intersection
        let p_error = gamma(5) * Vector3f::from(p_hit).abs();

        // Initialize _SurfaceInteraction_ from parametric information
        let isect = self.object_to_world.tranform_surface_interaction(&SurfaceInteraction::new(&p_hit, &p_error, &Point2f::new(u, v),
                                                    &-ray.d,&-dpdu,&-dpdv,&-dndu,&-dndv,
                                                    ray.time, Some(self), 0));

        // Update _tHit_ for quadric intersection
        Some((isect, t_shape_hit.v as Float))
    }

    fn intersect_p(&self, ray: &Ray, test_alpha_texture: bool) -> bool {
        let _p = Profiler::instance().profile("Sphere::intersect_p");
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
        let oz = EFloat::new(ray.o.z, o_err.z);
        let dx = EFloat::new(ray.d.x, d_err.x);
        let dy = EFloat::new(ray.d.y, d_err.y);
        let dz = EFloat::new(ray.d.z, d_err.z);
        let a = dx * dx + dy * dy + dz * dz;
        let b = (dx * ox + dy * oy + dz * oz) * 2.0;
        let c = ox * ox + oy * oy + oz * oz - EFloat::new(self.radius, 0.0) * EFloat::new(self.radius, 0.0);

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

        // Compute sphere hit position and $\phi$
        p_hit = ray.point_at_time(t_shape_hit.v);

        // Refine sphere intersection point
        p_hit *= self.radius / p_hit.distance(&Point3f::new(0.0, 0.0, 0.0));
        if p_hit.x == 0.0 && p_hit.y == 0.0 {
            p_hit.x = 1e-5 * self.radius;
        }
        phi = p_hit.y.atan2(p_hit.x);
        if phi < 0.0 {
            phi += 2.0 * PI;
        }

        // Test sphere intersection against clipping parameters
        if (self.z_min > -self.radius && p_hit.z < self.z_min)
            || (self.z_max < self.radius && p_hit.z > self.z_max)
            || phi > self.phi_max
        {
            if t_shape_hit == t1 {
                return false;
            }
            if t1.upper_bound() > ray.t_max {
                return false;
            }
            t_shape_hit = t1;
            // Compute sphere hit position and $\phi$
            p_hit = ray.point_at_time(t_shape_hit.v);

            // Refine sphere intersection point
            p_hit *= self.radius / p_hit.distance(&Point3f::default());
            if p_hit.x == 0.0 && p_hit.y == 0.0 {
                p_hit.x = 1e-5 * self.radius;
            }
            phi = p_hit.y.atan2(p_hit.x);
            if phi < 0.0 {
                phi += 2.0 * PI;
            }
            if self.z_min > -self.radius && p_hit.z < self.z_min
            || (self.z_max < self.radius && p_hit.z > self.z_max)
            || phi > self.phi_max {
                return false;
            }
        }
        true
    }

    fn area(&self) -> Float {
        self.phi_max * self.radius * (self.z_max - self.z_min)
    }

    fn sample(&self, u: &Point2f, pdf: &mut Float) -> Box<dyn Interaction> {
        let mut p_obj = Point3f::new(0.0, 0.0, 0.0) + self.radius * uniform_sample_sphere(u);
        let mut it = SimpleInteraction::default();
        it.n = self.object_to_world.transform_normal(&Normal3f::new(p_obj.x, p_obj.y, p_obj.z)).normalize();
        if self.reverse_orientation { it.n *= -1.0; }
        // Reproject _p_obj_ to sphere surface and compute _p_obj_error_
        p_obj *= self.radius / p_obj.distance(&Point3f::default());
        let p_obj_error = gamma(5) * Vector3f::from(p_obj).abs();
        it.p = self.object_to_world.transform_point_with_abs_error(&p_obj, &p_obj_error, &mut it.p_error);
        *pdf = 1.0 / self.area();
        Box::new(it)
    }

    fn sample_with_point(&self, iref: &dyn Interaction, u: &Point2f, pdf: &mut Float) -> Box<dyn Interaction> {
        let p_center = self.object_to_world.transform_point(&Point3f::default());

        // Sample uniformly on sphere if $\pt{}$ is inside it
        let p_origin =
            offset_ray_origin(&iref.get_p(), &iref.get_p_error(), &iref.get_n(), &(p_center - iref.get_p()));
        if p_origin.distance_squared(&p_center) <= self.radius * self.radius {
            let intr = self.sample(&u, pdf);
            let wi = intr.get_p() - iref.get_p();
            if wi.length_squared() == 0.0 {
                *pdf = 0.0;
            }
            else {
                // Convert from area measure returned by Sample() call above to
                // solid angle measure.
                wi = wi.normalize();
                *pdf *= iref.get_p().distance_squared(&intr.get_p()) / dot_normal_vec(&intr.get_n(), &-wi).abs();
            }
            if pdf.is_infinite() {
                *pdf = 0.0;
                }
            return intr;
        }

        // Sample sphere uniformly inside subtended cone

        // Compute coordinate system for sphere sampling
        let dc = iref.get_p().distance(&p_center);
        let inv_dc = 1.0 / dc;
        let wc = (p_center - iref.get_p()) * inv_dc;
        let wc_x: Vector3f;
        let wc_y: Vector3f;
        coordinate_system(&wc, &mut wc_x, &mut wc_y);

        // Compute $\theta$ and $\phi$ values for sample in cone
        let sin_theta_max = self.radius * inv_dc;
        let sin_theta_max2 = sin_theta_max * sin_theta_max;
        let inv_sin_theta_max = 1.0 / sin_theta_max;
        let cos_theta_max = (1.0 - sin_theta_max2).max(0.0).sqrt();

        let cos_theta  = (cos_theta_max - 1.0) * u.x + 1.0;
        let sin_theta2 = 1.0 - cos_theta * cos_theta;

        if sin_theta_max2 < 0.000685230 /* sin^2(1.5 deg) */ {
            /* Fall back to a Taylor series expansion for small angles, where
            the standard approach suffers from severe cancellation errors */
            sin_theta2 = sin_theta_max2 * u.x;
            cos_theta = (1.0 - sin_theta2).sqrt();
        }

        // Compute angle $\alpha$ from center of sphere to sampled point on surface
        let cos_alpha = sin_theta2 * inv_sin_theta_max +
            cos_theta * (1.0 - sin_theta2 * inv_sin_theta_max * inv_sin_theta_max).max(0.0).sqrt();
        let sin_alpha = (1.0 - cos_alpha*cos_alpha).max(0.0).sqrt();
        let phi = u.y * 2.0 * PI;

        // Compute surface normal and sampled point on sphere
        let n_world =
            spherical_direction_vec3(sin_alpha, cos_alpha, phi, &-wc_x, &-wc_y, &-wc);
        let p_world = p_center + Point3f::new(n_world.x, n_world.y, n_world.z) * self.radius;

        // Return _Interaction_ for sampled point on sphere
        let mut it = SimpleInteraction::default();
        it.p = p_world;
        it.p_error = gamma(5) * Vector3f::from(p_world).abs();
        it.n = n_world.into();
        if self.reverse_orientation { it.n *= -1.0; }

        // Uniform cone PDF.
        *pdf = 1.0 / (2.0 * PI * (1.0 - cos_theta_max));

        Box::new(it)
    }

    fn pdf_with_point(&self, iref: &dyn Interaction, wi: &Vector3f) -> Float {
        let p_center = self.object_to_world.transform_point(&Point3f::default());
        // Return uniform PDF if point is inside sphere
        let p_origin =
            offset_ray_origin(&iref.get_p(), &iref.get_p_error(), &iref.get_n(), &(p_center - iref.get_p()));
        if p_origin.distance_squared(&p_center) <= self.radius * self.radius {
            return Shape::pdf_with_point(self, iref, wi);
        }

        // Compute general sphere PDF
        let sin_theta_max2 = self.radius * self.radius / iref.get_p().distance_squared(&p_center);
        let cos_theta_max = (1.0 - sin_theta_max2).max(0.0).sqrt();
        uniform_cone_pdf(cos_theta_max)
    }

    fn solid_angle(&self, p: &Point3f, n_samples: i32) -> Float {
        let p_center = self.object_to_world.transform_point(&Point3f::default());
        if p.distance_squared(&p_center) <= self.radius * self.radius {
            return 4.0 * PI;
        }
        let sin_theta2 = self.radius * self.radius / p.distance_squared(&p_center);
        let cos_theta = (1.0 - sin_theta2).max(0.0).sqrt();
        (2.0 * PI * (1.0 - cos_theta))
    }
}