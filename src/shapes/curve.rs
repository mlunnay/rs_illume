use crate::core::pbrt::{Float, lerp, float_to_bits};
use crate::core::geometry::{Bounding3, Vector3f, Vector2f, Ray, Point2f, Point3f, Normal3f, Bounds3f, coordinate_system, dot_normal_vec};
use crate::core::interaction::SurfaceInteraction;
use crate::core::interaction::{Interaction, SimpleInteraction};
use crate::core::transform::Transform;
use crate::core::material::Material;
use crate::core::profiler::Profiler;
use crate::core::stats_accumulator::StatsAccumulator;
use crate::core::shape::Shape;
use std::sync::Arc;
use num::clamp;

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum CurveType {
    Flat,
    Cylinder,
    Ribbon
}

#[derive(Clone)]
pub struct CurveCommon {
    pub curve_type: CurveType,
    pub cp_obj: [Point3f; 4],
    pub width: [Float; 2],
    pub n: [Normal3f; 2],
    pub normal_angle: Float,
    pub inv_sin_normal_angle: Float,
}

impl CurveCommon {
    pub fn new(
        c: &[Point3f; 4],
        width0: Float,
        width1: Float,
        curve_type: CurveType,
        norm: Option<[Normal3f; 2]>,
    ) -> Self {
        StatsAccumulator::instance().report_counter(String::from("Scene/Curves"), 1);
        if let Some(norm) = norm {
            let n0: Normal3f = norm[0].normalize();
            let n1: Normal3f = norm[1].normalize();
            let normal_angle: Float =
                clamp(n0.dot(&n1), 0.0, 1.0).acos();
            let inv_sin_normal_angle: Float = 1.0 / normal_angle.sin();
            CurveCommon {
                curve_type,
                cp_obj: [c[0], c[1], c[2], c[3]],
                width: [width0, width1],
                n: [n0, n1],
                normal_angle,
                inv_sin_normal_angle,
            }
        } else {
            CurveCommon {
                curve_type,
                cp_obj: [c[0], c[1], c[2], c[3]],
                width: [width0, width1],
                n: [Normal3f::default(); 2],
                normal_angle: 0.0,
                inv_sin_normal_angle: 0.0,
            }
        }
    }
}

pub struct Curve {
    common: Arc<CurveCommon>,
    u_min: Float,
    u_max: Float,
    // inherited from class Shape (see shape.h)
    pub object_to_world: Transform,
    pub world_to_object: Transform,
    pub reverse_orientation: bool,
    pub transform_swaps_handedness: bool,
    pub material: Option<Arc<dyn Material + Send + Sync>>
}

impl Curve {
    pub fn new(
        object_to_world: Transform,
        world_to_object: Transform,
        reverse_orientation: bool,
        common: Arc<CurveCommon>,
        u_min: Float,
        u_max: Float
    ) -> Curve {
        Curve {
            common,
            u_min,
            u_max,
            object_to_world,
            world_to_object,
            reverse_orientation,
            transform_swaps_handedness: object_to_world.swaps_handness(),
            material: None
        }
    }

    pub fn recursive_intersect(
        &self,
        ray: &Ray,
        cp: &[Point3f; 4],
        ray_to_object: &Transform,
        u0: Float,
        u1: Float,
        depth: u32
    ) -> Option<(SurfaceInteraction, Float)> {
        let ray_length = ray.d.length();

        if depth > 0 {
            // Split curve segment into sub-segments and test for intersection
            let cp_split = [Point3f::default(); 7];
            subdivide_bezier(cp, &cp_split);

            // For each of the two segments, see if the ray's bounding box
            // overlaps the segment before recursively checking for
            // intersection with it.
            let hit: Option<(SurfaceInteraction, Float)> = None;
            let u = [u0, (u0 + u1) / 2.0, u1];
            // Pointer to the 4 control poitns for the current segment.
            for seg in 0..2 {
                let cps: &[Point3f] = &cp[seg * 3..seg * 3 + 4];
                let max_width =
                    lerp(u[seg], self.common.width[0], self.common.width[1]).max(
                        lerp(u[seg + 1], self.common.width[0], self.common.width[1]));

                // As above, check y first, since it most commonly lets us exit
                // out early.
                if cps[0].y.max(cps[1].y).max(cps[2].y.max(cps[3].y)) +
                        0.5 * max_width < 0.0 ||
                    cps[0].y.min(cps[1].y).min(cps[2].y.min(cps[3].y)) -
                        0.5 * max_width > 0.0 {
                        continue;
                    }
                if cps[0].x.max(cps[1].x).max(cps[2].x.max(cps[3].x)) +
                        0.5 * max_width < 0.0 ||
                    cps[0].x.min(cps[1].x).min(cps[2].x.min(cps[3].x)) -
                        0.5 * max_width > 0.0 {
                    continue;
                }

                let z_max = ray_length * ray.t_max;
                if cps[0].z.max(cps[1].z).max(cps[2].z.max(cps[3].z)) +
                        0.5 * max_width < 0.0 ||
                    cps[0].z.min(cps[1].z).min(cps[2].z.min(cps[3].z)) -
                        0.5 * max_width > z_max {
                    continue;
                }

                hit = self.recursive_intersect(&ray, &[cps[0], cps[1], cps[2], cps[3]], ray_to_object,
                                        u[seg], u[seg + 1], depth - 1);
                // If we found an intersection and this is a shadow ray,
                // we can exit out immediately.
                if let Some((isect, t_hit)) = hit {
                    if t_hit == 0.0 {
                        return hit;
                    }
                }
            }
            hit
        } else {
            // Intersect ray with curve segment

            // Test ray against segment endpoint boundaries

            // Test sample point against tangent perpendicular at curve start
            let edge =
                (cp[1].y - cp[0].y) * -cp[0].y + cp[0].x * (cp[0].x - cp[1].x);
            if edge < 0.0 {
                return None;
            }

            // Test sample point against tangent perpendicular at curve end
            edge = (cp[2].y - cp[3].y) * -cp[3].y + cp[3].x * (cp[3].x - cp[2].x);
            if edge < 0.0 {
                return None;
            }

            // Compute line $w$ that gives minimum distance to sample point
            let segment_direction = Point2f::from(cp[3]) - Point2f::from(cp[0]);
            let denom = segment_direction.length_squared();
            if denom == 0.0 {
                return None;
            }
            let w = Vector2f{x: -cp[0].x, y: -cp[0].y}.dot(&segment_direction) / denom;

            // Compute $u$ coordinate of curve intersection point and _hit_width_
            let u = clamp(lerp(w, u0, u1), u0, u1);
            let hit_width = lerp(u, self.common.width[0], self.common.width[1]);
            let n_hit ;
            if self.common.curve_type == CurveType::Ribbon {
                // Scale _hit_width_ based on ribbon orientation
                let sin0 = ((1.0 - u) * self.common.normal_angle).sin() *
                            self.common.inv_sin_normal_angle;
                let sin1 =
                    (u * self.common.normal_angle).sin() * self.common.inv_sin_normal_angle;
                n_hit = sin0 * self.common.n[0] + sin1 * self.common.n[1];
                hit_width *= dot_normal_vec(&n_hit, &ray.d).abs() / ray_length;
            }

            // Test intersection point against curve width
            let dpcdw = Vector3f::default();
            let pc = eval_bezier(cp, clamp(w, 0.0, 1.0), Some(&mut dpcdw));
            let pt_curve_dist2 = pc.x * pc.x + pc.y * pc.y;
            if pt_curve_dist2 > hit_width * hit_width * 0.25 {
                return None;
            }
            let z_max = ray_length * ray.t_max;
            if pc.z < 0.0 || pc.z > z_max {
                return None;
            }

            // Compute $v$ coordinate of curve intersection point
            let pt_curve_dist = pt_curve_dist2.sqrt();
            let edge_func = dpcdw.x * -pc.y + pc.x * dpcdw.y;
            let v = if edge_func > 0.0 { 0.5 + pt_curve_dist / hit_width }
                else { 0.5 - pt_curve_dist / hit_width };

            // Compute hit _t_ and partial derivatives for curve intersection
            // FIXME: this tHit isn't quite right for ribbons...
            let t_hit = pc.z / ray_length;
            // Compute error bounds for curve intersection
            let p_error = Vector3f::new(2.0 * hit_width, 2.0 * hit_width, 2.0 * hit_width);

            // Compute $\dpdu$ and $\dpdv$ for curve intersection
            let dpdu = Vector3f::default();
            let dpdv = Vector3f::default();
            eval_bezier(&self.common.cp_obj, u, Some(&mut dpdu));

            if self.common.curve_type == CurveType::Ribbon {
                dpdv = dpdu.cross(&Vector3f::from(n_hit)).normalize() * hit_width;
            }
            else {
                // Compute curve $\dpdv$ for flat and cylinder curves
                let dpdu_plane = ray_to_object.inverse().transform_vector(&dpdu);
                let dpdv_plane =
                    Vector3f::new (-dpdu_plane.y, dpdu_plane.x, 0.0).normalize() *
                    hit_width;
                if self.common.curve_type == CurveType::Cylinder {
                    // Rotate _dpdv_plane_ to give cylindrical appearance
                    let theta = lerp(v, -90., 90.);
                    let rot = Transform::rotate(-theta, &dpdu_plane);
                    dpdv_plane = rot.transform_vector(&dpdv_plane);
                }
                dpdv = ray_to_object.transform_vector(&dpdv_plane);
            }
            let isect = self.object_to_world.tranform_surface_interaction(&SurfaceInteraction::new(
                ray.point_at_time(t_hit), p_error, Point2f::new(u, v), -ray.d, dpdu, dpdv,
                Normal3f::default(), Normal3f::default(), ray.time, Some(self), 0));
            StatsAccumulator::instance().report_percentage(String::from("Intersections/Ray-curve intersection tests"), 1, 0);
            Some((isect, t_hit))
        }
    }
}

impl Shape for Curve {
    fn object_bound(&self) -> Box<dyn Bounding3<Float>> {
        // Compute object-space control points for curve segment, _cp_obj_
        let cp_obj = [Point3f::default(); 4];
        cp_obj[0] = blossom_bezier(&self.common.cp_obj, self.u_min, self.u_min, self.u_min);
        cp_obj[1] = blossom_bezier(&self.common.cp_obj, self.u_min, self.u_min, self.u_max);
        cp_obj[2] = blossom_bezier(&self.common.cp_obj, self.u_min, self.u_max, self.u_max);
        cp_obj[3] = blossom_bezier(&self.common.cp_obj, self.u_max, self.u_max, self.u_max);
        let b =
            Bounds3f::new(cp_obj[0], cp_obj[1]).union(&Bounds3f::new(cp_obj[2], cp_obj[3]));
        let width = [lerp(self.u_min, self.common.width[0], self.common.width[1]),
                    lerp(self.u_max, self.common.width[0], self.common.width[1])];
        Box::new(b.aabb().expand(width[0].max(width[1]) * 0.5))
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
        let _p = Profiler::instance().profile("Curve::intersect");
        StatsAccumulator::instance().report_percentage(String::from("Intersections/Ray-curve intersection tests"), 0, 1);
        // Transform _Ray_ to object space
        let o_err = Vector3f::default();
        let d_err = Vector3f::default();
        let ray = self.world_to_object.transform_ray_with_error(&ray, &mut o_err, &mut d_err);

        // Compute object-space control points for curve segment, _cp_obj_
        let cp_obj = [
            blossom_bezier(&self.common.cp_obj, self.u_min, self.u_min, self.u_min),
            blossom_bezier(&self.common.cp_obj, self.u_min, self.u_min, self.u_max),
            blossom_bezier(&self.common.cp_obj, self.u_min, self.u_max, self.u_max),
            blossom_bezier(&self.common.cp_obj, self.u_max, self.u_max, self.u_max)
            ];

        // Project curve control points to plane perpendicular to ray

        // Be careful to set the "up" direction passed to LookAt() to equal the
        // vector from the first to the last control points.  In turn, this
        // helps orient the curve to be roughly parallel to the x axis in the
        // ray coordinate system.
        //
        // In turn (especially for curves that are approaching stright lines),
        // we get curve bounds with minimal extent in y, which in turn lets us
        // early out more quickly in recursiveIntersect().
        let dx = ray.d.cross(&(cp_obj[3] - cp_obj[0]));
        if dx.length_squared() == 0.0 {
            // If the ray and the vector between the first and last control
            // points are parallel, dx will be zero.  Generate an arbitrary xy
            // orientation for the ray coordinate system so that intersection
            // tests can proceeed in this unusual case.
            let dy ;
            coordinate_system(&ray.d, &mut dx, &mut dy);
        }

        let object_to_ray = Transform::look_at(&ray.o, &(ray.o + ray.d), &dx);
        let cp = [object_to_ray.transform_point(&cp_obj[0]), object_to_ray.transform_point(&cp_obj[1]),
            object_to_ray.transform_point(&cp_obj[2]), object_to_ray.transform_point(&cp_obj[3])];

        // Before going any further, see if the ray's bounding box intersects
        // the curve's bounding box. We start with the y dimension, since the y
        // extent is generally the smallest (and is often tiny) due to our
        // careful orientation of the ray coordinate ysstem above.
        let max_width = lerp(self.u_min, self.common.width[0], self.common.width[1]).max(
            lerp(self.u_max, self.common.width[0], self.common.width[1]));
        if cp[0].y.max(cp[1].y).max(cp[2].y.max(cp[3].y)) +
                0.5 * max_width < 0.0 ||
            cp[0].y.min(cp[1].y).min(cp[2].y.min(cp[3].y)) -
                0.5 * max_width > 0.0 {
                return None;
            }

        // Check for non-overlap in x.
        if cp[0].x.max(cp[1].x).max(cp[2].x.max(cp[3].x)) +
                0.5 * max_width < 0.0 ||
            cp[0].x.min(cp[1].x).min(cp[2].x.min(cp[3].x)) -
                0.5 * max_width > 0.0 {
            return None;
        }

        // Check for non-overlap in z.
        let ray_length = ray.d.length();
        let z_max = ray_length * ray.t_max;
        if cp[0].z.max(cp[1].z).max(cp[2].z.max(cp[3].z)) +
                0.5 * max_width < 0.0 ||
            cp[0].z.min(cp[1].z).min(cp[2].z.min(cp[3].z)) -
                0.5 * max_width > z_max {
                return None;
            }

        // Compute refinement depth for curve, _max_depth_
        let mut l0: Float = 0.0;
        for i in 0..2 {
            l0 = l0.max( 
                (cp[i].x - 2.0 * cp[i + 1].x + cp[i + 2].x).abs().max(
                        (cp[i].y - 2.0 * cp[i + 1].y + cp[i + 2].y).abs()).max(
                (cp[i].z - 2.0 * cp[i + 1].z + cp[i + 2].z).abs()));
        }

        let eps = self.common.width[0].max(self.common.width[1]) * 0.05;  // width / 20
        let log2 = |v: f32| {
            if v < 1.0 {
                return 0;
            }
            let bits = float_to_bits(v);
            // https://graphics.stanford.edu/~seander/bithacks.html#IntegerLog
            // (With an additional add so get round-to-nearest rather than
            // round down.)
            (bits >> 23) - 127 + if bits & (1 << 22) != 0 { 1 } else { 0 }
        };
        // Compute log base 4 by dividing log2 in half.
        let r0 = log2(1.41421356237 * 6.0 * l0 / (8.0 * eps)) / 2;
        let max_depth = clamp(r0, 0, 10);
        StatsAccumulator::instance().report_int_distribution(String::from("Intersections/Curve refinement level"), max_depth as i64);

        self.recursive_intersect(&ray, &cp, &object_to_ray.inverse(), self.u_min,
                                self.u_max, max_depth)
    }

    fn area(&self) -> Float {
        // Compute object-space control points for curve segment, _cpObj_
        let cp_obj = [
            blossom_bezier(&self.common.cp_obj, self.u_min, self.u_min, self.u_min),
            blossom_bezier(&self.common.cp_obj, self.u_min, self.u_min, self.u_max),
            blossom_bezier(&self.common.cp_obj, self.u_min, self.u_max, self.u_max),
            blossom_bezier(&self.common.cp_obj, self.u_max, self.u_max, self.u_max)
            ];
        let width0 = lerp(self.u_min, self.common.width[0], self.common.width[1]);
        let width1 = lerp(self.u_max, self.common.width[0], self.common.width[1]);
        let avg_width = (width0 + width1) * 0.5;
        let approxLength = 0.0;
        for i in 0..3 {
            approxLength += cp_obj[i].distance(&cp_obj[i + 1]);
        }
        approxLength * avg_width
    }

    fn sample(&self, u: &Point2f, pdf: &mut Float) -> Box<dyn Interaction> {
        error!("Curve does not implement sample");
        Box::new(SimpleInteraction::default())
    }
}

fn blossom_bezier(p: &[Point3f; 4], u0: Float, u1: Float, u2: Float) -> Point3f {
    let a = [lerp(u0, p[0], p[1]), lerp(u0, p[1], p[2]),
                    lerp(u0, p[2], p[3])];
    let b = [lerp(u1, a[0], a[1]), lerp(u1, a[1], a[2])];
    b[0].lerp(&b[1], u2)
}

fn subdivide_bezier(cp: &[Point3f; 4], cp_split: &[Point3f; 7]) {
    cp_split[0] = cp[0];
    cp_split[1] = (cp[0] + cp[1]) / 2.0;
    cp_split[2] = (cp[0] + 2.0 * cp[1] + cp[2]) / 4.0;
    cp_split[3] = (cp[0] + 3.0 * cp[1] + 3.0 * cp[2] + cp[3]) / 8.0;
    cp_split[4] = (cp[1] + 2.0 * cp[2] + cp[3]) / 4.0;
    cp_split[5] = (cp[2] + cp[3]) / 2.0;
    cp_split[6] = cp[3];
}

fn eval_bezier(
    cp: &[Point3f; 4],
    u: Float,
    deriv: Option<&mut Vector3f>
) -> Point3f {
    let cp1 = [lerp(u, cp[0], cp[1]), lerp(u, cp[1], cp[2]),
                      lerp(u, cp[2], cp[3])];
    let cp2 = [lerp(u, cp1[0], cp1[1]), lerp(u, cp1[1], cp1[2])];
    if let Some(deriv) = deriv {
        if (cp2[1] - cp2[0]).length_squared() > 0.0 {
            *deriv = 3.0 * (cp2[1] - cp2[0]);
        }
        else {
            // For a cubic Bezier, if the first three control points (say) are
            // coincident, then the derivative of the curve is legitimately (0,0,0)
            // at u=0.  This is problematic for us, though, since we'd like to be
            // able to compute a surface normal there.  In that case, just punt and
            // take the difference between the first and last control points, which
            // ain't great, but will hopefully do.
            *deriv = cp[3] - cp[0];
        }
    }
    cp2[0].lerp(&cp2[1], u)
}