use crate::core::pbrt::{Float, gamma, consts::PI};
use crate::core::geometry::{Bounding3, Vector3f, Ray, Point2f, Point3f, Normal3f, Bounds3f, Union, coordinate_system};
use crate::core::surface_interaction::SurfaceInteraction;
use crate::core::interaction::{Interaction, SimpleInteraction};
use crate::core::texture::Texture;
use crate::core::transform::Transform;
use crate::core::material::Material;
use crate::core::profiler::Profiler;
use crate::core::stats_accumulator::StatsAccumulator;
use crate::core::shape::Shape;
use crate::core::sampling::uniform_sample_triangle;
use std::sync::Arc;
use std::mem::size_of;
use num::clamp;

#[derive(Clone)]
pub struct TriangleMesh {
    pub n_triangles: u32,
    pub vertex_indicies: Vec<u32>,
    pub p: Vec<Point3f>,
    pub n: Vec<Normal3f>,
    pub s: Vec<Vector3f>,
    pub uv: Vec<Point2f>,
    pub alpha_mask: Option<Arc<dyn Texture<Float> + Send + Sync>>,
    pub shadow_alpha_mask: Option<Arc<dyn Texture<Float> + Send + Sync>>,
    pub face_indices: Vec<u32>,
    // inherited from class Shape (see shape.h)
    pub object_to_world: Transform,
    pub world_to_object: Transform,
    pub reverse_orientation: bool,
    pub transform_swaps_handedness: bool,
    pub material: Option<Arc<dyn Material + Send + Sync>>,
}

impl TriangleMesh {
    pub fn new(
        object_to_world: Transform,
        world_to_object: Transform,
        reverse_orientation: bool,
        n_triangles: u32,
        vertex_indicies: Vec<u32>,
        p: Vec<Point3f>,
        n: Vec<Normal3f>,
        s: Vec<Vector3f>,
        uv: Vec<Point2f>,
        alpha_mask: Option<Arc<dyn Texture<Float> + Send + Sync>>,
        shadow_alpha_mask: Option<Arc<dyn Texture<Float> + Send + Sync>>,
        face_indices: Vec<u32>,
    ) -> TriangleMesh {
        // Transform mesh vertices to world space
        let p: Vec<_> = p.iter().map(|pt| object_to_world.transform_point(pt)).collect();
        let n: Vec<_> = n.iter().map(|norm| object_to_world.transform_normal(norm)).collect();
        let s: Vec<_> = s.iter().map(|t| object_to_world.transform_vector(t)).collect();

        StatsAccumulator::instance().report_ratio(String::from("Scene/Triangles per triangle mesh"), n_triangles as i64, 1);
        
        TriangleMesh {
            object_to_world,
            world_to_object,
            reverse_orientation,
            transform_swaps_handedness: object_to_world.swaps_handness(),
            n_triangles,
            vertex_indicies,
            p,
            n,
            s,
            uv,
            alpha_mask,
            shadow_alpha_mask,
            face_indices,
            material: None
        }
    }
}

#[derive(Clone)]
pub struct Triangle {
    pub mesh: Arc<TriangleMesh>,
    index: u32,
    face_index: u32,
}

impl Triangle {
    pub fn new(
        mesh: Arc<TriangleMesh>,
        index: u32,
        face_index: u32
    ) -> Triangle {
        Triangle {
            mesh,
            index,
            face_index
        }
    }
    
    pub fn get_uvs(&self) -> [Point2f; 3] {
        if self.mesh.uv.is_empty() {
            [
                Point2f::new(0.0, 0.0),
                Point2f::new(1.0, 0.0),
                Point2f::new(1.0, 1.0),
            ]
        }
        else {
            [
                self.mesh.uv[self.mesh.vertex_indicies[(self.index * 3 + 0) as usize] as usize],
                self.mesh.uv[self.mesh.vertex_indicies[(self.index * 3 + 1) as usize] as usize],
                self.mesh.uv[self.mesh.vertex_indicies[(self.index * 3 + 2) as usize] as usize],
            ]
        }
    }
}

impl Shape for Triangle {
    fn object_bound(&self) -> Box<dyn Bounding3<Float>> {
        let p0 = self.mesh.p[self.mesh.vertex_indicies[(self.index * 3 + 0) as usize] as usize];
        let p1 = self.mesh.p[self.mesh.vertex_indicies[(self.index * 3 + 1) as usize] as usize];
        let p2 = self.mesh.p[self.mesh.vertex_indicies[(self.index * 3 + 2) as usize] as usize];

        Box::new(Bounds3f::new(
            self.mesh.world_to_object.transform_point(&p0),
            self.mesh.world_to_object.transform_point(&p1)
        ).union(&self.mesh.world_to_object.transform_point(&p2)))
    }

    fn world_bound(&self) -> Box<dyn Bounding3<Float>> {
        let p0 = self.mesh.p[self.mesh.vertex_indicies[(self.index * 3 + 0) as usize] as usize];
        let p1 = self.mesh.p[self.mesh.vertex_indicies[(self.index * 3 + 1) as usize] as usize];
        let p2 = self.mesh.p[self.mesh.vertex_indicies[(self.index * 3 + 2) as usize] as usize];

        Box::new(Bounds3f::new(p0, p1).union(&p2))
    }

    fn get_reverse_orientation(&self) -> bool {
        self.mesh.reverse_orientation
    }

    fn get_transform_swaps_handedness(&self) -> bool {
        self.mesh.transform_swaps_handedness
    }

    fn intersect(&self, ray: &Ray, test_alpha_texture: bool) -> Option<(SurfaceInteraction, Float)> {
        let _p = Profiler::instance().profile("Triangle::intersect");
        let i0 = self.mesh.vertex_indicies[(self.index * 3) as usize] as usize;
        let i1 = self.mesh.vertex_indicies[(self.index * 3 + 1) as usize] as usize;
        let i2 = self.mesh.vertex_indicies[(self.index * 3 + 2) as usize] as usize;
        // Get triangle vertices in _p0_, _p1_, and _p2_
        let p0 = &self.mesh.p[i0];
        let p1 = &self.mesh.p[i1];
        let p2 = &self.mesh.p[i2];

        // Perform ray--triangle intersection test

        // Transform triangle vertices to ray coordinate space

        // Translate vertices based on ray origin
        let mut p0t = *p0 - Vector3f::from(ray.o);
        let mut p1t = *p1 - Vector3f::from(ray.o);
        let mut p2t = *p2 - Vector3f::from(ray.o);

        // Permute components of triangle vertices and ray direction
        let kz = ray.d.abs().max_dimension();
        let kx = kz + 1;
        if kx == 3 {
            kx = 0;
        }
        let ky = kx + 1;
        if ky == 3 {
            ky = 0;
        }
        let d = ray.d.permute(kx, ky, kz);
        p0t = p0t.permute(kx, ky, kz);
        p1t = p1t.permute(kx, ky, kz);
        p2t = p2t.permute(kx, ky, kz);

        // Apply shear transformation to translated vertex positions
        let sx = -d.x / d.z;
        let sy = -d.y / d.z;
        let sz = 1.0 / d.z;
        p0t.x += sx * p0t.z;
        p0t.y += sy * p0t.z;
        p1t.x += sx * p1t.z;
        p1t.y += sy * p1t.z;
        p2t.x += sx * p2t.z;
        p2t.y += sy * p2t.z;

        // Compute edge function coefficients _e0_, _e1_, and _e2_
        let e0 = p1t.x * p2t.y - p1t.y * p2t.x;
        let e1 = p2t.x * p0t.y - p2t.y * p0t.x;
        let e2 = p0t.x * p1t.y - p0t.y * p1t.x;

        // Fall back to double precision test at triangle edges
        if size_of::<Float>() == size_of::<f32>()
            && (e0 == 0.0 || e1 == 0.0 || e2 == 0.0) {
            let p2txp1ty = p2t.x as f64 * p1t.y as f64;
            let p2typ1tx = p2t.y as f64 * p1t.x as f64;
            e0 = (p2typ1tx - p2txp1ty) as Float;
            let p0txp2ty = p0t.x as f64 * p2t.y as f64;
            let p0typ2tx = p0t.y as f64 * p2t.x as f64;
            e1 = (p0typ2tx - p0txp2ty) as Float;
            let p1txp0ty = p1t.x as f64 * p0t.y as f64;
            let p1typ0tx = p1t.y as f64 * p0t.x as f64;
            e2 = (p1typ0tx - p1txp0ty) as Float;
        }

        // Perform triangle edge and determinant tests
        if (e0 < 0.0 || e1 < 0.0 || e2 < 0.0) && (e0 > 0.0 || e1 > 0.0 || e2 > 0.0) {
            StatsAccumulator::instance().report_percentage(String::from("Intersections/Ray-triangle intersection tests"), 0, 1);
            return None;
        }
        let det = e0 + e1 + e2;
        if det == 0.0 {
            StatsAccumulator::instance().report_percentage(String::from("Intersections/Ray-triangle intersection tests"), 0, 1);
            return None;
        }

        // Compute scaled hit distance to triangle and test against ray $t$ range
        p0t.z *= sz;
        p1t.z *= sz;
        p2t.z *= sz;
        let t_scaled = e0 * p0t.z + e1 * p1t.z + e2 * p2t.z;
        if det < 0.0 && (t_scaled >= 0.0 || t_scaled < ray.t_max * det) {
            StatsAccumulator::instance().report_percentage(String::from("Intersections/Ray-triangle intersection tests"), 0, 1);
            return None;
        }
        else if det > 0.0 && (t_scaled <= 0.0 || t_scaled > ray.t_max * det) {
            StatsAccumulator::instance().report_percentage(String::from("Intersections/Ray-triangle intersection tests"), 0, 1);
            return None;
        }

        // Compute barycentric coordinates and $t$ value for triangle intersection
        let inv_det = 1.0 / det;
        let b0 = e0 * inv_det;
        let b1 = e1 * inv_det;
        let b2 = e2 * inv_det;
        let t = t_scaled * inv_det;

        // Ensure that computed triangle $t$ is conservatively greater than zero

        // Compute $\delta_z$ term for triangle $t$ error bounds
        let max_zt = Vector3f::new(p0t.z, p1t.z, p2t.z).abs().max_component();
        let delta_z = gamma(3) * max_zt;

        // Compute $\delta_x$ and $\delta_y$ terms for triangle $t$ error bounds
        let max_xt = Vector3f::new(p0t.x, p1t.x, p2t.x).abs().max_component();
        let max_yt = Vector3f::new(p0t.y, p1t.y, p2t.y).abs().max_component();
        let delta_x = gamma(5) * (max_xt + max_zt);
        let delta_y = gamma(5) * (max_yt + max_zt);

        // Compute $\delta_e$ term for triangle $t$ error bounds
        let delta_e =
            2.0 * (gamma(2) * max_xt * max_yt + delta_y * max_xt + delta_x * max_yt);

        // Compute $\delta_t$ term for triangle $t$ error bounds and check _t_
        let max_e = Vector3f::new(e0, e1, e2).abs().max_component();
        let delta_t = 3.0 *
                    (gamma(3) * max_e * max_zt + delta_e * max_zt + delta_z * max_e) *
                    inv_det.abs();
        if t <= delta_t {
            StatsAccumulator::instance().report_percentage(String::from("Intersections/Ray-triangle intersection tests"), 0, 1);
            return None;
        }

        // Compute triangle partial derivatives
        let dpdu ;
        let dpdv ;
        let uv = self.get_uvs();

        // Compute deltas for triangle partial derivatives
        let duv02 = uv[0] - uv[2];
        let duv12 = uv[1] - uv[2];
        let dp02 = *p0 - *p2;
        let dp12 = *p1 - *p2;
        let determinant = duv02.x * duv12.y - duv02.x * duv12.y;
        let degenerate_u_v = determinant.abs() < 1e-8;
        if !degenerate_u_v {
            let invdet = 1.0 / determinant;
            dpdu = (duv12.y * dp02 - duv02.y * dp12) * invdet;
            dpdv = (-duv12.x * dp02 + duv02.x * dp12) * invdet;
        }
        if degenerate_u_v || dpdu.cross(&dpdv).length_squared() == 0.0 {
            // Handle zero determinant for triangle partial derivative matrix
            let ng = (*p2 - *p0).cross(&(*p1 - *p0));
            if ng.length_squared() == 0.0 {
                // The triangle is actually degenerate; the intersection is
                // bogus.
                StatsAccumulator::instance().report_percentage(String::from("Intersections/Ray-triangle intersection tests"), 0, 1);
                return None;
            }

            coordinate_system(&ng.normalize(), &mut dpdu, &mut dpdv);
        }

        // Compute error bounds for triangle intersection
        let x_abs_sum =
            (b0 * p0.x).abs() + (b1 * p1.x).abs() + (b2 * p2.x).abs();
        let y_abs_sum =
            (b0 * p0.y).abs() + (b1 * p1.y).abs() + (b2 * p2.y).abs();
        let z_abs_sum =
            (b0 * p0.z).abs() + (b1 * p1.z).abs() + (b2 * p2.z).abs();
        let p_error = gamma(7) * Vector3f::new(x_abs_sum, y_abs_sum, z_abs_sum);

        // Interpolate $(u,v)$ parametric coordinates and hit point
        let p_hit = b0 * *p0 + b1 * *p1 + b2 * *p2;
        let uv_hit = b0 * uv[0] + b1 * uv[1] + b2 * uv[2];

        // Test intersection against alpha texture, if present
        if test_alpha_texture {
            if let Some(alpha_mask) = self.mesh.alpha_mask {
                let isect_local = SurfaceInteraction::new(&p_hit, &Vector3f::default(), &uv_hit, &-ray.d,
                    &dpdu, &dpdv, &Normal3f::default(),
                    &Normal3f::default(), ray.time, Some(self), self.face_index);
                if alpha_mask.evaluate(&isect_local) == 0.0 {
                    StatsAccumulator::instance().report_percentage(String::from("Intersections/Ray-triangle intersection tests"), 0, 1);
                    return None;
                }
            }
        }

        // Fill in _SurfaceInteraction_ from triangle hit
        let isect = SurfaceInteraction::new(&p_hit, &p_error, &uv_hit, &-ray.d, &dpdu, &dpdv,
                                    &Normal3f::default(), &Normal3f::default(), ray.time,
                                    Some(self), self.face_index);

        // Override surface normal in _isect_ for triangle
        let mut normal = dp02.cross(&dp12).normalize();
        if self.mesh.reverse_orientation ^ self.mesh.transform_swaps_handedness {
            normal = -normal;
        }
        isect.n = Normal3f::from(normal);
        isect.shading.n = Normal3f::from(normal);

        if !self.mesh.n.is_empty() || !self.mesh.s.is_empty() {
            // Initialize _Triangle_ shading geometry

            // Compute shading normal _ns_ for triangle
            let mut ns: Normal3f;
            if !self.mesh.n.is_empty() {
                ns = b0 * self.mesh.n[i0] + b1 * self.mesh.n[i1] + b2 * self.mesh.n[i2];
                if ns.length_squared() > 0.0 {
                    ns = ns.normalize();
                }
                else {
                    ns = isect.n;
                }
            } else {
                ns = isect.n;
            }

            // Compute shading tangent _ss_ for triangle
            let ss: Vector3f;
            if !self.mesh.s.is_empty() {
                ss = b0 * self.mesh.s[i0] + b1 * self.mesh.s[i1] + b2 * self.mesh.s[i2];
                if ss.length_squared() > 0.0 {
                    ss = ss.normalize();
                } else {
                    ss = isect.dpdu.normalize();
                }
            } else {
                ss = isect.dpdu.normalize();
            }

            // Compute shading bitangent _ts_ for triangle and adjust _ss_
            let ts = ss.cross(&Vector3f::from(ns));
            if ts.length_squared() > 0.0 {
                ts = ts.normalize();
                ss = ts.cross(&Vector3f::from(ns));
            } else {
                coordinate_system(&Vector3f::from(ns), &mut ss, &mut ts);
            }

            // Compute $\dndu$ and $\dndv$ for triangle shading geometry
            let dndu: Normal3f;
            let dndv: Normal3f;
            if !self.mesh.n.is_empty() {
                // Compute deltas for triangle partial derivatives of normal
                let duv02 = uv[0] - uv[2];
                let duv12 = uv[1] - uv[2];
                let dn1 = self.mesh.n[i0] - self.mesh.n[i2];
                let dn2 = self.mesh.n[i1] - self.mesh.n[i2];
                let determinant = duv02.x * duv12.y - duv02.y * duv12.x;
                let degenerate_u_v = determinant.abs() < 1e-8;
                if degenerate_u_v {
                    // We can still compute dndu and dndv, with respect to the
                    // same arbitrary coordinate system we use to compute dpdu
                    // and dpdv when this happens. It's important to do this
                    // (rather than giving up) so that ray differentials for
                    // rays reflected from triangles with degenerate
                    // parameterizations are still reasonable.
                    let dn = Vector3f::from(self.mesh.n[i2] - self.mesh.n[i0]).cross(&Vector3f::from(self.mesh.n[i1] - self.mesh.n[i0]));
                    if dn.length_squared() == 0.0 {
                        dndu = Normal3f::default();
                        dndv = Normal3f::default();
                    }
                    else {
                        let dnu = Vector3f::default();
                        let dnv = Vector3f::default();
                        coordinate_system(&dn, &mut dnu, &mut dnv);
                        dndu = Normal3f::from(dnu);
                        dndv = Normal3f::from(dnv);
                    }
                } else {
                    let inv_det = 1.0 / determinant;
                    dndu = (duv12.y * dn1 - duv02.y * dn2) * inv_det;
                    dndv = (-duv12.x * dn1 + duv02.x * dn2) * inv_det;
                }
            } else {
                dndu = Normal3f::default();
                dndv = Normal3f::default();
            }
            if self.mesh.reverse_orientation {
                ts = -ts;
            }
            isect.set_shading_geometry(&ss, &ts, &dndu, &dndv, true);
        }

        StatsAccumulator::instance().report_percentage(String::from("Intersections/Ray-triangle intersection tests"), 0, 1);
        Some((isect, t as Float))
    }

    fn intersect_p(&self, ray: &Ray, test_alpha_texture: bool) -> bool {
        let _p = Profiler::instance().profile("Triangle::intersect_p");
        let i0 = self.mesh.vertex_indicies[(self.index * 3) as usize] as usize;
        let i1 = self.mesh.vertex_indicies[(self.index * 3 + 1) as usize] as usize;
        let i2 = self.mesh.vertex_indicies[(self.index * 3 + 2) as usize] as usize;
        // Get triangle vertices in _p0_, _p1_, and _p2_
        let p0 = &self.mesh.p[i0];
        let p1 = &self.mesh.p[i1];
        let p2 = &self.mesh.p[i2];

        // Perform ray--triangle intersection test

        // Transform triangle vertices to ray coordinate space

        // Translate vertices based on ray origin
        let mut p0t = *p0 - Vector3f::from(ray.o);
        let mut p1t = *p1 - Vector3f::from(ray.o);
        let mut p2t = *p2 - Vector3f::from(ray.o);

        // Permute components of triangle vertices and ray direction
        let kz = ray.d.abs().max_dimension();
        let kx = kz + 1;
        if kx == 3 {
            kx = 0;
        }
        let ky = kx + 1;
        if ky == 3 {
            ky = 0;
        }
        let d = ray.d.permute(kx, ky, kz);
        p0t = p0t.permute(kx, ky, kz);
        p1t = p1t.permute(kx, ky, kz);
        p2t = p2t.permute(kx, ky, kz);

        // Apply shear transformation to translated vertex positions
        let sx = -d.x / d.z;
        let sy = -d.y / d.z;
        let sz = 1.0 / d.z;
        p0t.x += sx * p0t.z;
        p0t.y += sy * p0t.z;
        p1t.x += sx * p1t.z;
        p1t.y += sy * p1t.z;
        p2t.x += sx * p2t.z;
        p2t.y += sy * p2t.z;

        // Compute edge function coefficients _e0_, _e1_, and _e2_
        let e0 = p1t.x * p2t.y - p1t.y * p2t.x;
        let e1 = p2t.x * p0t.y - p2t.y * p0t.x;
        let e2 = p0t.x * p1t.y - p0t.y * p1t.x;

        // Fall back to double precision test at triangle edges
        if size_of::<Float>() == size_of::<f32>()
            && (e0 == 0.0 || e1 == 0.0 || e2 == 0.0) {
            let p2txp1ty = p2t.x as f64 * p1t.y as f64;
            let p2typ1tx = p2t.y as f64 * p1t.x as f64;
            e0 = (p2typ1tx - p2txp1ty) as Float;
            let p0txp2ty = p0t.x as f64 * p2t.y as f64;
            let p0typ2tx = p0t.y as f64 * p2t.x as f64;
            e1 = (p0typ2tx - p0txp2ty) as Float;
            let p1txp0ty = p1t.x as f64 * p0t.y as f64;
            let p1typ0tx = p1t.y as f64 * p0t.x as f64;
            e2 = (p1typ0tx - p1txp0ty) as Float;
        }

        // Perform triangle edge and determinant tests
        if (e0 < 0.0 || e1 < 0.0 || e2 < 0.0) && (e0 > 0.0 || e1 > 0.0 || e2 > 0.0) {
            StatsAccumulator::instance().report_percentage(String::from("Intersections/Ray-triangle intersection tests"), 0, 1);
            return false;
        }
        let det = e0 + e1 + e2;
        if det == 0.0 {
            StatsAccumulator::instance().report_percentage(String::from("Intersections/Ray-triangle intersection tests"), 0, 1);
            return false;
        }

        // Compute scaled hit distance to triangle and test against ray $t$ range
        p0t.z *= sz;
        p1t.z *= sz;
        p2t.z *= sz;
        let t_scaled = e0 * p0t.z + e1 * p1t.z + e2 * p2t.z;
        if det < 0.0 && (t_scaled >= 0.0 || t_scaled < ray.t_max * det) {
            StatsAccumulator::instance().report_percentage(String::from("Intersections/Ray-triangle intersection tests"), 0, 1);
            return false;
        }
        else if det > 0.0 && (t_scaled <= 0.0 || t_scaled > ray.t_max * det) {
            StatsAccumulator::instance().report_percentage(String::from("Intersections/Ray-triangle intersection tests"), 0, 1);
            return false;
        }

        // Compute barycentric coordinates and $t$ value for triangle intersection
        let inv_det = 1.0 / det;
        let b0 = e0 * inv_det;
        let b1 = e1 * inv_det;
        let b2 = e2 * inv_det;
        let t = t_scaled * inv_det;

        // Ensure that computed triangle $t$ is conservatively greater than zero

        // Compute $\delta_z$ term for triangle $t$ error bounds
        let max_zt = Vector3f::new(p0t.z, p1t.z, p2t.z).abs().max_component();
        let delta_z = gamma(3) * max_zt;

        // Compute $\delta_x$ and $\delta_y$ terms for triangle $t$ error bounds
        let max_xt = Vector3f::new(p0t.x, p1t.x, p2t.x).abs().max_component();
        let max_yt = Vector3f::new(p0t.y, p1t.y, p2t.y).abs().max_component();
        let delta_x = gamma(5) * (max_xt + max_zt);
        let delta_y = gamma(5) * (max_yt + max_zt);

        // Compute $\delta_e$ term for triangle $t$ error bounds
        let delta_e =
            2.0 * (gamma(2) * max_xt * max_yt + delta_y * max_xt + delta_x * max_yt);

        // Compute $\delta_t$ term for triangle $t$ error bounds and check _t_
        let max_e = Vector3f::new(e0, e1, e2).abs().max_component();
        let delta_t = 3.0 *
                    (gamma(3) * max_e * max_zt + delta_e * max_zt + delta_z * max_e) *
                    inv_det.abs();
        if t <= delta_t {
            StatsAccumulator::instance().report_percentage(String::from("Intersections/Ray-triangle intersection tests"), 0, 1);
            return false;
        }

        if test_alpha_texture && (self.mesh.alpha_mask.is_some() || self.mesh.shadow_alpha_mask.is_some()) {
            // Compute triangle partial derivatives
            let dpdu ;
            let dpdv ;
            let uv = self.get_uvs();

            // Compute deltas for triangle partial derivatives
            let duv02 = uv[0] - uv[2];
            let duv12 = uv[1] - uv[2];
            let dp02 = *p0 - *p2;
            let dp12 = *p1 - *p2;
            let determinant = duv02.x * duv12.y - duv02.x * duv12.y;
            let degenerate_u_v = determinant.abs() < 1e-8;
            if !degenerate_u_v {
                let invdet = 1.0 / determinant;
                dpdu = (duv12.y * dp02 - duv02.y * dp12) * invdet;
                dpdv = (-duv12.x * dp02 + duv02.x * dp12) * invdet;
            }
            if degenerate_u_v || dpdu.cross(&dpdv).length_squared() == 0.0 {
                // Handle zero determinant for triangle partial derivative matrix
                let ng = (*p2 - *p0).cross(&(*p1 - *p0));
                if ng.length_squared() == 0.0 {
                    // The triangle is actually degenerate; the intersection is
                    // bogus.
                    StatsAccumulator::instance().report_percentage(String::from("Intersections/Ray-triangle intersection tests"), 0, 1);
                    return false;
                }

                coordinate_system(&ng.normalize(), &mut dpdu, &mut dpdv);
            }

            // Compute error bounds for triangle intersection
            let x_abs_sum =
                (b0 * p0.x).abs() + (b1 * p1.x).abs() + (b2 * p2.x).abs();
            let y_abs_sum =
                (b0 * p0.y).abs() + (b1 * p1.y).abs() + (b2 * p2.y).abs();
            let z_abs_sum =
                (b0 * p0.z).abs() + (b1 * p1.z).abs() + (b2 * p2.z).abs();
            let p_error = gamma(7) * Vector3f::new(x_abs_sum, y_abs_sum, z_abs_sum);

            // Interpolate $(u,v)$ parametric coordinates and hit point
            let p_hit = b0 * *p0 + b1 * *p1 + b2 * *p2;
            let uv_hit = b0 * uv[0] + b1 * uv[1] + b2 * uv[2];

            // Test intersection against alpha texture, if present
            let isect_local = SurfaceInteraction::new(&p_hit, &Vector3f::default(), &uv_hit, &-ray.d,
                        &dpdu, &dpdv, &Normal3f::default(),
                        &Normal3f::default(), ray.time, Some(self), self.face_index);
            
            if let Some(alpha_mask) = self.mesh.alpha_mask {
                if alpha_mask.evaluate(&isect_local) == 0.0 {
                    StatsAccumulator::instance().report_percentage(String::from("Intersections/Ray-triangle intersection tests"), 0, 1);
                    return false;
                }
            }
            if let Some(shadow_alpha_mask) = self.mesh.shadow_alpha_mask {
                if shadow_alpha_mask.evaluate(&isect_local) == 0.0 {
                    StatsAccumulator::instance().report_percentage(String::from("Intersections/Ray-triangle intersection tests"), 0, 1);
                    return false;
                }
            }
        }

        true
    }

    fn area(&self) -> Float {
        let p0 = &self.mesh.p[self.mesh.vertex_indicies[(self.index * 3) as usize] as usize];
        let p1 = &self.mesh.p[self.mesh.vertex_indicies[(self.index * 3 + 1) as usize] as usize];
        let p2 = &self.mesh.p[self.mesh.vertex_indicies[(self.index * 3 + 2) as usize] as usize];
        0.5 * (*p1 - *p0).cross(&(*p2 - *p0)).length()
    }

    fn sample(&self, u: &Point2f, pdf: &mut Float) -> Box<dyn Interaction> {
        let b = uniform_sample_triangle(&u);
        // Get triangle vertices in _p0_, _p1_, and _p2_
        let p0 = &self.mesh.p[self.mesh.vertex_indicies[(self.index * 3) as usize] as usize];
        let p1 = &self.mesh.p[self.mesh.vertex_indicies[(self.index * 3 + 1) as usize] as usize];
        let p2 = &self.mesh.p[self.mesh.vertex_indicies[(self.index * 3 + 2) as usize] as usize];
        let it = SimpleInteraction::default();
        it.p = b[0] * *p0 + b[1] * *p1 + (1.0 - b[0] - b[1]) * *p2;
        // Compute surface normal for sampled point on triangle
        it.n = Normal3f::from((*p1 - *p0).cross(&(*p2 - *p0))).normalize();
        // Ensure correct orientation of the geometric normal; follow the same
        // approach as was used in Triangle::Intersect().
        if !self.mesh.n.is_empty() {
            let ns = Normal3f::from(b[0] * self.mesh.n[self.mesh.vertex_indicies[(self.index * 3) as usize] as usize] +
                        b[1] * self.mesh.n[self.mesh.vertex_indicies[(self.index * 3 + 1) as usize] as usize] +
                        (1.0 - b[0] - b[1]) * self.mesh.n[self.mesh.vertex_indicies[(self.index * 3 + 2) as usize] as usize]);
            it.n = it.n.face_forward(&ns);
        } else if self.mesh.reverse_orientation ^ self.mesh.transform_swaps_handedness {
            it.n *= -1.0;
        }

        // Compute error bounds for sampled point on triangle
        let p_abs_sum = (b[0] * *p0).abs() + (b[1] * *p1).abs() + ((1.0 - b[0] - b[1]).abs() * *p2);
        it.p_error = gamma(6) * Vector3f::new(p_abs_sum.x, p_abs_sum.y, p_abs_sum.z);
        *pdf = 1.0 / self.area();
        Box::new(it)
    }

    fn solid_angle(&self, p: &Point3f, n_samples: i32) -> Float {
        // Project the vertices into the unit sphere around p.
        let p_sphere = [
            (self.mesh.p[self.mesh.vertex_indicies[(self.index * 3) as usize] as usize] - *p).normalize(),
            (self.mesh.p[self.mesh.vertex_indicies[(self.index * 3 + 1) as usize] as usize] - *p).normalize(),
            (self.mesh.p[self.mesh.vertex_indicies[(self.index * 3 + 2) as usize] as usize] - *p).normalize()
        ];

        // http://math.stackexchange.com/questions/9819/area-of-a-spherical-triangle
        // Girard's theorem: surface area of a spherical triangle on a unit
        // sphere is the 'excess angle' alpha+beta+gamma-pi, where
        // alpha/beta/gamma are the interior angles at the vertices.
        //
        // Given three vertices on the sphere, a, b, c, then we can compute,
        // for example, the angle c->a->b by
        //
        // cos theta =  Dot(Cross(c, a), Cross(b, a)) /
        //              (Length(Cross(c, a)) * Length(Cross(b, a))).
        //
        let cross01 = p_sphere[0].cross(&p_sphere[1]);
        let cross12 = p_sphere[1].cross(&p_sphere[2]);
        let cross20 = p_sphere[2].cross(&p_sphere[0]);

        // Some of these vectors may be degenerate. In this case, we don't want
        // to normalize them so that we don't hit an assert. This is fine,
        // since the corresponding dot products below will be zero.
        if cross01.length_squared() > 0.0 { cross01 = cross01.normalize(); }
        if cross12.length_squared() > 0.0 { cross12 = cross12.normalize(); }
        if cross20.length_squared() > 0.0 { cross20 = cross20.normalize(); }

        // We only need to do three cross products to evaluate the angles at
        // all three vertices, though, since we can take advantage of the fact
        // that Cross(a, b) = -Cross(b, a).
        (clamp(cross01.dot(&-cross12), -1.0, 1.0).acos() +
            clamp(cross12.dot(&-cross20), -1.0, 1.0).acos() +
            clamp(cross20.dot(&-cross01), -1.0, 1.0).acos() - PI).abs()
    }
}