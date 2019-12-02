use super::geometry::{Vector3f, Normal3f, Point3f, Ray, Bounding3};
use super::matrix::Matrix4x4;
use super::pbrt::{Float, radians, gamma};
use super::surface_interaction::SurfaceInteraction;
use std::ops::{Mul, Add};
use std::sync::{Arc, RwLock};

#[derive(Debug, Copy, Clone)]
pub struct Transform {
    pub m: Matrix4x4,
    pub m_inv: Matrix4x4,
}

macro_rules! not_one {
    ($e: expr) => {
        $e < 0.999 || $e > 1.001
    };
}

impl Transform {
    pub fn new(
        t00: Float,
        t01: Float,
        t02: Float,
        t03: Float,
        t10: Float,
        t11: Float,
        t12: Float,
        t13: Float,
        t20: Float,
        t21: Float,
        t22: Float,
        t23: Float,
        t30: Float,
        t31: Float,
        t32: Float,
        t33: Float
    ) -> Transform {
        let m = Matrix4x4::new(t00, t01, t02, t03, t10, t11, t12, t13, t20, t21, t22, t23, t30, t31, t32, t33);
        Transform{
            m,
            m_inv: m.inverse()
        }
    }

    /// Return the inverse of this Transform.
    pub fn inverse(&self) -> Transform {
        Transform{
            m: self.m.transpose(),
            m_inv: self.m.inverse()
        }
    }

    /// Test if this Transform is the identity tranformation.
    pub fn is_identity(&self) -> bool {
        self.m.m[0][0] == 1.0 && self.m.m[0][1] == 0.0 && self.m.m[0][1] == 0.0 && self.m.m[0][1] == 0.0 &&
        self.m.m[0][0] == 0.0 && self.m.m[0][1] == 1.0 && self.m.m[0][1] == 0.0 && self.m.m[0][1] == 0.0 &&
        self.m.m[0][0] == 0.0 && self.m.m[0][1] == 0.0 && self.m.m[0][1] == 1.0 && self.m.m[0][1] == 0.0 &&
        self.m.m[0][0] == 0.0 && self.m.m[0][1] == 0.0 && self.m.m[0][1] == 0.0 && self.m.m[0][1] == 1.0
    }

    /// Create a Transform representing a translation.
    pub fn translate(delta: &Vector3f) -> Transform {
        Transform{
            m: Matrix4x4::new(1.0, 0.0, 0.0, delta.x,
                0.0, 1.0, 0.0, delta.y,
                0.0, 0.0, 1.0, delta.z,
                0.0, 0.0, 0.0, 1.0
                ),
            m_inv: Matrix4x4::new(1.0, 0.0, 0.0, -delta.x,
                0.0, 1.0, 0.0, -delta.y,
                0.0, 0.0, 1.0, -delta.z,
                0.0, 0.0, 0.0, 1.0
                )
        }
    }

    /// Creates a Transform that represents a scale.
    pub fn scale(x: Float, y: Float, z: Float) -> Transform {
        Transform{
            m: Matrix4x4::new(x, 0.0, 0.0, 0.0,
                0.0, y, 0.0, 0.0,
                0.0, 0.0, z, 0.0,
                0.0, 0.0, 0.0, 1.0
                ),
            m_inv: Matrix4x4::new(1.0 / x, 0.0, 0.0, 0.0,
                0.0, 1.0 / y, 0.0, 0.0,
                0.0, 0.0, 1.0 / z, 0.0,
                0.0, 0.0, 0.0, 1.0
                )
        }
    }

    /// Test if this Transform has a scaling term.
    pub fn has_scale(&self) -> bool {
        let la2 = self.transform_vector(&Vector3f::new(1.0, 0.0, 0.0)).length_squared();
        let lb2 = self.transform_vector(&Vector3f::new(0.0, 1.0, 0.0)).length_squared();
        let lc2 = self.transform_vector(&Vector3f::new(0.0, 0.0, 1.0)).length_squared();
        not_one!(la2) || not_one!(lb2) || not_one!(lc2)
    }

    /// Create a Transform of a clock-wise rotation around the X axis in degrees.
    pub fn rotate_x(theta: Float) -> Transform {
        let theta = radians(theta);
        let sin_theta = theta.sin();
        let cos_theta = theta.cos();
        let m = Matrix4x4::new(
            1.0, 0.0, 0.0, 0.0,
            0.0, cos_theta, -sin_theta, 0.0,
            0.0, sin_theta, cos_theta, 0.0,
            0.0, 0.0, 0.0, 1.0
        );
        Transform{
            m,
            m_inv: m.transpose()
        }
    }

    /// Create a Transform of a clock-wise rotation around the Y axis in degrees.
    pub fn rotate_y(theta: Float) -> Transform {
        let theta = radians(theta);
        let sin_theta = theta.sin();
        let cos_theta = theta.cos();
        let m = Matrix4x4::new(
            cos_theta, 0.0, sin_theta, 0.0,
            0.0, 1.0, 0.0, 0.0,
            -sin_theta, 0.0, cos_theta, 0.0,
            0.0, 0.0, 0.0, 1.0
        );
        Transform{
            m,
            m_inv: m.transpose()
        }
    }

    /// Create a Transform of a clock-wise rotation around the Z axis in degrees.
    pub fn rotate_z(theta: Float) -> Transform {
        let theta = radians(theta);
        let sin_theta = theta.sin();
        let cos_theta = theta.cos();
        let m = Matrix4x4::new(
            cos_theta, -sin_theta, 0.0, 0.0,
            sin_theta, cos_theta, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0
        );
        Transform{
            m,
            m_inv: m.transpose()
        }
    }

    pub fn rotate(theta: Float, axis: &Vector3f) -> Transform {
        let theta = radians(theta);
        let a = axis.normalize();
        let sin_theta = theta.sin();
        let cos_theta = theta.cos();
        let mut m = Matrix4x4::default();
        // compute rotation of first basis vector
        m.m[0][0] = a.x * a.x + (1.0 - a.x * a.x) * cos_theta;
        m.m[0][1] = a.x * a.y * (1.0 - cos_theta) - a.z * sin_theta;
        m.m[0][2] = a.x * a.z * (1.0 - cos_theta) + a.y * sin_theta;
        m.m[0][3] = 0.0;
        // compute rotations of second basis vectors
        m.m[1][0] = a.x * a.y * (1.0 - cos_theta) + a.z * sin_theta;
        m.m[1][1] = a.y * a.y + (1.0 - a.y * a.y) * cos_theta;
        m.m[1][2] = a.y * a.z * (1.0 - cos_theta) - a.x * sin_theta;
        m.m[1][3] = 0.0;
        // compute rotations of third basis vectors
        m.m[2][0] = a.x * a.z * (1.0 - cos_theta) - a.y * sin_theta;
        m.m[2][1] = a.y * a.z * (1.0 - cos_theta) + a.x * sin_theta;
        m.m[2][2] = a.z * a.z + (1.0 - a.z * a.z) * cos_theta;
        m.m[2][3] = 0.0;
        Transform{
            m,
            m_inv: m.inverse()
        }
    }

    /// Create a transform so that the origin is at pos and rotation is towards look in relation to the up Vector.
    pub fn look_at(pos: &Point3f, look: &Point3f, up: &Vector3f) -> Transform {
        let mut camera_to_world = Matrix4x4::default();
        // Initialize fourth column of viewing matrix
        camera_to_world.m[0][3] = pos.x;
        camera_to_world.m[1][3] = pos.y;
        camera_to_world.m[2][3] = pos.z;
        camera_to_world.m[3][3] = 1.0;
        // Initialize first three columns of viewing matrix
        let dir = (*look - *pos).normalize();
        let right = up.normalize().cross(&dir).normalize();
        let new_up = dir.cross(&right);
        camera_to_world.m[0][0] = right.x;
        camera_to_world.m[1][0] = right.y;
        camera_to_world.m[2][0] = right.z;
        camera_to_world.m[3][0] = 0.;
        camera_to_world.m[0][1] = new_up.x;
        camera_to_world.m[1][1] = new_up.y;
        camera_to_world.m[2][1] = new_up.z;
        camera_to_world.m[3][1] = 0.;
        camera_to_world.m[0][2] = dir.x;
        camera_to_world.m[1][2] = dir.y;
        camera_to_world.m[2][2] = dir.z;
        camera_to_world.m[3][2] = 0.;
        Transform{
            m: camera_to_world.inverse(),
            m_inv: camera_to_world
        }
    }

    /// Transform a given Point3f.
    pub fn transform_point(&self, p: &Point3f) -> Point3f {
        let x = self.m.m[0][0] * p.x + self.m.m[0][1] * p.y + self.m.m[0][2] * p.z + self.m.m[0][3];
        let y = self.m.m[1][0] * p.x + self.m.m[1][1] * p.y + self.m.m[1][2] * p.z + self.m.m[1][3];
        let z = self.m.m[2][0] * p.x + self.m.m[2][1] * p.y + self.m.m[2][2] * p.z + self.m.m[2][3];
        let w = self.m.m[3][0] * p.x + self.m.m[3][1] * p.y + self.m.m[3][2] * p.z + self.m.m[3][3];
        if w == 1.0 {
            Point3f::new(x, y, z)
        }
        else {
            Point3f::new(x / w, y / w, z / w)
        }
    }

    /// Transform a Point3f setting the value of p_error to the absolute error that this Transform introduces.
    pub fn transform_point_with_error(&self, p: &Point3f, p_error: &mut Vector3f) -> Point3f {
        let x = p.x;
        let y = p.y;
        let z = p.z;
        // Compute transformed coordinates from point _pt_
        let xp = (self.m.m[0][0] * x + self.m.m[0][1] * y) + (self.m.m[0][2] * z + self.m.m[0][3]);
        let yp = (self.m.m[1][0] * x + self.m.m[1][1] * y) + (self.m.m[1][2] * z + self.m.m[1][3]);
        let zp = (self.m.m[2][0] * x + self.m.m[2][1] * y) + (self.m.m[2][2] * z + self.m.m[2][3]);
        let wp = (self.m.m[3][0] * x + self.m.m[3][1] * y) + (self.m.m[3][2] * z + self.m.m[3][3]);

        // Compute absolute error for transformed point
        let x_abs_sum = (self.m.m[0][0] * x).abs() + (self.m.m[0][1] * y).abs() +
                    (self.m.m[0][2] * z).abs() + (self.m.m[0][3]).abs();
        let y_abs_sum = (self.m.m[1][0] * x).abs() + (self.m.m[1][1] * y).abs() +
                    (self.m.m[1][2] * z).abs() + (self.m.m[1][3]).abs();
        let z_abs_sum = (self.m.m[2][0] * x).abs() + (self.m.m[2][1] * y).abs() +
                    (self.m.m[2][2] * z).abs() + (self.m.m[2][3]).abs();
        *p_error = Vector3f::new(x_abs_sum, y_abs_sum, z_abs_sum) * gamma(3);
        assert!(wp != 0.0, "wp({}) != 0.0", wp);
        if wp == 1.0 {
            Point3f::new(xp, yp, zp)
        }
        else {
            Point3f::new(xp / wp, yp / wp, zp / wp)
        }
    }

    /// Transform a Point3f with an error factor as pt_error and return the transformed Point3f and the accumlated error in abs_error.
    pub fn transform_point_with_abs_error(&self, p: &Point3f, pt_error: &Vector3f, abs_error: &mut Vector3f) -> Point3f {
        let x = p.x;
        let y = p.y;
        let z = p.z;
        // Compute transformed coordinates from point _pt_
        let xp = (self.m.m[0][0] * x + self.m.m[0][1] * y) + (self.m.m[0][2] * z + self.m.m[0][3]);
        let yp = (self.m.m[1][0] * x + self.m.m[1][1] * y) + (self.m.m[1][2] * z + self.m.m[1][3]);
        let zp = (self.m.m[2][0] * x + self.m.m[2][1] * y) + (self.m.m[2][2] * z + self.m.m[2][3]);
        let wp = (self.m.m[3][0] * x + self.m.m[3][1] * y) + (self.m.m[3][2] * z + self.m.m[3][3]);
        abs_error.x = (gamma(3) + 1.0 as Float) *
            (self.m.m[0][0].abs() * pt_error.x + self.m.m[0][1].abs() * pt_error.y +
            self.m.m[0][2].abs() * pt_error.z) +
            gamma(3) * ((self.m.m[0][0] * x).abs() + (self.m.m[0][1] * y).abs() +
            (self.m.m[0][2] * z).abs() + self.m.m[0][3].abs());
        abs_error.y = (gamma(3) + 1.0 as Float) *
            (self.m.m[1][0].abs() * pt_error.x + self.m.m[1][1].abs() * pt_error.y +
            self.m.m[1][2].abs() * pt_error.z) +
            gamma(3) * ((self.m.m[1][0] * x).abs() + (self.m.m[1][1] * y).abs() +
            (self.m.m[1][2] * z).abs() + self.m.m[1][3].abs());
        abs_error.z = (gamma(3) + 1.0 as Float) *
            (self.m.m[2][0].abs() * pt_error.x + self.m.m[2][1].abs() * pt_error.y +
            self.m.m[2][2].abs() * pt_error.z) +
            gamma(3) * ((self.m.m[2][0] * x).abs() + (self.m.m[2][1] * y).abs() +
            (self.m.m[2][2] * z).abs() + self.m.m[0][3].abs());
        assert!(wp != 0.0, "wp({}) != 0.0", wp);
        if wp == 1.0 {
            Point3f::new(xp, yp, zp)
        }
        else {
            Point3f::new(xp / wp, yp / wp, zp / wp)
        }
    }

    /// Transform a given Vector3f.
    pub fn transform_vector(&self, v: &Vector3f) -> Vector3f {
        Vector3f{
            x: self.m.m[0][0] * v.x + self.m.m[0][1] * v.y + self.m.m[0][2] * v.z,
            y: self.m.m[1][0] * v.x + self.m.m[1][1] * v.y + self.m.m[1][2] * v.z,
            z: self.m.m[2][0] * v.x + self.m.m[2][1] * v.y + self.m.m[2][2] * v.z
        }
    }

    /// Transform a given Vector3f and set abs_error to the absolute error that this Transform introduces.
    pub fn transform_vector_with_error(&self, v: &Vector3f, abs_error: &mut Vector3f) -> Vector3f {
        let x = v.x;
        let y = v.y;
        let z = v.z;
        abs_error.x = gamma(3) * ((self.m.m[0][0] * x).abs() + (self.m.m[0][1] * y).abs() +
            (self.m.m[0][2] * z).abs());
        abs_error.y = gamma(3) * ((self.m.m[1][0] * x).abs() + (self.m.m[1][1] * y).abs() +
            (self.m.m[1][2] * z).abs());
        abs_error.z = gamma(3) * ((self.m.m[2][0] * x).abs() + (self.m.m[2][1] * y).abs() +
            (self.m.m[2][2] * z).abs());
        Vector3f::new(self.m.m[0][0] * x + self.m.m[0][1] * y + self.m.m[0][2] * z,
            self.m.m[1][0] * x + self.m.m[1][1] * y + self.m.m[1][2] * z,
            self.m.m[2][0] * x + self.m.m[2][1] * y + self.m.m[2][2] * z)
    }

    /// Transform a given Vector3f with an existing error v_error and return the given Vector3f and set abs_error to the accumulated error.
    pub fn transform_vector_with_abs_error(&self, v: &Vector3f, v_error: &Vector3f, abs_error: &mut Vector3f) -> Vector3f {
        let x = v.x;
        let y = v.y;
        let z = v.z;
        abs_error.x = (gamma(3) + 1.0) *
            (self.m.m[0][0].abs() * v_error.x + self.m.m[0][1].abs() * v_error.y +
            self.m.m[0][2].abs() * v_error.z) +
            gamma(3) * ((self.m.m[0][0] * x).abs() + (self.m.m[0][1] * y).abs() +
            (self.m.m[0][2] * z).abs());
        abs_error.y = (gamma(3) + 1.0) *
            (self.m.m[1][0].abs() * v_error.x + self.m.m[1][1].abs() * v_error.y +
            self.m.m[1][2].abs() * v_error.z) +
            gamma(3) * ((self.m.m[1][0] * x).abs() + (self.m.m[1][1] * y).abs() +
            (self.m.m[1][2] * z).abs());
        abs_error.z = (gamma(3) + 1.0) *
            (self.m.m[2][0].abs() * v_error.x + self.m.m[2][1].abs() * v_error.y +
            self.m.m[2][2].abs() * v_error.z) +
            gamma(3) * ((self.m.m[2][0] * x).abs() + (self.m.m[2][1] * y).abs() +
            (self.m.m[2][2] * z).abs());
            Vector3f::new(self.m.m[0][0] * x + self.m.m[0][1] * y + self.m.m[0][2] * z,
                self.m.m[1][0] * x + self.m.m[1][1] * y + self.m.m[1][2] * z,
                self.m.m[2][0] * x + self.m.m[2][1] * y + self.m.m[2][2] * z)
    }

    /// Transform a given Normal3f.
    pub fn transform_normal(&self, n: &Normal3f) -> Normal3f {
        Normal3f{
            x: self.m.m[0][0] * n.x + self.m.m[1][0] * n.y + self.m.m[2][0] * n.z,
            y: self.m.m[0][1] * n.x + self.m.m[1][1] * n.y + self.m.m[2][1] * n.z,
            z: self.m.m[0][2] * n.x + self.m.m[1][2] * n.y + self.m.m[2][2] * n.z,
        }
    }

    /// Transform a given Ray.
    pub fn transform_ray(&self, r: &Ray) -> Ray {
        // this combines both transform for ray and ray differential
        let mut o_error = Vector3f::default();
        let mut o = self.transform_point_with_error(&r.o, &mut o_error);
        let d = self.transform_vector(&r.d);
        // Offset ray origin to edge of error bounds and compute _t_max_
        let length_squared = d.length_squared();
        let mut t_max = r.t_max;
        if length_squared > 0.0 {
            let dt = d.abs().dot(&o_error) / length_squared;
            o += d * dt;
            t_max -= dt;
        }
        let medium = if let Some(ref medium) = r.medium {
            Some(medium.clone())
        }
        else {
            None
        };
        if let Some(mut rd) = r.differential {
            rd.rx_origin = self.transform_point(&rd.rx_origin);
            rd.ry_origin = self.transform_point(&rd.ry_origin);
            rd.rx_direction = self.transform_vector(&rd.rx_direction);
            rd.ry_direction = self.transform_vector(&rd.ry_direction);
            Ray{
                o,
                d,
                t_max,
                medium,
                differential: Some(rd),
                time: r.time
            }
        }
        else {
            Ray{
                o,
                d,
                t_max,
                medium,
                differential: None,
                time: r.time
            }
        }
    }

    /// Transform a Ray setting error introduced for the origin in o_error and direction in d_error.
    pub fn transform_ray_with_error(&self, r: &Ray, o_error: &mut Vector3f, d_error: &mut Vector3f) -> Ray {
        // this combines both transform for ray and ray differential
        let mut o = self.transform_point_with_error(&r.o, o_error);
        let d = self.transform_vector_with_error(&r.d, d_error);
        // Offset ray origin to edge of error bounds and compute _t_max_
        let length_squared = d.length_squared();
        let t_max = r.t_max;
        if length_squared > 0.0 {
            let dt = d.abs().dot(&o_error) / length_squared;
            o += d * dt;
            // t_max -= dt;
        }
        let medium = if let Some(ref medium) = r.medium {
            Some(medium.clone())
        }
        else {
            None
        };
        if let Some(mut rd) = r.differential {
            rd.rx_origin = self.transform_point(&rd.rx_origin);
            rd.ry_origin = self.transform_point(&rd.ry_origin);
            rd.rx_direction = self.transform_vector(&rd.rx_direction);
            rd.ry_direction = self.transform_vector(&rd.ry_direction);
            Ray{
                o,
                d,
                t_max,
                medium,
                differential: Some(rd),
                time: r.time
            }
        }
        else {
            Ray{
                o,
                d,
                t_max,
                medium,
                differential: None,
                time: r.time
            }
        }
    }

    /// Transform a Ray with given origin and direction errors, setting the accumlative error in o_error_out and d_error_out.
    pub fn transform_ray_with_abs_error(&self, r: &Ray, o_error_in: &Vector3f, d_error_in: &Vector3f, o_error_out: &mut Vector3f, d_error_out: &mut Vector3f) -> Ray {
        // this combines both transform for ray and ray differential
        let mut o = self.transform_point_with_abs_error(&r.o, o_error_in, o_error_out);
        let d = self.transform_vector_with_abs_error(&r.d, d_error_in, d_error_out);
        // Offset ray origin to edge of error bounds and compute _t_max_
        let length_squared = d.length_squared();
        let t_max = r.t_max;
        if length_squared > 0.0 {
            let dt = d.abs().dot(&o_error_out) / length_squared;
            o += d * dt;
            // t_max -= dt;
        }
        let medium = if let Some(ref medium) = r.medium {
            Some(medium.clone())
        }
        else {
            None
        };
        if let Some(mut rd) = r.differential {
            rd.rx_origin = self.transform_point(&rd.rx_origin);
            rd.ry_origin = self.transform_point(&rd.ry_origin);
            rd.rx_direction = self.transform_vector(&rd.rx_direction);
            rd.ry_direction = self.transform_vector(&rd.ry_direction);
            Ray{
                o,
                d,
                t_max,
                medium,
                differential: Some(rd),
                time: r.time
            }
        }
        else {
            Ray{
                o,
                d,
                t_max,
                medium,
                differential: None,
                time: r.time
            }
        }
    }

    /// Transform a given Bounds3f.
    pub fn transform_bounds<T>(&self, b: Box<dyn Bounding3<T>>) -> Box<dyn Bounding3<Float>>
    where
    T: Copy + std::ops::Sub<Output=T> + PartialOrd + num::NumCast + Add<Output=T> + Mul<Output=T>
    {
        b.transform(self)
    }

    /// Transform a given SurfaceInteraction.
    pub fn tranform_surface_interaction(&self, si: &SurfaceInteraction) -> SurfaceInteraction {
        let ret = SurfaceInteraction::default();
        // Transform _p_ and _pError_ in _SurfaceInteraction_
        ret.p = self.transform_point_with_abs_error(&si.p, &si.p_error, &mut ret.p_error);

        // Transform remaining members of _SurfaceInteraction_
        ret.n = self.transform_normal(&si.n).normalize();
        ret.wo = self.transform_vector(&si.wo).normalize();
        ret.time = si.time;
        ret.medium_interface = si.medium_interface.clone();
        ret.uv = si.uv;
        ret.shape = si.shape;
        ret.dpdu = self.transform_vector(&si.dpdu);
        ret.dpdv = self.transform_vector(&si.dpdv);
        ret.dndu = self.transform_normal(&si.dndu);
        ret.dndv = self.transform_normal(&si.dndv);
        ret.shading.n = self.transform_normal(&si.shading.n).normalize();
        ret.shading.dpdu = self.transform_vector(&si.shading.dpdu);
        ret.shading.dpdv = self.transform_vector(&si.shading.dpdv);
        ret.shading.dndu = self.transform_normal(&si.shading.dndu);
        ret.shading.dndv = self.transform_normal(&si.shading.dndv);
        ret.dudx = si.dudx.clone();
        ret.dvdx = si.dvdx.clone();
        ret.dudy = si.dudy.clone();
        ret.dvdy = si.dvdy.clone();
        ret.dpdx = Arc::new(RwLock::new(self.transform_vector(&si.dpdx.read().unwrap())));
        ret.dpdy = Arc::new(RwLock::new(self.transform_vector(&si.dpdy.read().unwrap())));
        ret.bsdf = si.bsdf;
        ret.bssrdf = si.bssrdf;
        ret.primitive = si.primitive;
        ret.shading.n = ret.shading.n.face_forward(&ret.n);
        ret.face_index = si.face_index;
        return ret;
    }

    /// Tests if handness is changed by this Transform. 
    pub fn swaps_handness(&self) -> bool {
        let det = self.m.m[0][0] * (self.m.m[1][1] * self.m.m[2][2] - self.m.m[1][2] * self.m.m[2][1]) -
            self.m.m[0][1] * (self.m.m[1][0] * self.m.m[2][2] - self.m.m[1][2] * self.m.m[2][0]) +
            self.m.m[0][2] * (self.m.m[1][0] * self.m.m[2][1] - self.m.m[1][1] * self.m.m[2][0]);
        det < 0.0
    }

    /// Create a transform into an orthographic space.
    pub fn orthographic(z_near: Float, z_far: Float) -> Transform {
        Transform::scale(1.0, 1.0, 1.0 / (z_far - z_near))
    }

    /// Create a transform into a perspective space.
    pub fn perspective(fov: Float, n: Float, f: Float) -> Transform {
        // Perform projective divide for perspective projection
        let persp = Matrix4x4::new(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, f / (f - n),
            -f * n / (f - n), 0.0, 0.0, 1.0, 0.0);
        
        // Scale canonical perspective view to specified field of view
        let inv_tan_ang = 1.0 / (radians(fov) / 2.0).tan();
        Transform::scale(inv_tan_ang, inv_tan_ang, 1.0)
    }
}

impl Default for Transform {
    fn default() -> Self {
        Transform{
            m: Matrix4x4::default(),
            m_inv: Matrix4x4::default()
        }
    }
}

impl PartialEq for Transform {
    fn eq(&self, rhs: &Transform) -> bool {
        self.m == rhs.m && self.m_inv == rhs.m_inv
    }
}

impl Mul for Transform {
    type Output = Transform;
    fn mul(self, rhs: Transform) -> Transform {
        Transform{
            m: self.m.mul(&rhs.m),
            m_inv: rhs.m_inv.mul(&self.m_inv)
        }
    }
}

impl From<Matrix4x4> for Transform {
    fn from(m: Matrix4x4) -> Transform {
        Transform{
            m: m.clone(),
            m_inv: m.inverse()
        }
    }
}

pub fn solve_linear_system2x2(a: [[Float; 2]; 2], b: [Float; 2], x0: &mut Float, x1: &mut Float) -> bool {
    let det = a[0][0] * a[1][1] - a[0][1] * a[1][0];
    if det.abs() < 1e-10 {
        return false;
    }
    *x0 = (a[1][1] * b[0] - a[0][1] * b[1]) / det;
    *x1 = (a[0][0] * b[1] - a[1][0] * b[0]) / det;
    if x0.is_nan() || *x1.is_nan() {
        return false;
    }
    true
}