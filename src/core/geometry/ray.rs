use crate::core::pbrt::{Float, next_float_up, next_float_down, consts::PI};
use super::{Vector3f, Point3f, Normal3f};
use crate::core::medium::Medium;
use std::sync::Arc;
use num;
use std::fmt;

/// A Ray.
#[derive(Clone)]
#[repr(C)]
pub struct Ray {
    pub o: Point3f,
    pub d: Vector3f,
    pub t_max: Float,
    pub time: Float,
    pub medium: Option<Arc<dyn Medium + Send + Sync>>,
    /// In C++ differential is a subclass of Ray.
    pub differential: Option<RayDifferential>
}

impl Ray {
    pub fn new(o: Point3f, d: Vector3f) -> Ray {
        Ray{
            o,
            d,
            t_max: num::Float::infinity(),
            time: num::Zero::zero(),
            medium: None,
            differential: None
        }
    }

    /// Return the Point3 at a given time along the ray.
    /// This is a replacement for the C++ overloaded function application operator.
    pub fn point_at_time(&self, t: Float) -> Point3f {
        self.o + self.d * t
    }

    /// Update the scale diffentials for an estimated sampling space s.
    pub fn scale_differentials(&mut self, s: Float) {
        if let Some(d) = self.differential.iter_mut().next() {
            d.rx_origin = self.o + (d.rx_origin - self.o) * s;
            d.ry_origin = self.o + (d.ry_origin - self.o) * s;
            d.rx_direction = self.d + (d.rx_direction - self.d) * s;
            d.ry_direction = self.d + (d.ry_direction - self.d) * s;
        }
    }
}

impl Default for Ray {
    fn default() -> Ray {
        Ray{
            o: Point3f::zero(),
            d: Vector3f::zero(),
            t_max: num::Float::infinity(),
            time: num::Zero::zero(),
            medium: None,
            differential: None
        }
    }
}

impl fmt::Display for Ray {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "[o={}, d={}, t_max={}, time={}]", self.o, self.d, self.t_max, self.time)
    }
}

/// Auxillary rays for Texture antialiasing.
#[derive(Default, Clone, Copy, Debug)]
pub struct RayDifferential {
    pub rx_origin: Point3f,
    pub ry_origin: Point3f,
    pub rx_direction: Vector3f,
    pub ry_direction: Vector3f
}

/// Offset a ray origin along the surface normal accounting for floating point errors.
pub fn offset_ray_origin(
    p: &Point3f,
    p_error: &Vector3f,
    n: &Normal3f,
    w: &Vector3f) -> Point3f 
{
    let nv: Vector3f = (*n).into();
    let d = nv.abs().dot(p_error);
    let mut offset: Vector3f = Vector3f::from(*n) * d;
    if w.dot(&nv) < 0.0 {
        offset = -offset;
    }
    let mut po = *p + offset;
    // Round offset point _po_ away from _p_
    for i in 0..3 {
        if offset[i] > 0.0 {
            po[i] = next_float_up(po[i]);
        }
        else if offset[i] < 0.0 {
            po[i] = next_float_down(po[i]);
        }
    }
    return po;
}

/// Calculate appropriate direction vector from two angles.
pub fn spherical_direction(sin_theta: Float, cos_theta: Float, phi: Float) -> Vector3f {
    Vector3f {
        x: sin_theta * phi.cos(),
        y: sin_theta * phi.sin(),
        z: cos_theta,
    }
}

/// Take three basis vectors representing the x, y, and z axes and
/// return the appropriate direction vector with respect to the
/// coordinate frame defined by them.
pub fn spherical_direction_vec3(
    sin_theta: Float,
    cos_theta: Float,
    phi: Float,
    x: &Vector3f,
    y: &Vector3f,
    z: &Vector3f,
) -> Vector3f {
    *x * (sin_theta * phi.cos()) + *y * (sin_theta * phi.sin()) + *z * cos_theta
}

/// Conversion of a direction to spherical angles. Note that
/// **spherical_theta()** assumes that the vector **v** has been
/// normalized before being passed in.
pub fn spherical_theta(v: &Vector3f) -> Float {
    num::clamp(v.z, -1.0, 1.0).acos()
}

/// Conversion of a direction to spherical angles.
pub fn spherical_phi(v: &Vector3f) -> Float {
    let p: Float = v.y.atan2(v.x);
    if p < 0.0 {
        p + 2.0 * PI
    } else {
        p
    }
}