use crate::core::pbrt::{Float, consts::PI};
use super::Vector3f;

/// Convert theta and phi to a direction vector.
#[inline]
pub fn spherical_direction(sin_theta: Float, cos_theta: Float, phi: Float) -> Vector3f {
    Vector3f::new(sin_theta * phi.cos(), sin_theta * phi.sin(), cos_theta)
}

/// Convert theta and phi to a direction vector in the coordinate system x,y,z.
#[inline]
pub fn spherical_direction_xyz(
    sin_theta: Float,
    cos_theta: Float,
    phi: Float,
    x: &Vector3f,
    y: &Vector3f,
    z: &Vector3f
) -> Vector3f {
    sin_theta * phi.cos() * *x + sin_theta * phi.sin() * *y + cos_theta * *z
}

/// Convert a direction vector into theta part of it spherical angle
#[inline]
pub fn spherical_theta(v: &Vector3f) -> Float {
    num::clamp(v.z, -1.0, 1.0).acos()
}
/// Convert a direction vector into phi part of it spherical angle
#[inline]
pub fn spherical_phi(v: &Vector3f) -> Float {
    let p = v.y.atan2(v.x);
    if p < 0.0 { p + 2.0 * PI } else { p }
}
