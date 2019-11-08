use super::pbrt::Float;
use super::matrix::Matrix4x4;
use super::geometry::Vector3f;
use std::ops::{Add, AddAssign, Sub, SubAssign, Div, DivAssign, Mul, MulAssign, Neg};
use super::transform::Transform;

#[derive(Debug, Copy, Clone, PartialEq, PartialOrd)]
pub struct Quaternion {
    pub v: Vector3f,
    pub w: Float
}

impl Quaternion {
    pub fn new(x: Float, y: Float, z: Float, w: Float) -> Quaternion {
        Quaternion{
            v: Vector3f{x, y, z},
            w
        }
    }

    /// Calculate the dot product between this Quaternion and another.
    pub fn dot(&self, q2: &Quaternion) -> Float {
        self.v.dot(&q2.v) + self.w * q2.w
    }

    /// Normalize this Quaternion.
    pub fn normalize(&self) -> Quaternion {
        *self / self.dot(&self).sqrt()
    }

    /// Interpolate between two Quaternions using spherical interpolation.
    pub fn slerp(&self, q2: &Quaternion, t: Float) -> Quaternion {
        let cos_theta = self.dot(q2);
        if cos_theta > 0.9995 {
            (*self * (1.0 - t) + *q2 * t).normalize()
        }
        else {
            let theta: Float = num::clamp(cos_theta, -1.0, 1.0).acos();
            let thetap = theta * t;
            let qperp = (*q2 - *self * cos_theta).normalize();
            *self * thetap.cos() + qperp * thetap.sin()
    }
    }
}

impl Default for Quaternion {
    fn default() -> Quaternion {
        Quaternion{
            v: Vector3f::default(),
            w: 1.0
        }
    }
}

impl Neg for Quaternion {
    type Output = Quaternion;
    fn neg(self) -> Quaternion {
        Quaternion{
            v: -self.v,
            w: -self.w
        }
    }
}

impl Add for Quaternion {
    type Output = Quaternion;
    fn add(self, rhs: Quaternion) -> Quaternion {
        Quaternion{
            v: self.v + rhs.v,
            w: self.w + rhs.w
        }
    }
}

impl AddAssign for Quaternion {
    fn add_assign(&mut self, rhs: Quaternion) {
        self.v += rhs.v;
        self.w += rhs.w;
    }
}

impl Sub for Quaternion {
    type Output = Quaternion;
    fn sub(self, rhs: Quaternion) -> Quaternion {
        Quaternion{
            v: self.v - rhs.v,
            w: self.w - rhs.w
        }
    }
}

impl SubAssign for Quaternion {
    fn sub_assign(&mut self, rhs: Quaternion) {
        self.v -= rhs.v;
        self.w -= rhs.w;
    }
}

impl Mul<Float> for Quaternion {
    type Output = Quaternion;
    fn mul(self, rhs: Float) -> Quaternion {
        Quaternion{
            v: self.v * rhs,
            w: self.w * rhs
        }
    }
}

impl MulAssign<Float> for Quaternion {
    fn mul_assign(&mut self, rhs: Float) {
        self.v *= rhs;
        self.w *= rhs;
    }
}

impl Div<Float> for Quaternion {
    type Output = Quaternion;
    fn div(self, rhs: Float) -> Quaternion {
        Quaternion{
            v: self.v / rhs,
            w: self.w / rhs
        }
    }
}

impl DivAssign<Float> for Quaternion {
    fn div_assign(&mut self, rhs: Float) {
        self.v /= rhs;
        self.w /= rhs;
    }
}

impl From<Quaternion> for Transform {
    fn from(q: Quaternion) -> Transform {
        let xx = q.v.x * q.v.x;
        let yy = q.v.y * q.v.y;
        let zz = q.v.z * q.v.z;
        let xy = q.v.x * q.v.y;
        let xz = q.v.x * q.v.z;
        let yz = q.v.y * q.v.z;
        let wx = q.v.x * q.w;
        let wy = q.v.y * q.w;
        let wz = q.v.z * q.w;

        let mut m = Matrix4x4::default();
        m.m[0][0] = 1.0 - 2.0 * (yy + zz);
        m.m[0][1] = 2.0 * (xy + wz);
        m.m[0][2] = 2.0 * (xz - wy);
        m.m[1][0] = 2.0 * (xy - wz);
        m.m[1][1] = 1.0 - 2.0 * (xx + zz);
        m.m[1][2] = 2.0 * (yz + wx);
        m.m[2][0] = 2.0 * (xz + wy);
        m.m[2][1] = 2.0 * (yz - wx);
        m.m[2][2] = 1.0 - 2.0 * (xx + yy);

        // Transpose since we are left-handed.  Ugh.
        Transform{
            m: m.transpose(),
            m_inv: m
        }
    }
}

impl From<Transform> for Quaternion {
    fn from(t: Transform) -> Quaternion {
        let m = t.m;
        let trace = m.m[0][0] + m.m[1][1] + m.m[2][2];
        let mut v = Vector3f::default();
        let w: Float;
        if trace > 0.0 {
            // Compute w from matrix trace, then xyz
            // 4w^2 = m[0][0] + m[1][1] + m[2][2] + m[3][3] (but m[3][3] == 1)
            let mut s = (trace + 1.0).sqrt();
            w = s / 2.0;
            s = 0.5 / s;
            v.x = (m.m[2][1] - m.m[1][2]) * s;
            v.y = (m.m[0][2] - m.m[2][0]) * s;
            v.z = (m.m[1][0] - m.m[0][1]) * s;
        } else {
            // Compute largest of $x$, $y$, or $z$, then remaining components
            let nxt = [1, 2, 0];
            let mut q = [0 as Float; 3];
            let mut i = 0;
            if m.m[1][1] > m.m[0][0] { i = 1 };
            if m.m[2][2] > m.m[i][i] { i = 2 };
            let j = nxt[i];
            let k = nxt[j];
            let mut s = ((m.m[i][i] - (m.m[j][j] + m.m[k][k])) + 1.0).sqrt();
            q[i] = s * 0.5;
            if s != 0.0 { s = 0.5 / s };
            w = (m.m[k][j] - m.m[j][k]) * s;
            q[j] = (m.m[j][i] + m.m[i][j]) * s;
            q[k] = (m.m[k][i] + m.m[i][k]) * s;
            v.x = q[0];
            v.y = q[1];
            v.z = q[2];
        }
        Quaternion{
            v,
            w
        }
    }
}