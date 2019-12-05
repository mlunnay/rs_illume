use std::ops::*;
use std::fmt;
use super::super::pbrt::Float;
use super::Vector3;
use num;

pub type Normal3f = Normal3<Float>;

/// Representation of a surface normal.
#[derive(Debug, Default, Copy, Clone, PartialEq, PartialOrd)]
#[repr(C)]
pub struct Normal3<T> {
    pub x: T,
    pub y: T,
    pub z: T
}

impl<T> Normal3<T> {
    pub fn new(x: T, y: T, z: T) -> Self {
        Normal3::<T>{x, y, z}
    }

    /// Creates a new Normal3 with all components 0.
    #[inline]
    pub fn zero() -> Normal3<T>
    where
    T: num::Zero
    {
        Normal3::<T>{
            x: T::zero(),
            y: T::zero(),
            z: T::zero()
        }
    }

    /// Returns the dot product of two Normal3.
    pub fn dot(&self, o: &Normal3<T>) -> T
    where
    T: Copy + Add<Output = T> + Mul<Output = T>
    {
        self.x * o.x + self.y * o.y + self.z * o.z
    }

    /// Make sure that the Normal3 faces the same hemisphere as the given Normal3.
    pub fn face_forward(&self, o: &Normal3<T>) -> Normal3<T>
    where
    T: Copy + PartialOrd + Add<Output = T> + Mul<Output = T> + num::NumCast + Neg<Output = T>
    {
        if self.dot(o) < num::cast(0).unwrap() { -(*self) } else { *self }
    }

    /// Calculate the squared length of the Normal3.
    #[inline]
    pub fn length_squared(&self) -> T
    where
    T: Copy + Add<T, Output = T> + Mul<T, Output = T>
    {
        self.x * self.x + self.y * self.y + self.z * self.z
    }

    /// Calculate the length of the Normal3.
    #[inline]
    pub fn length(&self) -> Float
    where
    T: Copy + Add<T, Output = T> + Mul<T, Output = T> + Into<Float> {
        Float::sqrt((self.x * self.x + self.y * self.y + self.z * self.z).into())
    }

    pub fn normalize(&self) -> Normal3<Float>
    where
    T: Copy + Add<T, Output = T> + Mul<T, Output = T> + Into<Float>
    {
        let length: Float = 1.0 as Float / self.length();
        Normal3::<Float>{
            x: self.x.into() * length,
            y: self.y.into() * length,
            z: self.z.into() * length,
        }
    }

    /// Return the absolute values of the Normal3.
    pub fn abs(&self) -> Normal3<T>
    where
    T: num::Signed
    {
        Normal3::<T>{
            x: self.x.abs(),
            y: self.y.abs(),
            z: self.z.abs()
        }
    }
}

impl<T> From<Vector3<T>> for Normal3<T> {
    fn from(v: Vector3<T>) -> Normal3<T> {
        Normal3::<T>{
            x: v.x,
            y: v.y,
            z: v.z
        }
    }
}

impl<T> From<Normal3<T>> for Vector3<T> {
    fn from(n: Normal3<T>) -> Vector3<T> {
        Vector3::<T>{
            x: n.x,
            y: n.y,
            z: n.z
        }
    }
}

impl<T> Index<u8> for Normal3<T> {
    type Output = T;
    fn index(&self, index: u8) -> &T {
        match index {
            0 => &self.x,
            1 => &self.y,
            2 => &self.z,
            _ => panic!("Index out of range for Normal3"),
        }
    }
}

impl<T> IndexMut<u8> for Normal3<T> {
    fn index_mut(&mut self, index: u8) -> &mut T {
        match index {
            0 => &mut self.x,
            1 => &mut self.y,
            2 => &mut self.z,
            _ => panic!("Index out of range for Normal3"),
        }
    }
}

impl<T> Neg for Normal3<T>
where
T: Neg<Output = T>
{
    type Output = Normal3<T>;
    fn neg(self) -> Normal3<T> {
        Normal3::<T>{
            x: -self.x,
            y: -self.y,
            z: -self.z
        }
    }
}

impl<T> Add for Normal3<T>
where
T: Copy + Add<Output = T>
{
    type Output = Normal3<T>;
    fn add(self, o: Normal3<T>) -> Normal3<T> {
        Normal3::<T>{
            x: self.x + o.x,
            y: self.y + o.y,
            z: self.z + o.z
        }
    }
}

impl<T> AddAssign for Normal3<T>
where
T: Copy + AddAssign
{
    fn add_assign(&mut self, o: Normal3<T>) {
        self.x += o.x;
        self.y += o.y;
        self.z += o.z;
    }
}

impl<T> Sub for Normal3<T>
where
T: Copy + Sub<Output = T>
{
    type Output = Normal3<T>;
    fn sub(self, o: Normal3<T>) -> Normal3<T> {
        Normal3::<T>{
            x: self.x - o.x,
            y: self.y - o.y,
            z: self.z - o.z
        }
    }
}

impl<T> SubAssign for Normal3<T>
where
T: Copy + SubAssign
{
    fn sub_assign(&mut self, o: Normal3<T>) {
        self.x -= o.x;
        self.y -= o.y;
        self.z -= o.z;
    }
}

impl<T> Mul<T> for Normal3<T>
where
T: Copy + Mul<Output = T>
{
    type Output = Normal3<T>;
    fn mul(self, o: T) -> Normal3<T> {
        Normal3::<T>{
            x: self.x * o,
            y: self.y * o,
            z: self.z * o
        }
    }
}

reverse_mul_scalar!(Normal3, {x, y, z});

impl<T> MulAssign<T> for Normal3<T>
where
T: Copy + MulAssign
{
    fn mul_assign(&mut self, o: T) {
        self.x *= o;
        self.y *= o;
        self.z *= o;
    }
}

impl<T> Div<T> for Normal3<T>
where
T: Copy + Div<Output = T>
{
    type Output = Normal3<T>;
    fn div(self, o: T) -> Normal3<T> {
        Normal3::<T>{
            x: self.x / o,
            y: self.y / o,
            z: self.z / o
        }
    }
}

impl<T> DivAssign<T> for Normal3<T>
where
T: Copy + DivAssign
{
    fn div_assign(&mut self, o: T) {
        self.x /= o;
        self.y /= o;
        self.z /= o;
    }
}

impl<T> fmt::Display for Normal3<T>
where
T: fmt::Display
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "[ {}, {}, {} ]", self.x, self.y, self.z)
    }
}

/// Calculate the dot product between a Vector3 and a Normal3.
pub fn dot_vec_normal<T>(v: &Vector3<T>, n: &Normal3<T>) -> T
where
    T: Copy + Add<T, Output = T> + Mul<T, Output = T>
{
    v.x * n.x + v.y * n.y + v.z * n.z
}

/// Calculate the dot product between a Normal3 and a Vector3.
pub fn dot_normal_vec<T>(n: &Normal3<T>, v: &Vector3<T>) -> T
where
    T: Copy + Add<T, Output = T> + Mul<T, Output = T>
{
    v.x * n.x + v.y * n.y + v.z * n.z
}

/// Make sure that the Normal3 faces the same hemisphere as the given Vector3.
pub fn face_forward_vec_normal<T>(v: &Vector3<T>, n: &Normal3<T>) -> Normal3<T>
where
T: Copy + PartialOrd + Add<Output = T> + Mul<Output = T> + num::NumCast + Neg<Output = T>
{
    if dot_vec_normal(v, n) < num::cast(0).unwrap() { -(*n) } else { *n }
}

/// Make sure that the Normal3 faces the same hemisphere as the given Vector3.
pub fn face_forward_normal_vec<T>(n: &Normal3<T>, v: &Vector3<T>) -> Normal3<T>
where
T: Copy + PartialOrd + Add<Output = T> + Mul<Output = T> + num::NumCast + Neg<Output = T>
{
    if dot_vec_normal(v, n) < num::cast(0).unwrap() { -(*n) } else { *n }
}
