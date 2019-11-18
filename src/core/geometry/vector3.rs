use std::ops::*;
use num;
use super::super::pbrt::Float;
use super::Point3;

pub type Vector3f = Vector3<Float>;
pub type Vector3i = Vector3<i32>;

/// Representation of a 3D Vector.
#[derive(Debug, Default, Copy, Clone, PartialEq, PartialOrd)]
#[repr(C)]
pub struct Vector3<T> {
    pub x: T,
    pub y: T,
    pub z: T
}

impl<T> Vector3<T> {
    #[inline]
    pub fn new(x: T, y: T, z: T) -> Vector3<T> {
        Vector3::<T>{x, y, z}
    }

    #[inline]
    pub fn zero() -> Vector3<T>
    where
    T: num::Zero
    {
        Vector3::<T>{x: T::zero(), y: T::zero(), z: T::zero()}
    }

    #[inline]
    pub fn has_nans(&self) -> bool
    where
    T: num::Float
    {
        self.x.is_nan() || self.y.is_nan() || self.z.is_nan()
    }

    /// Calculate the squared length of the Vector3.
    #[inline]
    pub fn length_squared(&self) -> T
    where
    T: Copy + Add<T, Output = T> + Mul<T, Output = T>
    {
        self.x * self.x + self.y * self.y + self.z * self.z
    }

    /// Calculate the length of the Vector3.
    #[inline]
    pub fn length(&self) -> Float
    where
    T: Copy + Add<T, Output = T> + Mul<T, Output = T> + Into<Float> {
        Float::sqrt((self.x * self.x + self.y * self.y + self.z * self.z).into())
    }

    /// Calculate the dot product of two Vector3.
    pub fn dot(&self, o: &Vector3<T>) -> T
    where
    T: Copy + Add<T, Output = T> + Mul<T, Output = T>
    {
        self.x * o.x + self.y * o.y + self.z * o.z
    }

    /// Calculate the cross product of two Vector3
    pub fn cross(&self, o: &Vector3<T>) -> Vector3<Float>
    where
    T: Copy + Add<T, Output = T> + Mul<T, Output = T> + Into<f64>
    {
        let v1x: f64 = self.x.into() ;
        let v1y: f64 = self.y.into();
        let v1z: f64 = self.z.into();
        let v2x: f64 = o.x.into();
        let v2y: f64 = o.y.into();
        let v2z: f64 = o.z.into();
        Vector3::<Float>{
            x: ((v1y * v2z) - (v1z * v2y)) as Float,
            y: ((v1z * v2x) - (v1x * v2z)) as Float,
            z: ((v1x * v2y) - (v1y * v2x)) as Float
        }
    }

    /// Returns the Vector3 normalized to unit length.
    pub fn normalize(&self) -> Vector3<Float>
    where
    T: Copy + Add<T, Output = T> + Mul<T, Output = T> + Into<Float>
    {
        let length: Float = 1.0 as Float / self.length();
        Vector3::<Float>{
            x: self.x.into() * length,
            y: self.y.into() * length,
            z: self.z.into() * length,
        }
    }

    /// Returns the smallest of x and y.
    pub fn min_component(&self) -> T
    where
    T: Copy + PartialOrd
    {
        if self.x < self.y { self.x } else if self.y < self.z { self.y } else { self.z }
    }

    /// Returns the largest of x and y.
    pub fn max_component(&self) -> T
    where
    T: Copy + PartialOrd
    {
        if self.x > self.y { self.x } else if self.y > self.z { self.y } else { self.z }
    }

    /// Returns a component wise minimum.
    #[inline]
    pub fn min(&self, other: &Vector3<T>) -> Vector3<T>
    where
    T: Copy + PartialOrd
    {
        Vector3::<T>{
            x: if self.x < other.x { self.x } else { other.x },
            y: if self.y < other.y { self.y } else { other.y },
            z: if self.z < other.z { self.z } else { other.y }
        }
    }

    /// Returns a component wise maximum.
    #[inline]
    pub fn max(&self, other: &Vector3<T>) -> Vector3<T>
    where
    T: Copy + PartialOrd
    {
        Vector3::<T>{
            x: if self.x > other.x { self.x } else { other.x },
            y: if self.y > other.y { self.y } else { other.y },
            z: if self.z > other.z { self.z } else { other.z }
        }
    }

    /// Retuns the index of the component with the greatest value.
    pub fn max_dimension(&self) -> usize
    where
    T: Copy + PartialOrd
    {
        if self.x > self.y {
            if self.x > self.z { 0 } else { 2 }
        } else {
            if self.y > self.z { 1 } else { 2 }
        }
    }

    /// Permute the coordinate values according to the povided permutation.
    pub fn permute(&self, x: usize, y: usize, z: usize) -> Vector3<T>
    where
    T: Copy
    {
        let v = [self.x, self.y, self.z];
        Vector3::<T>{
            x: v[x],
            y: v[y],
            z: v[z]
        }
    }

    /// Return the absolute coordinate values for this Vector3
    #[inline]
    pub fn abs(&self) -> Vector3<T>
    where
    T: num::Signed
    {
        Vector3::<T>{
            x: self.x.abs(),
            y: self.y.abs(),
            z: self.z.abs(),
        }
    }

    /// Cast method as std::convert::from cant be done for casting to same type but different parameters.
    #[inline]
    pub fn cast<U>(&self) -> Vector3<U>
    where
    T: Copy + num::NumCast,
    U: num::NumCast
    {
        Vector3::<U>{
            x: num::cast(self.x).unwrap(),
            y: num::cast(self.y).unwrap(),
            z: num::cast(self.z).unwrap()
        }
    }
}

impl<T> Index<u8> for Vector3<T> {
    type Output = T;
    fn index(&self, index: u8) -> &T {
        match index {
            0 => &self.x,
            1 => &self.y,
            2 => &self.z,
            _ => panic!("Index out of range for Vector3"),
        }
    }
}

impl<T> IndexMut<u8> for Vector3<T> {
    fn index_mut(&mut self, index: u8) -> &mut T {
        match index {
            0 => &mut self.x,
            1 => &mut self.y,
            2 => &mut self.z,
            _ => panic!("Index out of range for Vector3"),
        }
    }
}

impl<T> Index<usize> for Vector3<T> {
    type Output = T;
    fn index(&self, index: usize) -> &T {
        match index {
            0 => &self.x,
            1 => &self.y,
            2 => &self.z,
            _ => panic!("Index out of range for Vector3"),
        }
    }
}

impl<T> IndexMut<usize> for Vector3<T> {
    fn index_mut(&mut self, index: usize) -> &mut T {
        match index {
            0 => &mut self.x,
            1 => &mut self.y,
            2 => &mut self.z,
            _ => panic!("Index out of range for Vector3"),
        }
    }
}

impl<T> Neg for Vector3<T>
where
T: Neg<Output = T>
{
    type Output = Self;

    fn neg(self) -> Vector3<T> {
        Vector3::<T>{
            x: -self.x,
            y: -self.y,
            z: -self.z
        }
    }
}

impl<T> Div<Vector3<T>> for Vector3<T> 
where
T: Copy + Div<T, Output = T>
{
    type Output = Self;
    #[inline]
    fn div(self, rhs: Self) -> Self {
        Vector3::<T>{
          x: self.x / rhs.x,
          y: self.y / rhs.y,
          z: self.z / rhs.z
        }
    }
}

impl<T> DivAssign<Vector3<T>> for Vector3<T>
where
T: DivAssign
{
    #[inline]
    fn div_assign(&mut self, rhs: Self) {
        self.x /= rhs.x;
        self.y /= rhs.y;
		self.z /= rhs.z;
    }
}

impl Div<Float> for Vector3<Float>
{
    type Output = Self;
    #[inline]
    fn div(self, rhs: Float) -> Self {
        assert_ne!(rhs, 0.0);
        let inv = 1.0 / rhs;
        Vector3::<Float>{
            x: self.x * inv,
            y: self.y * inv,
	        z: self.z * inv
        }
    }
}

impl DivAssign<Float> for Vector3<Float>
{
    #[inline]
    fn div_assign(&mut self, rhs: Float) {
        assert_ne!(rhs, 0.0);
        let inv = 1.0 / rhs;
        self.x *= inv;
        self.y *= inv;
        self.z *= inv;
    }
}

impl<T> Mul<Vector3<T>> for Vector3<T>
where
T: Mul<T, Output = T>
{
    type Output = Self;
    #[inline]
    fn mul(self, rhs: Self) -> Self {
        Vector3::<T>{
            x: self.x * rhs.x,
            y: self.y * rhs.y,
			z: self.z * rhs.z
        }
    }
}

impl<T> MulAssign<Vector3<T>> for Vector3<T>
where
T: MulAssign
{
    #[inline]
    fn mul_assign(&mut self, rhs: Self) {
        self.x *= rhs.x;
        self.y *= rhs.y;
		self.z *= rhs.z;
    }
}

impl<T> Mul<T> for Vector3<T>
where
T: Copy + Mul<T, Output = T>
{
    type Output = Self;
    #[inline]
    fn mul(self, rhs: T) -> Self {
        Vector3::<T>{
            x: self.x * rhs,
            y: self.y * rhs,
			z: self.z * rhs
        }
    }
}

reverse_mul_scalar!(Vector3, {x, y, z});

impl<T> MulAssign<T> for Vector3<T>
where
T: Copy + MulAssign
{
    #[inline]
    fn mul_assign(&mut self, rhs: T) {
        self.x *= rhs;
        self.y *= rhs;
        self.z *= rhs;
    }
}

impl<T> Add<Vector3<T>> for Vector3<T>
where
T: Add<T, Output = T>
{
    type Output = Self;
    #[inline]
    fn add(self, rhs: Self) -> Self {
        Vector3::<T>{
            x: self.x + rhs.x,
            y: self.y + rhs.y,
			z: self.z + rhs.z
        }
    }
}

impl<T> AddAssign<Vector3<T>> for Vector3<T>
where
T: AddAssign
{
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        self.x += rhs.x;
        self.y += rhs.y;
		self.z += rhs.z;
    }
}

impl<T> Add<T> for Vector3<T>
where
T: Copy + Add<T, Output = T>
{
    type Output = Self;
    #[inline]
    fn add(self, rhs: T) -> Self {
        Vector3::<T>{
            x: self.x + rhs,
            y: self.y + rhs,
			z: self.z + rhs
        }
    }
}

impl<T> AddAssign<T> for Vector3<T>
where
T: Copy + AddAssign
{
    #[inline]
    fn add_assign(&mut self, rhs: T) {
        self.x += rhs;
        self.y += rhs;
        self.z += rhs;
    }
}

impl<T> Sub<Vector3<T>> for Vector3<T>
where
T: Sub<T, Output = T>
{
    type Output = Self;
    #[inline]
    fn sub(self, rhs: Self) -> Self {
        Vector3::<T>{
            x: self.x - rhs.x,
            y: self.y - rhs.y,
			z: self.z - rhs.z
        }
    }
}

impl<T> SubAssign<Vector3<T>> for Vector3<T>
where
T: SubAssign
{
    #[inline]
    fn sub_assign(&mut self, rhs: Self) {
        self.x -= rhs.x;
        self.y -= rhs.y;
		self.z -= rhs.z;
    }
}

impl<T> Sub<T> for Vector3<T>
where
T: Copy + Sub<T, Output = T>
{
    type Output = Self;
    #[inline]
    fn sub(self, rhs: T) -> Self {
        Vector3::<T>{
            x: self.x - rhs,
            y: self.y - rhs,
			z: self.z - rhs
        }
    }
}

impl<T> SubAssign<T> for Vector3<T>
where
T: Copy + SubAssign
{
    #[inline]
    fn sub_assign(&mut self, rhs: T) {
        self.x -= rhs;
        self.y -= rhs;
        self.z -= rhs;
    }
}

impl<T> From<(T, T, T)> for Vector3<T> {
    fn from(v: (T, T, T)) -> Vector3<T> {
        Vector3::<T>{
            x: v.0,
            y: v.1,
            z: v.2
        }
    }
}

impl<T> From<[T; 3]> for Vector3<T>
where
T: Copy
{
    fn from(v: [T; 3]) -> Vector3<T> {
        Vector3::<T>{
            x: v[0],
            y: v[1],
            z: v[2]
        }
    }
}

impl<T, U> From<Point3<U>> for Vector3<T>
where
T: num::NumCast,
U: num::NumCast
{
    fn from(p: Point3<U>) -> Vector3<T> {
        Vector3::<T>{
            x: num::cast(p.x).unwrap(),
            y: num::cast(p.y).unwrap(),
            z: num::cast(p.z).unwrap()
        }
    }
}

/// Construct a local coordinate system given only a single 3D vector.
/// Expects v1 to be normalized.
pub fn coordinate_system(v1: &Vector3<Float>, v2: &mut Vector3<Float>, v3: &mut Vector3<Float>) {
    if v1.x.abs() > v1.y.abs() {
        let length = (v1.x * v1.x + v1.z * v1.z).sqrt();
        *v2 = Vector3::<Float>{
            x: -v1.z / length,
            y: 0.0 as Float,
            z: v1.x / length
        }
    }
    else {
        let length = Float::sqrt((v1.y * v1.y + v1.z * v1.z).into());
        *v2 = Vector3::<Float>{
            x: 0.0 as Float,
            y: v1.z / length,
            z: -v1.y / length
        }
    }
    *v3 = v1.cross(&*v2);
}