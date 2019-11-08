use std::ops::*;
use num;
use super::super::pbrt::Float;
use super::Vector3;
use super::Point2;

pub type Point3f = Point3<Float>;
pub type Point3i = Point3<i32>;

/// A 2D Point.
#[derive(Debug, Default, Copy, Clone, PartialEq, PartialOrd)]
#[repr(C)]
pub struct Point3<T> {
    pub x: T,
    pub y: T,
    pub z: T
}

impl<T> Point3<T> {
    pub fn new(x: T, y: T, z: T) -> Point3<T> {
        Point3::<T>{x, y, z}
    }

    /// Create a new Point3 with both components 0.
    pub fn zero() -> Point3<T>
    where
    T: num::Zero
    {
        Point3::<T>{
            x: T::zero(),
            y: T::zero(),
            z: T::zero()
        }
    }
    
    /// Cast method as std::convert::from cant be done for casting to same type but different parameters.
    #[inline]
    pub fn cast<U>(&self) -> Point3<U>
    where
    T: Copy + num::NumCast,
    U: num::NumCast
    {
        Point3::<U>{
            x: num::cast(self.x).unwrap(),
            y: num::cast(self.y).unwrap(),
            z: num::cast(self.y).unwrap()
        }
    }

    /// Calculate the squared distance between two Point3.
    #[inline]
    pub fn distance_squared(&self, o: &Point3<T>) -> T
    where
    T: Copy + Add<T, Output = T> + Mul<T, Output = T>
    {
        self.x * o.x + self.y * o.y + self.z * o.z
    }

    /// Calculate the distance between two Point3.
    #[inline]
    pub fn distance(&self, o: &Point3<T>) -> Float
    where
    T: num::NumCast + Copy + Add<T, Output = T> + Mul<T, Output = T> + Into<Float> {
        Float::sqrt((self.x * o.x + self.y * o.y + self.z * o.z).into())
    }

    /// Returns a component wise minimum.
    #[inline]
    pub fn min(&self, other: &Point3<T>) -> Point3<T>
    where
    T: Copy + PartialOrd
    {
        Point3::<T>{
            x: if self.x < other.x { self.x } else { other.x },
            y: if self.y < other.y { self.y } else { other.y },
            z: if self.z < other.z { self.z } else { other.z }
        }
    }

    /// Returns a component wise maximum.
    #[inline]
    pub fn max(&self, other: &Point3<T>) -> Point3<T>
    where
    T: Copy + PartialOrd
    {
        Point3::<T>{
            x: if self.x > other.x { self.x } else { other.x },
            y: if self.y > other.y { self.y } else { other.y },
            z: if self.z > other.z { self.z } else { other.z }
        }
    }

    /// Return the absolute coordinate values for this Point3
    #[inline]
    pub fn abs(&self) -> Point3<T>
    where
    T: num::Signed
    {
        Point3::<T>{
            x: self.x.abs(),
            y: self.y.abs(),
            z: self.y.abs()
        }
    }

    /// Return the ceiling of the coordinate values for this Point3
    #[inline]
    pub fn ceil(&self) -> Point3<T>
    where
    T: num::Float
    {
        Point3::<T>{
            x: self.x.ceil(),
            y: self.y.ceil(),
            z: self.y.ceil()
        }
    }

    /// Return the floor of the coordinate values for this Point3
    #[inline]
    pub fn floor(&self) -> Point3<T>
    where
    T: num::Float
    {
        Point3::<T>{
            x: self.x.floor(),
            y: self.y.floor(),
            z: self.y.floor()
        }
    }

    /// Permute the coordinate values according to the povided permutation.
    pub fn permute(&self, x: usize, y: usize, z: usize) -> Point3<T>
    where
    T: Copy
    {
        let v = [self.x, self.y];
        Point3::<T>{
            x: v[x],
            y: v[y],
            z: v[z]
        }
    }
}

impl Point3f {
    pub fn lerp(&self, p2: &Point3f, t: Float) -> Point3f {
        *self * (1.0 as Float - t) + *p2 * t
    }
}

impl<T> Index<u8> for Point3<T> {
    type Output = T;
    fn index(&self, index: u8) -> &T {
        match index {
            0 => &self.x,
            1 => &self.y,
            2 => &self.z,
            _ => panic!("Index out of range for Point3"),
        }
    }
}

impl<T> IndexMut<u8> for Point3<T> {
    fn index_mut(&mut self, index: u8) -> &mut T {
        match index {
            0 => &mut self.x,
            1 => &mut self.y,
            2 => &mut self.z,
            _ => panic!("Index out of range for Point3"),
        }
    }
}

impl<T> Neg for Point3<T>
where
T: Neg<Output = T>
{
    type Output = Point3<T>;
    fn neg(self) -> Point3<T> {
        Point3::<T>{
            x: -self.x,
            y: -self.y,
            z: -self.z
        }
    }
}

impl<T> Add<Vector3<T>> for Point3<T>
where
T: Copy + Add<Output = T>
{
    type Output = Point3<T>;
    fn add(self, rhs: Vector3<T>) -> Point3<T> {
        Point3::<T>{
            x: self.x + rhs.x,
            y: self.y + rhs.y,
			z: self.z + rhs.z
        }
    }
}

impl<T> AddAssign<Vector3<T>> for Point3<T>
where
T: Copy + AddAssign
{
    fn add_assign(&mut self, rhs: Vector3<T>) {
        self.x += rhs.x;
        self.y += rhs.y;
		self.z += rhs.z;
    }
}

impl<T> Add<Point3<T>> for Point3<T>
where
T: Copy + Add<Output = T>
{
    type Output = Point3<T>;
    fn add(self, rhs: Point3<T>) -> Point3<T> {
        Point3::<T>{
            x: self.x + rhs.x,
            y: self.y + rhs.y,
			z: self.z + rhs.z
        }
    }
}

impl<T> AddAssign<Point3<T>> for Point3<T>
where
T: Copy + AddAssign
{
    fn add_assign(&mut self, rhs: Point3<T>) {
        self.x += rhs.x;
        self.y += rhs.y;
		self.z += rhs.z;
    }
}

impl<T> Sub for Point3<T>
where
T: Copy + Sub<Output = T>
{
    type Output = Vector3<T>;
    fn sub(self, rhs: Point3<T>) -> Vector3<T> {
        Vector3::<T>{
            x: self.x - rhs.x,
            y: self.y - rhs.y,
		    z: self.z - rhs.z
        }
    }
}

impl<T> Sub<Vector3<T>> for Point3<T>
where
T: Copy + Sub<Output = T>
{
    type Output = Point3<T>;
    fn sub(self, rhs: Vector3<T>) -> Point3<T> {
        Point3::<T>{
            x: self.x - rhs.x,
            y: self.y - rhs.y,
		    z: self.z - rhs.z
        }
    }
}

impl<T> SubAssign<Vector3<T>> for Point3<T>
where
T: Copy + SubAssign
{
    fn sub_assign(&mut self, rhs: Vector3<T>) {
        self.x -= rhs.x;
        self.y -= rhs.y;
        self.z -= rhs.z;
    }
}

impl Div<Float> for Point3<Float>
{
    type Output = Point3<Float>;
    #[inline]
    fn div(self, rhs: Float) -> Point3<Float> {
        assert_ne!(rhs, 0.0);
        let inv = 1.0 / rhs;
        Point3::<Float>{
          x: self.x * inv,
          y: self.y * inv,
          z: self.z * inv
        }
    }
}

impl DivAssign<Float> for Point3<Float>
{
    #[inline]
    fn div_assign(&mut self, rhs: Float) {
        assert_ne!(rhs, 0.0);
        let inv = 1.0 / rhs;
        self.x /= inv;
        self.y /= inv;
        self.z /= inv;
    }
}

impl<T> Mul<T> for Point3<T>
  where
  T: Copy + Mul<Output = T>
  {
    type Output = Self;
    #[inline]
    fn mul(self, rhs: T) -> Self {
        Point3::<T>{
          x: self.x * rhs,
          y: self.y * rhs,
          z: self.z * rhs
        }
    }
}

impl<T> MulAssign<T> for Point3<T>
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

reverse_mul_scalar!(Point3, {x, y, z});

impl<T> Add<T> for Point3<T>
  where
  T: Copy + Add<Output = T>
  {
    type Output = Self;
    #[inline]
    fn add(self, rhs: T) -> Self {
        Point3::<T>{
          x: self.x + rhs,
          y: self.y + rhs,
          z: self.z + rhs
        }
    }
}

impl<T> AddAssign<T> for Point3<T>
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

impl<T> Sub<T> for Point3<T>
  where
  T: Copy + Sub<Output = T>
  {
    type Output = Self;
    #[inline]
    fn sub(self, rhs: T) -> Self {
        Point3::<T>{
          x: self.x - rhs,
          y: self.y - rhs,
          z: self.z - rhs
        }
    }
}

impl<T> SubAssign<T> for Point3<T>
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

impl<T, U> From<Vector3<U>> for Point3<T>
where
T: num::NumCast,
U: num::NumCast
{
    fn from(v: Vector3<U>) -> Point3<T> {
        Point3::<T>{
            x: num::cast(v.x).unwrap(),
            y: num::cast(v.y).unwrap(),
            z: num::cast(v.z).unwrap()
        }
    }
}

impl<T, U> From<Point2<U>> for Point3<T>
where
T: num::NumCast + num::Zero,
U: num::NumCast
{
    fn from(v: Point2<U>) -> Point3<T> {
        Point3::<T>{
            x: num::cast(v.x).unwrap(),
            y: num::cast(v.y).unwrap(),
            z: T::zero()
        }
    }
}