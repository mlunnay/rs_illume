use std::ops::*;
use std::fmt;
use num;
use super::super::pbrt::Float;
use super::Vector2;
use super::Point3;

pub type Point2f = Point2<Float>;
pub type Point2i = Point2<i32>;

/// A 2D Point.
#[derive(Debug, Default, Copy, Clone, PartialEq, PartialOrd)]
#[repr(C)]
pub struct Point2<T> {
    pub x: T,
    pub y: T
}

impl<T> Point2<T> {
    pub fn new(x: T, y: T) -> Point2<T> {
        Point2::<T>{x, y}
    }

    /// Create a new Point2 with both components 0.
    pub fn zero() -> Point2<T>
    where
    T: num::Zero
    {
        Point2::<T>{
            x: T::zero(),
            y: T::zero()
        }
    }
    /// Cast method as std::convert::from cant be done for casting to same type but different parameters.
    #[inline]
    pub fn cast<U>(&self) -> Point2<U>
    where
    T: Copy + num::NumCast,
    U: num::NumCast
    {
        Point2::<U>{
            x: num::cast(self.x).unwrap(),
            y: num::cast(self.y).unwrap()
        }
    }

    /// Calculate the squared distance between two Point2.
    #[inline]
    pub fn distance_squared(&self, o: &Point2<T>) -> T
    where
    T: Copy + Sub<T, Output = T> + Mul<T, Output = T> + Add<Output = T>
    {
        (*self - *o).length_squared()
    }

    /// Calculate the distance between two Point2.
    #[inline]
    pub fn distance(&self, o: &Point2<T>) -> Float
    where
    T: num::NumCast + Copy + Add<T, Output = T> + Mul<T, Output = T> + Into<Float> + Sub<Output=T> {
        (*self - *o).length()
    }

    /// Returns a component wise minimum.
    #[inline]
    pub fn min(&self, other: &Point2<T>) -> Point2<T>
    where
    T: Copy + PartialOrd
    {
        Point2::<T>{
            x: if self.x < other.x { self.x } else { other.x },
            y: if self.y < other.y { self.y } else { other.y }
        }
    }

    /// Returns a component wise maximum.
    #[inline]
    pub fn max(&self, other: &Point2<T>) -> Point2<T>
    where
    T: Copy + PartialOrd
    {
        Point2::<T>{
            x: if self.x > other.x { self.x } else { other.x },
            y: if self.y > other.y { self.y } else { other.y }
        }
    }

    /// Return the absolute coordinate values for this Point2
    #[inline]
    pub fn abs(&self) -> Point2<T>
    where
    T: num::Signed
    {
        Point2::<T>{
            x: self.x.abs(),
            y: self.y.abs()
        }
    }

    /// Return the ceiling of the coordinate values for this Point2
    #[inline]
    pub fn ceil(&self) -> Point2<T>
    where
    T: num::Float
    {
        Point2::<T>{
            x: self.x.ceil(),
            y: self.y.ceil()
        }
    }

    /// Return the floor of the coordinate values for this Point2
    #[inline]
    pub fn floor(&self) -> Point2<T>
    where
    T: num::Float
    {
        Point2::<T>{
            x: self.x.floor(),
            y: self.y.floor()
        }
    }

    /// Permute the coordinate values according to the povided permutation.
    pub fn permute(&self, x: usize, y: usize) -> Point2<T>
    where
    T: Copy
    {
        let v = [self.x, self.y];
        Point2::<T>{
            x: v[x],
            y: v[y]
        }
    }
}

impl Point2f {
    pub fn lerp(&self, p2: &Point2f, t: Float) -> Point2f {
        *self * (1.0 as Float - t) + *p2 * t
    }
}

impl<T> Index<u8> for Point2<T> {
    type Output = T;
    fn index(&self, index: u8) -> &T {
        match index {
            0 => &self.x,
            1 => &self.y,
            _ => panic!("Index out of range for Point2"),
        }
    }
}

impl<T> IndexMut<u8> for Point2<T> {
    fn index_mut(&mut self, index: u8) -> &mut T {
        match index {
            0 => &mut self.x,
            1 => &mut self.y,
            _ => panic!("Index out of range for Point2"),
        }
    }
}

impl<T> Add<Vector2<T>> for Point2<T>
where
T: Copy + Add<Output = T>
{
    type Output = Point2<T>;
    fn add(self, rhs: Vector2<T>) -> Point2<T> {
        Point2::<T>{
            x: self.x + rhs.x,
            y: self.y + rhs.y
        }
    }
}

impl<T> AddAssign<Vector2<T>> for Point2<T>
where
T: Copy + AddAssign
{
    fn add_assign(&mut self, rhs: Vector2<T>) {
        self.x += rhs.x;
        self.y += rhs.y;
    }
}

impl<T> Add<Point2<T>> for Point2<T>
where
T: Copy + Add<Output = T>
{
    type Output = Point2<T>;
    fn add(self, rhs: Point2<T>) -> Point2<T> {
        Point2::<T>{
            x: self.x + rhs.x,
            y: self.y + rhs.y
        }
    }
}

impl<T> AddAssign<Point2<T>> for Point2<T>
where
T: Copy + AddAssign
{
    fn add_assign(&mut self, rhs: Point2<T>) {
        self.x += rhs.x;
        self.y += rhs.y;
    }
}

impl<T> Sub for Point2<T>
where
T: Copy + Sub<Output = T>
{
    type Output = Vector2<T>;
    fn sub(self, rhs: Point2<T>) -> Vector2<T> {
        Vector2::<T>{
            x: self.x - rhs.x,
            y: self.y - rhs.y
        }
    }
}

impl<T> Sub<Vector2<T>> for Point2<T>
where
T: Copy + Sub<Output = T>
{
    type Output = Point2<T>;
    fn sub(self, rhs: Vector2<T>) -> Point2<T> {
        Point2::<T>{
            x: self.x - rhs.x,
            y: self.y - rhs.y
        }
    }
}

impl<T> SubAssign<Vector2<T>> for Point2<T>
where
T: Copy + SubAssign
{
    fn sub_assign(&mut self, rhs: Vector2<T>) {
        self.x -= rhs.x;
        self.y -= rhs.y;
    }
}

impl Div<Float> for Point2<Float>
{
    type Output = Point2<Float>;
    #[inline]
    fn div(self, rhs: Float) -> Point2<Float> {
        assert_ne!(rhs, 0.0);
        let inv = 1.0 / rhs;
        Point2::<Float>{
          x: self.x * inv,
          y: self.y * inv
        }
    }
}

impl DivAssign<Float> for Point2<Float>
{
    #[inline]
    fn div_assign(&mut self, rhs: Float) {
        assert_ne!(rhs, 0.0);
        let inv = 1.0 / rhs;
        self.x /= inv;
        self.y /= inv;
    }
}

impl<T> Mul<T> for Point2<T>
  where
  T: Copy + Mul<Output = T>
  {
    type Output = Self;
    #[inline]
    fn mul(self, rhs: T) -> Self {
        Point2::<T>{
          x: self.x * rhs,
          y: self.y * rhs
        }
    }
}

reverse_mul_scalar!(Point2, {x, y});

impl<T> MulAssign<T> for Point2<T>
where
T: Copy + MulAssign
{
    #[inline]
    fn mul_assign(&mut self, rhs: T) {
        self.x *= rhs;
        self.y *= rhs;
    }
}

impl<T> Add<T> for Point2<T>
  where
  T: Copy + Add<Output = T>
  {
    type Output = Self;
    #[inline]
    fn add(self, rhs: T) -> Self {
        Point2::<T>{
          x: self.x + rhs,
          y: self.y + rhs
        }
    }
}

impl<T> AddAssign<T> for Point2<T>
where
T: Copy + AddAssign
{
    #[inline]
    fn add_assign(&mut self, rhs: T) {
        self.x += rhs;
        self.y += rhs;
    }
}

impl<T> Sub<T> for Point2<T>
  where
  T: Copy + Sub<Output = T>
  {
    type Output = Self;
    #[inline]
    fn sub(self, rhs: T) -> Self {
        Point2::<T>{
          x: self.x - rhs,
          y: self.y - rhs
        }
    }
}

impl<T> SubAssign<T> for Point2<T>
where
T: Copy + SubAssign
{
    #[inline]
    fn sub_assign(&mut self, rhs: T) {
        self.x -= rhs;
        self.y -= rhs;
    }
}

impl<T, U> From<Vector2<U>> for Point2<T>
where
T: num::NumCast,
U: num::NumCast
{
    fn from(v: Vector2<U>) -> Point2<T> {
        Point2::<T>{
            x: num::cast(v.x).unwrap(),
            y: num::cast(v.y).unwrap()
        }
    }
}

impl<T, U> From<Point3<U>> for Point2<T>
where
T: num::NumCast,
U: num::NumCast
{
    fn from(v: Point3<U>) -> Point2<T> {
        Point2::<T>{
            x: num::cast(v.x).unwrap(),
            y: num::cast(v.y).unwrap()
        }
    }
}

impl<T> From<T> for Point2<T> {
    fn from(v: T) -> Point2<T> {
        Point2::<T>{
            x: v,
            y: v
        }
    }
}

impl<T> fmt::Display for Point2<T>
where
T: fmt::Display
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "[ {}, {} ]", self.x, self.y)
    }
}