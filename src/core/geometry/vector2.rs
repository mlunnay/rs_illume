use std::ops::*;
use std::fmt;
use num;
use super::super::pbrt::Float;
use super::Point2;

pub type Vector2f = Vector2<Float>;
pub type Vector2i = Vector2<i32>;

/// Representation of a 2D Vector.
#[derive(Debug, Default, Copy, Clone, PartialEq, PartialOrd)]
#[repr(C)]
pub struct Vector2<T> {
    pub x: T,
    pub y: T
}

impl<T> Vector2<T> {
    #[inline]
    pub fn new(x: T, y: T) -> Vector2<T> {
        Vector2::<T>{x, y}
    }

    #[inline]
    pub fn zero() -> Vector2<T>
    where
    T: num::Zero
    {
        Vector2::<T>{x: T::zero(), y: T::zero()}
    }

    #[inline]
    pub fn has_nans(&self) -> bool
    where
    T: num::Float
    {
        self.x.is_nan() || self.y.is_nan()
    }

    /// Calculate the squared length of the Vector2.
    #[inline]
    pub fn length_squared(&self) -> T
    where
    T: Copy + Add<T, Output = T> + Mul<T, Output = T>
    {
        self.x * self.x + self.y * self.y
    }

    /// Calculate the length of the Vector2.
    #[inline]
    pub fn length(&self) -> Float
    where
    T: num::NumCast + Copy + Add<T, Output = T> + Mul<T, Output = T> + Into<Float> {
        Float::sqrt((self.x * self.x + self.y * self.y).into())
    }

    /// Calculate the dot product of two Vector2.
    pub fn dot(&self, o: &Vector2<T>) -> T
    where
    T: Copy + Add<T, Output = T> + Mul<T, Output = T>
    {
        self.x * o.x + self.y * o.y
    }

    /// Returns the Vector2 normalized to unit length.
    pub fn normalize(&self) -> Vector2<Float>
    where
    T: Copy + num::NumCast + Add<T, Output = T> + Mul<T, Output = T> + Into<Float>
    {
        let length: Float = 1.0 as Float / self.length();
        Vector2::<Float>{
            x: self.x.into() * length,
            y: self.y.into() * length,
        }
    }

    /// Returns the smallest of x and y.
    pub fn min_component(&self) -> T
    where
    T: Copy + PartialOrd
    {
        if self.x < self.y { self.x } else { self.y }
    }

    /// Returns the largest of x and y.
    pub fn max_component(&self) -> T
    where
    T: Copy + PartialOrd
    {
        if self.x > self.y { self.x } else { self.y }
    }

    /// Returns a component wise minimum.
    #[inline]
    pub fn min(&self, other: &Vector2<T>) -> Vector2<T>
    where
    T: Copy + PartialOrd
    {
        Vector2::<T>{
            x: if self.x < other.x { self.x } else { other.x },
            y: if self.y < other.y { self.y } else { other.y }
        }
    }

    /// Returns a component wise maximum.
    #[inline]
    pub fn max(&self, other: &Vector2<T>) -> Vector2<T>
    where
    T: Copy + PartialOrd
    {
        Vector2::<T>{
            x: if self.x > other.x { self.x } else { other.x },
            y: if self.y > other.y { self.y } else { other.y }
        }
    }

    /// Retuns the index of the component with the greatest value.
    pub fn max_dimension(&self) -> usize
    where
    T: Copy + PartialOrd
    {
        if self.x > self.y { 0 } else { 1 }
    }

    /// Permute the coordinate values according to the povided permutation.
    pub fn permute(&self, x: usize, y: usize) -> Vector2<T>
    where
    T: Copy
    {
        let v = [self.x, self.y];
        Vector2::<T>{
            x: v[x],
            y: v[y]
        }
    }

    /// Return the absolute coordinate values for this Vector2
    #[inline]
    pub fn abs(&self) -> Vector2<T>
    where
    T: num::Signed
    {
        Vector2::<T>{
            x: self.x.abs(),
            y: self.y.abs()
        }
    }

    /// Cast method as std::convert::from cant be done for casting to same type but different parameters.
    #[inline]
    pub fn cast<U>(&self) -> Vector2<U>
    where
    T: Copy + num::NumCast,
    U: num::NumCast
    {
        Vector2::<U>{
            x: num::cast(self.x).unwrap(),
            y: num::cast(self.y).unwrap()
        }
    }
}

impl<T> Index<u8> for Vector2<T> {
    type Output = T;
    fn index(&self, index: u8) -> &T {
        match index {
            0 => &self.x,
            1 => &self.y,
            _ => panic!("Index out of range for Vector2"),
        }
    }
}

impl<T> IndexMut<u8> for Vector2<T> {
    fn index_mut(&mut self, index: u8) -> &mut T {
        match index {
            0 => &mut self.x,
            1 => &mut self.y,
            _ => panic!("Index out of range for Vector2"),
        }
    }
}

impl<T> Neg for Vector2<T>
where
T: Neg<Output = T>
{
    type Output = Self;

    fn neg(self) -> Vector2<T> {
        Vector2::<T>{
            x: -self.x,
            y: -self.y
        }
    }
}

impl<T> Div<Vector2<T>> for Vector2<T> 
where
T: Copy + Div<T, Output = T>
{
    type Output = Self;
    #[inline]
    fn div(self, rhs: Self) -> Self {
        Vector2::<T>{
            x: self.x / rhs.x,
            y: self.y / rhs.y
        }
    }
}

impl<T> DivAssign<Vector2<T>> for Vector2<T>
where
T: DivAssign
{
    #[inline]
    fn div_assign(&mut self, rhs: Self) {
        self.x /= rhs.x;
        self.y /= rhs.y;
    }
}

impl Div<Float> for Vector2<Float>
{
    type Output = Self;
    #[inline]
    fn div(self, rhs: Float) -> Self {
        assert_ne!(rhs, 0.0);
        let inv = 1.0 / rhs;
        Vector2::<Float>{
            x: self.x * inv,
            y: self.y * inv
        }
    }
}

impl DivAssign<Float> for Vector2<Float>
{
    #[inline]
    fn div_assign(&mut self, rhs: Float) {
        assert_ne!(rhs, 0.0);
        let inv = 1.0 / rhs;
        self.x *= inv;
        self.y *= inv;
    }
}

impl<T> Mul<Vector2<T>> for Vector2<T>
where
T: Mul<T, Output = T>
{
    type Output = Self;
    #[inline]
    fn mul(self, rhs: Self) -> Self {
        Vector2::<T>{
            x: self.x * rhs.x,
            y: self.y * rhs.y
        }
    }
}

reverse_mul_scalar!(Vector2, {x, y});

impl<T> MulAssign<Vector2<T>> for Vector2<T>
where
T: MulAssign
{
    #[inline]
    fn mul_assign(&mut self, rhs: Self) {
            self.x *= rhs.x;
            self.y *= rhs.y;
    }
}

impl<T> Mul<T> for Vector2<T>
where
T: Copy + Mul<T, Output = T>
{
    type Output = Self;
    #[inline]
    fn mul(self, rhs: T) -> Self {
        Vector2::<T>{
            x: self.x * rhs,
            y: self.y * rhs
        }
    }
}

impl<T> MulAssign<T> for Vector2<T>
where
T: Copy + MulAssign
{
    #[inline]
    fn mul_assign(&mut self, rhs: T) {
        self.x *= rhs;
        self.y *= rhs;
    }
}

impl<T> Add<Vector2<T>> for Vector2<T>
where
T: Add<T, Output = T>
{
    type Output = Self;
    #[inline]
    fn add(self, rhs: Self) -> Self {
        Vector2::<T>{
            x: self.x + rhs.x,
            y: self.y + rhs.y
        }
    }
}

impl<T> AddAssign<Vector2<T>> for Vector2<T>
where
T: AddAssign
{
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
            self.x += rhs.x;
            self.y += rhs.y;
    }
}

impl<T> Add<T> for Vector2<T>
where
T: Copy + Add<T, Output = T>
{
    type Output = Self;
    #[inline]
    fn add(self, rhs: T) -> Self {
        Vector2::<T>{
            x: self.x + rhs,
            y: self.y + rhs
        }
    }
}

impl<T> AddAssign<T> for Vector2<T>
where
T: Copy + AddAssign
{
    #[inline]
    fn add_assign(&mut self, rhs: T) {
        self.x += rhs;
        self.y += rhs;
    }
}

impl<T> Sub<Vector2<T>> for Vector2<T>
where
T: Sub<T, Output = T>
{
    type Output = Self;
    #[inline]
    fn sub(self, rhs: Self) -> Self {
        Vector2::<T>{
            x: self.x - rhs.x,
            y: self.y - rhs.y
        }
    }
}

impl<T> SubAssign<Vector2<T>> for Vector2<T>
where
T: SubAssign
{
    #[inline]
    fn sub_assign(&mut self, rhs: Self) {
            self.x -= rhs.x;
            self.y -= rhs.y;
    }
}

impl<T> Sub<T> for Vector2<T>
where
T: Copy + Sub<T, Output = T>
{
    type Output = Self;
    #[inline]
    fn sub(self, rhs: T) -> Self {
        Vector2::<T>{
            x: self.x - rhs,
            y: self.y - rhs
        }
    }
}

impl<T> SubAssign<T> for Vector2<T>
where
T: Copy + SubAssign
{
    #[inline]
    fn sub_assign(&mut self, rhs: T) {
        self.x -= rhs;
        self.y -= rhs;
    }
}

impl<T> From<(T, T)> for Vector2<T> {
    fn from(v: (T, T)) -> Vector2<T> {
        Vector2::<T>{
            x: v.0,
            y: v.1
        }
    }
}

impl<T> From<[T; 2]> for Vector2<T>
where
T: Copy
{
    fn from(v: [T; 2]) -> Vector2<T> {
        Vector2::<T>{
            x: v[0],
            y: v[1]
        }
    }
}

impl<T, U> From<Point2<U>> for Vector2<T>
where
T: num::NumCast,
U: num::NumCast
{
    fn from(p: Point2<U>) -> Vector2<T> {
        Vector2::<T>{
            x: num::cast(p.x).unwrap(),
            y: num::cast(p.y).unwrap()
        }
    }
}

impl<T> From<T> for Vector2<T> {
    fn from(v: T) -> Vector2<T> {
        Vector2::<T>{
            x: v,
            y: v
        }
    }
}

impl<T> fmt::Display for Vector2<T>
where
T: fmt::Display
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "[ {}, {} ]", self.x, self.y)
    }
}