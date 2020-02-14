use super::{Point2, Point2i, Point2f, Vector2};
use num;
use std::ops::{Index, IndexMut, Sub, Add, Div, DivAssign, Mul};
use std::default::Default;
use crate::core::pbrt::{lerp, Float};
use std::fmt;

pub type Bounds2i = Bounds2<i32>;
pub type Bounds2f = Bounds2<Float>;

/// Trait for bounding shapes.
/// This allows for a mix of bounding shapes not just AABB.
pub trait Bounding2<T>
where
T: Copy + PartialOrd + Sub<Output = T> + Mul<Output = T>
{
    /// Returns a new bounding shape that encompases two shapes.
    fn union(&self, b: &Self) -> Self;

    /// Returns a new bounding shape that encompases this shape and the given Point2.
    fn union_point(&self, p: &Point2<T>) -> Self;

    /// Test if a Point2 is inside this bounding shape.
    fn inside(&self, p: &Point2<T>) -> bool;

    /// The vector along the box diagonal.
    fn diagonal(&self) -> Vector2<T>;

    /// Returns the index for the axis with the greatest length.
    fn maximum_extent(&self) -> u8 {
        let d = self.diagonal();
        if d.x > d.y { 0 } else { 1 }
    }

    /// Returns the axis aligned bounding box that encompases this bounding object.
    fn aabb(&self) -> Bounds2<T>;

    /// Returns the area of the bounding object.
    fn area(&self) -> T;
}

/// An axis aligned bounding box.
#[derive(Debug, Copy, Clone, PartialEq, PartialOrd)]
#[repr(C)]
pub struct Bounds2<T> {
    pub min: Point2<T>,
    pub max: Point2<T>
}

impl<T> Bounds2<T> {
    pub fn new(min: Point2<T>, max: Point2<T>) -> Self
    where
    T: Copy + PartialOrd
    {
        Bounds2::<T>{
            min: min.min(&max),
            max: min.max(&max)
        }
    }

    /// Creates a Bounds2 that encompases a single point.
    pub fn from_point(p: Point2<T>) -> Bounds2<T>
    where
    T: Copy
    {
        Bounds2::<T>{
            min: p,
            max: p
        }
    }

    /// Returns the point of the corner at index.
    pub fn corner(&self, corner: u8) -> Point2<T>
    where
    T: Copy
    {
        assert!(corner < 4);
        Point2::<T>::new(self[corner & 1].x,
            self[if corner & 2 == 1 { 1 } else { 0 }].y)
    }

    /// Returns the intersecting Bounds2 for two shapes.
    pub fn intersect(&self, b: &Bounds2<T>) -> Bounds2<T>
    where
    T: Copy + PartialOrd
    {
        Bounds2::<T>{
            min: self.max.min(&b.min),
            max: self.min.max(&b.max)
        }
    }

    /// Test if this Bounds2 overlaps another shape.
    pub fn overlaps(&self, b: &Bounds2<T>) -> bool
    where
    T: PartialOrd
    {
        self.max.x >= b.min.x && self.min.x <= b.max.x &&
        self.max.y >= b.min.y && self.min.y <= b.max.y
    }

    /// Test if a Point2 is inside this Bounds2, ignoring points on the upper boundary.
    pub fn inside_exclusive(&self, p: &Point2<T>) -> bool
    where
    T: PartialOrd
    {
        p.x >= self.min.x && p.x < self.max.x &&
        p.y >= self.min.y && p.y < self.max.y
    }

    /// Linearly interpolates between the corners of the box by the given amount in each dimension.
    pub fn lerp(&self, t: &Point2<T>) -> Point2<T>
    where
    T: num::One + Copy + Add<Output = T> + Sub<Output = T>
    {
        Point2::<T>{
            x: lerp(t.x, self.min.x, self.max.x),
            y: lerp(t.y, self.min.y, self.max.y)
        }
    }

    /// returns the continuous position of a point relative to the corners of the box,
    /// where a point at the minimum corner has offset (0,0,0), a point at the maximum corner has offset (1,1,1)
    pub fn offset(&self, p: &Point2<T>) -> Vector2<T>
    where
    T: Copy + PartialOrd + DivAssign + Sub<Output = T>
    {
        let mut o = *p - self.min;
        if self.max.x > self.min.x {
            o.x /= self.max.x - self.min.x
        }
        if self.max.y > self.min.y {
            o.y /= self.max.y - self.min.y
        }
        o
    }

    pub fn bounding_sphere(&self, center: &mut Point2f, radius: &mut Float)
    where
    T: num::NumCast + Add<Output=T> + Copy + Div<Output=T> + Sub<T, Output=T> + PartialOrd + Mul<Output = T>,
    f32: std::convert::From<T>,
    Self: Bounding2<T>
    {
        *center = (self.min + self.max).cast() / 2.0;
        let center_copy: Point2<T> = center.cast();
        *radius = if self.inside(&center_copy) {
            center_copy.distance(&self.max)
        }
        else {
            0.0
        }
    }

    pub fn expand(&self, delta: T) -> Bounds2<T>
    where
    T: Copy + Sub<Output=T> + Add<Output=T>
    {
        Bounds2::<T>{
            min: self.min - delta,
            max: self.max + delta
        }
    }
}

impl<T> Bounding2<T> for Bounds2<T>
where
T: Copy + PartialOrd + Sub<Output = T> + Mul<Output = T>
 {
    fn union(&self, b: &Bounds2<T>) -> Bounds2<T> {
        Bounds2::<T>{
            min: self.min.min(&b.min),
            max: self.max.max(&b.max)
        }
    }

    fn union_point(&self, p: &Point2<T>) -> Bounds2<T> {
        Bounds2::<T>{
            min: self.min.min(&p),
            max: self.max.max(&p)
        }
    }

    fn inside(&self, p: &Point2<T>) -> bool {
        p.x >= self.min.x && p.x <= self.max.x &&
        p.y >= self.min.y && p.y <= self.max.y
    }

    fn diagonal(&self) -> Vector2<T> {
        self.max - self.min
    }

    fn aabb(&self) -> Bounds2<T> {
        *self
    }

    fn area(&self) -> T {
        let d = self.max - self.min;
        d.x * d.y
    }
}

impl<T> Default for Bounds2<T>
where 
T: num::Bounded
{
    /// A default constructor with min and max set to violate min < max.
    fn default() -> Bounds2<T> {
        Bounds2::<T>{
            min: Point2::<T>::new(T::max_value(), T::max_value()),
            max: Point2::<T>::new(T::min_value(), T::min_value())
        }
    }
}

unsafe impl<T> Send for Bounds2<T> {}

impl<T> Index<u8> for Bounds2<T> {
    type Output = Point2<T>;
    fn index(&self, index: u8) -> &Point2<T> {
        match index {
            0 => &self.min,
            1 => &self.max,
            _ => panic!("Index out of range for Bounds2"),
        }
    }
}

impl<T> IndexMut<u8> for Bounds2<T> {
    fn index_mut(&mut self, index: u8) -> &mut Point2<T> {
        match index {
            0 => &mut self.min,
            1 => &mut self.max,
            _ => panic!("Index out of range for Bounds2"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bounding_sphere() {
        let bb = Bounds2::<f32>::new(Point2::<f32>::new(0.0, 0.0), Point2::<f32>::new(1.0, 1.0));
        let mut center = Point2f::zero();
        let mut radius = 0.0;
        bb.bounding_sphere(&mut center, &mut radius);
        println!("{:?}, {}", center, radius);
        assert_eq!(center.x, 0.5);
        assert_eq!(center.y, 0.5);
        assert_eq!(radius, (0.5_f32 * 0.5 * 2.0).sqrt());
    }
}

// Iterator
pub struct Bounds2Iterator<'a> {
    p: Point2i,
    bounds: &'a Bounds2i
}

impl<'a> Iterator for Bounds2Iterator<'a> {
    type Item = Point2i;

    fn next(&mut self) -> Option<Point2i> {
        self.p.x += 1;
        if self.p.x == self.bounds.max.x {
            self.p.x = self.bounds.min.x;
            self.p.y += 1;
        }
        if self.p.y == self.bounds.max.y {
            None
        }
        else {
            Some(self.p)
        }
    }
}

impl<'a> IntoIterator for &'a Bounds2i {
    type Item = Point2i;
    type IntoIter = Bounds2Iterator<'a>;

    fn into_iter(self) -> Self::IntoIter {
        Bounds2Iterator{
            p: Point2i{x: self.min.x - 1, y: self.min.y},
            bounds: self
        }
    }
}

impl<T> fmt::Display for Bounds2<T>
where
T: fmt::Display
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "[ {} - {} ]", self.min, self.max)
    }
}