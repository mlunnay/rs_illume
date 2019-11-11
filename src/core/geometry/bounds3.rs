use super::{Point3, Point3i, Point3f, Vector3, Vector3f, Ray};
use num;
use std::ops::{Index, IndexMut, Sub, Add, Div, DivAssign, Mul};
use std::default::Default;
use crate::core::pbrt::{lerp, Float, gamma};
use crate::core::transform::Transform;

pub type Bounds3i = Bounds3<i32>;
pub type Bounds3f = Bounds3<Float>;

/// Trait for bounding shapes.
/// This allows for a mix of bounding shapes not just AABB.
pub trait Bounding3<T: Copy + PartialOrd + Sub<Output = T> + num::NumCast> {
    /// Test if a Point3 is inside this bounding shape.
    fn inside(&self, p: &Point3<T>) -> bool;

    /// The vector along the box diagonal.
    fn diagonal(&self) -> Vector3<T>;

    /// Test if the Ray intersects this Bounding3, setting the parametric range to hitt0 and hitt1.
    fn intersect_ray(&self, ray: &Ray, hitt0: &mut Float, hitt1: &mut Float) -> bool;

    /// Test if the Ray intersects this Bounding3. This takes the precomputed reciprical of the rays direction,
    /// and values indicating if the directions are negative.
    /// This should return true if the ray segment is inside the Bounding object, even if the intersections are not within the ray’s (0, t_max) range.
    fn intersect_p(&self, ray: &Ray, inv_dir: &Vector3f, dir_is_neg: [usize; 3]) -> bool;

    /// Transform the bounding shape.
    fn transform(&self, t: &Transform) -> Box<dyn Bounding3<Float>>;

    /// Return the axis aligned bounding box that encompases 
    fn aabb(&self) -> Bounds3<T>;

    /// Returns the union of this and another bounding object.
    /// This merges the axis aligned bounding boxes of both objects.
    fn union(&self, rhs: &dyn Bounding3<T>) -> Box<dyn Bounding3<T> + 'static>
    where
    T: 'static
    {
        let a = self.aabb();
        let b = rhs.aabb();
        Box::new(Bounds3::<T>{
            min: a.min.min(&b.min),
            max: a.max.max(&b.max)
        })
    }
}

/// An axis aligned bounding box.
#[derive(Debug, Copy, Clone, PartialEq, PartialOrd)]
#[repr(C)]
pub struct Bounds3<T> {
    pub min: Point3<T>,
    pub max: Point3<T>
}

impl<T> Bounds3<T> {
    pub fn new(min: Point3<T>, max: Point3<T>) -> Self
    where
    T: Copy + PartialOrd
    {
        Bounds3::<T>{
            min: min.min(&max),
            max: min.max(&max)
        }
    }

    /// Creates a Bounds3 that encompases a single point.
    pub fn from_point(p: Point3<T>) -> Bounds3<T>
    where
    T: Copy
    {
        Bounds3::<T>{
            min: p,
            max: p
        }
    }


    /// Cast method as std::convert::from cant be done for casting to same type but different parameters.
    #[inline]
    pub fn cast<U>(&self) -> Bounds3<U>
    where
    T: Copy + num::NumCast,
    U: num::NumCast
    {
        Bounds3::<U>{
            min: Point3::<U> {
                x: num::cast(self.min.x).unwrap(),
                y: num::cast(self.min.y).unwrap(),
                z: num::cast(self.min.z).unwrap()
            },
            max: Point3::<U> {
                x: num::cast(self.max.x).unwrap(),
                y: num::cast(self.max.y).unwrap(),
                z: num::cast(self.max.z).unwrap()
            }
        }
    }

    /// Returns the point of the corner at index.
    pub fn corner(&self, corner: u8) -> Point3<T>
    where
    T: Copy
    {
        assert!(corner < 4);
        Point3::<T>::new(self[corner & 1].x,
            self[if corner & 2 == 1 { 1 } else { 0 }].y,
            self[if corner & 4 == 1 { 1 } else { 0 }].z
            )
    }

    /// Returns the intersecting Bounds3 for two shapes.
    pub fn intersect(&self, b: &Bounds3<T>) -> Bounds3<T>
    where
    T: Copy + PartialOrd
    {
        Bounds3::<T>{
            min: self.max.min(&b.min),
            max: self.min.max(&b.max)
        }
    }

    /// Test if this Bounds3 overlaps another shape.
    pub fn overlaps(&self, b: &Bounds3<T>) -> bool
    where
    T: PartialOrd
    {
        self.max.x >= b.min.x && self.min.x <= b.max.x &&
        self.max.y >= b.min.y && self.min.y <= b.max.y
    }

    /// Test if a Point3 is inside this Bounds3, ignoring points on the upper boundary.
    pub fn inside_exclusive(&self, p: Point3<T>) -> bool
    where
    T: PartialOrd
    {
        p.x >= self.min.x && p.x < self.max.x &&
        p.y >= self.min.y && p.y < self.max.y
    }

    /// Linearly interpolates between the corners of the box by the given amount in each dimension.
    pub fn lerp(&self, t: Point3<T>) -> Point3<T>
    where
    T: num::One + Copy + Add<Output = T> + Sub<Output = T>
    {
        Point3::<T>{
            x: lerp(t.x, self.min.x, self.max.x),
            y: lerp(t.y, self.min.y, self.max.y),
            z: lerp(t.z, self.min.z, self.max.z)
        }
    }

    /// returns the continuous position of a point relative to the corners of the box,
    /// where a point at the minimum corner has offset (0,0,0), a point at the maximum corner has offset (1,1,1)
    pub fn offset(&self, p: &Point3<T>) -> Vector3<T>
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
        if self.max.z > self.min.z {
            o.z /= self.max.z - self.min.z
        }
        o
    }

    pub fn bounding_sphere(&self, center: &mut Point3f, radius: &mut Float)
    where
    T: num::NumCast + Add<Output=T> + Copy + Div<Output=T> + Sub<T, Output=T> + PartialOrd + Mul<Output = T>,
    f32: std::convert::From<T>
    {
        *center = (self.min + self.max).cast() / 2.0;
        let center_copy: Point3<T> = center.cast();
        *radius = if self.inside(&center_copy) {
            center_copy.distance(&self.max)
        }
        else {
            0.0
        }
    }

    /// Expands this Bounds3 by a given amount on each side.
    pub fn expand(&self, delta: T) -> Bounds3<T>
    where
    T: Sub<Output=T> + Add<Output=T>
    {
        Bounds3::<T>{
            min: Point3::<T>{
                x: self.min.x - delta,
                y: self.min.y - delta,
                z: self.min.z - delta,
            },
            max: Point3::<T>{
                x: self.min.x + delta,
                y: self.min.y + delta,
                z: self.min.z + delta,
            }
        }
    }
}

impl<T> Bounding3<T> for Bounds3<T>
where
T: Copy + PartialOrd + Sub<Output = T> + num::NumCast
{
    fn inside(&self, p: &Point3<T>) -> bool {
        p.x >= self.min.x && p.x <= self.max.x &&
        p.y >= self.min.y && p.y <= self.max.y
    }

    fn diagonal(&self) -> Vector3<T> {
        self.max - self.min
    }

    fn intersect_ray(&self, ray: &Ray, hitt0: &mut Float, hitt1: &mut Float) -> bool {
        let mut t0: Float = 0.0;
        let mut t1: Float = ray.t_max;
        let min: Point3f = self.min.cast();
        let max: Point3f = self.max.cast();
        for i in 0..3 {
            // update interval for _i_th bounding box slab
            let inv_ray_dir: Float = 1.0 as Float / ray.d[i];
            let mut t_near: Float = (min[i] - ray.o[i]) * inv_ray_dir;
            let mut t_far: Float = (max[i] - ray.o[i]) * inv_ray_dir;
            // update parametric interval from slab intersection $t$ values
            if t_near > t_far {
                std::mem::swap(&mut t_near, &mut t_far);
            }
            // update _t_far_ to ensure robust ray--bounds intersection
            t_far *= 1.0 as Float + 2.0 as Float * gamma(3_i32);
            if t_near > t0 {
                t0 = t_near;
            }
            if t_far < t1 {
                t1 = t_far;
            }
            if t0 > t1 {
                return false;
            }
        }
        *hitt0 = t0;
        *hitt1 = t1;
        true
    }

    fn intersect_p(&self, ray: &Ray, inv_dir: &Vector3f, dir_is_neg: [usize; 3]) -> bool {
        // check for ray intersection against $x$ and $y$ slabs
        let min_max: [Point3f; 2] = [self.min.cast(), self.max.cast()];
        let mut t_min: Float = (min_max[dir_is_neg[0]].x - ray.o.x) * inv_dir.x;
        let mut t_max: Float = (min_max[1 - dir_is_neg[0]].x - ray.o.x) * inv_dir.x;
        let ty_min: Float = (min_max[dir_is_neg[1]].y - ray.o.y) * inv_dir.y;
        let mut ty_max: Float = (min_max[1 - dir_is_neg[1]].y - ray.o.y) * inv_dir.y;
        // update _t_max_ and _ty_max_ to ensure robust bounds intersection
        t_max *= 1.0 + 2.0 * gamma(3_i32);
        ty_max *= 1.0 + 2.0 * gamma(3_i32);
        if t_min > ty_max || ty_min > t_max {
            return false;
        }
        if ty_min > t_min {
            t_min = ty_min;
        }
        if ty_max < t_max {
            t_max = ty_max;
        }
        // check for ray intersection against $z$ slab
        let tz_min: Float = (min_max[dir_is_neg[2]].z - ray.o.z) * inv_dir.z;
        let mut tz_max: Float = (min_max[1 - dir_is_neg[2]].z - ray.o.z) * inv_dir.z;
        // update _tz_max_ to ensure robust bounds intersection
        tz_max *= 1.0 + 2.0 * gamma(3_i32);
        if t_min > tz_max || tz_min > t_max {
            return false;
        }
        if tz_min > t_min {
            t_min = tz_min;
        }
        if tz_max < t_max {
            t_max = tz_max;
        }
        (t_min < ray.t_max) && (t_max > 0.0)
    }

    fn transform(&self, t: &Transform) -> Box<dyn Bounding3<Float>> {
        // improved version from the one in the book. http://www.realtimerendering.com/resources/GraphicsGems/gems/TransBox.c
        // take care of the translation by starting at it.
        let mut b_min = [t.m.m[0][3], t.m.m[1][3], t.m.m[2][3]];
        let mut b_max = [t.m.m[0][3], t.m.m[1][3], t.m.m[2][3]];
        let b: Bounds3f = self.cast();
        for i in 0..3 {
            for j in 0..3 {
                let a = t.m.m[i][j] * b.min[j as u8];
                let b = t.m.m[i][j] * b.max[j as u8];
                if a < b {
                    b_min[i] += a;
                    b_max[i] += b;
                }
                else {
                    b_min[i] += b;
                    b_max[i] += a;
                }
            }
        }
        Box::new(Bounds3f{
            min: Point3f{
                x: b_min[0],
                y: b_min[1],
                z: b_min[2]
            },
            max: Point3f {
                x: b_max[0],
                y: b_max[1],
                z: b_max[2]
            }
        })
    }
}



impl<T> Default for Bounds3<T>
where 
T: num::Bounded
{
    /// A default constructor with min and max set to violate min < max.
    fn default() -> Bounds3<T> {
        Bounds3::<T>{
            min: Point3::<T>::new(T::max_value(), T::max_value(), T::max_value()),
            max: Point3::<T>::new(T::min_value(), T::min_value(), T::min_value())
        }
    }
}

impl<T> Index<u8> for Bounds3<T> {
    type Output = Point3<T>;
    fn index(&self, index: u8) -> &Point3<T> {
        match index {
            0 => &self.min,
            1 => &self.max,
            _ => panic!("Index out of range for Bounds3"),
        }
    }
}

impl<T> IndexMut<u8> for Bounds3<T> {
    fn index_mut(&mut self, index: u8) -> &mut Point3<T> {
        match index {
            0 => &mut self.min,
            1 => &mut self.max,
            _ => panic!("Index out of range for Bounds3"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bounding_sphere() {
        let bb = Bounds3::<f32>::new(Point3::<f32>::new(0.0, 0.0, 0.0), Point3::<f32>::new(1.0, 1.0, 1.0));
        let mut center = Point3f::zero();
        let mut radius = 0.0;
        bb.bounding_sphere(&mut center, &mut radius);
        println!("{:?}, {}", center, radius);
        assert_eq!(center.x, 0.5);
        assert_eq!(center.y, 0.5);
        assert_eq!(radius, (0.5_f32 * 0.5 * 2.0).sqrt());
    }
}

// Iterator
// This isn't implemented in the book but may be useful.
pub struct Bounds3Iterator<'a> {
    p: Point3i,
    bounds: &'a Bounds3i
}

impl<'a> Iterator for Bounds3Iterator<'a> {
    type Item = Point3i;

    fn next(&mut self) -> Option<Point3i> {
        self.p.x += 1;
        if self.p.x == self.bounds.max.x {
            self.p.x = self.bounds.min.x;
            self.p.y += 1;
        }
        if self.p.y == self.bounds.max.y {
            self.p.y = self.bounds.min.y;
            self.p.z += 1;
        }
        if self.p.z == self.bounds.max.z {
            None
        }
        else {
            Some(self.p)
        }
    }
}

impl<'a> IntoIterator for &'a Bounds3i {
    type Item = Point3i;
    type IntoIter = Bounds3Iterator<'a>;

    fn into_iter(self) -> Self::IntoIter {
        Bounds3Iterator{
            p: Point3i{x: self.min.x - 1, y: self.min.y, z: self.min.z},
            bounds: self
        }
    }
}