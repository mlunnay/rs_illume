use super::pbrt::{Float, consts::PI, consts::FRAC_PI_2};
use std::ops::{Add, Sub, Mul};

/// A representation of intervals over real numbers.
#[derive(Debug, Default, Copy, Clone)]
pub struct Interval {
    pub low: Float,
    pub high: Float
}

impl Interval {
    pub fn new(v1: Float, v2: Float) -> Interval {
        Interval{
            low: v1.min(v2),
            high: v1.max(v2) 
        }
    }

    /// Create an Interval with both low and high set to the same value.
    pub fn from_single(v: Float) -> Interval {
        Interval{
            low: v,
            high: v
        }
    }

    /// Compute the sine on the interval. Assumes values are in the range [0,2π].
    pub fn sin(&self) -> Interval {
        assert!(self.low >= 0.0);
        assert!(self.high <= 2.0 * PI);
        let mut sin_low = self.low.sin();
        let mut sin_high = self.high.sin();
        if sin_low > sin_high {
            std::mem::swap(&mut sin_low, &mut sin_high);
        }
        if self.low < FRAC_PI_2 && self.high > FRAC_PI_2 {
            sin_high = 1.0;
        }
        if self.low < (3.0 / 2.0) * PI && self.high > (3.0 / 2.0) * PI {
            sin_low = -1.0;
        }
        Interval{
            low: sin_low,
            high: sin_high
        }
    }

    /// Compute the cosine on the interval. Assumes values are in the range [0,2π].
    pub fn cos(&self) -> Interval {
        assert!(self.low >= 0.0);
        assert!(self.high <= 2.0001 * PI);
        let mut cos_low = self.low.cos();
        let mut cos_high = self.high.cos();
        if cos_low > cos_high {
            std::mem::swap(&mut cos_low, &mut cos_high);
        }
        if self.low < PI && self.high> PI {
            cos_low = -1.0;
        }
        Interval{
            low: cos_low,
            high: cos_high
        }
    }
}

impl Add for Interval{
    type Output = Interval;
    fn add(self, rhs: Interval) -> Interval {
        Interval{
            low: self.low + rhs.low,
            high: self.high + rhs.high
        }
    }
}

impl Sub for Interval{
    type Output = Interval;
    fn sub(self, rhs: Interval) -> Interval {
        Interval{
            low: self.low - rhs.low,
            high: self.high - rhs.high
        }
    }
}

impl Mul for Interval{
    type Output = Interval;
    fn mul(self, rhs: Interval) -> Interval {
        let min_rhs_low = (self.low * rhs.low).min(self.high * rhs.low);
        let min_rhs_high = (self.low * rhs.high).min(self.high * rhs.high);
        let max_rhs_low = (self.low * rhs.low).max(self.high * rhs.low);
        let max_rhs_high = (self.low * rhs.high).max(self.high * rhs.high);
        Interval{
            low: min_rhs_low.min(min_rhs_high),
            high: max_rhs_low.max(max_rhs_high)
        }
    }
}

/// Finds the t value of any crossings of the motion equation over the given interval t_interval.
pub fn interval_find_zeros(
    c1: Float,
    c2: Float,
    c3: Float,
    c4: Float,
    c5: Float,
    theta: Float,
    t_interval: Interval,
    zeros: &mut Vec<Float>,
    depth: usize,
) {
    // evaluate motion derivative in interval form, return if no zeros
    let two_theta: Float = 2.0 as Float * theta;
    let range: Interval = Interval::new(c1, c1)
        + (Interval::new(c2, c2) + Interval::new(c3, c3) * t_interval)
            * (Interval::new(two_theta, two_theta) * t_interval).cos()
        + (Interval::new(c4, c4) + Interval::new(c5, c5) * t_interval)
            * (Interval::new(two_theta, two_theta) * t_interval).sin();
    if range.low > 0.0 as Float || range.high < 0.0 as Float || range.low == range.high {
        return;
    }
    if depth > 0_usize {
        // split _t_interval_ and check both resulting intervals
        let mid: Float = (t_interval.low + t_interval.high) * 0.5 as Float;
        interval_find_zeros(
            c1,
            c2,
            c3,
            c4,
            c5,
            theta,
            Interval::new(t_interval.low, mid),
            zeros,
            depth - 1_usize,
        );
        interval_find_zeros(
            c1,
            c2,
            c3,
            c4,
            c5,
            theta,
            Interval::new(mid, t_interval.high),
            zeros,
            depth - 1_usize,
        );
    } else {
        // use Newton's method to refine zero
        let mut t_newton: Float = (t_interval.low + t_interval.high) * 0.5 as Float;
        for _i in 0..4 {
            let f_newton: Float = c1
                + (c2 + c3 * t_newton) * (2.0 as Float * theta * t_newton).cos()
                + (c4 + c5 * t_newton) * (2.0 as Float * theta * t_newton).sin();
            let f_prime_newton: Float = (c3 + 2.0 as Float * (c4 + c5 * t_newton) * theta)
                * (2.0 as Float * t_newton * theta).cos()
                + (c5 - 2.0 as Float * (c2 + c3 * t_newton) * theta)
                    * (2.0 as Float * t_newton * theta).sin();
            if f_newton == 0.0 as Float || f_prime_newton == 0.0 as Float {
                break;
            }
            t_newton = t_newton - f_newton / f_prime_newton;
        }
        if t_newton >= t_interval.low - 1e-3 as Float && t_newton < t_interval.high + 1e-3 as Float
        {
            zeros.push(t_newton);
        }
    }
}