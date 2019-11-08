use super::pbrt::{next_float_down, next_float_up, consts::MACHINE_EPSILON};
use std::ops::{Add, Sub, Mul, Div};

/// EFloat tracks floating point errors to calculate error bounds.
pub struct EFloat {
    pub v: f32,
    pub low: f32,
    pub high: f32
}

impl EFloat {
    pub fn new(v: f32, err: f32) -> EFloat {
        let e = if err == 0.0 {
            EFloat{
                v,
                low: v,
                high: v
            }
        }
        else {
            EFloat {
                v,
                low: next_float_down(v - err),
                high: next_float_up(v + err)
            }
        };
        e.check();
        e
    }

    #[inline]
    pub fn new_with_check(v: f32, low: f32, high: f32) -> EFloat {
        let e = EFloat{v, low, high};
        e.check();
        e
    }

    pub fn get_absolute_error(&self) -> f32 {
        next_float_up((self.high - self.v).abs().max(self.v - self.low))
    }

    pub fn lower_bound(&self) -> f32 {
        self.low
    }

    pub fn upper_bound(&self) -> f32 {
        self.high
    }

    fn check(&self) {
        if self.low.is_finite() && self.high.is_finite() {
            assert!(self.low <= self.high);
        }
    }

    fn sqrt(&self) -> EFloat {
        let e = EFloat{
            v: self.v.sqrt(),
            low: next_float_down(self.low.sqrt()),
            high: next_float_up(self.high.sqrt())
        };
        e.check();
        e
    }

    fn abs(&self) -> EFloat {
        let e: EFloat = if self.low >= 0.0 {
            // The entire interval is greater than zero, so we're all set.
            return *self;
        }
        else if self.high <= 0.0 {
            // The entire interval is less than zero.
            EFloat{
                v: -self.v,
                low: -self.high,
                high: -self.low
            }
        }
        else {
            // The interval straddles zero.
            EFloat{
                v: self.v.abs(),
                low: 0.0,
                high: -self.low.max(self.high)
            }
        };
        e.check();
        e
    }
}

impl Default for EFloat {
    fn default() -> EFloat {
        EFloat{
            v: 0.0,
            low: 0.0,
            high: 0.0
        }
    }
}

impl PartialEq for EFloat {
    fn eq(&self, rhs: &EFloat) -> bool {
        self.v == rhs.v
    }
}

impl Add for EFloat {
    type Output = EFloat;
    fn add(self, rhs: EFloat) -> EFloat {
        let e = EFloat{
            v: self.v + rhs.v,
            low: next_float_down(self.low + rhs.low),
            high: next_float_up(self.high + rhs.high)
        };
        e.check();
        e
    }
}

impl Add<f32> for EFloat {
    type Output = EFloat;
    fn add(self, rhs: f32) -> EFloat {
        self + EFloat::new(rhs, rhs)
    }
}

impl Sub for EFloat {
    type Output = EFloat;
    fn sub(self, rhs: EFloat) -> EFloat {
        let e = EFloat{
            v: self.v - rhs.v,
            low: next_float_down(self.low - rhs.low),
            high: next_float_up(self.high - rhs.high)
        };
        e.check();
        e
    }
}

impl Sub<f32> for EFloat {
    type Output = EFloat;
    fn sub(self, rhs: f32) -> EFloat {
        self - EFloat::new(rhs, rhs)
    }
}

impl Mul for EFloat {
    type Output = EFloat;
    fn mul(self, rhs: EFloat) -> EFloat {
        let prod = [
            self.low * rhs.low,
            self.high * rhs.low,
            self.low * rhs.high,
            self.high * rhs.high
        ];
        let e = EFloat{
            v: self.v * rhs.v,
            low: next_float_down(prod[0].min(prod[1]).min(prod[2]).min(prod[3])),
            high: next_float_up(prod[0].max(prod[1]).max(prod[2]).max(prod[3]))
        };
        e.check();
        e
    }
}

impl Mul<f32> for EFloat {
    type Output = EFloat;
    fn mul(self, rhs: f32) -> EFloat {
        self * EFloat::new(rhs, rhs)
    }
}

impl Mul<EFloat> for f32 {
    type Output = EFloat;
    fn mul(self, rhs: EFloat) -> EFloat {
        EFloat::new(self, self) * rhs
    }
}

impl Div for EFloat {
    type Output = EFloat;
    fn div(self, rhs: EFloat) -> EFloat {
        let e = if rhs.low < 0.0 && rhs.high > 0.0 {
            // The interval we're dividing by straddles zero, so just
            // return an interval of everything.
            EFloat{
                v: self.v / rhs.v,
                low: std::f32::NEG_INFINITY,
                high: std::f32::INFINITY
            }
        }
        else {
            let div = [
                self.low / rhs.low,
                self.high / rhs.low,
                self.low / rhs.high,
                self.high / rhs.high
            ];
            EFloat{
                v: self.v / rhs.v,
                low: next_float_down(div[0].min(div[1]).min(div[2]).min(div[3])),
                high: next_float_up(div[0].max(div[1]).max(div[2]).max(div[3]))
            }
        };
        e.check();
        e
    }
}

impl Div<f32> for EFloat {
    type Output = EFloat;
    fn div(self, rhs: f32) -> EFloat {
        self / EFloat::new(rhs, rhs)
    }
}

impl From<EFloat> for f32 {
    fn from(e: EFloat) -> f32 {
        e.v
    }
}

impl From<EFloat> for f64 {
    fn from(e: EFloat) -> f64 {
        e.v.into()
    }
}

/// Find solution(s) of the quadratic equation at<sup>2</sup> + bt + c = 0 using
/// *EFloat* instead of *Float* for error bounds.
pub fn quadratic(a: EFloat, b: EFloat, c: EFloat, t0: &mut EFloat, t1: &mut EFloat) -> bool {
    let discrim: f64 = b.v as f64 * b.v as f64 - 4.0f64 * a.v as f64 * c.v as f64;
    if discrim < 0.0 {
        false
    } else {
        let root_discrim: f64 = discrim.sqrt();
        let float_root_discrim: EFloat = EFloat::new(
            root_discrim as f32,
            MACHINE_EPSILON as f32 * root_discrim as f32,
        );
        // compute quadratic _t_ values
        let q: EFloat;
        if b.v < 0.0f32 {
            q = (b - float_root_discrim) * -0.5f32;
        } else {
            q = (b + float_root_discrim) * -0.5f32;
        }
        *t0 = q / a;
        *t1 = c / q;
        if (*t0).v > (*t1).v {
            let swap: EFloat = *t0;
            *t0 = *t1;
            *t1 = swap;
        }
        true
    }
}