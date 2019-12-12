//! Type definitions and constants.
use std::ops::{Sub, Add, Mul, BitAnd, Not, SubAssign, BitOrAssign, Shr, Div};
use super::spectrum::*;

#[cfg(feature = "sampled_spectrum")]
pub type Spectrum = SampledSpectrum;
#[cfg(not(feature = "sampled_spectrum"))]
pub type Spectrum = RGBSpectrum;

#[cfg(feature = "float_as_double")]
mod float_mod {
    pub type Float = f64;
    pub mod consts {
        pub use std::f64::consts::*;
        pub const MACHINE_EPSILON: f64 = std::f64::EPSILON * 0.5;
    }
}
#[cfg(not(feature = "float_as_double"))]
mod float_mod {
    pub type Float = f32;
    pub mod consts {
        pub use std::f32::consts::*;
        pub const MACHINE_EPSILON: f32 = std::f32::EPSILON * 0.5;
    }
}

pub use float_mod::Float;
pub mod consts {
    use super::float_mod::Float;
    pub use super::float_mod::consts::*;
    pub const FRAC_PI_180: Float = PI / 180.0;
    pub const FRAC_180_PI: Float = 180.0 / PI;
    pub const SHADOW_EPSILON: Float = 0.0001;
    pub const INV_PI: Float = 1.0 / PI;
    pub const INV_2_PI: Float = 1.0 / PI * 2.0;
    pub const INV_4_PI: Float = 1.0 / PI * 4.0;
}

/// Linearly interpolate between two values.
#[inline]
pub fn lerp<S, T>(t: S, a: T, b: T) -> T
where
S: num::One + Sub<S, Output = S> + Copy,
T: Add<T, Output=T> + Mul<S, Output=T>
{
    a * (S::one() - t) + b * t
}

/// Convert an angle from radians into degrees.
pub fn radians(deg: Float) -> Float {
    consts::FRAC_PI_180 * deg
}

/// Convert an angle from degrees into radians.
pub fn degrees(rad: Float) -> Float {
    consts::FRAC_180_PI * rad
}

/// Use **unsafe**
/// [std::mem::transmute_copy][transmute_copy]
/// to convert *f32* to *u32*.
///
/// [transmute_copy]: https://doc.rust-lang.org/std/mem/fn.transmute_copy.html
pub fn float_to_bits(f: f32) -> u32 {
    // uint64_t ui;
    // memcpy(&ui, &f, sizeof(double));
    // return ui;
    let rui: u32;
    unsafe {
        let ui: u32 = std::mem::transmute_copy(&f);
        rui = ui;
    }
    rui
}

/// Use **unsafe**
/// [std::mem::transmute_copy][transmute_copy]
/// to convert *f32* to *u32*.
///
/// [transmute_copy]: https://doc.rust-lang.org/std/mem/fn.transmute_copy.html
pub fn double_to_bits(f: f64) -> u64 {
    // uint64_t ui;
    // memcpy(&ui, &f, sizeof(double));
    // return ui;
    let rui: u64;
    unsafe {
        let ui: u64 = std::mem::transmute_copy(&f);
        rui = ui;
    }
    rui
}

/// Use **unsafe**
/// [std::mem::transmute_copy][transmute_copy]
/// to convert *u32* to *f32*.
///
/// [transmute_copy]: https://doc.rust-lang.org/std/mem/fn.transmute_copy.html
pub fn bits_to_float(ui: u32) -> f32 {
    // float f;
    // memcpy(&f, &ui, sizeof(uint32_t));
    // return f;
    let rf: f32;
    unsafe {
        let f: f32 = std::mem::transmute_copy(&ui);
        rf = f;
    }
    rf
}

/// Use **unsafe**
/// [std::mem::transmute_copy][transmute_copy]
/// to convert *u32* to *f32*.
///
/// [transmute_copy]: https://doc.rust-lang.org/std/mem/fn.transmute_copy.html
pub fn bits_to_double(ui: u64) -> f64 {
    // float f;
    // memcpy(&f, &ui, sizeof(uint32_t));
    // return f;
    let rf: f64;
    unsafe {
        let f: f64 = std::mem::transmute_copy(&ui);
        rf = f;
    }
    rf
}

/// Bump a floating-point value up to the next greater representable
/// floating-point value.
pub fn next_float_up(v: f32) -> f32 {
    if v.is_infinite() && v > 0.0 {
        v
    } else {
        let new_v: f32;
        if v == -0.0 {
            new_v = 0.0;
        } else {
            new_v = v;
        }
        let mut ui: u32 = float_to_bits(new_v);
        if new_v >= 0.0 {
            ui += 1;
        } else {
            ui -= 1;
        }
        bits_to_float(ui)
    }
}

/// Bump a floating-point value down to the next smaller representable
/// floating-point value.
pub fn next_float_down(v: f32) -> f32 {
    if v.is_infinite() && v < 0.0 {
        v
    } else {
        let new_v: f32;
        if v == 0.0 {
            new_v = -0.0;
        } else {
            new_v = v;
        }
        let mut ui: u32 = float_to_bits(new_v);
        if new_v > 0.0 {
            ui -= 1;
        } else {
            ui += 1;
        }
        bits_to_float(ui)
    }
}

/// Error propagation.
pub fn gamma(n: i32) -> Float {
    (n as Float * consts::MACHINE_EPSILON) / (1.0 - n as Float * consts::MACHINE_EPSILON)
}

pub fn gamma_correct(value: Float) -> Float {
    if value < 0.0031308 {
        12.92 * value
    }
    else {
        1.055 * value.powf(1.0 / 2.4) - 0.055
    }
}

pub fn inverse_gamma_correct(value: Float) -> Float {
    if value <= 0.04045 {
        value * 1.0 / 12.92
    }
    else {
        ((value + 0.055) * 1.0 / 1.055).powf(2.4)
    }
}

/// A version of modulus where the result is positive for negative values
#[inline]
pub fn mod_t<T>(a: T, b: T) -> T
where
T: num::Zero + Copy + Div<Output=T> + Mul<Output=T> + Sub<Output=T> + PartialOrd
{
    let result = a - (a / b) * b;
    if result < num::Zero::zero() {
        result + b
    } else {
        result
    }
}

#[inline]
pub fn log2_int_u32(v: u32) -> i32 {
    31 - v.leading_zeros() as i32
}

#[inline]
pub fn log2_int_i32(v: i32) -> i32 {
    31 - v.leading_zeros() as i32
}

#[inline]
pub fn is_power_of_2<T>(v: T) -> bool
where
T: BitAnd<Output=T> + Not<Output=T> + num::Zero + num::One + PartialOrd + Sub<Output=T>
{
    v > num::Zero::zero() && !(v & (v - num::One::one())) > num::Zero::zero()
}

pub fn round_up_pow2<T>(mut v: T) -> T
where
T: SubAssign + BitOrAssign + Add<Output=T> + Shr<Output=T> + num::NumCast
{
    v -= num::cast(1).unwrap();
    v |= v >> num::cast(1).unwrap();
    v |= v >> num::cast(2).unwrap();
    v |= v >> num::cast(4).unwrap();
    v |= v >> num::cast(8).unwrap();
    v |= v >> num::cast(16).unwrap();
    v + num::cast(1).unwrap()
}

/// Specialized version of round_up_pow2 for i32 to avoid casting and unwraps
pub fn round_up_pow2_i32(v: i32) -> i32 {
    v -= 1;
    v |= v >> 1;
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
    v + 1
}

/// Specialized version of round_up_pow2 for i32 to avoid casting and unwraps
pub fn round_up_pow2_i64(v: i64) -> i64 {
    v -= 1;
    v |= v >> 1;
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
    v + 1
}

pub fn find_interval<T>(size: usize, predicate: T) -> usize
where
T: Fn(usize) -> bool
{
    let mut first = 0;
    let mut len = size;
    while len > 0 {
        let half = len >> 1;
        let middle = first + half;
        // Bisect range based on value of _pred_ at _middle_
        if predicate(middle) {
            first = middle + 1;
            len -= half + 1;
        } else {
            len = half;
        }
    }
    num::clamp(first.saturating_sub(1), 0, size.saturating_sub(2))
}

#[inline]
pub fn quadratic(
    a: Float,
    b: Float,
    c: Float,
    t0: &mut Float,
    t1: &mut Float
) -> bool {
    // Find quadratic discriminant
    let discrim = b as f64 * b as f64 - 4.0 * a as f64 * c as f64;
    if discrim < 0.0 { return false; }
    let root_discrim = discrim.sqrt();

    // Compute quadratic _t_ values
    let q = if b < 0.0 {
        -0.5 * (b as f64 - root_discrim)
    } else {
        -0.5 * (b as f64 + root_discrim)
    };
    *t0 = num::cast::cast(q / a as f64).unwrap();
    *t1 = num::cast::cast(c as f64 / q).unwrap();
    if *t0 > *t1 {
        std::mem::swap(t0, t1);
    }
    true
}

#[inline]
pub fn erf_inv(mut x: Float) -> Float {
    let mut p: Float;
    x = num::clamp(x, -0.99999, 0.99999);
    let mut w = -((1.0 - x) * (1.0 + x)).ln();
    if w < 5.0 {
        w = w - 2.5;
        p = 2.81022636e-08;
        p = 3.43273939e-07 + p * w;
        p = -3.5233877e-06 + p * w;
        p = -4.39150654e-06 + p * w;
        p = 0.00021858087 + p * w;
        p = -0.00125372503 + p * w;
        p = -0.00417768164 + p * w;
        p = 0.246640727 + p * w;
        p = 1.50140941 + p * w;
    } else {
        w = w.sqrt() - 3.0;
        p = -0.000200214257;
        p = 0.000100950558 + p * w;
        p = 0.00134934322 + p * w;
        p = -0.00367342844 + p * w;
        p = 0.00573950773 + p * w;
        p = -0.0076224613 + p * w;
        p = 0.00943887047 + p * w;
        p = 1.00167406 + p * w;
        p = 2.83297682 + p * w;
    }
    p * x
}
#[inline]
pub fn erf(mut x: Float) -> Float {
    // constants
    let a1: Float = 0.254829592;
    let a2: Float = -0.284496736;
    let a3: Float = 1.421413741;
    let a4: Float = -1.453152027;
    let a5: Float = 1.061405429;
    let p: Float = 0.3275911;

    // Save the sign of x
    let mut sign = 1.0;
    if x < 0.0 {
        sign = -1.0;
    }
    x = x.abs();

    // A&S formula 7.1.2.6
    let t = 1.0 / (1.0 + p * x);
    let y = 1.0 - (((((a5 * t + a4) * t) + a3) * t + a2) * t + a1) * t * (-x * x).exp();
    sign * y
}
