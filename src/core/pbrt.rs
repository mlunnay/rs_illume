//! Type definitions and constants.
use std::ops::{Sub, Add, Mul};
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

#[inline]
pub fn log2_int_u32(v: u32) -> i32 {
    31 - v.leading_zeros() as i32
}

#[inline]
pub fn log2_int_i32(v: i32) -> i32 {
    31 - v.leading_zeros() as i32
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