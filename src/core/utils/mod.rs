use crate::core::pbrt::Float;
use num::cast;

pub mod slice_extension;

/// Analogous to the %g format of the C printf function, this method switches
/// between naive and scientific notation for floating-point numbers when the
/// number being printed becomes so small that printing leading zeroes could end
/// up larger than the scientific notation, or so large that we would be forced
/// to print more significant digits than requested.
/// Modified from https://github.com/rust-lang/rfcs/issues/844#issuecomment-426525833.
pub fn float_to_string_general(x: Float, sig_digits: usize) -> String {
    let mut precision = sig_digits - 1;
    let log_x = x.abs().log10();
    if (log_x >= -3.0 && log_x <= cast(sig_digits).unwrap()) || x == 0.0 {
        // Print using naive notation
        if x != 0.0 {
            // Since Rust's precision controls number of digits after the
            // decimal point, we must adjust it depending on magnitude in order
            // to operate at a constant number of significant digits.
            precision = (precision as isize - log_x.trunc() as isize) as usize;

            // Numbers smaller than 1 must get one extra digit since the leading
            // zero does not count as a significant digit.
            if log_x < 0.0 { precision += 1 }
        }
        format!("{:.1$}", x, precision)
    } else {
        // Print using scientific notation
        format!("{:.1$e}", x, precision)
    }
}