//! Utility macros used accross multiple files.

/// Reverse multiplication for a generic type
macro_rules! reverse_mul {
    ($T:ident, { $($field:ident),+ }, $U:ty) => {
        impl Mul<$T<$U>> for $U {
            type Output = $T<$U>;
            fn mul(self, rhs: $T<$U>) -> Self::Output {
                $T::new($(rhs.$field * self),+)
            }
        }
    };
}

macro_rules! reverse_mul_scalar {
    ($T:ident, { $($field:ident),+ }) => {
        reverse_mul!($T, {$($field),+}, usize);
        reverse_mul!($T, {$($field),+}, u8);
        reverse_mul!($T, {$($field),+}, u16);
        reverse_mul!($T, {$($field),+}, u32);
        reverse_mul!($T, {$($field),+}, u64);
        reverse_mul!($T, {$($field),+}, isize);
        reverse_mul!($T, {$($field),+}, i8);
        reverse_mul!($T, {$($field),+}, i16);
        reverse_mul!($T, {$($field),+}, i32);
        reverse_mul!($T, {$($field),+}, i64);
        reverse_mul!($T, {$($field),+}, f32);
        reverse_mul!($T, {$($field),+}, f64);
    };
}