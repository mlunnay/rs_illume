use std::sync::atomic::*;
use super::pbrt::Float;

#[cfg(feature = "float_as_double")]
use super::pbrt::{bits_to_double as bits_to_float, double_to_bits as float_to_bits};
#[cfg(not(feature = "float_as_double"))]
use super::pbrt::{float_to_bits, bits_to_float};

#[cfg(feature = "float_as_double")]
use std::sync::atomic::AtomicU64 as Atomic;
#[cfg(not(feature = "float_as_double"))]
use std::sync::atomic::AtomicU32 as Atomic;


/// An implementation of an atomic floating point scalar.
/// Note that the sum (a+b)+c is not necessarily equal to the sum a+(b+c), and
/// the result can be different across multiple program executions. This is acceptable
/// in certain situations such as in Film::splat() and is prefferable to locking.
pub struct AtomicFloat {
    bits: Atomic
}

impl AtomicFloat {
    pub fn new(v: Float) -> AtomicFloat {
        AtomicFloat{
            bits: Atomic::new(float_to_bits(v))
        }
    }

    pub fn load(&self) -> Float {
        bits_to_float(self.bits.load(Ordering::SeqCst))
    }

    pub fn store(&self, v: Float) {
        self.bits.store(float_to_bits(v), Ordering::SeqCst);
    }

    pub fn add(&self, v: Float) {
        let mut old_bits: u32 = self.bits.load(Ordering::Relaxed);
        loop {
            let f: Float = bits_to_float(old_bits);
            let new_bits: u32 = float_to_bits(f + v);
            match self.bits.compare_exchange_weak(
                old_bits,
                new_bits,
                Ordering::SeqCst,
                Ordering::Relaxed,
            ) {
                Ok(_) => {
                    break;
                }
                Err(x) => {
                    old_bits = x;
                }
            }
        }
    }
}

impl Clone for AtomicFloat {
    fn clone(&self) -> Self {
        let bits = self.bits.load(Ordering::SeqCst);
        AtomicFloat {
            bits: Atomic::new(bits)
        }
    }
}

impl Default for AtomicFloat {
    fn default() -> AtomicFloat {
        AtomicFloat{
            bits: Atomic::new(0)
        }
    }
}

impl From<&AtomicFloat> for Float {
    fn from(f: &AtomicFloat) -> Float {
        bits_to_float(f.bits.load(Ordering::SeqCst))
    }
}