use hexf::*;
use super::pbrt::Float;

#[cfg(feature = "float_as_double")]
pub const ONE_MINUS_EPSILON: f64 = hexf64!("0x1.fffffffffffffp-1");
#[cfg(not(feature = "float_as_double"))]
pub const ONE_MINUS_EPSILON: f32 = hexf32!("0x1.fffffep-1");

pub const PCG32_DEFAULT_STATE: u64 = 0x853c49e6748fea9b;
pub const PCG32_DEFAULT_STREAM: u64 = 0xda3e39cb94b95bdb;
pub const PCG32_MULT: u64 = 0x5851f42d4c957f2d;

/// PCG pseudo-random number generator
#[derive(Debug, Copy, Clone)]
pub struct Rng {
    state: u64,
    inc: u64
}

impl Rng {
    pub fn new(seed: u64) -> Rng {
        let rng = Rng {
            state: PCG32_DEFAULT_STATE,
            inc: PCG32_DEFAULT_STREAM
        };
        rng.set_sequence(seed);
        rng
    }

    pub fn set_sequence(&mut self, initseq: u64) {
        self.state = 0;
        self.inc = initseq.wrapping_shl(1) | 1;
        self.uniform_uint32();
        self.state = self.state.wrapping_add(PCG32_DEFAULT_STATE);
        self.uniform_uint32();
    }

    pub fn uniform_uint32(&mut self) -> u32 {
        let oldstate: u64 = self.state;
        // C++: state = oldstate * PCG32_MULT + inc;
        self.state = oldstate.wrapping_mul(PCG32_MULT).wrapping_add(self.inc);
        // C++: uint32_t xorshifted = (uint32_t)(((oldstate >> 18u) ^ oldstate) >> 27u);
        let xorshifted: u32 = (oldstate.wrapping_shr(18) ^ oldstate).wrapping_shr(27) as u32;
        // C++: uint32_t rot = (uint32_t)(oldstate >> 59u);
        let rot: u32 = oldstate.wrapping_shr(59) as u32;
        // C++: return (xorshifted >> rot) | (xorshifted << ((~rot + 1u) & 31));
        // bitwise not in Rust is ! (not the ~ operator like in C)
        xorshifted.wrapping_shr(rot) | xorshifted.wrapping_shl(!rot.wrapping_add(1_u32) & 31)
    }
    pub fn uniform_uint32_bounded(&mut self, b: u32) -> u32 {
        // bitwise not in Rust is ! (not the ~ operator like in C)
        let threshold = (!b + 1) & b;
        loop {
            let r = self.uniform_uint32();
            if r >= threshold {
                return r % b;
            }
        }
    }
    pub fn uniform_float(&mut self) -> Float {
        (self.uniform_uint32() as Float * hexf32!("0x1.0p-32") as Float)
            .min(ONE_MINUS_EPSILON)
    }

    pub fn advance(&mut self, idelta: i64) {
        let mut cur_mult = PCG32_MULT;
        let mut cur_plus = self.inc;
        let mut acc_mult = 1_u64;
        let mut acc_plus = 0_u64;
        let mut delta = idelta;

        while delta > 1 {
            if delta & 1 != 0 {
                acc_mult *= cur_mult;
                acc_plus = acc_plus * cur_mult + cur_plus;
            }
            cur_plus = (cur_mult + 1) * cur_plus;
            cur_mult *= cur_mult;
            delta /= 2;
        } 
        self.state = acc_mult.wrapping_mul(self.state).wrapping_add(acc_plus);
    }
}

impl Default for Rng {
    fn default() -> Rng {
        Rng {
            state: PCG32_DEFAULT_STATE,
            inc: PCG32_DEFAULT_STREAM
        }
    }
}