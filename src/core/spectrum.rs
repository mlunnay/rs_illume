use super::pbrt::{Float, lerp, find_interval};
use std::ops::{Add, AddAssign, Sub, Div, DivAssign, Mul, MulAssign, Neg, Index, IndexMut};

lazy_static!{
    static ref SAMPLED_SPECTRUM_GLOBALS: SampledSpectrumGlobals = SampledSpectrumGlobals::new();
}

const SAMPLED_LAMBDA_START: Float = 400.0;
const SAMPLED_LAMBDA_END: Float = 700.0;
const N_SPECTRAL_SAMPLES: usize = 60;

macro_rules! CoefficientSpectrumMethods {
    ($n_spectrum_samples: expr) => {
        pub fn is_black(&self) -> bool {
            for i in 0..$n_spectrum_samples {
                if self.c[i] != 0.0 {
                    return false;
                }
            }
            true
        }

        pub fn sqrt(&self) -> Self {
            let mut ret = Self::default();
            for i in 0..$n_spectrum_samples {
                ret.c[i] = self.c[i].sqrt();
            }
            assert!(!ret.has_nans());
            ret
        }

        pub fn clamp(&self, low: Float, high: Float) -> Self {
            let mut ret = Self::default();
            for i in 0..$n_spectrum_samples {
                ret.c[i] = num::clamp(self.c[i], low, high);
            }
            assert!(!ret.has_nans());
            ret
        }

        pub fn max_component_value(&self) -> Float {
            let mut m = self.c[0];
            for i in 1..$n_spectrum_samples {
                m = m.max(self.c[i]);
            }
            m
        }

        pub fn has_nans(&self) -> bool {
            for i in 1..$n_spectrum_samples {
                if self.c[i].is_nan() {
                    return true;
                }
            }
            false
        }

        pub fn exp(&self) -> Self {
            let mut ret = Self::default();
            for i in 0..$n_spectrum_samples {
                ret.c[i] = self.c[i].exp();
            }
            assert!(!ret.has_nans());
            ret
        }

        pub fn pow(&self, e: Float) -> Self {
            let mut ret = Self::default();
            for i in 0..$n_spectrum_samples {
                ret.c[i] = self.c[i].powf(e);
            }
            assert!(!ret.has_nans());
            ret
        }

        pub fn lerp(&self, s2: &Self, t: Float) -> Self {
            *self * (1.0 - t) + *s2 * t
        }

        const n_samples: usize = $n_spectrum_samples;
    };
}

macro_rules! CoefficientSpectrumImpl {
    ($T: ident, $n_spectrum_samples: expr) => {
        impl std::fmt::Display for $T {
            fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                f.debug_list().entries(self.c.iter()).finish()
            }
        }

        impl Neg for $T {
            type Output = $T;
            fn neg(self) -> $T {
                let ret = Self::default();
                for i in 0..$n_spectrum_samples {
                    ret.c[i] = -self.c[i];
                }
                ret
            }
        }

        impl Index<usize> for $T {
            type Output = Float;
            fn index(&self, i: usize) -> Float {
                self.c[i]
            }
        }

        impl IndexMut<usize> for $T {
            fn index_mut(&self, i: usize) -> &mut Float {
                &mut self.c[i]
            }
        }

        impl Add for $T {
            type Output = Self;
            fn add(self, s2: Self) -> Self {
                assert!(!s2.has_nans());
                let ret = Self::default();
                for i in 0..$n_spectrum_samples {
                    ret.c[i] = self.c[i] + s2.c[i];
                }
                ret
            }
        }

        impl AddAssign for $T {
            fn add_assign(&mut self, s2: Self) {
                assert!(!s2.has_nans());
                for i in 0..$n_spectrum_samples {
                    self.c[i] += s2.c[i];
                }
            }
        }

        impl Sub for $T {
            type Output = Self;
            fn sub(self, s2: Self) -> Self {
                assert!(!s2.has_nans());
                let ret = Self::default();
                for i in 0..$n_spectrum_samples {
                    ret.c[i] = self.c[i] - s2.c[i];
                }
                ret
            }
        }

        impl Div for $T {
            type Output = Self;
            fn div(self, s2: Self) -> Self {
                assert!(!s2.has_nans());
                let ret = Self::default();
                for i in 0..$n_spectrum_samples {
                    ret.c[i] = self.c[i] / s2.c[i];
                }
                ret
            }
        }

        impl Mul for $T {
            type Output = Self;
            fn mul(self, s2: Self) -> Self {
                assert!(!s2.has_nans());
                let ret = Self::default();
                for i in 0..$n_spectrum_samples {
                    ret.c[i] = self.c[i] * s2.c[i];
                }
                ret
            }
        }

        impl MulAssign for $T {
            fn mul_assign(&mut self, s2: Self) {
                assert!(!s2.has_nans());
                for i in 0..$n_spectrum_samples {
                    self.c[i] *= s2.c[i];
                }
            }
        }

        impl Mul<Float> for $T {
            type Output = Self;
            fn mul(self, a: Float) -> Self {
                let ret = Self::default();
                for i in 0..$n_spectrum_samples {
                    ret.c[i] = self.c[i] * a;
                }
                assert!(!ret.has_nans());
                ret
            }
        }

        impl MulAssign<Float> for $T {
            fn mul_assign(&mut self, a: Float) {
                for i in 0..$n_spectrum_samples {
                    self.c[i] *= a;
                }
                assert!(!self.has_nans());
            }
        }

        impl Mul<$T> for Float {
            type Output = $T;
            fn mul(self, s: $T) -> $T {
                assert!(!self.is_nan() && !s.has_nans());
                let mut ret = $T::default();
                for i in 0..$n_spectrum_samples {
                    ret.c[i] = s.c[i] * self;
                }
                ret
            }
        }

        impl Div<Float> for $T {
            type Output = $T;
            fn div(self, a: Float) -> $T {
                assert_ne!(a, 0.0);
                assert!(!a.is_nan());
                let mut ret = $T::default();
                for i in 0..$n_spectrum_samples {
                    ret.c[i] = self.c[i] / a;
                }
                ret
            }
        }

        impl DivAssign<Float> for $T {
            fn div_assign(&mut self, a: Float) {
                assert_ne!(a, 0.0);
                assert!(!a.is_nan());
                for i in 0..$n_spectrum_samples {
                    self.c[i] /= a;
                }
            }
        }

        impl PartialEq for $T {
            fn eq(&self, s2: &$T) -> bool {
                for i in 0..$n_spectrum_samples {
                    if self.c[i] != s2.c[i] {
                        return false
                    }
                }
                true
            }
        }
    };
}

#[derive(Copy, Clone)]
pub struct SampledSpectrum {
    c: [Float; N_SPECTRAL_SAMPLES]
}

impl SampledSpectrum {
    pub fn new(v: Float) -> SampledSpectrum {
        SampledSpectrum{
            c: [v; N_SPECTRAL_SAMPLES]
        }
    }

    pub fn from_sampled(
        lambda: &[Float],
        v: &[Float]
    ) -> SampledSpectrum {
        // Sort samples if unordered, use sorted for returned spectrum
        if !spectrum_samples_sorted(lambda) {
            let slambda = lambda.clone();
            let sv = v.clone();
            sort_spectrum_sample(&mut slambda, &mut sv);
            return SampledSpectrum::from_sampled(slambda, v);
        }
        let mut r = SampledSpectrum::default();
        for i in 0..N_SPECTRAL_SAMPLES {
            // Compute average value of given SPD over $i$th sample's range
            let lambda0 = lerp(i as Float / N_SPECTRAL_SAMPLES as Float,
                SAMPLED_LAMBDA_START, SAMPLED_LAMBDA_END);
            let lambda1 = lerp((i as Float + 1.0) / N_SPECTRAL_SAMPLES as Float,
                SAMPLED_LAMBDA_START, SAMPLED_LAMBDA_END);
            r.c[i] = average_spectrum_samples(lambda, v, lambda0, lambda1);
        }
        r
    }

    pub fn from_rgb(rgb: &[Float; 3], spectrum_type: SpectrumType) -> SampledSpectrum {
        let mut r = SampledSpectrum::default();
        if spectrum_type == SpectrumType::Reflectance {
            // Convert reflectance spectrum to RGB
            if rgb[0] <= rgb[1] && rgb[0] <= rgb[2] {
                // Compute reflectance _SampledSpectrum_ with _rgb[0]_ as minimum
                r += rgb[0] * SAMPLED_SPECTRUM_GLOBALS.rgb_refl2_spect_white;
                if rgb[1] <= rgb[2] {
                    r += (rgb[1] - rgb[0]) * SAMPLED_SPECTRUM_GLOBALS.rgb_refl2_spect_cyan;
                    r += (rgb[2] - rgb[1]) * SAMPLED_SPECTRUM_GLOBALS.rgb_refl2_spect_blue;
                } else {
                    r += (rgb[2] - rgb[0]) * SAMPLED_SPECTRUM_GLOBALS.rgb_refl2_spect_cyan;
                    r += (rgb[1] - rgb[2]) * SAMPLED_SPECTRUM_GLOBALS.rgb_refl2_spect_green;
                }
            } else if rgb[1] <= rgb[0] && rgb[1] <= rgb[2] {
                // Compute reflectance _SampledSpectrum_ with _rgb[1]_ as minimum
                r += rgb[1] * SAMPLED_SPECTRUM_GLOBALS.rgb_refl2_spect_white;
                if rgb[0] <= rgb[2] {
                    r += (rgb[0] - rgb[1]) * SAMPLED_SPECTRUM_GLOBALS.rgb_refl2_spect_magenta;
                    r += (rgb[2] - rgb[0]) * SAMPLED_SPECTRUM_GLOBALS.rgb_refl2_spect_blue;
                } else {
                    r += (rgb[2] - rgb[1]) * SAMPLED_SPECTRUM_GLOBALS.rgb_refl2_spect_magenta;
                    r += (rgb[0] - rgb[2]) * SAMPLED_SPECTRUM_GLOBALS.rgb_refl2_spect_red;
                }
            } else {
                // Compute reflectance _SampledSpectrum_ with _rgb[2]_ as minimum
                r += rgb[2] * SAMPLED_SPECTRUM_GLOBALS.rgb_refl2_spect_white;
                if rgb[0] <= rgb[1] {
                    r += (rgb[0] - rgb[2]) * SAMPLED_SPECTRUM_GLOBALS.rgb_refl2_spect_yellow;
                    r += (rgb[1] - rgb[0]) * SAMPLED_SPECTRUM_GLOBALS.rgb_refl2_spect_green;
                } else {
                    r += (rgb[1] - rgb[2]) * SAMPLED_SPECTRUM_GLOBALS.rgb_refl2_spect_yellow;
                    r += (rgb[0] - rgb[1]) * SAMPLED_SPECTRUM_GLOBALS.rgb_refl2_spect_red;
                }
            }
            r *= 0.94;
        } else {
            // Convert illuminant spectrum to RGB
            if rgb[0] <= rgb[1] && rgb[0] <= rgb[2] {
                // Compute illuminant _SampledSpectrum_ with _rgb[0]_ as minimum
                r += rgb[0] * SAMPLED_SPECTRUM_GLOBALS.rgb_illum2_spect_white;
                if rgb[1] <= rgb[2] {
                    r += (rgb[1] - rgb[0]) * SAMPLED_SPECTRUM_GLOBALS.rgb_illum2_spect_cyan;
                    r += (rgb[2] - rgb[1]) * SAMPLED_SPECTRUM_GLOBALS.rgb_illum2_spect_blue;
                } else {
                    r += (rgb[2] - rgb[0]) * SAMPLED_SPECTRUM_GLOBALS.rgb_illum2_spect_cyan;
                    r += (rgb[1] - rgb[2]) * SAMPLED_SPECTRUM_GLOBALS.rgb_illum2_spect_green;
                }
            } else if rgb[1] <= rgb[0] && rgb[1] <= rgb[2] {
                // Compute illuminant _SampledSpectrum_ with _rgb[1]_ as minimum
                r += rgb[1] * SAMPLED_SPECTRUM_GLOBALS.rgb_illum2_spect_white;
                if rgb[0] <= rgb[2] {
                    r += (rgb[0] - rgb[1]) * SAMPLED_SPECTRUM_GLOBALS.rgb_illum2_spect_magenta;
                    r += (rgb[2] - rgb[0]) * SAMPLED_SPECTRUM_GLOBALS.rgb_illum2_spect_blue;
                } else {
                    r += (rgb[2] - rgb[1]) * SAMPLED_SPECTRUM_GLOBALS.rgb_illum2_spect_magenta;
                    r += (rgb[0] - rgb[2]) * SAMPLED_SPECTRUM_GLOBALS.rgb_illum2_spect_red;
                }
            } else {
                // Compute illuminant _SampledSpectrum_ with _rgb[2]_ as minimum
                r += rgb[2] * SAMPLED_SPECTRUM_GLOBALS.rgb_illum2_spect_white;
                if rgb[0] <= rgb[1] {
                    r += (rgb[0] - rgb[2]) * SAMPLED_SPECTRUM_GLOBALS.rgb_illum2_spect_yellow;
                    r += (rgb[1] - rgb[0]) * SAMPLED_SPECTRUM_GLOBALS.rgb_illum2_spect_green;
                } else {
                    r += (rgb[1] - rgb[2]) * SAMPLED_SPECTRUM_GLOBALS.rgb_illum2_spect_yellow;
                    r += (rgb[0] - rgb[1]) * SAMPLED_SPECTRUM_GLOBALS.rgb_illum2_spect_red;
                }
            }
            r *= 0.86445;
        }
        r.clamp(0.0, num::Float::infinity());
        r
    }

    pub fn from_xyz(xyz: &[Float; 3], spectrum_type: SpectrumType) -> SampledSpectrum {
        let mut rgb: [Float; 3] = [0.0; 3];
        xyz_to_rgb(xyz, &mut rgb);
        SampledSpectrum::from_rgb(&rgb, spectrum_type)
    }

    pub fn to_rgb(&self, rgb: &mut [Float; 3]) {
        let mut xyz: [Float; 3] = [0.0; 3];
        self.to_xyz(&mut xyz);
        xyz_to_rgb(&xyz, rgb)
    }

    pub fn to_xyz(&self, xyz: &mut [Float; 3]) {
        xyz[0] = 0.0;
        xyz[1] = 0.0;
        xyz[2] = 0.0;
        for i in 0..N_SPECTRAL_SAMPLES {
            xyz[0] += SAMPLED_SPECTRUM_GLOBALS.x.c[i] * self.c[i];
            xyz[1] += SAMPLED_SPECTRUM_GLOBALS.y.c[i] * self.c[i];
            xyz[2] += SAMPLED_SPECTRUM_GLOBALS.z.c[i] * self.c[i];
        }
        let scale = (SAMPLED_LAMBDA_END - SAMPLED_LAMBDA_START) as Float /
            (CIE_Y_INTEGRAL * N_SPECTRAL_SAMPLES as Float);
        xyz[0] *= scale;
        xyz[1] *= scale;
        xyz[2] *= scale;
    }

    pub fn to_rgb_spectrum(&self) -> RGBSpectrum {
        let mut rgb: [Float; 3] = [0.0; 3];
        self.to_rgb(&mut rgb);
        RGBSpectrum::from_rgb(&rgb)
    }

    pub fn y(&self) -> Float {
        let mut yy: Float = 0.0;
        for i in 0..N_SPECTRAL_SAMPLES {
            yy += SAMPLED_SPECTRUM_GLOBALS.y.c[i] * self.c[i];
        }
        yy * (SAMPLED_LAMBDA_END - SAMPLED_LAMBDA_START) as Float / 
            (CIE_Y_INTEGRAL * N_SPECTRAL_SAMPLES as Float)
    }

    CoefficientSpectrumMethods!(N_SPECTRAL_SAMPLES);
}

CoefficientSpectrumImpl!(SampledSpectrum, N_SPECTRAL_SAMPLES);

impl Default for SampledSpectrum {
    fn default() -> SampledSpectrum {
        SampledSpectrum{
            c: [0.0; N_SPECTRAL_SAMPLES]
        }
    }
}

impl From<RGBSpectrum> for SampledSpectrum {
    fn from(r: &RGBSpectrum) -> Self {
        let mut rgb: [Float; 3] = [0.0; 3];
        r.to_rgb(&mut rgb);
        SampledSpectrum::from_rgb(&rgb, SpectrumType::Reflectance)
    }
}

#[derive(Debug, Copy, Clone)]
pub struct RGBSpectrum {
    c: [Float; 3]
}

impl RGBSpectrum {
    pub fn new(v: Float) -> RGBSpectrum {
        RGBSpectrum{
            c: [v; 3]
        }
    }

    pub fn from_rgb(rgb: &[Float; 3]) -> RGBSpectrum {
        let mut s = RGBSpectrum::default();
        s.c[0] = rgb[0];
        s.c[1] = rgb[1];
        s.c[2] = rgb[2];
        assert!(!s.has_nans());
        s
    }

    pub fn from_xyz(xyz: &[Float; 3], spectrum_type: SpectrumType) -> RGBSpectrum {
        let mut r = RGBSpectrum::default();
        xyz_to_rgb(xyz, &mut r.c);
        r
    }

    pub fn from_sampled(
        lambda: &[Float],
        v: &[Float]
    ) -> RGBSpectrum {
        // Sort samples if unordered, use sorted for returned spectrum
        if !spectrum_samples_sorted(lambda) {
            let slambda = lambda.clone();
            let sv = v.clone();
            sort_spectrum_sample(&mut slambda, &mut sv);
            return RGBSpectrum::from_sampled(slambda, v);
        }
        let mut xyz: [Float; 3] = [0.0, 0.0, 0.0];
        for i in 0..N_CIE_SAMPLES {
            let val = interpolate_spectrum_samples(lambda, v, CIE_lambda[i]);
            xyz[0] += val * CIE_X[i];
            xyz[1] += val * CIE_Y[i];
            xyz[2] += val * CIE_Z[i];
        }
        let scale = (CIE_lambda[N_CIE_SAMPLES - 1] - CIE_lambda[0]) / (CIE_Y_INTEGRAL * N_CIE_SAMPLES as Float);
        xyz[0] *= scale;
        xyz[1] *= scale;
        xyz[2] *= scale;
        RGBSpectrum::from_rgb(&xyz)
    }

    pub fn to_rgb(&self, rgb: &mut [Float; 3]) {
        rgb[0] = self.c[0];
        rgb[1] = self.c[1];
        rgb[2] = self.c[2];
    }

    pub fn to_xyz(&self, xyz: &mut [Float; 3]) {
        rgb_to_xyz(&self.c, xyz);
    }

    pub fn y(&self) -> Float {
        let y_weight: [Float; 3] = [0.212671, 0.715160, 0.072169];
        y_weight[0] * self.c[0] + y_weight[1] * self.c[1] + y_weight[2] * self.c[2]
    }

    CoefficientSpectrumMethods!(3);
}

CoefficientSpectrumImpl!(RGBSpectrum, 3);

impl Default for RGBSpectrum {
    fn default() -> RGBSpectrum {
        RGBSpectrum {
            c: [0.0; 3]
        }
    }
}

struct SampledSpectrumGlobals {
    pub x: SampledSpectrum,
    pub y: SampledSpectrum,
    pub z: SampledSpectrum,
    pub rgb_refl2_spect_white: SampledSpectrum,
    pub rgb_refl2_spect_cyan: SampledSpectrum,
    pub rgb_refl2_spect_magenta: SampledSpectrum,
    pub rgb_refl2_spect_yellow: SampledSpectrum,
    pub rgb_refl2_spect_red: SampledSpectrum,
    pub rgb_refl2_spect_green: SampledSpectrum,
    pub rgb_refl2_spect_blue: SampledSpectrum,
    pub rgb_illum2_spect_white: SampledSpectrum,
    pub rgb_illum2_spect_cyan: SampledSpectrum,
    pub rgb_illum2_spect_magenta: SampledSpectrum,
    pub rgb_illum2_spect_yellow: SampledSpectrum,
    pub rgb_illum2_spect_red: SampledSpectrum,
    pub rgb_illum2_spect_green: SampledSpectrum,
    pub rgb_illum2_spect_blue: SampledSpectrum,
}

impl SampledSpectrumGlobals {
    pub fn new() -> SampledSpectrumGlobals {
        let mut g = SampledSpectrumGlobals {
            x: SampledSpectrum::default(),
            y: SampledSpectrum::default(),
            z: SampledSpectrum::default(),
            rgb_refl2_spect_white: SampledSpectrum::default(),
            rgb_refl2_spect_cyan: SampledSpectrum::default(),
            rgb_refl2_spect_magenta: SampledSpectrum::default(),
            rgb_refl2_spect_yellow: SampledSpectrum::default(),
            rgb_refl2_spect_red: SampledSpectrum::default(),
            rgb_refl2_spect_green: SampledSpectrum::default(),
            rgb_refl2_spect_blue: SampledSpectrum::default(),
            rgb_illum2_spect_white: SampledSpectrum::default(),
            rgb_illum2_spect_cyan: SampledSpectrum::default(),
            rgb_illum2_spect_magenta: SampledSpectrum::default(),
            rgb_illum2_spect_yellow: SampledSpectrum::default(),
            rgb_illum2_spect_red: SampledSpectrum::default(),
            rgb_illum2_spect_green: SampledSpectrum::default(),
            rgb_illum2_spect_blue: SampledSpectrum::default(),
        };

        // SampledSpectrum::Init()
        for i in 0..N_SPECTRAL_SAMPLES {
            let wl0 = lerp(i as Float / N_SPECTRAL_SAMPLES as Float,
                SAMPLED_LAMBDA_START, SAMPLED_LAMBDA_END);
            let wl1 = lerp((i as Float + 1.0) / N_SPECTRAL_SAMPLES as Float,
                SAMPLED_LAMBDA_START, SAMPLED_LAMBDA_END);
            
            // Compute XYZ matching functions for _SampledSpectrum_
            g.x.c[i] = average_spectrum_samples(&CIE_lambda, &CIE_X, wl0, wl1);
            g.y.c[i] = average_spectrum_samples(&CIE_lambda, &CIE_Y, wl0, wl1);
            g.z.c[i] = average_spectrum_samples(&CIE_lambda, &CIE_Z, wl0, wl1);

            // Compute RGB to spectrum functions for _SampledSpectrum_
            g.rgb_refl2_spect_white.c[i] = average_spectrum_samples(
                &RGB2SpectLambda, &RGBRefl2SpectWhite, wl0, wl1);
            g.rgb_refl2_spect_cyan.c[i] = average_spectrum_samples(
                &RGB2SpectLambda, &RGBRefl2SpectCyan, wl0, wl1);
            g.rgb_refl2_spect_magenta.c[i] = average_spectrum_samples(
                &RGB2SpectLambda, &RGBRefl2SpectMagenta, wl0, wl1);
            g.rgb_refl2_spect_yellow.c[i] = average_spectrum_samples(
                &RGB2SpectLambda, &RGBRefl2SpectYellow, wl0, wl1);
            g.rgb_refl2_spect_red.c[i] = average_spectrum_samples(
                &RGB2SpectLambda, &RGBRefl2SpectRed, wl0, wl1);
            g.rgb_refl2_spect_green.c[i] = average_spectrum_samples(
                &RGB2SpectLambda, &RGBRefl2SpectGreen, wl0, wl1);
            g.rgb_refl2_spect_blue.c[i] = average_spectrum_samples(
                &RGB2SpectLambda, &RGBRefl2SpectBlue, wl0, wl1);
            
            g.rgb_illum2_spect_white.c[i] = average_spectrum_samples(
                &RGB2SpectLambda, &RGBIllum2SpectWhite, wl0, wl1);
            g.rgb_illum2_spect_cyan.c[i] = average_spectrum_samples(
                &RGB2SpectLambda, &RGBIllum2SpectCyan, wl0, wl1);
            g.rgb_illum2_spect_magenta.c[i] = average_spectrum_samples(
                &RGB2SpectLambda, &RGBIllum2SpectMagenta, wl0, wl1);
            g.rgb_illum2_spect_yellow.c[i] = average_spectrum_samples(
                &RGB2SpectLambda, &RGBIllum2SpectYellow, wl0, wl1);
            g.rgb_illum2_spect_red.c[i] = average_spectrum_samples(
                &RGB2SpectLambda, &RGBIllum2SpectRed, wl0, wl1);
            g.rgb_illum2_spect_green.c[i] = average_spectrum_samples(
                &RGB2SpectLambda, &RGBIllum2SpectGreen, wl0, wl1);
            g.rgb_illum2_spect_blue.c[i] = average_spectrum_samples(
                &RGB2SpectLambda, &RGBIllum2SpectBlue, wl0, wl1);
        }
        g
    }
}

pub fn spectrum_samples_sorted(lambda: &[Float]) -> bool {
    for i in 0..lambda.len() - 1 {
        if lambda[i] > lambda[i + 1] {
            return false;
        }
    }
    true
}

pub fn sort_spectrum_sample(
    lambda: &mut [Float],
    vals: &mut [Float]
) {
    assert!(lambda.len() == vals.len());
    let mut sort_vec: Vec<(Float, Float)> = Vec::with_capacity(lambda.len());
    for i in 0..lambda.len() {
        sort_vec.push((lambda[i], vals[i]));
    }
    sort_vec.sort_by(|a,b| a.0.partial_cmp(&b.0).unwrap());
    for i in 0..lambda.len() {
        lambda[i] = sort_vec[i].0;
        vals[i] = sort_vec[i].1;
    }
}

pub fn average_spectrum_samples(
    lambda: &[Float],
    vals: &[Float],
    lambda_start: Float,
    lambda_end: Float
) -> Float {
    let n = lambda.len();
    for i in 0..n {
        assert!(lambda[i + 1] > lambda[i]);
    }
    assert!(lambda_start < lambda_end);
    // Handle cases with out-of-bounds range or single sample only
    if lambda_end <= lambda[0] || n == 1 {
        return vals[0];
    }
    if lambda_start >= lambda[n - 1] {
        return vals[n - 1];
    }
    let mut sum: Float = 0.0;
    // Add contributions of constant segments before/after samples
    if lambda_start < lambda[0] {
        sum += vals[0] * (lambda[0] - lambda_start);
    }
    if lambda_end > lambda[n - 1] {
        sum += vals[n - 1] * (lambda_end - lambda[n - 1]);
    }

    // Advance to first relevant wavelength segment
    let i = 0_usize;
    while lambda_start > lambda[i + 1] {
        i += 1;
    }
    assert!(i + 1 < n);

    // Loop over wavelength sample segments and add contributions
    let interp = |w: Float, i: usize| lerp((w - lambda[i]) / (lambda[i + 1] - lambda[i]), vals[i], vals[i + 1]);
    while i + 1 < n && lambda_end >= lambda[i] {
        let seg_lambda_start = lambda_start.max(lambda[i]);
        let seg_lambda_end = lambda_end.min(lambda[i + 1]);
        sum += 0.5 * (interp(seg_lambda_start, i) + interp(seg_lambda_end, i)) *
            (seg_lambda_end - seg_lambda_start);
        i += 1;
    }
    sum / (lambda_end - lambda_start)
}

/// Given a piecewise-linear SPD with values in vIn[] at corresponding
/// wavelengths lambdaIn[], where lambdaIn is assumed to be sorted but may
/// be irregularly spaced, resample the spectrum over the range of
/// wavelengths [lambdaMin, lambdaMax], with a total of nOut wavelength
/// samples between lambdaMin and lamdbaMax (including those at
/// endpoints). The resampled spectrum values are written to vOut.
///
/// In general, this is a standard sampling and reconstruction problem, with
/// the complication that for any given invocation, some of the
/// reconstruction points may involve upsampling the input distribution and
/// others may involve downsampling. For upsampling, we just point-sample,
/// and for downsampling, we apply a box filter centered around the
/// destination wavelength with total width equal to the sample spacing.
pub fn resample_linear_spectrum(
    lambda_in: &[Float],
    v_in: &[Float],
    lambda_min: Float,
    lambda_max: Float,
    v_out: &mut [Float]
) {
    let n_in = v_in.len();
    let n_out = v_out.len();
    assert!(n_out >= 2);
    for i in 0..n_in - 1 {
        assert!(lambda_in[i + 1] > lambda_in[i]);
    }
    assert!(lambda_min < lambda_max);

    // Spacing between samples in the output distribution.
    let delta = (lambda_max - lambda_min) / (n_out - 1) as Float;

    // We assume that the SPD is constant outside of the specified
    // wavelength range, taking on the respectively first/last SPD value
    // for out-of-range wavelengths.
    //
    // To make this convention fit cleanly into the code below, we create
    // virtual samples in the input distribution with index -1 for the
    // sample before the first valid sample and index nIn for the sample
    // after the last valid sample. In turn, can place those virtual
    // samples beyond the endpoints of the target range so that we can
    // always assume that the source range is broader than the target
    // range, which in turn lets us not worry about various boundary cases
    // below.

    // The wavelengths of the virtual samples at the endpoints are set so
    // that they are one destination sample spacing beyond the destination
    // endpoints.  (Note that this potentially means that if we swept along
    // indices from -1 to nIn, we wouldn't always see a monotonically
    // increasing set of wavelength values. However, this isn't a problem
    // since we only access these virtual samples if the destination range
    // is wider than the source range.)
    let lamdba_in_clamped = |index: i32| {
        assert!(index >= -1 && index <= n_in as i32);
        if index == -1 {
            assert!(lambda_min - delta < lambda_in[0]);
            return lambda_min - delta;
        } else if index == n_in as i32 {
            assert!(lambda_max + delta > lambda_in[n_in - 1]);
            return lambda_max + delta;
        }
        lambda_in[index as usize]
    };

    // Due to the piecewise-constant assumption, the SPD values outside the
    // specified range are given by the valid endpoints.
    let v_in_clamped = |index: i32| {
        assert!(index >= -1 && index <= n_in as i32);
        v_in[num::clamp(index as usize, 0, n_in - 1)]
    };

    // Helper that upsamples ors downsample the given SPD at the given
    // wavelength lambda.
    let resample = |lambda: Float| {
        // Handle the edge cases first so that we don't need to worry about
        // them in the following.
        //
        // First, if the entire filtering range for the destination is
        // outside of the range of valid samples, we can just return the
        // endpoint value.
        if lambda + delta / 2.0 <= lambda_in[0] {
            return v_in[0];
        }
        if lambda - delta / 2.0 >= lambda_in[n_in - 1] {
            return v_in[n_in - 1];
        }
        // Second, if there's only one sample, then the SPD has the same
        // value everywhere, and we're done.
        if n_in == 1 {
            return v_in[0];
        }

        // Otherwise, find indices into the input SPD that bracket the
        // wavelength range [lambda-delta, lambda+delta]. Note that this is
        // a 2x wider range than we will actually filter over in the end.
        let mut start: i32 = 0;
        let mut end: i32 = 0;
        if lambda - delta < lambda_in[0] {
            // Virtual sample at the start, as described above.
            start = -1;
        } else {
            start = find_interval(n_in, |i| lambda_in[i] <= lambda - delta) as i32;
            assert!(start >= 0 && start < n_in as i32);
        }

        if lambda + delta > lambda_in[n_in - 1] {
            // Virtual sample at the end, as described above.
            end = n_in as i32;
        } else {
            // Linear search from the starting point. (Presumably more
            // efficient than a binary search from scratch, or doesn't
            // matter either way.)
            end = if start > 0 { start } else { 0 };
            while end < n_in as i32 && lambda + delta > lambda_in[end as usize] {
                end += 1;
            }
        }

        if end - start == 2 && lamdba_in_clamped(start) <= lambda - delta &&
            lambda_in[start as usize + 1] == lambda &&
            lamdba_in_clamped(end) >= lambda + delta {
            // Downsampling: special case where the input and output
            // wavelengths line up perfectly, so just return the
            // corresponding point sample at lambda.
            return v_in[start as usize + 1];
        } else if end - start == 1 {
            // Downsampling: evaluate the piecewise-linear function at
            // lambda.
            let t = (lambda - lamdba_in_clamped(start)) /
                (lamdba_in_clamped(end) - lamdba_in_clamped(start));
            assert!(t >= 0.0 && t <= 1.0);
            return lerp(t, v_in_clamped(start), v_in_clamped(end));
        } else {
            // Upsampling: use a box filter and average all values in the
            // input spectrum from lambda +/- delta / 2.
            return average_spectrum_samples(lambda_in, v_in, lambda - delta / 2.0, lambda + delta / 2.0);
        }
    };

    // For each destination sample, compute the wavelength lambda for the
    // sample and then resample the source SPD distribution at that point.
    for out_offset in 0..n_out {
        // TODO: Currently, resample() does a binary search each time,
        // even though we could do a single sweep across the input array,
        // since we're resampling it at a regular and increasing set of
        // lambdas. It would be nice to polish that up.
        let lambda = lerp(out_offset as Float / (n_out - 1) as Float, lambda_min, lambda_max);
        v_out[out_offset] = resample(lambda);
    }
}

#[inline]
pub fn xyz_to_rgb(xyz: &[Float; 3], rgb: &mut [Float; 3]) {
    rgb[0] = 3.240479 * xyz[0] - 1.537150 * xyz[1] - 0.498535 * xyz[2];
    rgb[1] = -0.969256 * xyz[0] + 1.875991 * xyz[1] + 0.041556 * xyz[2];
    rgb[2] = 0.055648 * xyz[0] - 0.204043 * xyz[1] + 1.057311 * xyz[2];
}

#[inline]
pub fn rgb_to_xyz(rgb: &[Float; 3], xyz: &mut [Float; 3]) {
    xyz[0] = 0.412453 * rgb[0] + 0.357580 * rgb[1] + 0.180423 * rgb[2];
    xyz[1] = 0.212671 * rgb[0] + 0.715160 * rgb[1] + 0.072169 * rgb[2];
    xyz[2] = 0.019334 * rgb[0] + 0.119193 * rgb[1] + 0.950227 * rgb[2];
}

#[derive(Debug, Copy, Clone, PartialEq, PartialOrd)]
pub enum SpectrumType {
    Reflectance,
    Illuminant
}

pub fn interpolate_spectrum_samples(
    lambda: &[Float],
    vals: &[Float],
    l: Float
) -> Float {
    let n = lambda.len();
    for i in 0..n {
        assert!(lambda[i + 1] > lambda[i]);
    }
    if l <= lambda[0] {
        return vals[0];
    }
    if l >= lambda[n - 1] {
        return vals[n - 1];
    }
    let offset = find_interval(n, |index| lambda[index] <= l);
    assert!(l >= lambda[offset] && l <= lambda[offset + 1]);
    let t = (l - lambda[offset]) / (lambda[offset + 1] - lambda[offset]);
    lerp(t, vals[offset], vals[offset + 1])
}

pub fn blackbody(
    lambda: &[Float],
    t: Float,
    le: &mut [Float]
) {
    let n = lambda.len();
    if t <= 0.0 {
        for i in 0..n {
            le[i] = 0.0;
        }
        return;
    }
    const c: Float = 299792458.0;
    const h: Float = 6.62606957e-34;
    const kb: Float = 1.3806488e-23;
    for i in 0..n {
        // Compute emitted radiance for blackbody at wavelength _lambda[i]_
        let l = lambda[i] * 1e-9;
        let lambda5 = (l * l) * (l * l) * l;
        le[i] = (2.0 * h * c * c) /
            (lambda5 * (((h * c) / (l * kb * t)).exp() - 1.0));
        assert!(!le[i].is_nan());
    }
}

pub fn blackbody_normalized(
    lambda: &[Float],
    t: Float,
    le: &mut [Float]
) {
    blackbody(lambda, t, le);
    // Normalize _Le_ values based on maximum blackbody radiance
    let lambda_max = [2.8977721e-3 / t * 1e9];
    let mut max_l: [Float; 1] = [0.0];
    blackbody(&lambda_max, t, &mut max_l);
    for i in 0..lambda.len() {
        le[i] /= max_l[0];
    }
}

pub const CIE_Y_INTEGRAL: Float = 106.856895;
pub const N_RGB2_SPECT_SAMPLES: usize = 32;


pub const N_CIE_SAMPLES: usize = 471;
pub const CIE_X: [Float; N_CIE_SAMPLES] = [
    // CIE X function values
    0.0001299000,   0.0001458470,   0.0001638021,   0.0001840037,
    0.0002066902,   0.0002321000,   0.0002607280,   0.0002930750,
    0.0003293880,   0.0003699140,   0.0004149000,   0.0004641587,
    0.0005189860,   0.0005818540,   0.0006552347,   0.0007416000,
    0.0008450296,   0.0009645268,   0.001094949,    0.001231154,
    0.001368000,    0.001502050,    0.001642328,    0.001802382,
    0.001995757,    0.002236000,    0.002535385,    0.002892603,
    0.003300829,    0.003753236,    0.004243000,    0.004762389,
    0.005330048,    0.005978712,    0.006741117,    0.007650000,
    0.008751373,    0.01002888,     0.01142170,     0.01286901,
    0.01431000,     0.01570443,     0.01714744,     0.01878122,
    0.02074801,     0.02319000,     0.02620736,     0.02978248,
    0.03388092,     0.03846824,     0.04351000,     0.04899560,
    0.05502260,     0.06171880,     0.06921200,     0.07763000,
    0.08695811,     0.09717672,     0.1084063,      0.1207672,
    0.1343800,      0.1493582,      0.1653957,      0.1819831,
    0.1986110,      0.2147700,      0.2301868,      0.2448797,
    0.2587773,      0.2718079,      0.2839000,      0.2949438,
    0.3048965,      0.3137873,      0.3216454,      0.3285000,
    0.3343513,      0.3392101,      0.3431213,      0.3461296,
    0.3482800,      0.3495999,      0.3501474,      0.3500130,
    0.3492870,      0.3480600,      0.3463733,      0.3442624,
    0.3418088,      0.3390941,      0.3362000,      0.3331977,
    0.3300411,      0.3266357,      0.3228868,      0.3187000,
    0.3140251,      0.3088840,      0.3032904,      0.2972579,
    0.2908000,      0.2839701,      0.2767214,      0.2689178,
    0.2604227,      0.2511000,      0.2408475,      0.2298512,
    0.2184072,      0.2068115,      0.1953600,      0.1842136,
    0.1733273,      0.1626881,      0.1522833,      0.1421000,
    0.1321786,      0.1225696,      0.1132752,      0.1042979,
    0.09564000,     0.08729955,     0.07930804,     0.07171776,
    0.06458099,     0.05795001,     0.05186211,     0.04628152,
    0.04115088,     0.03641283,     0.03201000,     0.02791720,
    0.02414440,     0.02068700,     0.01754040,     0.01470000,
    0.01216179,     0.009919960,    0.007967240,    0.006296346,
    0.004900000,    0.003777173,    0.002945320,    0.002424880,
    0.002236293,    0.002400000,    0.002925520,    0.003836560,
    0.005174840,    0.006982080,    0.009300000,    0.01214949,
    0.01553588,     0.01947752,     0.02399277,     0.02910000,
    0.03481485,     0.04112016,     0.04798504,     0.05537861,
    0.06327000,     0.07163501,     0.08046224,     0.08973996,
    0.09945645,     0.1096000,      0.1201674,      0.1311145,
    0.1423679,      0.1538542,      0.1655000,      0.1772571,
    0.1891400,      0.2011694,      0.2133658,      0.2257499,
    0.2383209,      0.2510668,      0.2639922,      0.2771017,
    0.2904000,      0.3038912,      0.3175726,      0.3314384,
    0.3454828,      0.3597000,      0.3740839,      0.3886396,
    0.4033784,      0.4183115,      0.4334499,      0.4487953,
    0.4643360,      0.4800640,      0.4959713,      0.5120501,
    0.5282959,      0.5446916,      0.5612094,      0.5778215,
    0.5945000,      0.6112209,      0.6279758,      0.6447602,
    0.6615697,      0.6784000,      0.6952392,      0.7120586,
    0.7288284,      0.7455188,      0.7621000,      0.7785432,
    0.7948256,      0.8109264,      0.8268248,      0.8425000,
    0.8579325,      0.8730816,      0.8878944,      0.9023181,
    0.9163000,      0.9297995,      0.9427984,      0.9552776,
    0.9672179,      0.9786000,      0.9893856,      0.9995488,
    1.0090892,      1.0180064,      1.0263000,      1.0339827,
    1.0409860,      1.0471880,      1.0524667,      1.0567000,
    1.0597944,      1.0617992,      1.0628068,      1.0629096,
    1.0622000,      1.0607352,      1.0584436,      1.0552244,
    1.0509768,      1.0456000,      1.0390369,      1.0313608,
    1.0226662,      1.0130477,      1.0026000,      0.9913675,
    0.9793314,      0.9664916,      0.9528479,      0.9384000,
    0.9231940,      0.9072440,      0.8905020,      0.8729200,
    0.8544499,      0.8350840,      0.8149460,      0.7941860,
    0.7729540,      0.7514000,      0.7295836,      0.7075888,
    0.6856022,      0.6638104,      0.6424000,      0.6215149,
    0.6011138,      0.5811052,      0.5613977,      0.5419000,
    0.5225995,      0.5035464,      0.4847436,      0.4661939,
    0.4479000,      0.4298613,      0.4120980,      0.3946440,
    0.3775333,      0.3608000,      0.3444563,      0.3285168,
    0.3130192,      0.2980011,      0.2835000,      0.2695448,
    0.2561184,      0.2431896,      0.2307272,      0.2187000,
    0.2070971,      0.1959232,      0.1851708,      0.1748323,
    0.1649000,      0.1553667,      0.1462300,      0.1374900,
    0.1291467,      0.1212000,      0.1136397,      0.1064650,
    0.09969044,     0.09333061,     0.08740000,     0.08190096,
    0.07680428,     0.07207712,     0.06768664,     0.06360000,
    0.05980685,     0.05628216,     0.05297104,     0.04981861,
    0.04677000,     0.04378405,     0.04087536,     0.03807264,
    0.03540461,     0.03290000,     0.03056419,     0.02838056,
    0.02634484,     0.02445275,     0.02270000,     0.02108429,
    0.01959988,     0.01823732,     0.01698717,     0.01584000,
    0.01479064,     0.01383132,     0.01294868,     0.01212920,
    0.01135916,     0.01062935,     0.009938846,    0.009288422,
    0.008678854,    0.008110916,    0.007582388,    0.007088746,
    0.006627313,    0.006195408,    0.005790346,    0.005409826,
    0.005052583,    0.004717512,    0.004403507,    0.004109457,
    0.003833913,    0.003575748,    0.003334342,    0.003109075,
    0.002899327,    0.002704348,    0.002523020,    0.002354168,
    0.002196616,    0.002049190,    0.001910960,    0.001781438,
    0.001660110,    0.001546459,    0.001439971,    0.001340042,
    0.001246275,    0.001158471,    0.001076430,    0.0009999493,
    0.0009287358,   0.0008624332,   0.0008007503,   0.0007433960,
    0.0006900786,   0.0006405156,   0.0005945021,   0.0005518646,
    0.0005124290,   0.0004760213,   0.0004424536,   0.0004115117,
    0.0003829814,   0.0003566491,   0.0003323011,   0.0003097586,
    0.0002888871,   0.0002695394,   0.0002515682,   0.0002348261,
    0.0002191710,   0.0002045258,   0.0001908405,   0.0001780654,
    0.0001661505,   0.0001550236,   0.0001446219,   0.0001349098,
    0.0001258520,   0.0001174130,   0.0001095515,   0.0001022245,
    0.00009539445,  0.00008902390,  0.00008307527,  0.00007751269,
    0.00007231304,  0.00006745778,  0.00006292844,  0.00005870652,
    0.00005477028,  0.00005109918,  0.00004767654,  0.00004448567,
    0.00004150994,  0.00003873324,  0.00003614203,  0.00003372352,
    0.00003146487,  0.00002935326,  0.00002737573,  0.00002552433,
    0.00002379376,  0.00002217870,  0.00002067383,  0.00001927226,
    0.00001796640,  0.00001674991,  0.00001561648,  0.00001455977,
    0.00001357387,  0.00001265436,  0.00001179723,  0.00001099844,
    0.00001025398,  0.000009559646, 0.000008912044, 0.000008308358,
    0.000007745769, 0.000007221456, 0.000006732475, 0.000006276423,
    0.000005851304, 0.000005455118, 0.000005085868, 0.000004741466,
    0.000004420236, 0.000004120783, 0.000003841716, 0.000003581652,
    0.000003339127, 0.000003112949, 0.000002902121, 0.000002705645,
    0.000002522525, 0.000002351726, 0.000002192415, 0.000002043902,
    0.000001905497, 0.000001776509, 0.000001656215, 0.000001544022,
    0.000001439440, 0.000001341977, 0.000001251141];

pub const CIE_Y: [Float; N_CIE_SAMPLES] = [
    // CIE Y function values
    0.000003917000,  0.000004393581,  0.000004929604,  0.000005532136,
    0.000006208245,  0.000006965000,  0.000007813219,  0.000008767336,
    0.000009839844,  0.00001104323,   0.00001239000,   0.00001388641,
    0.00001555728,   0.00001744296,   0.00001958375,   0.00002202000,
    0.00002483965,   0.00002804126,   0.00003153104,   0.00003521521,
    0.00003900000,   0.00004282640,   0.00004691460,   0.00005158960,
    0.00005717640,   0.00006400000,   0.00007234421,   0.00008221224,
    0.00009350816,   0.0001061361,    0.0001200000,    0.0001349840,
    0.0001514920,    0.0001702080,    0.0001918160,    0.0002170000,
    0.0002469067,    0.0002812400,    0.0003185200,    0.0003572667,
    0.0003960000,    0.0004337147,    0.0004730240,    0.0005178760,
    0.0005722187,    0.0006400000,    0.0007245600,    0.0008255000,
    0.0009411600,    0.001069880,     0.001210000,     0.001362091,
    0.001530752,     0.001720368,     0.001935323,     0.002180000,
    0.002454800,     0.002764000,     0.003117800,     0.003526400,
    0.004000000,     0.004546240,     0.005159320,     0.005829280,
    0.006546160,     0.007300000,     0.008086507,     0.008908720,
    0.009767680,     0.01066443,      0.01160000,      0.01257317,
    0.01358272,      0.01462968,      0.01571509,      0.01684000,
    0.01800736,      0.01921448,      0.02045392,      0.02171824,
    0.02300000,      0.02429461,      0.02561024,      0.02695857,
    0.02835125,      0.02980000,      0.03131083,      0.03288368,
    0.03452112,      0.03622571,      0.03800000,      0.03984667,
    0.04176800,      0.04376600,      0.04584267,      0.04800000,
    0.05024368,      0.05257304,      0.05498056,      0.05745872,
    0.06000000,      0.06260197,      0.06527752,      0.06804208,
    0.07091109,      0.07390000,      0.07701600,      0.08026640,
    0.08366680,      0.08723280,      0.09098000,      0.09491755,
    0.09904584,      0.1033674,       0.1078846,       0.1126000,
    0.1175320,       0.1226744,       0.1279928,       0.1334528,
    0.1390200,       0.1446764,       0.1504693,       0.1564619,
    0.1627177,       0.1693000,       0.1762431,       0.1835581,
    0.1912735,       0.1994180,       0.2080200,       0.2171199,
    0.2267345,       0.2368571,       0.2474812,       0.2586000,
    0.2701849,       0.2822939,       0.2950505,       0.3085780,
    0.3230000,       0.3384021,       0.3546858,       0.3716986,
    0.3892875,       0.4073000,       0.4256299,       0.4443096,
    0.4633944,       0.4829395,       0.5030000,       0.5235693,
    0.5445120,       0.5656900,       0.5869653,       0.6082000,
    0.6293456,       0.6503068,       0.6708752,       0.6908424,
    0.7100000,       0.7281852,       0.7454636,       0.7619694,
    0.7778368,       0.7932000,       0.8081104,       0.8224962,
    0.8363068,       0.8494916,       0.8620000,       0.8738108,
    0.8849624,       0.8954936,       0.9054432,       0.9148501,
    0.9237348,       0.9320924,       0.9399226,       0.9472252,
    0.9540000,       0.9602561,       0.9660074,       0.9712606,
    0.9760225,       0.9803000,       0.9840924,       0.9874812,
    0.9903128,       0.9928116,       0.9949501,       0.9967108,
    0.9980983,       0.9991120,       0.9997482,       1.0000000,
    0.9998567,       0.9993046,       0.9983255,       0.9968987,
    0.9950000,       0.9926005,       0.9897426,       0.9864444,
    0.9827241,       0.9786000,       0.9740837,       0.9691712,
    0.9638568,       0.9581349,       0.9520000,       0.9454504,
    0.9384992,       0.9311628,       0.9234576,       0.9154000,
    0.9070064,       0.8982772,       0.8892048,       0.8797816,
    0.8700000,       0.8598613,       0.8493920,       0.8386220,
    0.8275813,       0.8163000,       0.8047947,       0.7930820,
    0.7811920,       0.7691547,       0.7570000,       0.7447541,
    0.7324224,       0.7200036,       0.7074965,       0.6949000,
    0.6822192,       0.6694716,       0.6566744,       0.6438448,
    0.6310000,       0.6181555,       0.6053144,       0.5924756,
    0.5796379,       0.5668000,       0.5539611,       0.5411372,
    0.5283528,       0.5156323,       0.5030000,       0.4904688,
    0.4780304,       0.4656776,       0.4534032,       0.4412000,
    0.4290800,       0.4170360,       0.4050320,       0.3930320,
    0.3810000,       0.3689184,       0.3568272,       0.3447768,
    0.3328176,       0.3210000,       0.3093381,       0.2978504,
    0.2865936,       0.2756245,       0.2650000,       0.2547632,
    0.2448896,       0.2353344,       0.2260528,       0.2170000,
    0.2081616,       0.1995488,       0.1911552,       0.1829744,
    0.1750000,       0.1672235,       0.1596464,       0.1522776,
    0.1451259,       0.1382000,       0.1315003,       0.1250248,
    0.1187792,       0.1127691,       0.1070000,       0.1014762,
    0.09618864,      0.09112296,      0.08626485,      0.08160000,
    0.07712064,      0.07282552,      0.06871008,      0.06476976,
    0.06100000,      0.05739621,      0.05395504,      0.05067376,
    0.04754965,      0.04458000,      0.04175872,      0.03908496,
    0.03656384,      0.03420048,      0.03200000,      0.02996261,
    0.02807664,      0.02632936,      0.02470805,      0.02320000,
    0.02180077,      0.02050112,      0.01928108,      0.01812069,
    0.01700000,      0.01590379,      0.01483718,      0.01381068,
    0.01283478,      0.01192000,      0.01106831,      0.01027339,
    0.009533311,     0.008846157,     0.008210000,     0.007623781,
    0.007085424,     0.006591476,     0.006138485,     0.005723000,
    0.005343059,     0.004995796,     0.004676404,     0.004380075,
    0.004102000,     0.003838453,     0.003589099,     0.003354219,
    0.003134093,     0.002929000,     0.002738139,     0.002559876,
    0.002393244,     0.002237275,     0.002091000,     0.001953587,
    0.001824580,     0.001703580,     0.001590187,     0.001484000,
    0.001384496,     0.001291268,     0.001204092,     0.001122744,
    0.001047000,     0.0009765896,    0.0009111088,    0.0008501332,
    0.0007932384,    0.0007400000,    0.0006900827,    0.0006433100,
    0.0005994960,    0.0005584547,    0.0005200000,    0.0004839136,
    0.0004500528,    0.0004183452,    0.0003887184,    0.0003611000,
    0.0003353835,    0.0003114404,    0.0002891656,    0.0002684539,
    0.0002492000,    0.0002313019,    0.0002146856,    0.0001992884,
    0.0001850475,    0.0001719000,    0.0001597781,    0.0001486044,
    0.0001383016,    0.0001287925,    0.0001200000,    0.0001118595,
    0.0001043224,    0.00009733560,   0.00009084587,   0.00008480000,
    0.00007914667,   0.00007385800,   0.00006891600,   0.00006430267,
    0.00006000000,   0.00005598187,   0.00005222560,   0.00004871840,
    0.00004544747,   0.00004240000,   0.00003956104,   0.00003691512,
    0.00003444868,   0.00003214816,   0.00003000000,   0.00002799125,
    0.00002611356,   0.00002436024,   0.00002272461,   0.00002120000,
    0.00001977855,   0.00001845285,   0.00001721687,   0.00001606459,
    0.00001499000,   0.00001398728,   0.00001305155,   0.00001217818,
    0.00001136254,   0.00001060000,   0.000009885877,  0.000009217304,
    0.000008592362,  0.000008009133,  0.000007465700,  0.000006959567,
    0.000006487995,  0.000006048699,  0.000005639396,  0.000005257800,
    0.000004901771,  0.000004569720,  0.000004260194,  0.000003971739,
    0.000003702900,  0.000003452163,  0.000003218302,  0.000003000300,
    0.000002797139,  0.000002607800,  0.000002431220,  0.000002266531,
    0.000002113013,  0.000001969943,  0.000001836600,  0.000001712230,
    0.000001596228,  0.000001488090,  0.000001387314,  0.000001293400,
    0.000001205820,  0.000001124143,  0.000001048009,  0.0000009770578,
    0.0000009109300, 0.0000008492513, 0.0000007917212, 0.0000007380904,
    0.0000006881098, 0.0000006415300, 0.0000005980895, 0.0000005575746,
    0.0000005198080, 0.0000004846123, 0.0000004518100];

pub const CIE_Z: [Float; N_CIE_SAMPLES] = [
    // CIE Z function values
    0.0006061000,
    0.0006808792,
    0.0007651456,
    0.0008600124,
    0.0009665928,
    0.001086000,
    0.001220586,
    0.001372729,
    0.001543579,
    0.001734286,
    0.001946000,
    0.002177777,
    0.002435809,
    0.002731953,
    0.003078064,
    0.003486000,
    0.003975227,
    0.004540880,
    0.005158320,
    0.005802907,
    0.006450001,
    0.007083216,
    0.007745488,
    0.008501152,
    0.009414544,
    0.01054999,
    0.01196580,
    0.01365587,
    0.01558805,
    0.01773015,
    0.02005001,
    0.02251136,
    0.02520288,
    0.02827972,
    0.03189704,
    0.03621000,
    0.04143771,
    0.04750372,
    0.05411988,
    0.06099803,
    0.06785001,
    0.07448632,
    0.08136156,
    0.08915364,
    0.09854048,
    0.1102000,
    0.1246133,
    0.1417017,
    0.1613035,
    0.1832568,
    0.2074000,
    0.2336921,
    0.2626114,
    0.2947746,
    0.3307985,
    0.3713000,
    0.4162091,
    0.4654642,
    0.5196948,
    0.5795303,
    0.6456000,
    0.7184838,
    0.7967133,
    0.8778459,
    0.9594390,
    1.0390501,
    1.1153673,
    1.1884971,
    1.2581233,
    1.3239296,
    1.3856000,
    1.4426352,
    1.4948035,
    1.5421903,
    1.5848807,
    1.6229600,
    1.6564048,
    1.6852959,
    1.7098745,
    1.7303821,
    1.7470600,
    1.7600446,
    1.7696233,
    1.7762637,
    1.7804334,
    1.7826000,
    1.7829682,
    1.7816998,
    1.7791982,
    1.7758671,
    1.7721100,
    1.7682589,
    1.7640390,
    1.7589438,
    1.7524663,
    1.7441000,
    1.7335595,
    1.7208581,
    1.7059369,
    1.6887372,
    1.6692000,
    1.6475287,
    1.6234127,
    1.5960223,
    1.5645280,
    1.5281000,
    1.4861114,
    1.4395215,
    1.3898799,
    1.3387362,
    1.2876400,
    1.2374223,
    1.1878243,
    1.1387611,
    1.0901480,
    1.0419000,
    0.9941976,
    0.9473473,
    0.9014531,
    0.8566193,
    0.8129501,
    0.7705173,
    0.7294448,
    0.6899136,
    0.6521049,
    0.6162000,
    0.5823286,
    0.5504162,
    0.5203376,
    0.4919673,
    0.4651800,
    0.4399246,
    0.4161836,
    0.3938822,
    0.3729459,
    0.3533000,
    0.3348578,
    0.3175521,
    0.3013375,
    0.2861686,
    0.2720000,
    0.2588171,
    0.2464838,
    0.2347718,
    0.2234533,
    0.2123000,
    0.2011692,
    0.1901196,
    0.1792254,
    0.1685608,
    0.1582000,
    0.1481383,
    0.1383758,
    0.1289942,
    0.1200751,
    0.1117000,
    0.1039048,
    0.09666748,
    0.08998272,
    0.08384531,
    0.07824999,
    0.07320899,
    0.06867816,
    0.06456784,
    0.06078835,
    0.05725001,
    0.05390435,
    0.05074664,
    0.04775276,
    0.04489859,
    0.04216000,
    0.03950728,
    0.03693564,
    0.03445836,
    0.03208872,
    0.02984000,
    0.02771181,
    0.02569444,
    0.02378716,
    0.02198925,
    0.02030000,
    0.01871805,
    0.01724036,
    0.01586364,
    0.01458461,
    0.01340000,
    0.01230723,
    0.01130188,
    0.01037792,
    0.009529306,
    0.008749999,
    0.008035200,
    0.007381600,
    0.006785400,
    0.006242800,
    0.005749999,
    0.005303600,
    0.004899800,
    0.004534200,
    0.004202400,
    0.003900000,
    0.003623200,
    0.003370600,
    0.003141400,
    0.002934800,
    0.002749999,
    0.002585200,
    0.002438600,
    0.002309400,
    0.002196800,
    0.002100000,
    0.002017733,
    0.001948200,
    0.001889800,
    0.001840933,
    0.001800000,
    0.001766267,
    0.001737800,
    0.001711200,
    0.001683067,
    0.001650001,
    0.001610133,
    0.001564400,
    0.001513600,
    0.001458533,
    0.001400000,
    0.001336667,
    0.001270000,
    0.001205000,
    0.001146667,
    0.001100000,
    0.001068800,
    0.001049400,
    0.001035600,
    0.001021200,
    0.001000000,
    0.0009686400,
    0.0009299200,
    0.0008868800,
    0.0008425600,
    0.0008000000,
    0.0007609600,
    0.0007236800,
    0.0006859200,
    0.0006454400,
    0.0006000000,
    0.0005478667,
    0.0004916000,
    0.0004354000,
    0.0003834667,
    0.0003400000,
    0.0003072533,
    0.0002831600,
    0.0002654400,
    0.0002518133,
    0.0002400000,
    0.0002295467,
    0.0002206400,
    0.0002119600,
    0.0002021867,
    0.0001900000,
    0.0001742133,
    0.0001556400,
    0.0001359600,
    0.0001168533,
    0.0001000000,
    0.00008613333,
    0.00007460000,
    0.00006500000,
    0.00005693333,
    0.00004999999,
    0.00004416000,
    0.00003948000,
    0.00003572000,
    0.00003264000,
    0.00003000000,
    0.00002765333,
    0.00002556000,
    0.00002364000,
    0.00002181333,
    0.00002000000,
    0.00001813333,
    0.00001620000,
    0.00001420000,
    0.00001213333,
    0.00001000000,
    0.000007733333,
    0.000005400000,
    0.000003200000,
    0.000001333333,
    0.000000000000,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0];

pub const CIE_lambda: [Float; N_CIE_SAMPLES] = [
    360.0, 361.0, 362.0, 363.0, 364.0, 365.0, 366.0, 367.0, 368.0, 369.0, 370.0, 371.0, 372.0, 373.0, 374.0,
    375.0, 376.0, 377.0, 378.0, 379.0, 380.0, 381.0, 382.0, 383.0, 384.0, 385.0, 386.0, 387.0, 388.0, 389.0,
    390.0, 391.0, 392.0, 393.0, 394.0, 395.0, 396.0, 397.0, 398.0, 399.0, 400.0, 401.0, 402.0, 403.0, 404.0,
    405.0, 406.0, 407.0, 408.0, 409.0, 410.0, 411.0, 412.0, 413.0, 414.0, 415.0, 416.0, 417.0, 418.0, 419.0,
    420.0, 421.0, 422.0, 423.0, 424.0, 425.0, 426.0, 427.0, 428.0, 429.0, 430.0, 431.0, 432.0, 433.0, 434.0,
    435.0, 436.0, 437.0, 438.0, 439.0, 440.0, 441.0, 442.0, 443.0, 444.0, 445.0, 446.0, 447.0, 448.0, 449.0,
    450.0, 451.0, 452.0, 453.0, 454.0, 455.0, 456.0, 457.0, 458.0, 459.0, 460.0, 461.0, 462.0, 463.0, 464.0,
    465.0, 466.0, 467.0, 468.0, 469.0, 470.0, 471.0, 472.0, 473.0, 474.0, 475.0, 476.0, 477.0, 478.0, 479.0,
    480.0, 481.0, 482.0, 483.0, 484.0, 485.0, 486.0, 487.0, 488.0, 489.0, 490.0, 491.0, 492.0, 493.0, 494.0,
    495.0, 496.0, 497.0, 498.0, 499.0, 500.0, 501.0, 502.0, 503.0, 504.0, 505.0, 506.0, 507.0, 508.0, 509.0,
    510.0, 511.0, 512.0, 513.0, 514.0, 515.0, 516.0, 517.0, 518.0, 519.0, 520.0, 521.0, 522.0, 523.0, 524.0,
    525.0, 526.0, 527.0, 528.0, 529.0, 530.0, 531.0, 532.0, 533.0, 534.0, 535.0, 536.0, 537.0, 538.0, 539.0,
    540.0, 541.0, 542.0, 543.0, 544.0, 545.0, 546.0, 547.0, 548.0, 549.0, 550.0, 551.0, 552.0, 553.0, 554.0,
    555.0, 556.0, 557.0, 558.0, 559.0, 560.0, 561.0, 562.0, 563.0, 564.0, 565.0, 566.0, 567.0, 568.0, 569.0,
    570.0, 571.0, 572.0, 573.0, 574.0, 575.0, 576.0, 577.0, 578.0, 579.0, 580.0, 581.0, 582.0, 583.0, 584.0,
    585.0, 586.0, 587.0, 588.0, 589.0, 590.0, 591.0, 592.0, 593.0, 594.0, 595.0, 596.0, 597.0, 598.0, 599.0,
    600.0, 601.0, 602.0, 603.0, 604.0, 605.0, 606.0, 607.0, 608.0, 609.0, 610.0, 611.0, 612.0, 613.0, 614.0,
    615.0, 616.0, 617.0, 618.0, 619.0, 620.0, 621.0, 622.0, 623.0, 624.0, 625.0, 626.0, 627.0, 628.0, 629.0,
    630.0, 631.0, 632.0, 633.0, 634.0, 635.0, 636.0, 637.0, 638.0, 639.0, 640.0, 641.0, 642.0, 643.0, 644.0,
    645.0, 646.0, 647.0, 648.0, 649.0, 650.0, 651.0, 652.0, 653.0, 654.0, 655.0, 656.0, 657.0, 658.0, 659.0,
    660.0, 661.0, 662.0, 663.0, 664.0, 665.0, 666.0, 667.0, 668.0, 669.0, 670.0, 671.0, 672.0, 673.0, 674.0,
    675.0, 676.0, 677.0, 678.0, 679.0, 680.0, 681.0, 682.0, 683.0, 684.0, 685.0, 686.0, 687.0, 688.0, 689.0,
    690.0, 691.0, 692.0, 693.0, 694.0, 695.0, 696.0, 697.0, 698.0, 699.0, 700.0, 701.0, 702.0, 703.0, 704.0,
    705.0, 706.0, 707.0, 708.0, 709.0, 710.0, 711.0, 712.0, 713.0, 714.0, 715.0, 716.0, 717.0, 718.0, 719.0,
    720.0, 721.0, 722.0, 723.0, 724.0, 725.0, 726.0, 727.0, 728.0, 729.0, 730.0, 731.0, 732.0, 733.0, 734.0,
    735.0, 736.0, 737.0, 738.0, 739.0, 740.0, 741.0, 742.0, 743.0, 744.0, 745.0, 746.0, 747.0, 748.0, 749.0,
    750.0, 751.0, 752.0, 753.0, 754.0, 755.0, 756.0, 757.0, 758.0, 759.0, 760.0, 761.0, 762.0, 763.0, 764.0,
    765.0, 766.0, 767.0, 768.0, 769.0, 770.0, 771.0, 772.0, 773.0, 774.0, 775.0, 776.0, 777.0, 778.0, 779.0,
    780.0, 781.0, 782.0, 783.0, 784.0, 785.0, 786.0, 787.0, 788.0, 789.0, 790.0, 791.0, 792.0, 793.0, 794.0,
    795.0, 796.0, 797.0, 798.0, 799.0, 800.0, 801.0, 802.0, 803.0, 804.0, 805.0, 806.0, 807.0, 808.0, 809.0,
    810.0, 811.0, 812.0, 813.0, 814.0, 815.0, 816.0, 817.0, 818.0, 819.0, 820.0, 821.0, 822.0, 823.0, 824.0,
    825.0, 826.0, 827.0, 828.0, 829.0, 830.0];

pub const RGB2SpectLambda: [Float; N_RGB2_SPECT_SAMPLES] = [
    380.000000, 390.967743, 401.935486, 412.903229, 423.870972, 434.838715,
    445.806458, 456.774200, 467.741943, 478.709686, 489.677429, 500.645172,
    511.612915, 522.580627, 533.548340, 544.516052, 555.483765, 566.451477,
    577.419189, 588.386902, 599.354614, 610.322327, 621.290039, 632.257751,
    643.225464, 654.193176, 665.160889, 676.128601, 687.096313, 698.064026,
    709.031738, 720.000000];

pub const RGBRefl2SpectWhite: [Float; N_RGB2_SPECT_SAMPLES] = [
    1.0618958571272863e+00, 1.0615019980348779e+00, 1.0614335379927147e+00,
    1.0622711654692485e+00, 1.0622036218416742e+00, 1.0625059965187085e+00,
    1.0623938486985884e+00, 1.0624706448043137e+00, 1.0625048144827762e+00,
    1.0624366131308856e+00, 1.0620694238892607e+00, 1.0613167586932164e+00,
    1.0610334029377020e+00, 1.0613868564828413e+00, 1.0614215366116762e+00,
    1.0620336151299086e+00, 1.0625497454805051e+00, 1.0624317487992085e+00,
    1.0625249140554480e+00, 1.0624277664486914e+00, 1.0624749854090769e+00,
    1.0625538581025402e+00, 1.0625326910104864e+00, 1.0623922312225325e+00,
    1.0623650980354129e+00, 1.0625256476715284e+00, 1.0612277619533155e+00,
    1.0594262608698046e+00, 1.0599810758292072e+00, 1.0602547314449409e+00,
    1.0601263046243634e+00, 1.0606565756823634e+00];

pub const RGBRefl2SpectCyan: [Float; N_RGB2_SPECT_SAMPLES] = [
    1.0414628021426751e+00,  1.0328661533771188e+00,  1.0126146228964314e+00,
    1.0350460524836209e+00,  1.0078661447098567e+00,  1.0422280385081280e+00,
    1.0442596738499825e+00,  1.0535238290294409e+00,  1.0180776226938120e+00,
    1.0442729908727713e+00,  1.0529362541920750e+00,  1.0537034271160244e+00,
    1.0533901869215969e+00,  1.0537782700979574e+00,  1.0527093770467102e+00,
    1.0530449040446797e+00,  1.0550554640191208e+00,  1.0553673610724821e+00,
    1.0454306634683976e+00,  6.2348950639230805e-01,  1.8038071613188977e-01,
    -7.6303759201984539e-03, -1.5217847035781367e-04, -7.5102257347258311e-03,
    -2.1708639328491472e-03, 6.5919466602369636e-04,  1.2278815318539780e-02,
    -4.4669775637208031e-03, 1.7119799082865147e-02,  4.9211089759759801e-03,
    5.8762925143334985e-03,  2.5259399415550079e-02];

pub const RGBRefl2SpectMagenta: [Float; N_RGB2_SPECT_SAMPLES] = [
    9.9422138151236850e-01,  9.8986937122975682e-01, 9.8293658286116958e-01,
    9.9627868399859310e-01,  1.0198955019000133e+00, 1.0166395501210359e+00,
    1.0220913178757398e+00,  9.9651666040682441e-01, 1.0097766178917882e+00,
    1.0215422470827016e+00,  6.4031953387790963e-01, 2.5012379477078184e-03,
    6.5339939555769944e-03,  2.8334080462675826e-03, -5.1209675389074505e-11,
    -9.0592291646646381e-03, 3.3936718323331200e-03, -3.0638741121828406e-03,
    2.2203936168286292e-01,  6.3141140024811970e-01, 9.7480985576500956e-01,
    9.7209562333590571e-01,  1.0173770302868150e+00, 9.9875194322734129e-01,
    9.4701725739602238e-01,  8.5258623154354796e-01, 9.4897798581660842e-01,
    9.4751876096521492e-01,  9.9598944191059791e-01, 8.6301351503809076e-01,
    8.9150987853523145e-01,  8.4866492652845082e-01];

pub const RGBRefl2SpectYellow: [Float; N_RGB2_SPECT_SAMPLES] = [
    5.5740622924920873e-03,  -4.7982831631446787e-03, -5.2536564298613798e-03,
    -6.4571480044499710e-03, -5.9693514658007013e-03, -2.1836716037686721e-03,
    1.6781120601055327e-02,  9.6096355429062641e-02,  2.1217357081986446e-01,
    3.6169133290685068e-01,  5.3961011543232529e-01,  7.4408810492171507e-01,
    9.2209571148394054e-01,  1.0460304298411225e+00,  1.0513824989063714e+00,
    1.0511991822135085e+00,  1.0510530911991052e+00,  1.0517397230360510e+00,
    1.0516043086790485e+00,  1.0511944032061460e+00,  1.0511590325868068e+00,
    1.0516612465483031e+00,  1.0514038526836869e+00,  1.0515941029228475e+00,
    1.0511460436960840e+00,  1.0515123758830476e+00,  1.0508871369510702e+00,
    1.0508923708102380e+00,  1.0477492815668303e+00,  1.0493272144017338e+00,
    1.0435963333422726e+00,  1.0392280772051465e+00];

pub const RGBRefl2SpectRed: [Float; N_RGB2_SPECT_SAMPLES] = [
    1.6575604867086180e-01,  1.1846442802747797e-01,  1.2408293329637447e-01,
    1.1371272058349924e-01,  7.8992434518899132e-02,  3.2205603593106549e-02,
    -1.0798365407877875e-02, 1.8051975516730392e-02,  5.3407196598730527e-03,
    1.3654918729501336e-02,  -5.9564213545642841e-03, -1.8444365067353252e-03,
    -1.0571884361529504e-02, -2.9375521078000011e-03, -1.0790476271835936e-02,
    -8.0224306697503633e-03, -2.2669167702495940e-03, 7.0200240494706634e-03,
    -8.1528469000299308e-03, 6.0772866969252792e-01,  9.8831560865432400e-01,
    9.9391691044078823e-01,  1.0039338994753197e+00,  9.9234499861167125e-01,
    9.9926530858855522e-01,  1.0084621557617270e+00,  9.8358296827441216e-01,
    1.0085023660099048e+00,  9.7451138326568698e-01,  9.8543269570059944e-01,
    9.3495763980962043e-01,  9.8713907792319400e-01];

pub const RGBRefl2SpectGreen: [Float; N_RGB2_SPECT_SAMPLES] = [
    2.6494153587602255e-03,  -5.0175013429732242e-03, -1.2547236272489583e-02,
    -9.4554964308388671e-03, -1.2526086181600525e-02, -7.9170697760437767e-03,
    -7.9955735204175690e-03, -9.3559433444469070e-03, 6.5468611982999303e-02,
    3.9572875517634137e-01,  7.5244022299886659e-01,  9.6376478690218559e-01,
    9.9854433855162328e-01,  9.9992977025287921e-01,  9.9939086751140449e-01,
    9.9994372267071396e-01,  9.9939121813418674e-01,  9.9911237310424483e-01,
    9.6019584878271580e-01,  6.3186279338432438e-01,  2.5797401028763473e-01,
    9.4014888527335638e-03,  -3.0798345608649747e-03, -4.5230367033685034e-03,
    -6.8933410388274038e-03, -9.0352195539015398e-03, -8.5913667165340209e-03,
    -8.3690869120289398e-03, -7.8685832338754313e-03, -8.3657578711085132e-06,
    5.4301225442817177e-03,  -2.7745589759259194e-03];

pub const RGBRefl2SpectBlue: [Float; N_RGB2_SPECT_SAMPLES] = [
    9.9209771469720676e-01,  9.8876426059369127e-01,  9.9539040744505636e-01,
    9.9529317353008218e-01,  9.9181447411633950e-01,  1.0002584039673432e+00,
    9.9968478437342512e-01,  9.9988120766657174e-01,  9.8504012146370434e-01,
    7.9029849053031276e-01,  5.6082198617463974e-01,  3.3133458513996528e-01,
    1.3692410840839175e-01,  1.8914906559664151e-02,  -5.1129770932550889e-06,
    -4.2395493167891873e-04, -4.1934593101534273e-04, 1.7473028136486615e-03,
    3.7999160177631316e-03,  -5.5101474906588642e-04, -4.3716662898480967e-05,
    7.5874501748732798e-03,  2.5795650780554021e-02,  3.8168376532500548e-02,
    4.9489586408030833e-02,  4.9595992290102905e-02,  4.9814819505812249e-02,
    3.9840911064978023e-02,  3.0501024937233868e-02,  2.1243054765241080e-02,
    6.9596532104356399e-03,  4.1733649330980525e-03];
pub const RGBIllum2SpectWhite: [Float; N_RGB2_SPECT_SAMPLES] = [
    1.1565232050369776e+00, 1.1567225000119139e+00, 1.1566203150243823e+00,
    1.1555782088080084e+00, 1.1562175509215700e+00, 1.1567674012207332e+00,
    1.1568023194808630e+00, 1.1567677445485520e+00, 1.1563563182952830e+00,
    1.1567054702510189e+00, 1.1565134139372772e+00, 1.1564336176499312e+00,
    1.1568023181530034e+00, 1.1473147688514642e+00, 1.1339317140561065e+00,
    1.1293876490671435e+00, 1.1290515328639648e+00, 1.0504864823782283e+00,
    1.0459696042230884e+00, 9.9366687168595691e-01, 9.5601669265393940e-01,
    9.2467482033511805e-01, 9.1499944702051761e-01, 8.9939467658453465e-01,
    8.9542520751331112e-01, 8.8870566693814745e-01, 8.8222843814228114e-01,
    8.7998311373826676e-01, 8.7635244612244578e-01, 8.8000368331709111e-01,
    8.8065665428441120e-01, 8.8304706460276905e-01];

pub const RGBIllum2SpectCyan: [Float; N_RGB2_SPECT_SAMPLES] = [
    1.1334479663682135e+00,  1.1266762330194116e+00,  1.1346827504710164e+00,
    1.1357395805744794e+00,  1.1356371830149636e+00,  1.1361152989346193e+00,
    1.1362179057706772e+00,  1.1364819652587022e+00,  1.1355107110714324e+00,
    1.1364060941199556e+00,  1.1360363621722465e+00,  1.1360122641141395e+00,
    1.1354266882467030e+00,  1.1363099407179136e+00,  1.1355450412632506e+00,
    1.1353732327376378e+00,  1.1349496420726002e+00,  1.1111113947168556e+00,
    9.0598740429727143e-01,  6.1160780787465330e-01,  2.9539752170999634e-01,
    9.5954200671150097e-02,  -1.1650792030826267e-02, -1.2144633073395025e-02,
    -1.1148167569748318e-02, -1.1997606668458151e-02, -5.0506855475394852e-03,
    -7.9982745819542154e-03, -9.4722817708236418e-03, -5.5329541006658815e-03,
    -4.5428914028274488e-03, -1.2541015360921132e-02];

pub const RGBIllum2SpectMagenta: [Float; N_RGB2_SPECT_SAMPLES] = [
    1.0371892935878366e+00,  1.0587542891035364e+00,  1.0767271213688903e+00,
    1.0762706844110288e+00,  1.0795289105258212e+00,  1.0743644742950074e+00,
    1.0727028691194342e+00,  1.0732447452056488e+00,  1.0823760816041414e+00,
    1.0840545681409282e+00,  9.5607567526306658e-01,  5.5197896855064665e-01,
    8.4191094887247575e-02,  8.7940070557041006e-05,  -2.3086408335071251e-03,
    -1.1248136628651192e-03, -7.7297612754989586e-11, -2.7270769006770834e-04,
    1.4466473094035592e-02,  2.5883116027169478e-01,  5.2907999827566732e-01,
    9.0966624097105164e-01,  1.0690571327307956e+00,  1.0887326064796272e+00,
    1.0637622289511852e+00,  1.0201812918094260e+00,  1.0262196688979945e+00,
    1.0783085560613190e+00,  9.8333849623218872e-01,  1.0707246342802621e+00,
    1.0634247770423768e+00,  1.0150875475729566e+00];

pub const RGBIllum2SpectYellow: [Float; N_RGB2_SPECT_SAMPLES] = [
    2.7756958965811972e-03,  3.9673820990646612e-03,  -1.4606936788606750e-04,
    3.6198394557748065e-04,  -2.5819258699309733e-04, -5.0133191628082274e-05,
    -2.4437242866157116e-04, -7.8061419948038946e-05, 4.9690301207540921e-02,
    4.8515973574763166e-01,  1.0295725854360589e+00,  1.0333210878457741e+00,
    1.0368102644026933e+00,  1.0364884018886333e+00,  1.0365427939411784e+00,
    1.0368595402854539e+00,  1.0365645405660555e+00,  1.0363938240707142e+00,
    1.0367205578770746e+00,  1.0365239329446050e+00,  1.0361531226427443e+00,
    1.0348785007827348e+00,  1.0042729660717318e+00,  8.4218486432354278e-01,
    7.3759394894801567e-01,  6.5853154500294642e-01,  6.0531682444066282e-01,
    5.9549794132420741e-01,  5.9419261278443136e-01,  5.6517682326634266e-01,
    5.6061186014968556e-01,  5.8228610381018719e-01];

pub const RGBIllum2SpectRed: [Float; N_RGB2_SPECT_SAMPLES] = [
    5.4711187157291841e-02,  5.5609066498303397e-02,  6.0755873790918236e-02,
    5.6232948615962369e-02,  4.6169940535708678e-02,  3.8012808167818095e-02,
    2.4424225756670338e-02,  3.8983580581592181e-03,  -5.6082252172734437e-04,
    9.6493871255194652e-04,  3.7341198051510371e-04,  -4.3367389093135200e-04,
    -9.3533962256892034e-05, -1.2354967412842033e-04, -1.4524548081687461e-04,
    -2.0047691915543731e-04, -4.9938587694693670e-04, 2.7255083540032476e-02,
    1.6067405906297061e-01,  3.5069788873150953e-01,  5.7357465538418961e-01,
    7.6392091890718949e-01,  8.9144466740381523e-01,  9.6394609909574891e-01,
    9.8879464276016282e-01,  9.9897449966227203e-01,  9.8605140403564162e-01,
    9.9532502805345202e-01,  9.7433478377305371e-01,  9.9134364616871407e-01,
    9.8866287772174755e-01,  9.9713856089735531e-01];

pub const RGBIllum2SpectGreen: [Float; N_RGB2_SPECT_SAMPLES] = [
    2.5168388755514630e-02,  3.9427438169423720e-02,  6.2059571596425793e-03,
    7.1120859807429554e-03,  2.1760044649139429e-04,  7.3271839984290210e-12,
    -2.1623066217181700e-02, 1.5670209409407512e-02,  2.8019603188636222e-03,
    3.2494773799897647e-01,  1.0164917292316602e+00,  1.0329476657890369e+00,
    1.0321586962991549e+00,  1.0358667411948619e+00,  1.0151235476834941e+00,
    1.0338076690093119e+00,  1.0371372378155013e+00,  1.0361377027692558e+00,
    1.0229822432557210e+00,  9.6910327335652324e-01,  -5.1785923899878572e-03,
    1.1131261971061429e-03,  6.6675503033011771e-03,  7.4024315686001957e-04,
    2.1591567633473925e-02,  5.1481620056217231e-03,  1.4561928645728216e-03,
    1.6414511045291513e-04,  -6.4630764968453287e-03, 1.0250854718507939e-02,
    4.2387394733956134e-02,  2.1252716926861620e-02];

pub const RGBIllum2SpectBlue: [Float; N_RGB2_SPECT_SAMPLES] = [
    1.0570490759328752e+00,  1.0538466912851301e+00,  1.0550494258140670e+00,
    1.0530407754701832e+00,  1.0579930596460185e+00,  1.0578439494812371e+00,
    1.0583132387180239e+00,  1.0579712943137616e+00,  1.0561884233578465e+00,
    1.0571399285426490e+00,  1.0425795187752152e+00,  3.2603084374056102e-01,
    -1.9255628442412243e-03, -1.2959221137046478e-03, -1.4357356276938696e-03,
    -1.2963697250337886e-03, -1.9227081162373899e-03, 1.2621152526221778e-03,
    -1.6095249003578276e-03, -1.3029983817879568e-03, -1.7666600873954916e-03,
    -1.2325281140280050e-03, 1.0316809673254932e-02,  3.1284512648354357e-02,
    8.8773879881746481e-02,  1.3873621740236541e-01,  1.5535067531939065e-01,
    1.4878477178237029e-01,  1.6624255403475907e-01,  1.6997613960634927e-01,
    1.5769743995852967e-01,  1.9069090525482305e-01];