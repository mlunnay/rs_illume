use crate::core::pbrt::{Float, erf, erf_inv, consts::PI};
use crate::core::geometry::{Vector3f, Point2f, spherical_direction};
use super::MicrofacetDistribution;
use crate::core::reflection::*;
use std::fmt;

pub struct BeckmannDistribution {
    alphax: Float,
    alphay: Float,

    // MicrofacetDistribution attributes
    sample_visible_area: bool
}

impl BeckmannDistribution {
    pub fn new(alphax: Float, alphay: Float, samplevis: bool) -> BeckmannDistribution {
        BeckmannDistribution {
            alphax,
            alphay,
            sample_visible_area: samplevis
        }
    }

    pub fn roughness_to_alpha(mut roughness: Float) -> Float {
        roughness = roughness.max(1e-3);
        let x = roughness.ln();
        1.62142 + 0.819955 * x + 0.1734 * x * x +
            0.0171201 * x * x * x + 0.000640711 * x * x * x * x
    }
}

impl MicrofacetDistribution for BeckmannDistribution {
    fn get_sample_visible_area(&self) -> bool {
        self.sample_visible_area
    }

    fn d(&self, wh: &Vector3f) -> Float {
        let tan2_theta = tan2_theta(wh);
        if tan2_theta.is_infinite() {
            return 0.0;
        }
        let cos4_theta = cos2_theta(wh) * cos2_theta(wh);
        (-tan2_theta * (cos2_phi(wh) / (self.alphax * self.alphax) +
            sin2_phi(wh) / (self.alphay * self.alphay))).exp() /
            (PI * self.alphax * self.alphay * cos4_theta)
    }

    fn lambda(&self, w: &Vector3f) -> Float {
        let abs_tan_theta = tan2_theta(w).abs();
        if abs_tan_theta.is_infinite() {
            return 0.0;
        }
        // Compute _alpha_ for direction _w_
        let alpha = (cos2_phi(w) * self.alphax * self.alphax + sin2_phi(w) * self.alphay * self.alphay).sqrt();
        let a = 1.0 / (alpha * abs_tan_theta);
        if a >= 1.6 {
            return 0.0;
        }
        (1.0 - 1.259 * a + 0.396 * a * a) / (3.535 * a + 2.181 * a * a)
    }

    fn sample_wh(&self, wo: &Vector3f, u: &Point2f) -> Vector3f {
        if self.sample_visible_area {
            // Sample full distribution of normals for Beckmann distribution

            // Compute $\tan^2 \theta$ and $\phi$ for Beckmann distribution sample
            let tan2_theta: Float = 0.0;
            let phi: Float = 0.0;
            if self.alphax == self.alphay {
                let log_sample = (1.0 - u[0]).ln();
                assert!(!log_sample.is_infinite());
                tan2_theta = -self.alphax * self.alphax * log_sample;
                phi = u[1] * 2.0 * PI;
            } else {
                // Compute _tan2Theta_ and _phi_ for anisotropic Beckmann
                // distribution
                let log_sample = (1.0 - u[0]).ln();
                assert!(!log_sample.is_infinite());
                phi = (self.alphay / self.alphax * (2.0 * PI * u[1] + 0.5 * PI).tan()).atan();
                if u[1] > 0.5 {
                    phi += PI;
                }
                let sin_phi = phi.sin();
                let cos_phi = phi.cos();
                let alphax2 = self.alphax * self.alphax;
                let alphay2 = self.alphay * self.alphay;
                tan2_theta = -log_sample /
                    (cos_phi * cos_phi / alphax2 + sin_phi * sin_phi / alphay2);
            }

            // Map sampled Beckmann angles to normal direction _wh_
            let cos_theta = 1.0 / (1.0 + tan2_theta).sqrt();
            let sin_theta = (1.0 - cos_theta * cos_theta).max(0.0).sqrt();
            let wh = spherical_direction(sin_theta, cos_theta, phi);
            return if !same_hemisphere(wo, &wh) {
                -wh
            } else {
                wh
            }
        } else {
            // Sample visible area of normals for Beckmann distribution
            let flip = wo.z < 0.0;
            let wh = beckmann_sample(if flip { &-*wo } else { wo }, self.alphax, self.alphay, u[0], u[1]);
            return if flip {
                -wh
            } else {
                wh
            }
        }
    }
}

impl fmt::Display for BeckmannDistribution {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "[ BeckmannDistribution alpha_x: {} alpha_y: {} ]", self.alphax, self.alphay)
    }
}

fn beckmann_sample11(
    cos_theta_i: Float,
    u1: Float,
    u2: Float,
    slope_x: &mut Float,
    slope_y: &mut Float
) {
    /* Special case (normal incidence) */
    if cos_theta_i > 0.9999 {
        let r = (-(1.0 - u1).ln()).sqrt();
        let sin_phi = (2.0 * PI * u2).sin();
        let cos_phi = (2.0 * PI * u2).cos();
        *slope_x = r * cos_phi;
        *slope_y = r * sin_phi;
        return;
    }

    /* The original inversion routine from the paper contained
       discontinuities, which causes issues for QMC integration
       and techniques like Kelemen-style MLT. The following code
       performs a numerical inversion with better behavior */
    let sin_theta_i = (1.0 - cos_theta_i * cos_theta_i).max(0.0).sqrt();
    let tan_theta_i = sin_theta_i / cos_theta_i;
    let cot_theta_i = 1.0 / tan_theta_i;

    /* Search interval -- everything is parameterized
       in the Erf() domain */
    let mut a: Float = -1.0;
    let mut c = erf(cot_theta_i);
    let sample_x = u1.max(1e-6);

    /* Start with a good initial guess */
    // Float b = (1-sample_x) * a + sample_x * c;

    /* We can do better (inverse of an approximation computed in
     * Mathematica) */
    let theta_i = cot_theta_i.acos();
    let fit = 1.0 + theta_i * (-0.876 + theta_i * (0.4265 - 0.0594 * theta_i));
    let mut b = c - (1.0 + c) * (1.0 - sample_x).powf(fit);

    /* Normalization factor for the CDF */
    static sqrt_pi_inv: Float = 1.0 / PI.sqrt();
    let normalization = 1.0 / (1.0 + c + sqrt_pi_inv * tan_theta_i * (-cot_theta_i * cot_theta_i).exp());

    for _ in 0..10 {
        /* Bisection criterion -- the oddly-looking
           Boolean expression are intentional to check
           for NaNs at little additional cost */
        if !(b >= a && b <= c) {
            b = 0.5 * (a + c);
        }

        /* Evaluate the CDF and its derivative
           (i.e. the density function) */
        let inv_erf = erf_inv(b);
        let value = normalization *
            (1.0 + b + sqrt_pi_inv * tan_theta_i * (-inv_erf * inv_erf).exp()) -
            sample_x;
        let derivative = normalization * (1.0 - inv_erf * tan_theta_i);

        if value.abs() < 1e-5 {
            break;
        }

        /* Update bisection intervals */
        if value > 0.0 {
            c = b;
        } else {
            a = b;
        }

        b -= value / derivative;
    }

    /* Now convert back into a slope value */
    *slope_x = erf_inv(b);

    /* Simulate Y component */
    *slope_y = erf_inv(2.0 * u2.max(1e-6) - 1.0);

    assert!(slope_x.is_finite());
    assert!(slope_y.is_finite());
}

fn beckmann_sample(wi: &Vector3f, alpha_x: Float, alpha_y: Float, u1: Float, u2: Float) -> Vector3f {
    // 1. stretch wi
    let wi_stretched =
        Vector3f::new(alpha_x * wi.x, alpha_y * wi.y, wi.z).normalize();

    // 2. simulate P22_{wi}(x_slope, y_slope, 1, 1)
    let mut slope_x: Float = 0.0;
    let mut slope_y: Float = 0.0;
    beckmann_sample11(cos_theta(&wi_stretched), u1, u2, &mut slope_x, &mut slope_y);

    // 3. rotate
    let tmp = cos_phi(&wi_stretched) * slope_x - sin_phi(&wi_stretched) * slope_y;
    slope_y = sin_phi(&wi_stretched) * slope_x + cos_phi(&wi_stretched) * slope_y;
    slope_x = tmp;

    // 4. unstretch
    slope_x = alpha_x * slope_x;
    slope_y = alpha_y * slope_y;

    // 5. compute normal
    Vector3f::new(-slope_x, -slope_y, 1.0).normalize()
}