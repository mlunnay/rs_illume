use crate::core::pbrt::{Float, consts::PI};
use crate::core::geometry::{Vector3f, Point2f, spherical_direction};
use super::MicrofacetDistribution;
use crate::core::reflection::*;
use std::fmt;

#[derive(Debug, Default, Clone, Copy)]
pub struct TrowbridgeReitzDistribution {
    alphax: Float,
    alphay: Float,

    // MicrofacetDistribution attributes
    sample_visible_area: bool
}

impl TrowbridgeReitzDistribution {
    pub fn new(alphax: Float, alphay: Float, samplevis: bool) -> TrowbridgeReitzDistribution {
        TrowbridgeReitzDistribution {
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

impl MicrofacetDistribution for TrowbridgeReitzDistribution {
    fn get_sample_visible_area(&self) -> bool {
        self.sample_visible_area
    }

    fn d(&self, wh: &Vector3f) -> Float {
        let tan2_theta = tan2_theta(wh);
        if tan2_theta.is_infinite() {
            return 0.0;
        }
        let cos4_theta = cos2_theta(wh) * cos2_theta(wh);
        let e = (cos2_phi(wh) / (self.alphax * self.alphax) + sin2_phi(wh) / (self.alphay * self.alphay)) *
            tan2_theta;
        1.0 / (PI * self.alphax * self.alphay * cos4_theta * (1.0 + e) * (1.0 + e))
    }

    fn lambda(&self, w: &Vector3f) -> Float {
        let abs_tan_theta = tan2_theta(w).abs();
        if abs_tan_theta.is_infinite() {
            return 0.0;
        }
        // Compute _alpha_ for direction _w_
        let alpha = (cos2_phi(w) * self.alphax * self.alphax + sin2_phi(w) * self.alphay * self.alphay).sqrt();
        let a = 1.0 / (alpha * abs_tan_theta);
        let alpha2_tan2_theta = (alpha * abs_tan_theta) * (alpha * abs_tan_theta);
        (-1.0 + (1.0 + alpha2_tan2_theta).sqrt()) / 2.0
    }

    fn sample_wh(&self, wo: &Vector3f, u: &Point2f) -> Vector3f {
        let mut wh = Vector3f::default();
        if self.sample_visible_area {
            let mut cos_theta: Float = 0.0;
            let mut phi = (2.0 * PI) * u[1];
            if self.alphax == self.alphay {
                let tan_theta2 = self.alphax * self.alphax * u[0] / (1.0 - u[0]);
                cos_theta = 1.0 / (1.0 + tan_theta2).sqrt();
            } else {
                phi = (self.alphay / self.alphax * (2.0 * PI * u[1] + 0.5 * PI).tan()).atan();
                if u[1] > 0.5 {
                    phi += PI;
                }
                let sin_phi = phi.sin();
                let cos_phi = phi.cos();
                let alphax2 = self.alphax * self.alphax;
                let alphay2 = self.alphay * self.alphay;
                let alpha2 = 1.0 / (cos_phi * cos_phi / alphax2 + sin_phi * sin_phi / alphay2);
                let tan_theta2 = alpha2 * u[0] / (1.0 - u[0]);
                cos_theta = 1.0 / (1.0 + tan_theta2).sqrt();
            }
            let sin_theta = (1.0 - cos_theta * cos_theta).max(0.0).sqrt();
            wh = spherical_direction(sin_theta, cos_theta, phi);
            if !same_hemisphere(wo, &wh) {
                wh = -wh
            }
        } else {
            let flip = wo.z < 0.0;
             wh = trowbridge_reitz_sample(if flip { &-*wo } else { wo }, self.alphax, self.alphay, u[0], u[1]);
            if flip {
                wh = -wh
            }
        }
        wh
    }
}

impl fmt::Display for TrowbridgeReitzDistribution {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "[ TrowbridgeReitzDistribution alpha_x: {} alpha_y: {} ]", self.alphax, self.alphay)
    }
}

fn trowbridge_reitz_sample11(
    cos_theta: Float,
    u1: Float,
    mut u2: Float,
    slope_x: &mut Float,
    slope_y: &mut Float
) {
    /* Special case (normal incidence) */
    if cos_theta > 0.9999 {
        let r = (u1 / (1.0 - u1)).sqrt();
        let phi = 6.28318530718 * u2;
        *slope_x = r * phi.cos();
        *slope_y = r * phi.sin();
        return;
    }

    let sin_theta = (1.0 - cos_theta * cos_theta).max(0.0).sqrt();
    let tan_theta = sin_theta / cos_theta;
    let a = 1.0 / tan_theta;
    let g1 = 2.0 / (1.0 + (1.0 + 1.0 / (a * a)).sqrt());

    // sample slope_x
    let a = 2.0 * u1 / g1 - 1.0;
    let tmp = 1.0 / (a * a - 1.0);
    if tmp > 1e10 {
        tmp = 1e10;
    }
    let b = tan_theta;
    let d = (b * b * tmp * tmp - (a * a - b * b) * tmp).max(0.0).sqrt();
    let slope_x_1 = b * tmp - d;
    let slope_x_2 = b * tmp + d;
    *slope_x = if a < 0.0 || slope_x_2 > 1.0 / tan_theta { slope_x_1 } else { slope_x_2 };

    // sample slope_y
    let mut s: Float = 0.0;
    if u2 > 0.5 {
        s = 1.0;
        u2 = 2.0 * (u2 - 0.5);
    } else {
        s = -1.0;
        u2 = 2.0 * (0.5 - u2);
    }
    let z = (u2 * (u2 * (u2 * 0.27385 - 0.73369) + 0.46341)) /
        (u2 * (u2 * (u2 * 0.093073 + 0.309420) - 1.000000) + 0.597999);
    *slope_y = s * z * (1.0 + *slope_x * *slope_x).sqrt();

    assert!(slope_y.is_finite());
}

fn trowbridge_reitz_sample(wi: &Vector3f, alpha_x: Float, alpha_y: Float, u1: Float, u2: Float) -> Vector3f {
    // 1. stretch wi
    let wi_stretched =
        Vector3f::new(alpha_x * wi.x, alpha_y * wi.y, wi.z).normalize();

    // 2. simulate P22_{wi}(x_slope, y_slope, 1, 1)
    let mut slope_x: Float = 0.0;
    let mut slope_y: Float = 0.0;
    trowbridge_reitz_sample11(cos_theta(&wi_stretched), u1, u2, &mut slope_x, &mut slope_y);

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