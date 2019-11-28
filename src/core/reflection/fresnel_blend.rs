use super::{BxDF, same_hemisphere, BxDFType, abs_cos_theta, cosine_sample_hemisphere};
use crate::core::pbrt::{Float, Spectrum, consts::{PI, INV_PI}};
use crate::core::geometry::{Vector3f, Point2f};
use crate::core::microfacet::MicrofacetDistribution;
use crate::core::rng::ONE_MINUS_EPSILON;
use crate::core::reflection::reflect;
use std::fmt;

#[derive(Copy, Clone)]
pub struct FresnelBlend {
    rd: Spectrum,
    rs: Spectrum,
    distribution: MicrofacetDistribution
}

impl FresnelBlend {
    pub fn new(rd: Spectrum, rs: Spectrum, distribution: MicrofacetDistribution) -> FresnelBlend {
        FresnelBlend{ rd, rs, distribution }
    }

    pub fn schlick_fresnel(&self, cos_theta: Float) -> Spectrum {
        self.rs + pow5(1.0 - cos_theta) * (Spectrum::new(1.0) - self.rs)
    }
}

impl BxDF for FresnelBlend {
    fn get_type(&self) -> u8 {
        BxDFType::BSDF_REFLECTION | BxDFType::BSDF_GLOSSY
    }

    fn f(
        &self,
        wo: &Vector3f,
        wi: &Vector3f
    ) -> Spectrum {
        let diffuse = (28.0 / (23.0 * PI)) * self.rd * (Spectrum::new(1.0) - self.rs) *
            (1.0 - pow5(1.0 - 0.5 * abs_cos_theta(wi))) *
            (1.0 - pow5(1.0 - 0.5 * abs_cos_theta(wo)));
        let mut wh = *wi + *wo;
        if wh.z == 0.0 && wh.y == 0.0 && wh.z == 0.0 {
            return Spectrum::new(0.0);
        }
        wh = wh.normalize();
        let specular = self.distribution.d(&wh) /
            (4.0 * wi.dot(&wh).abs() * abs_cos_theta(wi).max(abs_cos_theta(wo))) *
            self.schlick_fresnel(wi.dot(&wh));
        diffuse + specular
    }

    fn sample_f(
        &self,
        wo: &Vector3f,
        wi: &mut Vector3f,
        sample: &Point2f,
        pdf: &mut Float,
        sampled_type: &mut u8
    ) -> Spectrum {
        let mut u = sample;
        if u[0] < 0.5 {
            u[0] = (2.0 * u[0]).min(ONE_MINUS_EPSILON);
            // Cosine-sample the hemisphere, flipping the direction if necessary
            *wi = cosine_sample_hemisphere(&u);
            if wo.z < 0.0 {
                wi.z *= -1.0;
            }
        } else {
            u[0] = (2.0 * (u[0] - 0.5)).min(ONE_MINUS_EPSILON);
            // Sample microfacet orientation $\wh$ and reflected direction $\wi$
            let wh = self.distribution.sample_wh(wo, u);
            *wi = reflect(wo, wh);
            if !same_hemisphere(wo, wi) {
                return Spectrum::new(0.0);
            }
        }
        *pdf = self.pdf(wo, wi);
        self.f(wo, wi)
    }

    fn pdf(&self, wo: &Vector3f, wi: &Vector3f) -> Float {
        if !same_hemisphere(wo, wi) {
            return 0.0;
        }
        let wh = (*wo + *wi).normalize();
        let pdf_wh = self.distribution.pdf(wo, &wh);
        0.5 * (abs_cos_theta(wi) * INV_PI + pdf_wh / (4.0 * wo.dot(&wh)))
    }
}

impl fmt::Display for FresnelBlend {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "[ FresnelBlend rd: {} rs: {} distribution: {} ]",
            self.rd, self.rs, self.distribution)
    }
}

#[inline]
fn pow5(v: Float) -> Float {
    v * v * v * v * v
}