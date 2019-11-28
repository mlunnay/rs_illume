use super::{BxDF, Fresnel, FresnelDielectric, BxDFType, same_hemisphere, cos_theta};
use crate::core::pbrt::{Float, Spectrum};
use crate::core::geometry::{Vector3f, Point2f, Normal3f};
use crate::core::microfacet::MicrofacetDistribution;
use crate::core::reflection::refract;
use crate::core::material::TransportMode;
use std::fmt;

pub struct MicrofacetTransmission {
    t: Spectrum,
    distribution: Box<dyn MicrofacetDistribution>,
    eta_a: Float,
    eta_b: Float,
    fresnel: FresnelDielectric,
    mode: TransportMode
}

impl MicrofacetTransmission {
    pub fn new(
        t: Spectrum,
        distribution: Box<dyn MicrofacetDistribution>,
        eta_a: Float,
        eta_b: Float,
        mode: TransportMode
    ) -> MicrofacetTransmission {
        MicrofacetTransmission {
            t,
            distribution,
            eta_a,
            eta_b,
            fresnel: FresnelDielectric::new(eta_a, eta_b),
            mode
        }
    }
}

impl BxDF for MicrofacetTransmission {
    fn get_type(&self) -> u8 {
        BxDFType::BSDF_TRANSMISSION | BxDFType::BSDF_GLOSSY
    }

    fn f(
        &self,
        wo: &Vector3f,
        wi: &Vector3f
    ) -> Spectrum {
        if same_hemisphere(wo, wi) { // transmission only
            return Spectrum::new(0.0);
        }

        let cos_theta_o = cos_theta(wo);
        let cos_theta_i = cos_theta(wi);
        if cos_theta_i == 0.0 || cos_theta_o == 0.0 {
            return Spectrum::new(0.0);
        }

        // Compute $\wh$ from $\wo$ and $\wi$ for microfacet transmission
        let eta = if cos_theta(wo) > 0.0 { self.eta_b / self.eta_a } else { self.eta_a / self.eta_b };
        let mut wh = (*wo + *wi * eta).normalize();
        if wh.z < 0.0 {
            wh = -wh;
        }

        let f = self.fresnel.evaluate(wo.dot(&wh));
        let sqrt_denom = wo.dot(&wh) + eta * wi.dot(&wh);
        let factor = if self.mode == TransportMode::Radiance { 1.0 / eta } else { 1.0 };

        (Spectrum::new(1.0) - f) * self.t *
            (self.distribution.d(&wh) * self.distribution.g(wo, wi) * eta * eta *
            wi.dot(&wh).abs() * wo.dot(&wh).abs() * factor * factor /
            (cos_theta_i * cos_theta_o * sqrt_denom * sqrt_denom)).abs()
    }

    fn sample_f(
        &self,
        wo: &Vector3f,
        wi: &mut Vector3f,
        sample: &Point2f,
        pdf: &mut Float,
        sampled_type: &mut u8
    ) -> Spectrum {
        if wo.z == 0.0 {
            return Spectrum::new(0.0);
        }
        let wh = self.distribution.sample_wh(wo, sample);
        if wo.dot(&wh) < 0.0 { // should be rare
            return Spectrum::new(0.0);
        }

        let eta = if cos_theta(wo) > 0.0 { self.eta_a / self.eta_b } else { self.eta_b / self.eta_a };
        if !refract(wo, &Normal3f::from(wh), eta, wi) {
            return Spectrum::new(0.0);
        }
        *pdf = self.pdf(wo, wi);
        self.f(wo, wi)
    }

    fn pdf(&self, wo: &Vector3f, wi: &Vector3f) -> Float {
        if same_hemisphere(wo, wi) {
            return 0.0;
        }
        // Compute $\wh$ from $\wo$ and $\wi$ for microfacet transmission
        let eta = if cos_theta(wo) > 0.0 { self.eta_b / self.eta_a } else { self.eta_a / self.eta_b };
        let wh = (*wo + *wi * eta).normalize();

        // Compute change of variables _dwh\_dwi_ for microfacet transmission
        let sqrt_denom = wo.dot(&wh) + eta * wi.dot(&wh);
        let dwh_dwi = ((eta * eta * wo.dot(&wh)) / (sqrt_denom * sqrt_denom)).abs();
        self.distribution.pdf(wo, &wh) * dwh_dwi
    }
}

impl fmt::Display for MicrofacetTransmission {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "[ MicrofacetTransmission t: {} distribution: {} eta_a: {} eta_b: {} fresnel: {} mode: {} ]",
            self.t, self.distribution, self.eta_a, self.eta_b, self.fresnel,
            if self.mode == TransportMode::Radiance { "RADIANCE" } else { "IMPORTANCE" })
    }
}