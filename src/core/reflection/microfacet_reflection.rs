use super::{BxDF, Fresnel, BxDFType, abs_cos_theta, same_hemisphere};
use crate::core::pbrt::{Float, Spectrum};
use crate::core::geometry::{Vector3f, Point2f, Normal3f};
use crate::core::microfacet::MicrofacetDistribution;
use crate::core::reflection::reflect;
use std::fmt;

#[derive(Copy, Clone)]
pub struct MicrofacetReflection {
    r: Spectrum,
    distribution: MicrofacetDistribution,
    fresnel: Box<dyn Fresnel>
}

impl MicrofacetReflection {
    pub fn new(
        r: Spectrum,
        distribution: MicrofacetDistribution,
        fresnel: Box<dyn Fresnel>
    ) -> MicrofacetReflection {
        MicrofacetReflection { r, distribution, fresnel }
    }
}

impl BxDF for MicrofacetReflection {
    fn get_type(&self) -> u8 {
        BxDFType::BSDF_REFLECTION | BxDFType::BSDF_GLOSSY
    }

    fn f(
        &self,
        wo: &Vector3f,
        wi: &Vector3f
    ) -> Spectrum {
        let cos_theta_o = abs_cos_theta(wo);
        let cos_theta_i = abs_cos_theta(wi);
        let mut wh = *wi + *wo;
        // Handle degenerate cases for microfacet reflection
        if cos_theta_i == 0.0 || cos_theta_o == 0.0 {
            return Spectrum::new(0.0);
        }
        if wh.x == 0.0 && wh.y == 0.0 && wh.z == 0.0 {
            return Spectrum::new(0.0);
        }
        wh = wh.normalize();
        // For the Fresnel call, make sure that wh is in the same hemisphere
        // as the surface normal, so that TIR is handled correctly.
        let forward_wh = Vector3f::from(Normal3f::from(wh).face_forward(&Normal3f::new(0.0, 0.0, 1.0)));
        let f = self.fresnel.evaluate(wi.dot(&forward_wh));
        self.r * self.distribution.d(&wh) * self.distribution.g(wo, wi) * f /
            (4.0 * cos_theta_i * cos_theta_o)
        
    }

    fn sample_f(
        &self,
        wo: &Vector3f,
        wi: &mut Vector3f,
        sample: &Point2f,
        pdf: &mut Float,
        sampled_type: &mut u8
    ) -> Spectrum {
        // Sample microfacet orientation $\wh$ and reflected direction $\wi$
        if wo.z == 0.0 {
            return Spectrum::new(0.0);
        }
        let wh = self.distribution.sample_wh(wo, sample);
        if wo.dot(&wh) < 0.0 {  // should be rare
            return Spectrum::new(0.0);
        }
        *wi = reflect(wo, &wh);
        if !same_hemisphere(wo, &wh) {
            return Spectrum::new(0.0);
        }

        // Compute PDF of _wi_ for microfacet reflection
        *pdf = self.distribution.pdf(wo, &wh) / (4.0 * wo.dot(&wh));
        self.f(wo, &wi)
    }

    fn pdf(&self, wo: &Vector3f, wi: &Vector3f) -> Float {
        if same_hemisphere(wo, wi) {
            return 0.0;
        }
        let wh = (*wo + *wi).normalize();
        self.distribution.pdf(wo, &wh) / (4.0 * wo.dot(&wh))
    }
}

impl fmt::Display for MicrofacetReflection {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "[ MicrofacetReflection r: {} distribution: {} fresnel: {} ]", self.r, self.distribution, self.fresnel)
    }
}