use crate::core::pbrt::{Float, Spectrum, lerp};
use super::{fr_conductor, fr_dielectric, fr_schlick_spectrum};
use std::fmt;

pub trait Fresnel: fmt::Display {
    fn evaluate(&self, cos_i: Float) -> Spectrum;
}

#[derive(Debug, Default, Copy, Clone)]
pub struct FresnelNoOp {}

impl Fresnel for FresnelNoOp {
    fn evaluate(&self, cos_i: Float) -> Spectrum {
        Spectrum::new(0.0)
    }
}

impl fmt::Display for FresnelNoOp {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "[ FresnelNoOp ]")
    }
}

#[derive(Default, Debug, Copy, Clone)]
pub struct FresnelConductor {
    eta_i: Spectrum,
    eta_t: Spectrum,
    k: Spectrum
}

impl FresnelConductor {
    pub fn new(eta_i: Spectrum, eta_t: Spectrum, k: Spectrum) -> Self {
        FresnelConductor{ eta_i, eta_t, k }
    }
}

impl Fresnel for FresnelConductor {
    fn evaluate(&self, cos_i: Float) -> Spectrum {
        fr_conductor(cos_i.abs(), &self.eta_i, &self.eta_t, &self.k)
    }
}

impl fmt::Display for FresnelConductor {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "[ FresnelConductor eta_i: {} eta_t: {} k: {} ]", self.eta_i, self.eta_t, self.k)
    }
}

#[derive(Debug, Default, Copy, Clone)]
pub struct FresnelDielectric {
    eta_i: Float,
    eta_t: Float
}

impl FresnelDielectric {
    pub fn new(eta_i: Float, eta_t: Float) -> Self {
        FresnelDielectric { eta_i, eta_t }
    }
}

impl Fresnel for FresnelDielectric {
    fn evaluate(&self, cos_i: Float) -> Spectrum {
        Spectrum::new(fr_dielectric(cos_i, self.eta_i, self.eta_t))
    }
}

impl fmt::Display for FresnelDielectric {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "[ FresnelDielectric eta_i: {} eta_t: {} ]", self.eta_i, self.eta_t)
    }
}

/// Specialized Fresnel function used for the specular component, based on
/// a mixture between dielectric and the Schlick Fresnel approximation.
#[derive(Debug, Default, Copy, Clone)]
pub struct DisneyFresnel {
    r0: Spectrum,
    metallic: Float,
    eta: Float
}

impl DisneyFresnel {
    pub fn new(r0: Spectrum, metallic: Float, eta: Float) -> DisneyFresnel {
        DisneyFresnel{ r0, metallic, eta }
    }
}

impl Fresnel for DisneyFresnel {
    fn evaluate(&self, cos_i: Float) -> Spectrum {
        lerp(self.metallic,
            Spectrum::new(fr_dielectric(cos_i, 1.0, self.eta)),
            fr_schlick_spectrum(&self.r0, cos_i))
    }
}

impl fmt::Display for DisneyFresnel {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "[ DisneyFresnel r0: {} metallic: {} eta: {} ]", self.r0, self.metallic, self.eta)
    }
}