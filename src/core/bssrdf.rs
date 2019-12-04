use super::pbrt::{Float, Spectrum, consts::{PI, INV_4_PI}};
use super::profiler::Profiler;
use super::interaction::{Interaction, SurfaceInteraction, SimpleInteraction};
use super::geometry::{Vector3f, Point2f, Normal3f, dot_vec_normal};
use super::scene::Scene;
use super::material::{Material, TransportMode};
use super::reflection::{fr_dielectric, cos_theta, BSDF, BxDF, BxDFType};
use super::interpolation::{catmull_rom_weights, sample_catmull_rom_2d, integrate_catmull_rom, invert_catmull_rom};
use super::medium::phase_hg;
use obstack::Obstack;
use std::sync::Arc;
use std::fmt;
use rayon::prelude::*;
use num::Float as NumFloat;


pub trait BSSRDF {
    fn s(&self, pi: &SurfaceInteraction, wi: &Vector3f) -> Spectrum;
    fn sample_s(
        &self,
        scene: &Scene,
        u1: Float,
        u2: Point2f,
        arena: &Obstack,
        si: &mut SurfaceInteraction,
        pdf: &mut Float
    ) -> Spectrum;
}

pub trait SeparableBSSRDF: BSSRDF {
    fn sw(&self, w: &Vector3f) -> Spectrum;
    fn sp(&self, pi: &SurfaceInteraction) -> Spectrum;
    fn sample_sp(
        &self,
        scene: &Scene,
        u1: Float,
        u2: Point2f,
        arena: &Obstack,
        si: &mut SurfaceInteraction,
        pdf: &mut Float
    ) -> Spectrum;
    fn pdf_sp(&self, si: &SurfaceInteraction) -> Float;
}

pub trait SeparableBSSRDFProvider {
    fn sr(&self, d: Float) -> Spectrum;
    fn sample_sr(&self, ch: usize, u: Float) -> Float;
    fn pdf_sr(&self, ch: usize, u: Float) -> Float;
}

/// Inner implementation of the SeparableBSSRDF class from C++.
#[derive(Clone)]
pub struct SeparableBSSRDFImplementation<'a> {
    po: &'a SurfaceInteraction,
    eta: Float,
    ns: Normal3f,
    ss: Vector3f,
    ts: Vector3f,
    material: Arc<dyn Material>,
    mode: TransportMode
}

impl<'a> SeparableBSSRDFImplementation<'a> {
    pub fn new(
        po: &'a SurfaceInteraction,
        eta: Float, material: Arc<dyn Material>,
        mode: TransportMode
    ) -> Self {
        let ns = po.shading.n;
        let ss = po.shading.dpdu.normalize();
        let ts = Vector3f::from(ns).cross(&ss);
        Self{
            po,
            eta,
            ns,
            ss,
            ts,
            material,
            mode
        }
    }

    pub fn s(&self, provider: &impl SeparableBSSRDFProvider, pi: &SurfaceInteraction, wi: &Vector3f) -> Spectrum {
        let _profile = Profiler::instance().profile("BSSRDF::s()");
        let ft = fr_dielectric(cos_theta(&self.po.wo), 1.0, self.eta);
        (1.0 - ft) * self.sp(provider, pi) * self.sw(wi)
    }

    pub fn sw(&self, w: &Vector3f) -> Spectrum {
        let c = 1.0 - 2.0 * fresnel_moment1(1.0 / self.eta);
        Spectrum::new((1.0 - fr_dielectric(cos_theta(w), 1.0, self.eta)) / (c * PI))
    }

    pub fn sp(&self, provider: &impl SeparableBSSRDFProvider, pi: &SurfaceInteraction) -> Spectrum {
        provider.sr(self.po.p.distance(&pi.p))
    }

    pub fn sample_s(
        &self,
        provider: &impl SeparableBSSRDFProvider,
        scene: &Scene,
        u1: Float,
        u2: Point2f,
        arena: &Obstack,
        si: &mut SurfaceInteraction,
        pdf: &mut Float
    ) -> Spectrum {
        let _profile = Profiler::instance().profile("BSSRDF::sample_s()");
        let sp = self.sample_sp(provider, scene, u1, u2, arena, si, pdf);
        if !sp.is_black() {
            // Initialize material model at sampled surface interaction
            // TODO: sort out area issues
            let bsdf = arena.push_copy(BSDF::new(si, 1.0));
            bsdf.add(arena.push_copy(SeparableBSSRDFAdapter::new(self, self.mode, self.eta)));
            si.bsdf = bsdf;
            si.wo = Vector3f::from(si.shading.n);
        }
        sp
    }

    pub fn sample_sp(
        &self,
        provider: &impl SeparableBSSRDFProvider,
        scene: &Scene,
        mut u1: Float,
        u2: Point2f,
        arena: &Obstack,
        si: &mut SurfaceInteraction,
        pdf: &mut Float
    ) -> Spectrum {
        let _profile = Profiler::instance().profile("BSSRDF::s()");
        // Choose projection axis for BSSRDF sampling
        let mut vx: Vector3f = Vector3f::default();
        let mut vy: Vector3f = Vector3f::default();
        let mut vz: Vector3f = Vector3f::default();
        if u1 < 0.5 {
            vx = self.ss;
            vy = self.ts;
            vz = Vector3f::from(self.ns);
            u1 *= 2.0;
        } else if u1 < 0.75 {
            // Prepare for sampling rays with respect to _ss_
            vx = self.ts;
            vy = Vector3f::from(self.ns);
            vz = self.ss;
            u1 = (u1 - 0.5) * 4.0;
        } else {
            // Prepare for sampling rays with respect to _ts_
            vx = Vector3f::from(self.ns);
            vy = self.ss;
            vz = self.ts;
            u1 = (u1 - 0.75) * 4.0;
        }

        // Choose spectral channel for BSSRDF sampling
        let ch = num::clamp(u1 as usize * Spectrum::n_samples, 0, Spectrum::n_samples - 1);
        u1 = u1 * (Spectrum::n_samples - ch) as Float;

        // Sample BSSRDF profile in polar coordinates
        let r = provider.sample_sr(ch, u2[0]);
        if r < 0.0 {
            return Spectrum::new(0.0);
        }
        let phi = 2.0 * PI * u2[1];

        // Compute BSSRDF profile bounds and intersection height
        let r_max = provider.sample_sr(ch, 0.999);
        if r >= r_max {
            return Spectrum::new(0.0);
        }
        let l = 2.0 * (r_max * r_max - r * r).sqrt();

        // Compute BSSRDF sampling ray segment
        let mut base: SimpleInteraction = SimpleInteraction::default();
        base.p = self.po.p + r * (vx * phi.cos() + vy * phi.sin()) - l * vz * 0.5;
        base.time = self.po.time;
        let p_target = base.p + l * vz;

        // Intersect BSSRDF sampling ray against the scene geometry

        let chain: Vec<SurfaceInteraction> = Vec::new();
        // Accumulate chain of intersections along ray
        let mut n_found = 0_usize;
        loop {
            let r = base.spawn_ray_to_point(&p_target);
            if r.d == Vector3f::default() {
                break;
            }
            if let Some(si) = scene.intersect(&r) {
                base.p = si.p;
                base.time = si.time;
                base.p_error = si.p_error;
                base.wo = si.wo;
                base.n = si.n;
                base.medium_interface = si.medium_interface.clone();
                // Append admissible intersection to _IntersectionChain_
                if let Some(primitive) = si.primitive {
                    if let Some(material) = primitive.get_material() {
                        if Arc::ptr_eq(&self.material, &material) {
                            chain.push(si);
                            n_found += 1;
                        }
                    }
                }
            } else {
                break;
            }
        }

        // Randomly choose one of several intersections during BSSRDF sampling
        if n_found == 0 {
            return Spectrum::new(0.0);
        }
        let selected = num::clamp(u1 as usize * n_found, 0, n_found - 1);
        *si = chain[selected].clone();

        // Compute sample PDF and return the spatial BSSRDF term $\Sp$
        *pdf = self.pdf_sp(provider, &*si) / n_found as Float;
        self.sp(provider, &*si)
    }

    pub fn pdf_sp(&self, provider: &impl SeparableBSSRDFProvider, si: &SurfaceInteraction) -> Float {
        // Express $\pti-\pto$ and $\bold{n}_i$ with respect to local coordinates at
        // $\pto$
        let d = self.po.p - si.p;
        let d_local = Vector3f{
            x: self.ss.dot(&d),
            y: self.ts.dot(&d),
            z: self.ns.dot(&si.n)
        };
        let n_local = Normal3f{
            x: dot_vec_normal(&self.ss, &si.n),
            y: dot_vec_normal(&self.ts, &si.n),
            z: self.ns.dot(&si.n)
        };

        // Compute BSSRDF profile radius under projection along each axis
        let r_proj: [Float; 3] = [
            (d_local.y * d_local.y + d_local.z * d_local.z).sqrt(),
            (d_local.z * d_local.z + d_local.x * d_local.x).sqrt(),
            (d_local.x * d_local.x + d_local.y * d_local.y).sqrt(),
        ];

        // Return combined probability from all BSSRDF sampling strategies
        let mut pdf: Float = 0.0;
        let axis_prob: [Float; 3] = [0.25, 0.25, 0.5];
        let ch_prob = 1.0 / Spectrum::n_samples as Float;
        for axis in 0..3 {
            for ch in 0..Spectrum::n_samples {
                pdf += provider.pdf_sr(ch, r_proj[axis]) * n_local[axis as u8].abs() * ch_prob * axis_prob[axis];
            }
        }
        pdf
    }
}

#[derive(Clone)]
pub struct TabulatedBSSRDF<'a> {
    inner: SeparableBSSRDFImplementation<'a>,
    table: Arc<BSSRDFTable>,
    sigma_t: Spectrum,
    rho: Spectrum
}

impl<'a> TabulatedBSSRDF<'a> {
    pub fn new(
        po: &'a SurfaceInteraction,
        material: Arc<dyn Material>,
        mode: TransportMode,
        eta: Float,
        sigma_a: Spectrum,
        sigma_s: Spectrum,
        table: Arc<BSSRDFTable>
    ) -> TabulatedBSSRDF<'a> {
        let sigma_t = sigma_a + sigma_s;
        let rho = Spectrum::default();
        for c in 0..Spectrum::n_samples {
            rho[c] = if sigma_t[c] != 0.0 { sigma_s[c] / sigma_t[c] } else { 0.0 };
        }
        TabulatedBSSRDF {
            inner: SeparableBSSRDFImplementation::new(po, eta, material, mode),
            table,
            sigma_t,
            rho
        }
    }
}

impl<'a> SeparableBSSRDFProvider for TabulatedBSSRDF<'a> {
    fn sr(&self, r: Float) -> Spectrum {
        let mut sr = Spectrum::new(0.0);
        for ch in 0..Spectrum::n_samples {
            // Convert $r$ into unitless optical radius $r_{\roman{optical}}$
            let r_optical = r * self.sigma_t[ch];

            // Compute spline weights to interpolate BSSRDF on channel _ch_
            let mut rho_offset: usize = 0;
            let mut radius_offset: usize = 0;
            let mut rho_weights = [0.0 as Float; 4];
            let mut radius_weights = [0.0 as Float; 4];
            if !catmull_rom_weights(&self.table.rho_samples, self.rho[ch], &mut rho_offset, &mut rho_weights) ||
                !catmull_rom_weights(&self.table.radius_samples, r_optical, &mut radius_offset, &mut radius_weights) {
                continue;
            }

            // Set BSSRDF value _Sr[ch]_ using tensor spline interpolation
            let mut srv: Float = 0.0;
            for i in 0..4 {
                for j in 0..4 {
                    let weight = rho_weights[i] * radius_weights[j];
                    if weight != 0.0 {
                        srv += weight * self.table.eval_profile(rho_offset + i, radius_offset + j);
                    }
                }
            }

            // Cancel marginal PDF factor from tabulated BSSRDF profile
            if r_optical != 0.0 {
                sr /= 2.0 * PI * r_optical;
            }
        }

        sr.clamp(0.0, Float::infinity())
    }

    fn sample_sr(&self, ch: usize, u: Float) -> Float {
        if self.sigma_t[ch] == 0.0 {
            return -1.0;
        }
        sample_catmull_rom_2d(&self.table.rho_samples, &self.table.radius_samples,
            &self.table.profile, &self.table.profile_cdf, self.rho[ch], u, None, None) /
            self.sigma_t[ch]
    }

    fn pdf_sr(&self, ch: usize, r: Float) -> Float {
        // Convert $r$ into unitless optical radius $r_{\roman{optical}}$
        let r_optical = r * self.sigma_t[ch];

        // Compute spline weights to interpolate BSSRDF on channel _ch_
        let mut rho_offset: usize = 0;
        let mut radius_offset: usize = 0;
        let mut rho_weights = [0.0 as Float; 4];
        let mut radius_weights = [0.0 as Float; 4];
        if !catmull_rom_weights(&self.table.rho_samples, self.rho[ch], &mut rho_offset, &mut rho_weights) ||
            !catmull_rom_weights(&self.table.radius_samples, r_optical, &mut radius_offset, &mut radius_weights) {
            return 0.0;
        }

        // Return BSSRDF profile density for channel _ch_
        let mut sr: Float = 0.0;
        let mut rho_eff: Float = 0.0;
        for i in 0..4 {
            if rho_weights[i] == 0.0 {
                continue;
            }
            rho_eff += self.table.rho_eff[rho_offset + i] * rho_weights[i];
            for j in 0..4 {
                if radius_weights[j] == 0.0 {
                    continue;
                }
                sr += self.table.eval_profile(rho_offset + i, radius_offset + j) *
                    rho_weights[i] * radius_weights[j];
            }
        }

        // Cancel marginal PDF factor from tabulated BSSRDF profile
        if r_optical != 0.0 {
            sr /= 2.0 * PI * r_optical;
        }
        (sr * self.sigma_t[ch] * self.sigma_t[ch] / rho_eff).max(0.0)
    }
}

impl<'a> BSSRDF for TabulatedBSSRDF<'a> {
    fn s(&self, pi: &SurfaceInteraction, wi: &Vector3f) -> Spectrum {
        self.inner.s(self, pi, wi)
    }

    fn sample_s(
        &self,
        scene: &Scene,
        u1: Float,
        u2: Point2f,
        arena: &Obstack,
        si: &mut SurfaceInteraction,
        pdf: &mut Float
    ) -> Spectrum {
        self.inner.sample_s(self, scene, u1, u2, arena, si, pdf)
    }
}

impl<'a> SeparableBSSRDF for TabulatedBSSRDF<'a> {
    fn sw(&self, w: &Vector3f) -> Spectrum {
        self.inner.sw(w)
    }

    fn sp(&self, pi: &SurfaceInteraction) -> Spectrum {
        self.inner.sp(self, pi)
    }

    fn sample_sp(
        &self,
        scene: &Scene,
        u1: Float,
        u2: Point2f,
        arena: &Obstack,
        si: &mut SurfaceInteraction,
        pdf: &mut Float
    ) -> Spectrum {
        self.inner.sample_sp(self, scene, u1, u2, arena, si, pdf)
    }

    fn pdf_sp(&self, si: &SurfaceInteraction) -> Float {
        self.inner.pdf_sp(self, si)
    }
}

pub struct SeparableBSSRDFAdapter<'a, T>
where
T: SeparableBSSRDF
{
    bssrdf: &'a T,
    mode: TransportMode,
    eta: Float
}

impl<'a, T> SeparableBSSRDFAdapter<'a, T>
where
T: SeparableBSSRDF
{
    pub fn new(bssrdf: &'a T, mode: TransportMode, eta: Float) -> Self {
        Self { bssrdf, mode, eta }
    }
}

impl<'a, T> BxDF for SeparableBSSRDFAdapter<'a, T>
where
T: SeparableBSSRDF
{
    fn get_type(&self) -> u8 {
        BxDFType::BSDF_REFLECTION | BxDFType::BSDF_DIFFUSE
    }

    fn f(
        &self,
        wo: &Vector3f,
        wi: &Vector3f
    ) -> Spectrum {
        let mut f = self.bssrdf.sw(wi);
        // Update BSSRDF transmission term to account for adjoint light
        // transport
        if self.mode == TransportMode::Radiance {
            f *= self.eta * self.eta;
        }
        f
    }
}

impl<'a, T> fmt::Display for SeparableBSSRDFAdapter<'a, T>
where
T: SeparableBSSRDF
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "[ SeparableBSSRDFAdapter ]")
    }
}

#[derive(Debug, Default, Clone)]
pub struct BSSRDFTable {
    pub rho_samples: Vec<Float>,
    pub radius_samples: Vec<Float>,
    pub profile: Vec<Float>,
    pub rho_eff: Vec<Float>,
    pub profile_cdf: Vec<Float>
}

impl BSSRDFTable {
    pub fn new(n_rho_samples: usize, n_radius_samples: usize) -> BSSRDFTable {
        BSSRDFTable {
            rho_samples: vec![0.0 as Float; n_rho_samples],
            radius_samples: vec![0.0 as Float; n_radius_samples],
            profile: vec![0.0 as Float; n_radius_samples * n_rho_samples],
            rho_eff: vec![0.0 as Float; n_rho_samples],
            profile_cdf: vec![0.0 as Float; n_radius_samples * n_rho_samples]
        }
    }

    #[inline]
    pub fn eval_profile(&self, rho_index: usize, radius_index: usize) -> Float {
        self.profile[rho_index * self.radius_samples.len() + radius_index]
    }
}

pub fn fresnel_moment1(inv_eta: Float) -> Float {
    let eta2 = inv_eta * inv_eta;
    let eta3 = eta2 * inv_eta;
    let eta4 = eta3 * inv_eta;
    let eta5 = eta4 * inv_eta;
    if inv_eta < 1.0 {
        0.45966 - 1.73965 * inv_eta + 3.37668 * eta2 - 3.904945 * eta3 +
            2.49277 * eta4 - 0.68441 * eta5
    } else {
        -4.61686 + 11.1136 * inv_eta - 10.4646 * eta2 + 5.11455 * eta3 -
            1.27198 * eta4 + 0.12746 * eta5
    }
}

pub fn fresnel_moment2(inv_eta: Float) -> Float {
    let eta2 = inv_eta * inv_eta;
    let eta3 = eta2 * inv_eta;
    let eta4 = eta3 * inv_eta;
    let eta5 = eta4 * inv_eta;
    if inv_eta < 1.0 {
        0.27614 - 0.87350 * inv_eta + 1.12077 * eta2 - 0.65095 * eta3 +
            0.07883 * eta4 + 0.04860 * eta5
    } else {
        let r_eta = 1.0 / inv_eta;
        let r_eta2 = r_eta * r_eta;
        let r_eta3 = r_eta2 * r_eta;
        -547.033 + 45.3087 * r_eta3 - 218.725 * r_eta2 +
            458.843 * r_eta + 404.557 * inv_eta - 189.519 * eta2 +
            54.9327 * eta3 - 9.00603 * eta4 + 0.63942 * eta5
    }
}

pub fn beam_diffusion_ss(sigma_s: Float, sigma_a: Float, g: Float, eta: Float, r: Float) -> Float {
    const n_samples: usize = 100;
    let mut ed: Float = 0.0;
    // Precompute information for dipole integrand

    // Compute reduced scattering coefficients $\sigmaps, \sigmapt$ and albedo
    // $\rhop$
    let sigmap_s = sigma_s * (1.0 - g);
    let sigmap_t = sigma_a + sigmap_s;
    let rhop = sigmap_s / sigmap_t;

    // Compute non-classical diffusion coefficient $D_\roman{G}$ using
    // Equation (15.24)
    let d_g = (2.0 * sigma_a + sigmap_s) / (3.0 * sigmap_t * sigmap_t);

    // Compute effective transport coefficient $\sigmatr$ based on $D_\roman{G}$
    let sigma_tr = (sigma_a / d_g).sqrt();

    // Determine linear extrapolation distance $\depthextrapolation$ using
    // Equation (15.28)
    let fm1 = fresnel_moment1(eta);
    let fm2 = fresnel_moment2(eta);
    let ze = -2.0 * d_g * (1.0 + 3.0 * fm2) / (1.0 - 2.0 * fm1);

    // Determine exitance scale factors using Equations (15.31) and (15.32)
    let c_phi = 0.25 * (1.0 - 2.0 * fm1);
    let c_e = 0.5 * (1.0 - 3.0 * fm2);
    for i in 0..n_samples {
        // Sample real point source depth $\depthreal$
        let zr = -(1.0 - (i as Float + 0.5) / n_samples as Float).ln() / sigmap_t;

        // Evaluate dipole integrand $E_{\roman{d}}$ at $\depthreal$ and add to
        // _Ed_
        let zv = -zr + 2.0 * ze;
        let dr = (r * r + zr * zr).sqrt();
        let dv = (r * r + zv * zv).sqrt();

        // Compute dipole fluence rate $\dipole(r)$ using Equation (15.27)
        let phi_d = INV_4_PI / d_g * ((-sigma_tr * dr).exp() / dr -
                                     (-sigma_tr * dv).exp() / dv);

        // Compute dipole vector irradiance $-\N{}\cdot\dipoleE(r)$ using
        // Equation (15.27)
        let e_dn = INV_4_PI * (zr * (1.0 + sigma_tr * dr) *
                                  (-sigma_tr * dr).exp() / (dr * dr * dr) -
                              zv * (1.0 + sigma_tr * dv) *
                                  (-sigma_tr * dv).exp() / (dv * dv * dv));

        // Add contribution from dipole for depth $\depthreal$ to _Ed_
        let e = phi_d * c_phi + e_dn * c_e;
        let kappa = 1.0 - (-2.0 * sigmap_t * (dr + zr)).exp();
        ed += kappa * rhop * rhop * e;
    }
    ed / n_samples as Float
}

pub fn beam_diffusion_ms(sigma_s: Float, sigma_a: Float, g: Float, eta: Float, r: Float) -> Float {
    // Compute material parameters and minimum $t$ below the critical angle
    let sigma_t = sigma_a + sigma_s;
    let rho = sigma_s / sigma_t;
    let t_crit = r * (eta * eta - 1.0).sqrt();
    let mut ess: Float = 0.0;
    const n_samples: usize = 100;
    for i in 0..n_samples {
        // Evaluate single scattering integrand and add to _Ess_
        let ti = t_crit - (1.0 - (i as Float + 0.5) / n_samples as Float).ln() / sigma_t;

        // Determine length $d$ of connecting segment and $\cos\theta_\roman{o}$
        let d = (r * r + ti * ti).sqrt();
        let cos_theta_o = ti / d;

        // Add contribution of single scattering at depth $t$
        ess += rho * (-sigma_t * (d + t_crit)).exp() / (d * d) *
               phase_hg(cos_theta_o, g) * (1.0 - fr_dielectric(-cos_theta_o, 1.0, eta)) *
               cos_theta_o.abs();
    }
    ess / n_samples as Float
}

pub fn compute_beam_diffusion_bssrdf(g: Float, eta: Float, t: &mut BSSRDFTable) {
    // Choose radius values of the diffusion profile discretization
    t.radius_samples[0] = 0.0;
    t.radius_samples[1] = 2.5e-3;
    for i in 2..t.radius_samples.len() {
        t.radius_samples[i] = t.radius_samples[i - 1] * 1.2;
    }

    // Choose albedo values of the diffusion profile discretization
    for i in 0..t.rho_samples.len() {
        t.rho_samples[i] =
            (1.0 - (-8.0 * i as Float / (t.rho_samples.len() - 1) as Float).exp()) /
            (1.0 - (-8.0).exp());
    }
    (0..t.rho_samples.len()).into_par_iter().for_each(|i| {
        // Compute the diffusion profile for the _i_th albedo sample

        // Compute scattering profile for chosen albedo $\rho$
        for j in 0..t.radius_samples.len() {
            let rho = t.rho_samples[i];
            let r = t.radius_samples[j];
            t.profile[i * t.radius_samples.len() + j] =
                2.0 * PI * r * (beam_diffusion_ss(rho, 1.0 - rho, g, eta, r) +
                              beam_diffusion_ms(rho, 1.0 - rho, g, eta, r));
        }

        // Compute effective albedo $\rho_{\roman{eff}}$ and CDF for importance
        // sampling
        t.rho_eff[i] =
            integrate_catmull_rom(&t.radius_samples[0..i * t.radius_samples.len()],
                                &t.profile,
                                &mut t.profile_cdf);
    });
}

pub fn subsurface_from_diffuse(
    table: &BSSRDFTable,
    rho_eff: &Spectrum,
    mfp: &Spectrum,
    sigma_a: &mut Spectrum,
    sigma_s: &mut Spectrum
) {
    for c in 0..Spectrum::n_samples {
        let rho = invert_catmull_rom(&table.rho_samples, &table.rho_eff, rho_eff[c]);
        sigma_s[c] = rho / mfp[c];
        sigma_a[c] = (1.0 - rho) / mfp[c];
    }
}