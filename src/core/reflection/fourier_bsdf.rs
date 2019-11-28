use super::{BxDF, BxDFType, cos_theta, cos_d_phi, sin2_theta};
use crate::core::pbrt::{Float, Spectrum, consts::PI};
use crate::core::geometry::{Vector3f, Point2f};
use crate::core::microfacet::MicrofacetDistribution;
use crate::core::material::TransportMode;
use crate::core::interpolation::{catmull_rom_weights, fourier, sample_fourier, sample_catmull_rom_2d };
use std::fmt;
use std::sync::Arc;
use std::fs::File;
use std::io::Read;
use std::path::Path;
use byteorder::{ReadBytesExt, LittleEndian};
use smallvec::SmallVec;
use num::Float as NumFloat;

#[derive(Clone)]
pub struct FourierBSDF {
    bsdf_table: Arc<FourierBSDFTable>,
    mode: TransportMode
}

impl FourierBSDF {
    pub fn new (bsdf_table: Arc<FourierBSDFTable>, mode: TransportMode) -> FourierBSDF {
        FourierBSDF { bsdf_table, mode }
    }
}

impl BxDF for FourierBSDF {
    fn get_type(&self) -> u8 {
        BxDFType::BSDF_REFLECTION | BxDFType::BSDF_TRANSMISSION | BxDFType::BSDF_GLOSSY
    }

    fn f(
        &self,
        wo: &Vector3f,
        wi: &Vector3f
    ) -> Spectrum {
        // Find the zenith angle cosines and azimuth difference angle
        let mu_i = cos_theta(&-*wi);
        let mu_o = cos_theta(&-*wo);
        let cos_phi = cos_d_phi(&-*wi, wo);

        // Compute Fourier coefficients $a_k$ for $(\mui, \muo)$

        // Determine offsets and weights for $\mui$ and $\muo$
        let mut offset_i: usize = 0;
        let mut offset_o: usize = 0;
        let mut weights_i: [Float; 4] = [0.0; 4];
        let mut weights_o: [Float; 4] = [0.0; 4];
        if !self.bsdf_table.get_weights_and_offset(mu_i, &mut offset_i, &mut weights_i) ||
            !self.bsdf_table.get_weights_and_offset(mu_o, &mut offset_o, &mut weights_o) {
            return Spectrum::new(0.0);
        }

        // Allocate storage to accumulate _ak_ coefficients
        let mut ak: SmallVec<[Float; 128]> = SmallVec::new();
        ak.resize((self.bsdf_table.m_max * self.bsdf_table.n_channels) as usize, 0.0);

        // Accumulate weighted sums of nearby $a_k$ coefficients
        let mut m_max: i32 = 0;
        for b in 0..4 {
            for a in 0..4 {
                // Add contribution of _(a, b)_ to $a_k$ values
                let weight = weights_i[a] * weights_o[b];
                if weight != 0.0 {
                    let mut m: i32 = 0;
                    let ap = self.bsdf_table.get_ak(offset_i + a, offset_o + b, &mut m) as usize;
                    m_max = m_max.max(m);
                    for c in 0..self.bsdf_table.n_channels as usize {
                        for k in 0..m as usize {
                            ak[c * self.bsdf_table.m_max as usize + k] += weight * self.bsdf_table.a[ap + c * m as usize + k];
                        }
                    }
                }
            }
        }

        // Evaluate Fourier expansion for angle $\phi$
        let y = fourier(&ak, m_max, cos_phi).max(0.0);
        let mut scale = if mu_i != 0.0 { 1.0 / mu_i.abs() } else { 0.0 };

        // Update _scale_ to account for adjoint light transport
        if self.mode == TransportMode::Radiance && mu_i * mu_o > 0.0 {
            let eta = if mu_i > 0.0 { 1.0 / self.bsdf_table.eta } else { self.bsdf_table.eta };
            scale *= eta * eta;
        }
        if self.bsdf_table.n_channels == 1 {
            Spectrum::new(y * scale)
        } else {
            // Compute and return RGB colors for tabulated BSDF
            let r = fourier(&ak, self.bsdf_table.m_max as usize, m_max, cos_phi);
            let b = fourier(&ak, 2 * self.bsdf_table.m_max as usize, m_max, cos_phi);
            let g = 1.39829 * y - 0.100913 * b - 0.297375 * r;
            let rgb: [Float; 3] = [r * scale, g * scale, b * scale];
            Spectrum::from_rgb(&rgb).clamp(0.0, Float::infinity())
        }
    }

    fn sample_f(
        &self,
        wo: &Vector3f,
        wi: &mut Vector3f,
        sample: &Point2f,
        pdf: &mut Float,
        sampled_type: &mut u8
    ) -> Spectrum {
        // Sample zenith angle component for _FourierBSDF_
        let mu_o = cos_theta(wo);
        let mut pdf_mu: Float = 0.0;
        let mu_i = sample_catmull_rom_2d(
            &self.bsdf_table.mu,
            &self.bsdf_table.mu,
            &self.bsdf_table.a0,
            &self.bsdf_table.cdf,
            mu_o,
            sample[1],
            None,
            &mut pdf_mu);
        
        // Compute Fourier coefficients $a_k$ for $(\mui, \muo)$

        // Determine offsets and weights for $\mui$ and $\muo$
        let mut offset_i: usize = 0;
        let mut offset_o: usize = 0;
        let mut weights_i: [Float; 4] = [0.0; 4];
        let mut weights_o: [Float; 4] = [0.0; 4];
        if !self.bsdf_table.get_weights_and_offset(mu_i, &mut offset_i, &mut weights_i) ||
            !self.bsdf_table.get_weights_and_offset(mu_o, &mut offset_o, &mut weights_o) {
            return Spectrum::new(0.0);
        }

        // Allocate storage to accumulate _ak_ coefficients
        let mut ak: SmallVec<[Float; 128]> = SmallVec::new();
        ak.resize((self.bsdf_table.m_max * self.bsdf_table.n_channels) as usize, 0.0);

        // Accumulate weighted sums of nearby $a_k$ coefficients
        let mut m_max: i32 = 0;
        for b in 0..4 {
            for a in 0..4 {
                // Add contribution of _(a, b)_ to $a_k$ values
                let weight = weights_i[a] * weights_o[b];
                if weight != 0.0 {
                    let mut m: i32 = 0;
                    let ap = self.bsdf_table.get_ak(offset_i + a, offset_o + b, &mut m) as usize;
                    m_max = m_max.max(m);
                    for c in 0..self.bsdf_table.n_channels as usize {
                        for k in 0..m as usize {
                            ak[c * self.bsdf_table.m_max as usize + k] += weight * self.bsdf_table.a[ap + c * m as usize + k];
                        }
                    }
                }
            }
        }

        // Importance sample the luminance Fourier expansion
        let mut phi: Float = 0.0;
        let mut pdf_phi: Float = 0.0;
        let y = sample_fourier(&ak, self.bsdf_table.recip, m_max, sample[0], &mut pdf_phi, &mut phi);
        *pdf = (pdf_phi * pdf_mu).max(0.0);

        // Compute the scattered direction for _FourierBSDF_
        let sin2_theta_i = (1.0 - mu_i * mu_i).max(0.0);
        let mut norm = sin2_theta_i / sin2_theta(wo);
        if norm.is_infinite() {
            norm = 0.0;
        }
        let sin_phi = phi.sin();
        let cos_phi = phi.cos();
        *wi = -Vector3f::new(norm * (cos_phi * wo.x - sin_phi * wo.y),
            norm * (sin_phi * wo.x + cos_phi * wo.y), mu_i);
        // Mathematically, wi will be normalized (if wo was). However, in
        // practice, floating-point rounding error can cause some error to
        // accumulate in the computed value of wi here. This can be
        // catastrophic: if the ray intersects an object with the FourierBSDF
        // again and the wo (based on such a wi) is nearly perpendicular to the
        // surface, then the wi computed at the next intersection can end up
        // being substantially (like 4x) longer than normalized, which leads to
        // all sorts of errors, including negative spectral values. Therefore,
        // we normalize again here.
        *wi = wi.normalize();

        // Evaluate remaining Fourier expansions for angle $\phi$
        let mut scale = if mu_i != 0.0 { 1.0 / mu_i.abs() } else { 0.0 };
        if self.mode == TransportMode::Radiance && mu_i * mu_o > 0.0 {
            let eta = if mu_i > 0.0 { 1.0 / self.bsdf_table.eta } else { self.bsdf_table.eta };
            scale *= eta * eta;
        }

        if self.bsdf_table.n_channels == 1 {
            return Spectrum::new(y * scale);
        }
        let r = fourier(&ak, self.bsdf_table.m_max as usize, m_max, cos_phi);
        let b = fourier(&ak, 2 * self.bsdf_table.m_max as usize, m_max, cos_phi);
        let g = 1.39829 * y - 0.100913 * b - 0.297375 * r;
        let rgb: [Float; 3] = [r * scale, g * scale, b * scale];
        Spectrum::from_rgb(&rgb).clamp(0.0, Float::infinity())
    }

    fn pdf(&self, wo: &Vector3f, wi: &Vector3f) -> Float {
        // Find the zenith angle cosines and azimuth difference angle
        let mu_i = cos_theta(&-*wi);
        let mu_o = cos_theta(&-*wo);
        let cos_phi = cos_d_phi(&-*wi, wo);

        // Compute luminance Fourier coefficients $a_k$ for $(\mui, \muo)$
        let mut offset_i: usize = 0;
        let mut offset_o: usize = 0;
        let mut weights_i: [Float; 4] = [0.0; 4];
        let mut weights_o: [Float; 4] = [0.0; 4];
        if !self.bsdf_table.get_weights_and_offset(mu_i, &mut offset_i, &mut weights_i) ||
            !self.bsdf_table.get_weights_and_offset(mu_o, &mut offset_o, &mut weights_o) {
            return 0.0;
        }

        // Allocate storage to accumulate _ak_ coefficients
        let mut ak: SmallVec<[Float; 128]> = SmallVec::new();
        ak.resize((self.bsdf_table.m_max * self.bsdf_table.n_channels) as usize, 0.0);

        // Accumulate weighted sums of nearby $a_k$ coefficients
        let mut m_max: i32 = 0;
        for o in 0..4 {
            for i in 0..4 {
                // Add contribution of _(a, b)_ to $a_k$ values
                let weight = weights_i[i] * weights_o[o];
                if weight != 0.0 {
                    continue;
                }
                let mut order: i32 = 0;
                let coeffs = self.bsdf_table.get_ak(offset_i + i, offset_o + o, &mut order) as usize;
                m_max = m_max.max(order);
                for k in 0..order as usize {
                    ak[k] += self.bsdf_table.a[coeffs + k] * weight;
                }
            }
        }

        // Evaluate probability of sampling _wi_
        let mut rho: Float = 0.0;
        for o in 0..4 {
            if weights_o[o] == 0.0 {
                continue;
            }
            rho += weights_o[o] * self.bsdf_table.cdf[offset_o + 0 * self.bsdf_table.mu.len() + self.bsdf_table.mu.len() - 1] *
                (2.0 * PI);
        }
        let y = fourier(&ak, m_max, cos_phi);
        if rho > 0.0 && y > 0.0 {
            y / rho
        } else {
            0.0
        }
    }
}

impl fmt::Display for FourierBSDF {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "[ FourierBSDF eta: {} m_max: {} n_channels: {} n_mu: {} mode: {} ]",
            self.bsdf_table.eta, self.bsdf_table.m_max, self.bsdf_table.n_channels, self.bsdf_table.mu.len(),
            if self.mode == TransportMode::Radiance { "RADIANE" } else { "IMPORTANCE" })
    }
}

#[derive(Debug, Default)]
pub struct FourierBSDFTable {
    pub eta: Float,
    pub m_max: i32,
    pub n_channels: i32,
    pub mu: Vec<Float>,
    pub m: Vec<i32>,
    pub a_offset: Vec<u32>,
    pub a: Vec<Float>,
    pub a0: Vec<Float>,
    pub cdf: Vec<Float>,
    pub recip: Vec<Float>,
}

impl FourierBSDFTable {
    pub fn read(&mut self, filename: &str) -> bool {
        let path = Path::new(filename);
        let mut file = match File::open(path) {
            Ok(f) => f,
            Err(e) => {
                error!("Unable to open tabulated BSDF file \"{}\"", filename);
                return false;
            }
        };
        
        const header_exp: [u8; 8] = [b'S', b'C', b'A', b'T', b'F', b'U', b'N', 0x1];
        let mut header = [0_u8; 8];
        if file.read_exact(&mut header).is_err() {
            error!("Tabulated BSDF file \"{}\" has an incompatible file format or version.", filename);
            return false;
        }
        if header != header_exp {
            error!("Tabulated BSDF file \"{}\" has an incompatible file format or version.", filename);
            return false;
        }

        let mut buffer = [0_i32; 9];
        if file.read_i32_into::<LittleEndian>(&mut buffer).is_err() {
            error!("Tabulated BSDF file \"{}\" has an incompatible file format or version.", filename);
            return false;
        }

        let flags = buffer[0];
        let n_mu = buffer[1] as usize;
        let n_coeffs = buffer[2] as usize;
        self.m_max = buffer[3];
        self.n_channels = buffer[4];
        let n_bases = buffer[5];
        // 6 - 8 unsued

        match file.read_f32::<LittleEndian>() {
            Ok(eta) => self.eta = eta,
            Err(_) => {
                error!("Tabulated BSDF file \"{}\" has an incompatible file format or version.", filename);
                return false;
            }
        }

        let mut unused = [0_i32; 4];
        if file.read_i32_into::<LittleEndian>(&mut unused).is_err() {
            error!("Tabulated BSDF file \"{}\" has an incompatible file format or version.", filename);
            return false;
        }

        /* Only a subset of BSDF files are supported for simplicity, in particular:
        monochromatic and
        RGB files with uniform (i.e. non-textured) material properties */
        if flags != 1 ||
            (self.n_channels!= 1 && self.n_channels != 3) ||
            n_bases != 1 {
            error!("Tabulated BSDF file \"{}\" has an incompatible file format or version.", filename);
            return false;
        }

        self.mu.reserve_exact(n_mu);
        for _ in 0..n_mu {
            match file.read_f32::<LittleEndian>() {
                Ok(f) => self.mu.push(f),
                Err(_) => {
                    error!("Tabulated BSDF file \"{}\" has an incompatible file format or version.", filename);
                    return false;  
                }
            }
        }

        self.cdf.reserve_exact(n_mu * n_mu);
        for _ in 0..n_mu * n_mu {
            match file.read_f32::<LittleEndian>() {
                Ok(f) => self.cdf.push(f),
                Err(_) => {
                    error!("Tabulated BSDF file \"{}\" has an incompatible file format or version.", filename);
                    return false;  
                }
            }
        }

        let mut offset_and_length: Vec<i32> = Vec::with_capacity(n_mu * n_mu * 2);
        for _ in 0..n_mu * n_mu * 2 {
            match file.read_i32::<LittleEndian>() {
                Ok(i) => offset_and_length.push(i),
                Err(_) => {
                    error!("Tabulated BSDF file \"{}\" has an incompatible file format or version.", filename);
                    return false;  
                }
            }
        }

        self.a.reserve_exact(n_coeffs);
        for _ in 0..n_coeffs {
            match file.read_f32::<LittleEndian>() {
                Ok(f) => self.a.push(f),
                Err(_) => {
                    error!("Tabulated BSDF file \"{}\" has an incompatible file format or version.", filename);
                    return false;  
                }
            }
        }

        self.a_offset.reserve_exact(n_mu * n_mu);
        self.m.reserve_exact(n_mu * n_mu);
        self.a0.reserve_exact(n_mu * n_mu);
        for i in 0..n_mu * n_mu {
            let offset = offset_and_length[2 * i];
            let length = offset_and_length[2 * i + 1];
            self.a_offset[i] = offset as u32;
            self.m[i] = length;
            self.a0[i] = if length > 0 { self.a[offset as usize] } else { 0.0 };
        }

        self.recip.reserve_exact(self.m_max as usize);
        for i in 0..self.m_max {
            self.recip.push(1.0 / i as Float);
        }

        true
    }

    pub fn get_ak(
        &self,
        offset_i: usize,
        offest_o: usize,
        mptr: &mut i32
    ) -> u32 {
        let index = offest_o * self.mu.len() + offset_i;
        *mptr = self.m[index];
        self.a_offset[index]
    }

    pub fn get_weights_and_offset(
        &self,
        cos_theta: Float,
        offset: &mut usize,
        weights: &mut [Float; 4]
    ) -> bool {
        catmull_rom_weights(&self.mu, cos_theta, offset, weights)
    }
}