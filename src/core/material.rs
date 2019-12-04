use super::pbrt::Float;
use super::geometry::{Vector2f, Vector3f, Normal3f};
use super::interaction::{SurfaceInteraction};
use super::texture::Texture;
use std::sync::Arc;
use obstack::Obstack;

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum TransportMode {
    Radiance,
    Importance
}

pub trait Material {
    fn compute_scattering_function(&self, arena: &Obstack, isect: &mut SurfaceInteraction, mode: TransportMode, allow_multiple_lobes: bool);
}

// Material::bump
pub fn bump(d: Arc<dyn Texture<Float>>, si: &mut SurfaceInteraction) {
    // Compute offset positions and evaluate displacement texture
    let mut si_eval = si.clone();

    // Shift _siEval_ _du_ in the $u$ direction
    let du = 0.5 * (si.dudx.read().unwrap().abs() + si.dudy.read().unwrap().abs());
    // The most common reason for du to be zero is for ray that start from
    // light sources, where no differentials are available. In this case,
    // we try to choose a small enough du so that we still get a decently
    // accurate bump value.
    if du == 0.0 {
       du = 0.0005;
    }
    si_eval.p = si.p + du * si.shading.dpdu;
    si_eval.uv = si.uv + Vector2f::new(du, 0.0);
    si_eval.n = (Normal3f::from(si.shading.dpdu.cross(&si.shading.dpdv)) +
                         du * si.dndu).normalize();
    let u_displace = d.evaluate(&si_eval);

    // Shift _siEval_ _dv_ in the $v$ direction
    let dv = 0.5 * (si.dvdx.read().unwrap().abs() + si.dvdy.read().unwrap().abs());
    if dv == 0.0 {
        dv = 0.0005;
    }
    si_eval.p = si.p + dv * si.shading.dpdv;
    si_eval.uv = si.uv + Vector2f::new(0.0, dv);
    si_eval.n = (Normal3f::from(si.shading.dpdu.cross(&si.shading.dpdv)) +
                         dv * si.dndv).normalize();
    let v_displace = d.evaluate(si_eval);
    let displace = d.evaluate(*si);

    // Compute bump-mapped differential geometry
    let dpdu = si.shading.dpdu +
                    (u_displace - displace) / du * Vector3f::from(si.shading.n) +
                    displace * Vector3f::from(si.shading.dndu);
    let dpdv = si.shading.dpdv +
                    (v_displace - displace) / dv * Vector3f::from(si.shading.n) +
                    displace * Vector3f::from(si.shading.dndv);
    si.set_shading_geometry(dpdu, dpdv, &si.shading.dndu, &si.shading.dndv,
                           false);
}