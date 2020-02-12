use super::pbrt::{Float, Spectrum, consts::SHADOW_EPSILON};
use super::geometry::{Vector3f, Normal3f, Point2f, Point3f, Ray, offset_ray_origin, dot_normal_vec};
use super::medium::{Medium, MediumInterface, PhaseFunction};
use super::shape::Shape;
use super::primitive::Primitive;
use super::reflection::BSDF;
use super::bssrdf::BSSRDF;
use super::material::TransportMode;
use super::transform::solve_linear_system2x2;
use obstack::Obstack;

use std::sync::{Arc, RwLock};

pub trait Interaction {
    fn get_p(&self) -> Point3f;
    fn get_time(&self) -> Float;
    fn get_p_error(&self) -> Vector3f;
    fn get_wo(&self) -> Vector3f;
    fn get_n(&self) -> Normal3f;
    fn get_medium_interface(&self) -> Option<Arc<MediumInterface>> {
        None
    }
    
    fn is_surface_interaction(&self) -> bool {
        self.get_n() != Normal3f::default()
    }

    fn is_medium_interaction(&self) -> bool {
        !self.is_surface_interaction()
    }

    /// Spawn a ray from this Interaction going in direction d.
    fn spawn_ray(&self, d: &Vector3f) -> Ray {
        let o = offset_ray_origin(&self.get_p(), &self.get_p_error(), &self.get_n(), d);
        Ray{
            o,
            d: *d,
            t_max: num::Float::infinity(),
            time: self.get_time(),
            medium: self.get_medium(d),
            differential: None
        }
    }

    /// Spawn a ray from this Interaction going towards p.
    fn spawn_ray_to_point(&self, p: &Point3f) -> Ray {
        let d = *p - self.get_p();
        let o = offset_ray_origin(&self.get_p(), &self.get_p_error(), &self.get_n(), &d);
        Ray{
            o,
            d,
            t_max: 1.0 - SHADOW_EPSILON,
            time: self.get_time(),
            medium: self.get_medium(&d),
            differential: None
        }
    }

    /// Spawn a ray going from this Interaction towards another Interaction.
    fn spawn_ray_to(&self, i: &dyn Interaction) -> Ray {
        let o = offset_ray_origin(&self.get_p(), &self.get_p_error(), &self.get_n(), &(i.get_p() - self.get_p()));
        let target = offset_ray_origin(&i.get_p(), &i.get_p_error(), &i.get_n(), &(o - i.get_p()));
        let d = target - o;
        Ray{
            o,
            d,
            t_max: 1.0 - SHADOW_EPSILON,
            time: self.get_time(),
            medium: self.get_medium(&d),
            differential: None
        }
    }

    fn get_medium(&self, w: &Vector3f) -> Option<Arc<dyn Medium>> {
        match self.get_medium_interface() {
            Some(ref medium) => {
                if w.dot(&self.get_n().into()) > 0.0 {
                    medium.inside.clone()
                }
                else {
                    medium.outside.clone()
                }
            }
            None => None
        }
    }

    fn get_phase(&self) -> Option<Arc<dyn PhaseFunction>> {
        None
    }

    fn get_bsdf(&self) -> Option<BSDF> {
        None
    }

    fn get_shading(&self) -> Option<Shading> {
        None
    }
}

/// A Simple implementation of Interaction.
#[derive(Debug, Default, Clone)]
pub struct SimpleInteraction {
    pub p: Point3f,
    pub time: Float,
    pub p_error: Vector3f,
    pub wo: Vector3f,
    pub n: Normal3f,
    pub medium_interface: Option<Arc<MediumInterface>>
}

impl SimpleInteraction {
    pub fn new(
        p: Point3f,
        time: Float,
        p_error: Vector3f,
        wo: Vector3f,
        n: Normal3f
    ) -> SimpleInteraction {
        SimpleInteraction{
            p,
            time,
            p_error,
            wo,
            n,
            medium_interface: None
        }
    }
}

impl Interaction for SimpleInteraction {
    fn get_p(&self) -> Point3f {
        self.p
    }

    fn get_time(&self) -> Float {
        self.time
    }

    fn get_p_error(&self) -> Vector3f {
        self.p_error
    }

    fn get_wo(&self) -> Vector3f {
        self.wo
    }

    fn get_n(&self) -> Normal3f {
        self.n
    }

    fn get_medium_interface(&self) -> Option<Arc<MediumInterface>> {
        self.medium_interface
    }
}

#[derive(Default, Clone)]
pub struct MediumInteraction {
    // Interaction fields
    pub p: Point3f,
    pub time: Float,
    pub p_error: Vector3f,
    pub wo: Vector3f,
    pub n: Normal3f,
    pub medium_interface: Option<Arc<MediumInterface>>,
    
    pub phase: Option<Arc<dyn PhaseFunction>>
}

impl MediumInteraction {
    pub fn new(
        p: Point3f,
        wo: Vector3f,
        time: Float,
        medium_interface: Option<Arc<MediumInterface>>,
        phase: Option<Arc<dyn PhaseFunction>>
    ) -> MediumInteraction {
        MediumInteraction {
            p,
            time,
            p_error: Vector3f::default(),
            wo,
            n: Normal3f::default(),
            medium_interface,
            phase
        }
    }

    pub fn is_valid(&self) -> bool {
        self.phase.is_some()
    }
}

impl Interaction for MediumInteraction {
    fn get_p(&self) -> Point3f {
        self.p
    }

    fn get_time(&self) -> Float {
        self.time
    }

    fn get_p_error(&self) -> Vector3f {
        self.p_error
    }

    fn get_wo(&self) -> Vector3f {
        self.wo
    }

    fn get_n(&self) -> Normal3f {
        self.n
    }

    fn get_medium_interface(&self) -> Option<Arc<MediumInterface>> {
        self.medium_interface
    }

    fn get_phase(&self) -> Option<Arc<dyn PhaseFunction>> {
        self.phase
    }
}

#[derive(Default, Debug, Clone, Copy)]
pub struct Shading {
    pub n: Normal3f,
    pub dpdu: Vector3f,
    pub dpdv: Vector3f,
    pub dndu: Normal3f,
    pub dndv: Normal3f
}

#[derive(Default)]
pub struct SurfaceInteraction<'a> {
    // Interaction fields
    pub p: Point3f,
    pub time: Float,
    pub p_error: Vector3f,
    pub wo: Vector3f,
    pub n: Normal3f,
    pub medium_interface: Option<Arc<MediumInterface>>,

    pub uv: Point2f,
    pub dpdu: Vector3f,
    pub dpdv: Vector3f,
    pub dndu: Normal3f,
    pub dndv: Normal3f,
    pub shape: Option<&'a dyn Shape>,
    pub shading: Shading,
    pub primitive: Option<&'a dyn Primitive>,
    pub bsdf: Option<BSDF>,
    pub bssrdf: Option<Box<dyn BSSRDF>>,
    pub dpdx: RwLock<Vector3f>,
    pub dpdy: RwLock<Vector3f>,
    pub dudx: RwLock<Float>,
    pub dudy: RwLock<Float>,
    pub dvdx: RwLock<Float>,
    pub dvdy: RwLock<Float>,
    pub face_index: u32
}

impl<'a> SurfaceInteraction<'a> {
    pub fn new(
        p: Point3f,
        p_error: Vector3f,
        uv: Point2f,
        wo: Vector3f,
        dpdu: Vector3f,
        dpdv: Vector3f,
        dndu: Normal3f,
        dndv: Normal3f,
        time: Float,
        shape: Option<&'a dyn Shape>,
        face_index: u32
    ) -> SurfaceInteraction {
        let mut n = Normal3f::from(dpdu.cross(&dpdv).normalize());
        // Adjust normal based on orientation and handedness
        if let Some(shape) = shape {
            if shape.get_reverse_orientation() ^ shape.get_transform_swaps_handedness() {
                n *= -1.0;
            }
        }
        let shading = Shading {
            n,
            dpdu,
            dpdv,
            dndu,
            dndv
        };

        SurfaceInteraction {
            p,
            p_error,
            uv,
            wo,
            n,
            dpdu,
            dpdv,
            dndu,
            dndv,
            dpdx: RwLock::new(Vector3f::default()),
            dpdy: RwLock::new(Vector3f::default()),
            dudx: RwLock::new(0.0),
            dudy: RwLock::new(0.0),
            dvdx: RwLock::new(0.0),
            dvdy: RwLock::new(0.0),
            time,
            shape,
            medium_interface: None,
            face_index,
            shading: Shading::default(),
            primitive: None,
            bsdf: None,
            bssrdf: None
        }
    }

    pub fn set_shading_geometry(
        &mut self,
        dpdu: &Vector3f,
        dpdv: &Vector3f,
        dndu: &Normal3f,
        dndv: &Normal3f,
        orientation_is_authoritative: bool
    ) {
        // Compute _shading.n_ for _SurfaceInteraction_
        self.shading.n = Normal3f::from(dpdu.cross(dpdv).normalize());
        if orientation_is_authoritative {
            self.n = self.n.face_forward(&self.shading.n);
        } else {
            self.shading.n = self.shading.n.face_forward(&self.n);
        }

        // Initialize _shading_ partial derivative values
        self.shading.dpdu = *dpdu;
        self.shading.dpdv = *dpdv;
        self.shading.dndu = *dndu;
        self.shading.dndv = *dndv;
    }

    pub fn le(&self, w: &Vector3f) -> Spectrum {
        if let Some(primitive) = self.primitive {
            if let Some(area) = primitive.get_area_light() {
                return area.l(self, w);
            }
        }
        Spectrum::new(0.0)
    }

    pub fn computer_scattering_functions(
        &mut self,
        ray: &Ray,
        arena: &mut Obstack,
        allow_multiple_lobes: bool,
        mode: TransportMode
    ) {
        self.compute_differentials(ray);
        if let Some(primitive) = self.primitive {
            primitive.compute_scattering_function(arena, self, mode, allow_multiple_lobes);
        }
    }

    pub fn compute_differentials(
        &self,
        ray: &Ray
    ) {
        if let Some(rdiff) = ray.differential {
            // Estimate screen space change in $\pt{}$ and $(u,v)$

            // Compute auxiliary intersection points with plane
            let d = dot_normal_vec(&self.n, &Vector3f::new(self.p.x, self.p.y, self.p.z));
            let tx = -(dot_normal_vec(&self.n, &Vector3f::from(rdiff.rx_origin)) - d) / dot_normal_vec(&self.n, &rdiff.rx_direction);
            if !tx.is_finite() {
                let mut dudx = self.dudx.write().unwrap();
                *dudx = 0.0;
                let mut dudy = self.dudy.write().unwrap();
                *dudy = 0.0;
                let mut dvdx = self.dvdx.write().unwrap();
                *dvdx = 0.0;
                let mut dvdy = self.dvdy.write().unwrap();
                *dvdy = 0.0;
                let mut dpdx = self.dpdx.write().unwrap();
                *dpdx = Vector3f::default();
                let mut dpdy = self.dpdy.write().unwrap();
                *dpdy = Vector3f::default();
            }
            let px = rdiff.rx_origin + tx * rdiff.rx_direction;
            let ty: Float = -(dot_normal_vec(&self.n, &Vector3f::from(rdiff.ry_origin)) - d) / dot_normal_vec(&self.n, &rdiff.ry_direction);
            if !ty.is_finite() {
                let mut dudx = self.dudx.write().unwrap();
                *dudx = 0.0;
                let mut dudy = self.dudy.write().unwrap();
                *dudy = 0.0;
                let mut dvdx = self.dvdx.write().unwrap();
                *dvdx = 0.0;
                let mut dvdy = self.dvdy.write().unwrap();
                *dvdy = 0.0;
                let mut dpdx = self.dpdx.write().unwrap();
                *dpdx = Vector3f::default();
                let mut dpdy = self.dpdy.write().unwrap();
                *dpdy = Vector3f::default();
            }
            let py = rdiff.ry_origin + ty * rdiff.ry_direction;
            *self.dpdx.write().unwrap() = px - self.p;
            *self.dpdy.write().unwrap() = py - self.p;

            // Compute $(u,v)$ offsets at auxiliary points

            // Choose two dimensions to use for ray offset computation
            let dim: [usize; 2] = if self.n.x.abs() > self.n.y.abs() && self.n.x.abs() > self.n.z.abs() {
                [1, 2]
            } else if self.n.y.abs() > self.n.z.abs() {
                [0, 2]
            } else {
                [0, 1]
            };

            // Initialize _A_, _Bx_, and _By_ matrices for offset computation
            let a = [
                [self.dpdu[dim[0]], self.dpdv[dim[0]]],
                [self.dpdu[dim[1]], self.dpdv[dim[1]]]
            ];
            let bx = [px[dim[0] as u8] - self.p[dim[0] as u8], px[dim[1] as u8] - self.p[dim[1] as u8]];
            let by = [py[dim[0] as u8] - self.p[dim[0] as u8], py[dim[1] as u8] - self.p[dim[1] as u8]];
            { // so mutex write guard gets dropped straight away
                let mut dudx = self.dudx.write().unwrap();
                let mut dvdx = self.dvdx.write().unwrap();
                if !solve_linear_system2x2(a, bx, &mut dudx, &mut dvdx) {
                    *dudx = 0.0;
                    *dvdx = 0.0;
                }
            }
            { // so mutex write guard gets dropped straight away
                let mut dudy = self.dudy.write().unwrap();
                let mut dvdy = self.dvdy.write().unwrap();
                if !solve_linear_system2x2(a, by, &mut dudy, &mut dvdy) {
                    *dudy = 0.0;
                    *dvdy = 0.0;
                }
            }
        } else {
            let mut dudx = self.dudx.write().unwrap();
            *dudx = 0.0;
            let mut dudy = self.dudy.write().unwrap();
            *dudy = 0.0;
            let mut dvdx = self.dvdx.write().unwrap();
            *dvdx = 0.0;
            let mut dvdy = self.dvdy.write().unwrap();
            *dvdy = 0.0;
            let mut dpdx = self.dpdx.write().unwrap();
            *dpdx = Vector3f::default();
            let mut dpdy = self.dpdy.write().unwrap();
            *dpdy = Vector3f::default();
        }
    }
}

impl<'a> Interaction for SurfaceInteraction<'a> {
    fn get_p(&self) -> Point3f {
        self.p
    }

    fn get_time(&self) -> Float {
        self.time
    }

    fn get_p_error(&self) -> Vector3f {
        self.p_error
    }

    fn get_wo(&self) -> Vector3f {
        self.wo
    }

    fn get_n(&self) -> Normal3f {
        self.n
    }

    fn get_medium_interface(&self) -> Option<Arc<MediumInterface>> {
        self.medium_interface
    }

    fn get_bsdf(&self) -> Option<BSDF> {
        self.bsdf
    }

    fn get_shading(&self) -> Option<Shading> {
        Some(self.shading)
    }
}

impl<'a> Clone for SurfaceInteraction<'a> {
    fn clone(&self) -> Self {
        SurfaceInteraction {
            dpdx: RwLock::new(*self.dpdx.read().unwrap()),
            dpdy: RwLock::new(*self.dpdy.read().unwrap()),
            dudx: RwLock::new(*self.dudx.read().unwrap()),
            dudy: RwLock::new(*self.dudy.read().unwrap()),
            dvdx: RwLock::new(*self.dvdx.read().unwrap()),
            dvdy: RwLock::new(*self.dvdy.read().unwrap()),
            ..*self
        }
    }
}