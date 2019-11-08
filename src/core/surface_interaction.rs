use super::pbrt::Float;
use super::interaction::{Interaction};
use super::medium::MediumInterface;
use super::shape::Shape;
use super::primitive::Primitive;
use super::bssrdf::BSSRDF;
use super::reflection::BSDF;
use super::geometry::{Point2f, Point3f, Vector3f, Normal3f};
use std::sync::{Arc, RwLock};

#[derive(Default)]
pub struct SurfaceInteraction<'a> {
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
    pub shape: Option<&'a (dyn Shape + Send + Sync)>,
    pub shading: Shading,
    pub primitive: Option<&'a Primitive>,
    pub bsdf: Option<BSDF>,
    pub bssrdf: Option<BSSRDF>,
    pub dpdx: Arc<RwLock<Vector3f>>, 
    pub dpdy: Arc<RwLock<Vector3f>>,
    pub dudx: Arc<RwLock<Float>>,
    pub dvdx: Arc<RwLock<Float>>,
    pub dudy: Arc<RwLock<Float>>,
    pub dvdy: Arc<RwLock<Float>>,
    pub face_index: u32
}

impl<'a> SurfaceInteraction<'a> {
    pub fn new(
        p: &Point3f,
        p_error: &Vector3f,
        uv: &Point2f,
        wo: &Vector3f,
        dpdu: &Vector3f,
        dpdv: &Vector3f,
        dndu: &Normal3f,
        dndv: &Normal3f,
        time: Float,
        sh: Option<&'a (dyn Shape + Send + Sync)>,
        face_index: u32
    ) -> SurfaceInteraction<'a> {
        let nv: Vector3f = dpdu.cross(dpdv).normalize();
        let mut n: Normal3f = Normal3f {
            x: nv.x,
            y: nv.y,
            z: nv.z,
        };
        // initialize shading geometry from true geometry
        let mut shading: Shading = Shading {
            n,
            dpdu: Vector3f::from(*dpdu),
            dpdv: Vector3f::from(*dpdv),
            dndu: *dndu,
            dndv: *dndv,
        };
        if let Some(ref shape) = sh {
            // adjust normal based on orientation and handedness
            if shape.get_reverse_orientation() ^ shape.get_transform_swaps_handedness() {
                n *= -1.0 as Float;
                shading.n *= -1.0 as Float;
            }
        }
        if let Some(ref shape) = sh {
            SurfaceInteraction {
                p: *p,
                time,
                p_error: *p_error,
                wo: wo.normalize(),
                n,
                medium_interface: None,
                uv: *uv,
                dpdu: *dpdu,
                dpdv: *dpdv,
                dndu: *dndu,
                dndv: *dndv,
                dpdx: Arc::new(RwLock::new(Vector3f::default())),
                dpdy: Arc::new(RwLock::new(Vector3f::default())),
                dudx: Arc::new(RwLock::new(0.0 as Float)),
                dvdx: Arc::new(RwLock::new(0.0 as Float)),
                dudy: Arc::new(RwLock::new(0.0 as Float)),
                dvdy: Arc::new(RwLock::new(0.0 as Float)),
                primitive: None,
                shading,
                bsdf: None,
                bssrdf: None,
                shape: Some(shape.clone()),
                face_index
            }
        } else {
            SurfaceInteraction {
                p: *p,
                time,
                p_error: *p_error,
                wo: wo.normalize(),
                n,
                medium_interface: None,
                uv: *uv,
                dpdu: *dpdu,
                dpdv: *dpdv,
                dndu: *dndu,
                dndv: *dndv,
                dpdx: Arc::new(RwLock::new(Vector3f::default())),
                dpdy: Arc::new(RwLock::new(Vector3f::default())),
                dudx: Arc::new(RwLock::new(0.0 as Float)),
                dvdx: Arc::new(RwLock::new(0.0 as Float)),
                dudy: Arc::new(RwLock::new(0.0 as Float)),
                dvdy: Arc::new(RwLock::new(0.0 as Float)),
                primitive: None,
                shading,
                bsdf: None,
                bssrdf: None,
                shape: None,
                face_index
            }
        }
    }

    pub fn set_shading_geometry(
        &mut self,
        dpdus: &Vector3f,
        dpdvs: &Vector3f,
        dndus: &Normal3f,
        dndvs: &Normal3f,
        orientation_is_authoritative: bool
    ) {
        // Compute _shading.n_ for _SurfaceInteraction_
        self.shading.n = dpdus.cross(dpdvs).normalize().into();
        if orientation_is_authoritative {
            self.n = self.n.face_forward(&self.shading.n);
        }
        else {
            self.shading.n = self.shading.n.face_forward(&self.n);
        }

        // Initialize _shading_ partial derivative values
        self.shading.dpdu = *dpdus;
        self.shading.dpdv = *dpdvs;
        self.shading.dndu = *dndus;
        self.shading.dndv = *dndvs;
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
        self.medium_interface.clone()
    }
}

#[derive(Debug, Default, Copy, Clone)]
pub struct Shading {
    pub n: Normal3f,
    pub dpdu: Vector3f,
    pub dpdv: Vector3f,
    pub dndu: Normal3f,
    pub dndv: Normal3f
}