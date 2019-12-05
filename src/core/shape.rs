use super::pbrt::Float;
use super::geometry::{Bounding3, Vector3f, Ray, Point2f, Point3f, Normal3f, dot_normal_vec};
use super::interaction::SurfaceInteraction;
use super::interaction::Interaction;
use super::low_discrepancy::radical_inverse;

/// Interface for Shape objects.
pub trait Shape: Send + Sync {
    /// Returns the bounding shape in object coordinates.
    fn object_bound(&self) -> Box<dyn Bounding3<Float>>;
    
    /// Returns the bounding shape in world coordinates.
    fn world_bound(&self) -> Box<dyn Bounding3<Float>>;

    /// Should the Shapes orientation be reversed.
    fn get_reverse_orientation(&self) -> bool;

    /// Does the object to world transform swap the handedness of the coordinate system.
    fn get_transform_swaps_handedness(&self) -> bool;
    
    // Calculates if a Ray intersects this Shape. Returns an an Option with a tuple of surface interaction and parametric distance along the ray,
    /// or None if the is no intersection. 
    fn intersect(&self, ray: &Ray, test_alpha_texture: bool) -> Option<(SurfaceInteraction, Float)>;
    
    /// A predicate Ray intersection test. The default implementation just calls intersect and ignores the returned values.
    fn intersect_p(&self, ray: &Ray, test_alpha_texture: bool) -> bool {
        self.intersect(ray, test_alpha_texture).is_some()
    }

    /// Returns the surface area of the Shape.
    fn area(&self) -> Float;

    /// Returns surface information based on surface area distrubution.
    fn sample(&self, u: &Point2f, pdf: &mut Float) -> Box<dyn Interaction>;

    /// Return the pdf for the Shape. The default implementation is 1 over the surface area.
    fn pdf(&self, _iref: &dyn Interaction) -> Float {
        1.0 / self.area()
    }

    /// Sample a point on the shape given a reference point iref and
    /// return the PDF with respect to solid angle from iref.
    fn sample_with_point(&self, iref: &dyn Interaction, u: &Point2f, pdf: &mut Float) -> Box<dyn Interaction> {
        let intr = self.sample(&u, &mut pdf);
        let wi = intr.get_p() - iref.get_p();
        if wi.length_squared() == 0.0 {
            *pdf = 0.0;
        }
        else {
            wi = wi.normalize();
            // Convert from area measure, as returned by the Sample() call
            // above, to solid angle measure.
            *pdf *= iref.get_p().distance_squared(&intr.get_p()) / intr.get_n().dot(&Normal3f::from(-wi)).abs();
            if pdf.is_infinite() { *pdf = 0.0; }
        }
        intr
    }

    /// Sample density over solid angle.
    fn pdf_with_point(&self, iref: &dyn Interaction, wi: &Vector3f) -> Float {
        // Intersect sample ray with area light geometry
        let ray = iref.spawn_ray(&wi);
        let t_hit: Float;
        let isect_light: SurfaceInteraction;
        // Ignore any alpha textures used for trimming the shape when performing
        // this intersection. Hack for the "San Miguel" scene, where this is used
        // to make an invisible area light.
        match self.intersect(&ray, false) {
            Some((si, hit)) => {
                t_hit = hit;
                isect_light = si;
            }
            None => { return 0.0; }
        }

        // Convert light sample weight to solid angle measure
        let pdf = iref.get_p().distance_squared(&isect_light.p) /
                    dot_normal_vec(&isect_light.n, &wi).abs() * self.area();
        if pdf.is_infinite() { pdf = 0.0; }
        pdf
    }

    /// Returns the solid angle subtended by the shape w.r.t. the reference
    /// point p, given in world space. Some shapes compute this value in
    /// closed-form, while the default implementation uses Monte Carlo
    /// integration; the nSamples parameter determines how many samples are
    /// used in this case.
    fn solid_angle(&self, p: &Point3f, n_samples: i32) -> Float {
        let iref = SurfaceInteraction::new(*p,
            Vector3f::default(),
            Point2f::default(),
            Vector3f::new(0.0, 0.0, 1.0),
            Vector3f::default(),
            Vector3f::default(),
            Normal3f::default(),
            Normal3f::default(),
            0.0,
            None,
            0);
        let solid_angle = 0.0_f64;
        for i in 0..n_samples {
            let u = Point2f::new(radical_inverse(0, i as u64), radical_inverse(1, i as u64));
            let mut pdf: Float;
            let p_shape = self.sample_with_point(&iref, &u, &mut pdf);
            let r = Ray::new(*p, p_shape.get_p() - *p);
            r.t_max = 0.999;
            if pdf > 0.0 && !self.intersect_p(&r, true) {
                solid_angle += 1.0 / pdf as f64;
            }
        }
        num::cast(solid_angle / n_samples as f64).unwrap()
    }
}