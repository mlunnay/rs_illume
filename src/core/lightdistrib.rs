use super::pbrt::Float;
use super::geometry::{Bounding3, Bounds3f, Point2f, Point3f, Point3i, Vector3f, Normal3f};
use super::sampling::Distribution1D;
use super::scene::Scene;
use super::integrator::compute_light_power_distribution;
use super::low_discrepancy::radical_inverse;
use super::interaction::SimpleInteraction;
use super::medium::MediumInterface;
use super::light::VisibilityTester;
use super::stats_accumulator::StatsAccumulator;
use super::profiler::Profiler;
use std::sync::{Arc, /* RwLock ,*/ atomic::{AtomicU64, Ordering}};
use parking_lot::RwLock;
use num::clamp;

/// LightDistribution defines a general interface for classes that provide
/// probability distributions for sampling light sources at a given point in
/// space.
pub trait LightDistribution {
    /// Given a point |p| in space, this method returns a (hopefully
    /// effective) sampling distribution for light sources at that point.
    fn lookup(&self, p: &Point3f) -> Arc<Distribution1D>;
}

/// The simplest possible implementation of LightDistribution: this returns
/// a uniform distribution over all light sources, ignoring the provided
/// point. This approach works well for very simple scenes, but is quite
/// ineffective for scenes with more than a handful of light sources. (This
/// was the sampling method originally used for the PathIntegrator and the
/// VolPathIntegrator in the printed book, though without the
/// UniformLightDistribution class.)
pub struct UniformLightDistribution {
    distrib: Arc<Distribution1D>
}

impl UniformLightDistribution {
    pub fn new(scene: &Scene) -> UniformLightDistribution {
        let mut prob: Vec<Float> = vec![1.0 as Float; scene.lights.len()];
        UniformLightDistribution {
            distrib: Arc::new(Distribution1D::new(&prob))
        }
    }
}

impl LightDistribution for UniformLightDistribution {
    fn lookup(&self, p: &Point3f) -> Arc<Distribution1D> {
        self.distrib.clone()
    }
}

/// PowerLightDistribution returns a distribution with sampling probability
/// proportional to the total emitted power for each light. (It also ignores
/// the provided point |p|.)  This approach works well for scenes where
/// there the most powerful lights are also the most important contributors
/// to lighting in the scene, but doesn't do well if there are many lights
/// and if different lights are relatively important in some areas of the
/// scene and unimportant in others. (This was the default sampling method
/// used for the BDPT integrator and MLT integrator in the printed book,
/// though also without the PowerLightDistribution class.)
pub struct PowerLightDistribution {
    distrib: Arc<Distribution1D>
}

impl PowerLightDistribution {
    pub fn new(scene: &Scene) -> PowerLightDistribution {
        let distrib = if let Some(d) = compute_light_power_distribution(scene) {
            d
        } else {
            Arc::new(Distribution1D::new(&Vec::<Float>::new()))
        };
        PowerLightDistribution {
            distrib
        }
    }
}

impl LightDistribution for PowerLightDistribution {
    fn lookup(&self, p: &Point3f) -> Arc<Distribution1D> {
        self.distrib.clone()
    }
}

/// A spatially-varying light distribution that adjusts the probability of
/// sampling a light source based on an estimate of its contribution to a
/// region of space.  A fixed voxel grid is imposed over the scene bounds
/// and a sampling distribution is computed as needed for each voxel.
pub struct SpatialLightDistribution<'a> {
    scene: &'a Scene,
    n_voxels: [u32; 3],
    hash_table: Arc<Vec<HashEntry>>
}

// Voxel coordinates are packed into a uint64_t for hash table lookups;
// 10 bits are allocated to each coordinate.  invalidPackedPos is an impossible
// packed coordinate value, which we use to represent
const INVALID_PACKED_POS: u64 = 0xffffffffffffffff;

impl<'a> SpatialLightDistribution<'a> {
    /// Create a new SpatialLightDistribution.
    /// default max_voxels size is 64.
    pub fn new(scene: &'a Scene, max_voxels: usize) -> SpatialLightDistribution {
        // Compute the number of voxels so that the widest scene bounding box
        // dimension has maxVoxels voxels and the other dimensions have a number
        // of voxels so that voxels are roughly cube shaped.
        let b = scene.world_bound();
        let diag = b.diagonal();
        let bmax = diag[b.maximum_extent()];
        let mut n_voxels = [0_u32; 3];
        for i in 0..3 {
            n_voxels[i] = ((diag[i] / bmax * max_voxels as Float).round() as u32).max(1);
            // In the Lookup() method, we require that 20 or fewer bits be
            // sufficient to represent each coordinate value. It's fairly hard
            // to imagine that this would ever be a problem.
            assert!(n_voxels[i] < 1 << 20);
        }

        let hash_table_size = (4 * n_voxels[0] * n_voxels[1] * n_voxels[2]) as usize;
        let hash_table: Vec<HashEntry> = Vec::with_capacity(hash_table_size);
        for i in 0..hash_table_size {
            hash_table.push(HashEntry {
                packed_pos: AtomicU64::new(INVALID_PACKED_POS),
                distribution: RwLock::new(None)
            });
        }

        info!("SpatialLightDistribution: scene bounds {}, vexel res ({}, {}, {})", b, n_voxels[0], n_voxels[1], n_voxels[2]);

        SpatialLightDistribution {
            scene,
            n_voxels,
            hash_table: Arc::new(hash_table)
        }
    }

    fn compute_distribution(&self, pi: &Point3i) -> Distribution1D {
        StatsAccumulator::instance().report_counter(String::from("SpatialLightDistribution/Distributions created"), 1);
        StatsAccumulator::instance().report_ratio(String::from("SpatialLightDistribution/Lookups per distribution"), 0, 1);

        // Compute the world-space bounding box of the voxel corresponding to
        // |pi|.
        let p0 = Point3f::new(
            pi[0] as Float / self.n_voxels[0] as Float,
            pi[1] as Float / self.n_voxels[1] as Float,
            pi[2] as Float / self.n_voxels[2] as Float,
        );
        let p1 = Point3f::new(
            pi[0] as Float + 1.0 / self.n_voxels[0] as Float,
            pi[1] as Float + 1.0 / self.n_voxels[1] as Float,
            pi[2] as Float + 1.0 / self.n_voxels[2] as Float,
        );
        let voxel_bounds = Bounds3f::new(self.scene.world_bound().lerp(&p0),
            self.scene.world_bound().lerp(&p1)
        );

        // Compute the sampling distribution. Sample a number of points inside
        // voxelBounds using a 3D Halton sequence; at each one, sample each
        // light source and compute a weight based on Li/pdf for the light's
        // sample (ignoring visibility between the point in the voxel and the
        // point on the light source) as an approximation to how much the light
        // is likely to contribute to illumination in the voxel.
        let n_samples: usize = 128;
        let mut light_contrib = vec![0.0 as Float; self.scene.lights.len()];
        for i in 0..n_samples {
            let po = voxel_bounds.lerp(&Point3f::new(
                radical_inverse(0, i as u64), radical_inverse(1, i as u64), radical_inverse(2, i as u64)
            ));
            let mut intr = Box::new(SimpleInteraction::new(po, 0.0, Vector3f::default(), Vector3f::new(1.0, 0.0, 0.0), Normal3f::default()));
            intr.medium_interface = Some(Arc::new(MediumInterface::default()));
            
            // Use the next two Halton dimensions to sample a point on the
            // light source.
            let u = Point2f::new(radical_inverse(3, i as u64), radical_inverse(4, i as u64));
            for j in 0..self.scene.lights.len() {
                let mut pdf: Float = 0.0;
                let mut wi = Vector3f::default();
                let mut vis = VisibilityTester::default();
                let li = self.scene.lights[j].sample_li(intr, &u, &mut wi, &mut pdf, &mut vis);
                if pdf > 0.0 {
                    // TODO: look at tracing shadow rays / computing beam
                    // transmittance.  Probably shouldn't give those full weight
                    // but instead e.g. have an occluded shadow ray scale down
                    // the contribution by 10 or something.
                    light_contrib[j] += li.y() / pdf;
                }
            }
        }

        // We don't want to leave any lights with a zero probability; it's
        // possible that a light contributes to points in the voxel even though
        // we didn't find such a point when sampling above.  Therefore, compute
        // a minimum (small) weight and ensure that all lights are given at
        // least the corresponding probability.
        let sum_contrib = light_contrib.iter().sum();
        let avg_contrib = sum_contrib / (n_samples + light_contrib.len()) as Float;
        let min_contrib = if avg_contrib > 0.0 { 0.001 * avg_contrib } else { 1.0 };
        for i in 0..light_contrib.len() {
            vlog!(2, "Voxel pi = {}, light {} contrib = {}", pi, i, light_contrib[i]);
            light_contrib[i] = light_contrib[i].max(min_contrib);
        }
        info!("Initialized light distribution in voxel pi= {}, avgContrib = {}", pi, avg_contrib);

        // Compute a sampling distribution from the accumulated contributions.
        Distribution1D::new(&light_contrib)
    }
}

impl<'a> LightDistribution for SpatialLightDistribution<'a> {
    fn lookup(&self, p: &Point3f) -> Arc<Distribution1D> {
        let _p = Profiler::instance().profile("SpatialLightDistribution lookup");
        StatsAccumulator::instance().report_ratio(String::from("SpatialLightDistribution/Lookups per distribution"), 1, 0);

        // First, compute integer voxel coordinates for the given point |p|
        // with respect to the overall voxel grid.
        let offset = self.scene.world_bound().offset(p);
        let mut pi = Point3i::default();
        for i in 0..3_usize {
            // The clamp should almost never be necessary, but is there to be
            // robust to computed intersection points being slightly outside
            // the scene bounds due to floating-point roundoff error.
            pi[i as u8] = clamp((offset[i as u8] * self.n_voxels[i] as Float) as i32, 0, self.n_voxels[i] as i32 - 1);
        }

        // Pack the 3D integer voxel coordinates into a single 64-bit value.
        let packed_pos = ((pi[0] as u64) << 40) | ((pi[0] as u64) << 20) | pi[2] as u64;
        assert_ne!(packed_pos, INVALID_PACKED_POS);

        // Compute a hash value from the packed voxel coordinates.  We could
        // just take packedPos mod the hash table size, but since packedPos
        // isn't necessarily well distributed on its own, it's worthwhile to do
        // a little work to make sure that its bits values are individually
        // fairly random. For details of and motivation for the following, see:
        // http://zimbry.blogspot.ch/2011/09/better-bit-mixing-improving-on.html
        let mut hash = packed_pos;
        hash ^= hash >> 31;
        hash *= 0x7fb5d329728ea185;
        hash ^= hash >> 27;
        hash *= 0x81dadef4bc2dd44d;
        hash ^= hash >> 33;
        hash %= self.hash_table.len() as u64;
        assert!(hash >= 0);

        // Now, see if the hash table already has an entry for the voxel. We'll
        // use quadratic probing when the hash table entry is already used for
        // another value; step stores the square root of the probe step.
        let mut step: u64 = 1;
        let mut n_probes: usize = 0;
        loop {
            n_probes += 1;
            let mut entry = &self.hash_table[hash as usize];
            // Does the hash table entry at offset |hash| match the current point?
            let entry_packed_pos = entry.packed_pos.load(Ordering::Acquire);
            if entry_packed_pos == packed_pos {
                // Yes! Most of the time, there should already by a light
                // sampling distribution available.
                // this differs from the C++ code as the RwLock should ensure that 
                // the hash_table entries distribution is created.
                // It also means we cant profile the spin lock.
                // let option: Option<Arc<Distribution1D>> = entry.distribution.read().unwrap();
                if let Some(dist) = *entry.distribution.read() {
                    return dist.clone();
                };
            } else if entry_packed_pos != INVALID_PACKED_POS {
                // The hash table entry we're checking has already been
                // allocated for another voxel. Advance to the next entry with
                // quadratic probing.
                hash += step * step;
                if hash >= self.hash_table.len() as u64 {
                    hash %= self.hash_table.len() as u64;
                }
                step += 1;
            } else {
                // We have found an invalid entry. (Though this may have
                // changed since the load into entryPackedPos above.)  Use an
                // atomic compare/exchange to try to claim this entry for the
                // current position.
                let invalid = INVALID_PACKED_POS;
                if entry.packed_pos.compare_exchange_weak(invalid, packed_pos, Ordering::SeqCst, Ordering::Relaxed).is_ok() {
                    // Success; we've claimed this position for this voxel's
                    // distribution. Now compute the sampling distribution and
                    // add it to the hash table. As long as packedPos has been
                    // set but the entry's distribution pointer is nullptr, any
                    // other threads looking up the distribution for this voxel
                    // will spin wait until the distribution pointer is
                    // written.
                    let dist = Arc::new(self.compute_distribution(&pi));
                    let mut distribution = entry.distribution.write();
                    *distribution = Some(dist.clone());
                    return dist;
                }
            }
        }
    }
}

#[derive(Debug)]
struct HashEntry {
    packed_pos: AtomicU64,
    distribution: RwLock<Option<Arc<Distribution1D>>>
}