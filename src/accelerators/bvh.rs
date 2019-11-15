use crate::core::pbrt::Float;
use crate::core::primitive::Primitive;
use crate::core::light::Light;
use crate::core::material::{Material, TransportMode};
use crate::core::surface_interaction::SurfaceInteraction;
use crate::core::profiler::Profiler;
use crate::core::stats_accumulator::StatsAccumulator;
use crate::core::geometry::{Bounds3f, Point3f, Ray, Vector3f, Bounding3};
use crate::core::utils::slice_extension::SliceExtension;
use std::sync::{Arc, atomic::{AtomicUsize, Ordering}};
use obstack::{Obstack};
use std::mem;
use rayon::prelude::*;
use std::ptr;

#[derive(Debug, Clone, PartialEq)]
pub enum SplitMethod {
    SAH,
    HLBVH,
    Middle,
    EqualCounts
}

pub struct BVHAccel {
    max_prims_in_node: u8,  // show intent as maxPrimsInNode has a max value of 255
    split_method: SplitMethod,
    primitives: Vec<Arc<dyn Primitive>>,
    nodes: Vec<LinearBVHNode>
}

impl BVHAccel {
    pub fn new(
        p: Vec<Arc<dyn Primitive>>,
        max_prims_in_node: u8,
        split_method: SplitMethod
    ) -> BVHAccel {
        let _guard = Profiler::instance().profile("Acceleration structure creation");
        let num_primitives = p.len();
        
        if num_primitives == 0 {
            return BVHAccel {
                max_prims_in_node,
                split_method,
                primitives: p,
                nodes: Vec::new()
            };
        }

        // Initialize _primitiveInfo_ array for primitives
        let mut primitive_info: Vec<BVHPrimitiveInfo> = Vec::with_capacity(num_primitives);
        for i in 0..num_primitives {
            primitive_info.push(BVHPrimitiveInfo::new(i, p[i].world_bound()));
        }

        // Build BVH tree for primitives using _primitiveInfo_
        let mut arena = Obstack::with_initial_capacity(1024 * 1024);
        let mut total_nodes = 0;
        let mut ordered_prims: Vec<Arc<dyn Primitive>> = Vec::with_capacity(num_primitives);

        let root = if split_method == SplitMethod::HLBVH {
            BVHAccel::hlbvh_build(max_prims_in_node, &p,
                &mut arena, &mut primitive_info, &mut total_nodes,
                &mut ordered_prims)
        } else {
            BVHAccel::recursive_build(max_prims_in_node, split_method, &p,
                &mut arena, &mut primitive_info, 0,
                num_primitives, &mut total_nodes, &mut ordered_prims)
        };

        info!("BVH created with {} nodes for {} primitives ({:.2} MB), arena allocated {:.2} MB",
            total_nodes,
            num_primitives,
            (total_nodes * mem::size_of::<LinearBVHNode>()) as f32 / (1024.0 * 1024.0),
            arena.bytes_used() as f32 / (1024.0 * 1024.0)
        );

        let tree_bytes = total_nodes * mem::size_of::<LinearBVHNode>() +
            mem::size_of::<BVHAccel>() + num_primitives * mem::size_of_val(&*p[0]);
        StatsAccumulator::instance().report_memory_counter(String::from("Memory/BVH tree"), tree_bytes as i64);

        let mut nodes: Vec<LinearBVHNode> = vec![LinearBVHNode::default(); total_nodes];
        let mut offset = 0;

        BVHAccel::flatten_bvh_tree(root, &mut nodes, &mut offset);
        assert!(offset == total_nodes);

        BVHAccel {
            max_prims_in_node,
            split_method,
            primitives: ordered_prims,
            nodes: nodes
        }
    }

    pub fn recursive_build<'a>(
        max_prims_in_node: u8,
        split_method: SplitMethod,
        primitives: &Vec<Arc<dyn Primitive>>,
        arena: &'a mut Obstack,
        primitive_info: &mut Vec<BVHPrimitiveInfo>,
        start: usize,
        end: usize,
        total_nodes: &mut usize,
        ordered_prims: &mut Vec<Arc<dyn Primitive>>
    ) -> &'a mut BVHBuildNode<'a> {
        assert_ne!(start, end);
        let node = arena.push_copy(BVHBuildNode::new());
        *total_nodes += 1;
        let mut bounds = Bounds3f::default();
        for i in start..end {
            bounds = bounds.union(&*primitive_info[i].bounds).aabb()
        }
        let n_primitives = end - start;
        if n_primitives == 1 {
            // Create leaf _BVHBuildNode_
            let first_prim_offset = ordered_prims.len();
            // removed loop from https://github.com/mmp/pbrt-v3/blob/3f94503ae1777cd6d67a7788e06d67224a525ff4/src/accelerators/bvh.cpp#L251
            // as it will only ever run once as we have already established end - start == 1.
            let prim_num = primitive_info[start].primitive_number;
            ordered_prims.push(primitives[prim_num].clone());
            node.init_leaf(first_prim_offset, n_primitives, &bounds);
            return node;
        } else {
            // Compute bound of primitive centroids, choose split dimension _dim_
            let centroid_bounds = Bounds3f::default();
            for i in start..end {
                centroid_bounds = centroid_bounds.union(&*primitive_info[i].bounds).aabb();
            }
            let dim = centroid_bounds.maximum_extent();
            
            // Partition primitives into two sets and build children
            let mut mid = (start + end) / 2;
            if centroid_bounds.max[dim] == centroid_bounds.min[dim] {
                // Create leaf _BVHBuildNode_
                let first_prim_offset = ordered_prims.len();
                for i in start..end {
                    let prim_num = primitive_info[i].primitive_number;
                    ordered_prims.push(primitives[prim_num].clone());
                }
                node.init_leaf(first_prim_offset, n_primitives, &bounds);
                return node;
            } else {
                // Partition primitives based on _splitMethod_
                match split_method {
                    SplitMethod::Middle => {
                        // Partition primitives through node's midpoint
                        let pmid = (centroid_bounds.min[dim] + centroid_bounds.max[dim]) / 2.0;
                        // mid = partition(&mut primitive_info, start, end, |i| i.centroid[dim] < pmid);
                        mid = primitive_info[start..end].partition(|i| i.centroid[dim] < pmid) + start;
                        // For lots of prims with large overlapping bounding boxes, this
                        // may fail to partition; in that case don't break and do
                        // the same as EqualCounts. (no fallthrough in rust).
                        if mid == start || mid == end {
                            // Partition primitives into equally-sized subsets
                            mid = (start + end) / 2;
                            primitive_info[start..end].partial_sort(mid, |a,b| a.centroid[dim] < b.centroid[dim]);
                        }
                    }
                    SplitMethod::EqualCounts => {
                        // Partition primitives into equally-sized subsets
                        mid = (start + end) / 2;
                        primitive_info[start..end].partial_sort(mid, |a,b| a.centroid[dim] < b.centroid[dim]);
                    }
                    SplitMethod::SAH => {
                        // Partition primitives using approximate SAH
                        if n_primitives <= 2 {
                            // Partition primitives into equally-sized subsets
                            mid = (start + end) / 2;
                            primitive_info[start..end].partial_sort(mid, |a,b| a.centroid[dim] < b.centroid[dim]);
                        } else {
                            // Allocate _BucketInfo_ for SAH partition buckets
                            const n_buckets: usize = 12;
                            let mut buckets = [BucketInfo::default(); n_buckets];
                            // Initialize _BucketInfo_ for SAH partition buckets
                            for i in start..end {
                                let mut b = n_buckets * centroid_bounds.offset(&primitive_info[i].centroid)[dim] as usize;
                                if b == n_buckets {
                                    b = n_buckets - 1;
                                }
                                assert!(b >= 0 && b < n_buckets);
                                buckets[b].count +=  1;
                                buckets[b].bounds = buckets[b].bounds.union(&*primitive_info[i].bounds).aabb();
                            }

                            // Compute costs for splitting after each bucket
                            let mut cost: [Float; n_buckets - 1] = [0.0; n_buckets - 1];
                            for i in 0..n_buckets - 1 {
                                let mut b0 = Bounds3f::default();
                                let mut b1 = Bounds3f::default();
                                let mut count0 = 0_usize;
                                let mut count1 = 0_usize;
                                for j in 0..=i {
                                    b0 = b0.union(&buckets[j].bounds).aabb();
                                    count0 += buckets[j].count;
                                }
                                for j in (i + 1)..n_buckets {
                                    b1 = b1.union(&buckets[j].bounds).aabb();
                                    count1 += buckets[j].count;
                                }
                                cost[i] = 1.0 + (count0 as Float * b0.surface_area() +
                                    count1 as Float * b1.surface_area()) / bounds.surface_area();
                            }

                            // Find bucket to split at that minimizes SAH metric
                            let min_cost = cost[0];
                            let min_cost_split_bucket = 0_usize;
                            for i in 1..n_buckets - 1 {
                                if cost[i] < min_cost {
                                    min_cost = cost[i];
                                    min_cost_split_bucket = i;
                                }
                            }

                            // Either create leaf or split primitives at selected SAH
                            // bucket
                            let leaf_cost = n_primitives as Float;
                            if n_primitives > max_prims_in_node as usize || min_cost < leaf_cost {
                                mid = primitive_info[start..end].partition(|pi| {
                                    let b = n_buckets * centroid_bounds.offset(&pi.centroid)[dim] as usize;
                                    if b == n_buckets {
                                        b = n_buckets - 1;
                                    }
                                    assert!(b >= 0 && b < n_buckets);
                                    b <= min_cost_split_bucket
                                }) + start;
                            } else {
                                // Create leaf _BVHBuildNode_
                                let first_prim_offset = ordered_prims.len();
                                for i in start..end {
                                    let prim_num = primitive_info[i].primitive_number;
                                    ordered_prims.push(primitives[prim_num].clone());
                                }
                                node.init_leaf(first_prim_offset, n_primitives, &bounds);
                                return node;
                            }
                        }
                    }
                }

                node.init_interior(dim,
                    BVHAccel::recursive_build(max_prims_in_node, split_method, primitives, arena, primitive_info, start, mid, total_nodes, ordered_prims),
                    BVHAccel::recursive_build(max_prims_in_node, split_method, primitives, arena, primitive_info, mid, end, total_nodes, ordered_prims)
                );
            }
        }
        node
    }

    pub fn hlbvh_build<'a>(
        max_prims_in_node: u8,
        primitives: &Vec<Arc<dyn Primitive>>,
        arena: &'a mut Obstack,
        primitive_info: &Vec<BVHPrimitiveInfo>,
        total_nodes: &mut usize,
        ordered_prims: &mut Vec<Arc<dyn Primitive>>
    ) -> &'a mut BVHBuildNode<'a> {
        // Compute bounding box of all primitive centroids
        let mut bounds = Bounds3f::default();
        for pi in primitive_info {
            bounds = bounds.union(&Bounds3f::from_point(pi.centroid)).aabb();
        }
        let v = vec![0_u8; 100];
        v.par_iter();
        // Compute Morton indices of primitives
        let mut morton_prims: Vec<MortonPrimitive> = primitive_info.par_iter()
            .map(|pi| {
                let centroid_offset = bounds.offset(&pi.centroid);
                MortonPrimitive {
                    primitive_index: pi.primitive_number,
                    morton_code: encode_morton_3(&(centroid_offset * morton_scale as Float))
                }
            }).collect();

        // Radix sort primitive Morton indices
        radix_sort(&mut morton_prims);

        // Create LBVH treelets at bottom of BVH

        // Find intervals of primitives for each treelet
        let mut treelets_to_build: Vec<LBVHTreelet> = Vec::new();
        let mut start = 0_usize;
        for end in 1..=morton_prims.len() {
            const mask: u32 = 0b00111111111111000000000000000000;
            if end == morton_prims.len() ||
                morton_prims[start].morton_code & mask !=
                morton_prims[end].morton_code & mask {
                // Add entry to _treeletsToBuild_ for this treelet
                let n_primitives = end - start;
                let max_bvh_nodes = 2 * n_primitives;
                let mut nodes: Vec<&'a mut BVHBuildNode<'a>> = Vec::with_capacity(max_bvh_nodes);
                for i in 0..max_bvh_nodes {
                    nodes.push(arena.push_copy(BVHBuildNode::new()));
                }
                treelets_to_build.push(LBVHTreelet {
                    start_index: start,
                    n_primitives,
                    build_nodes: nodes
                });
                start = end;
            }
        }
        
        // Create LBVHs for treelets in parallel
        let mut atomic_total: AtomicUsize = AtomicUsize::new(0);
        let mut ordered_prims_offset: AtomicUsize = AtomicUsize::new(0);
        treelets_to_build.par_iter_mut().for_each(|tr| {
            // Generate _i_th LBVH treelet
            let nodes_created = 0_usize;
            const first_bit_index: i32 = 29 - 12;
            BVHAccel::emit_lbvh(max_prims_in_node, primitives,
                &tr.build_nodes.into_iter(), &primitive_info, &morton_prims[tr.start_index..],
                tr.n_primitives, &mut nodes_created, ordered_prims,
                ordered_prims_offset, first_bit_index);
            atomic_total.fetch_add(nodes_created, Ordering::SeqCst);
        });
        *total_nodes = atomic_total.load(Ordering::Acquire);

        // Create and return SAH BVH from LBVH treelets
        let mut finished_treelets: Vec<&'a mut BVHBuildNode<'a>> = Vec::with_capacity(treelets_to_build.len());
        for treelet in treelets_to_build {
            finished_treelets.push(treelet.build_nodes[0]);
        }

        return BVHAccel::build_upper_sah(&mut arena, &finished_treelets, 0,
            finished_treelets.len(), &mut total_nodes);
    }

    pub fn emit_lbvh<'a, T>(
        max_prims_in_node: u8,
        primitives: &Vec<Arc<dyn Primitive>>,
        // build_nodes: &mut obstack::Ref<'a, Vec<&'a BVHBuildNode<'a>>>,
        // build_nodes: &'b mut [&'a mut BVHBuildNode<'a>],
        build_nodes: &T,
        primitive_info: &Vec<BVHPrimitiveInfo>,
        morton_prims: &[MortonPrimitive],
        n_primitives: usize,
        total_nodes: &mut usize,
        ordered_prims: &mut Vec<Arc<dyn Primitive>>,
        ordered_prims_offset: AtomicUsize,
        bit_index: i32
    ) -> &'a mut BVHBuildNode<'a>
    where
    T: Iterator<Item = &'a mut BVHBuildNode<'a>>
    {
        assert!(n_primitives > 0);
        if bit_index == -1 || n_primitives < max_prims_in_node as usize {
            // Create and return leaf node of LBVH treelet
            *total_nodes += 1;
            let mut node = build_nodes.next().unwrap();
            let mut bounds = Bounds3f::default();
            let first_prim_offset = ordered_prims_offset.fetch_add(n_primitives, Ordering::SeqCst);
            for i in 0..n_primitives {
                let primitive_index = morton_prims[i].primitive_index;
                ordered_prims[first_prim_offset + i] = primitives[primitive_index];
                bounds = bounds.union(&*primitive_info[primitive_index].bounds).aabb();
            }
            node.init_leaf(first_prim_offset, n_primitives, &bounds);
            return node;
        } else {
            let mask = 1 << bit_index;
            // Advance to next subtree level if there's no LBVH split for this bit
            if morton_prims[0].morton_code & mask ==
                morton_prims[n_primitives - 1].morton_code & mask {
                return BVHAccel::emit_lbvh(max_prims_in_node, &primitives,
                    build_nodes, primitive_info, morton_prims, n_primitives,
                    &mut total_nodes, &mut ordered_prims, ordered_prims_offset,
                    bit_index - 1);
            }

            // Find LBVH split point for this dimension
            let mut search_start = 0_usize;
            let mut search_end = n_primitives - 1;
            while search_start + 1 != search_end {
                let mid = (search_start + search_end) / 2;
                if morton_prims[search_start].morton_code & mask ==
                    morton_prims[mid].morton_code & mask {
                    search_start = mid;
                } else {
                    assert_eq!(morton_prims[mid].morton_code & mask,
                        morton_prims[search_end].morton_code & mask);
                    search_end = mid;
                }
            }

            let split_offset = search_end;
            assert!(split_offset <= n_primitives - 1);
            assert_ne!(morton_prims[split_offset - 1].morton_code & mask,
                morton_prims[split_offset].morton_code & mask);

            // Create and return interior LBVH node
            *total_nodes += 1;
            let mut node = build_nodes.next().unwrap();
            let c1 = BVHAccel::emit_lbvh(max_prims_in_node, &primitives,
                build_nodes, primitive_info, &morton_prims[..split_offset], split_offset,
                &mut total_nodes, &mut ordered_prims, ordered_prims_offset,
                bit_index - 1);
            let c2 = BVHAccel::emit_lbvh(max_prims_in_node, &primitives,
                build_nodes, primitive_info, &morton_prims[split_offset..], n_primitives - split_offset,
                &mut total_nodes, &mut ordered_prims, ordered_prims_offset,
                bit_index - 1);
            let axis = (bit_index % 3) as u8;
            node.init_interior(axis, c1, c2);
            return node;
        }
    }

    pub fn build_upper_sah<'a>(
        arena: &'a mut Obstack,
        treelet_roots: &'a Vec<&'a mut BVHBuildNode<'a>>,
        start: usize,
        end: usize,
        total_nodes: &mut usize
    ) -> &'a mut BVHBuildNode<'a> {
        assert!(start < end);
        let n_nodes = end - start;
        if n_nodes == 1 {
            return treelet_roots[start];
        }
        *total_nodes += 1;
        let node = arena.push_copy(BVHBuildNode::new());

        // Compute bounds of all nodes under this HLBVH node
        let mut bounds = Bounds3f::default();
        for i in start..end {
            bounds = bounds.union(&treelet_roots[i].bounds).aabb();
        }

        // Compute bound of HLBVH node centroids, choose split dimension _dim_
        let mut centroid_bounds = Bounds3f::default();
        for i in start..end {
            let centroid = (treelet_roots[i].bounds.min + treelet_roots[i].bounds.max) * 0.5;
            centroid_bounds = centroid_bounds.union(&Bounds3f::from_point(centroid)).aabb();
        }
        let dim = centroid_bounds.maximum_extent();
        // FIXME: if this hits, what do we need to do?
        // Make sure the SAH split below does something... ?
        assert_ne!(centroid_bounds.max[dim], centroid_bounds.min[dim]);

        // Allocate _BucketInfo_ for SAH partition buckets
        const n_buckets: usize = 12;
        let mut buckets = [BucketInfo::default(); n_buckets];

        // Initialize _BucketInfo_ for HLBVH SAH partition buckets
        for i in start..end {
            let centroid = (treelet_roots[i].bounds.min[dim] +
                treelet_roots[i].bounds.max[dim]) * 0.5;
            let mut b = n_buckets * ((centroid - centroid_bounds.min[dim]) / 
                centroid_bounds.max[dim] - centroid_bounds.min[dim]) as usize;
            if b == n_buckets {
                b = n_buckets - 1;
            }
            assert!(b >= 0 && b < n_buckets);
            buckets[b].count += 1;
            buckets[b].bounds = buckets[b].bounds.union(&treelet_roots[i].bounds).aabb();
        }

        // Compute costs for splitting after each bucket
        let mut cost: [Float; n_buckets] = [0.0; n_buckets];
        for i in 0..n_buckets - 1 {
            let mut b0 = Bounds3f::default();
            let mut b1 = Bounds3f::default();
            let mut count0 = 0_usize;
            let mut count1 = 0_usize;
            for j in 0..=i {
                b0 = b0.union(&buckets[j].bounds).aabb();
                count0 += buckets[j].count;
            }
            for j in (i + 1)..n_buckets {
                b1 = b1.union(&buckets[j].bounds).aabb();
                count1 += buckets[j].count;
            }
            cost[i] = 1.0 + (count0 as Float * b0.surface_area() +
                count1 as Float * b1.surface_area()) / bounds.surface_area();
        }

        // Find bucket to split at that minimizes SAH metric
        let min_cost = cost[0];
        let min_cost_split_bucket = 0_usize;
        for i in 1..n_buckets - 1 {
            if cost[i] < min_cost {
                min_cost = cost[i];
                min_cost_split_bucket = i;
            }
        }

        // Split nodes and create interior HLBVH SAH node
        let mid = treelet_roots[start..end].partition(|node| {
            let centroid = (node.bounds.min[dim] + node.bounds.max[dim]) * 0.5;
            let mut b = n_buckets * ((centroid - centroid_bounds.min[dim]) / 
                centroid_bounds.max[dim] - centroid_bounds.min[dim]) as usize;
            if b == n_buckets {
                b = n_buckets - 1;
            }
            assert!(b >= 0 && b < n_buckets);
            b <= min_cost_split_bucket
        });
        assert!(mid > start && mid < end);
        node.init_interior(dim,
            BVHAccel::build_upper_sah(&mut arena, treelet_roots, start, mid, total_nodes),
            BVHAccel::build_upper_sah(&mut arena, treelet_roots, mid, end, total_nodes)
        );
        node
    }

    pub fn flatten_bvh_tree<'a>(
        node: &mut BVHBuildNode<'a>,
        nodes: &mut Vec<LinearBVHNode>,
        offset: &mut usize
    ) -> usize {
        let my_offset = *offset;
        *offset += 1;
        if node.n_primitives > 0 {
            assert!(node.child1.is_none() && node.child2.is_none());
            assert!(node.n_primitives < 65536);
            nodes[my_offset] = LinearBVHNode {
                bounds: node.bounds,
                offset: node.first_prim_offset as i32,
                n_primitives: node.n_primitives as u16,
                axis: 0_u8,
                pad: 0_u8
            };
        } else {
            // Create interior flattened BVH node
            if let Some(ref mut child1) = node.child1 {
                BVHAccel::flatten_bvh_tree(child1, nodes, offset);
            }
            if let Some(ref mut child2) = node.child2 {
                nodes[my_offset] = LinearBVHNode {
                    bounds: node.bounds,
                    offset: BVHAccel::flatten_bvh_tree(child2, nodes, offset) as i32,
                    n_primitives: 0_u16,
                    axis: node.split_axis,
                    pad: 0_u8
                };
            }
        }
        my_offset
    }
}

impl Primitive for BVHAccel {
    fn get_area_light(&self) -> Option<Arc<dyn Light>> {
        error!("BVHAccell::get_area_light() called; should have gone to GeometricPrimitive.");
        None
    }

    fn get_material(&self) -> Option<Arc<dyn Material>> {
        error!("BVHAccell::get_material() called; should have gone to GeometricPrimitive.");
        None
    }

    fn compute_scattering_function(&self, arena: &Obstack, isect: &mut SurfaceInteraction, mode: TransportMode, allow_multiple_lobes: bool) {
        error!("BVHAccell::compute_scattering_function() called; should have gone to GeometricPrimitive.");
    }

    fn world_bound(&self) -> Box<dyn Bounding3<Float>> {
        if self.nodes.len() > 0 {
            Box::new(self.nodes[0].bounds)
        } else {
            Box::new(Bounds3f::default())
        }
    }

    fn intersect(&self, ray: &Ray) -> Option<SurfaceInteraction> {
        if self.nodes.len() == 0 {
            return None;
        }
        let _guard = Profiler::instance().profile("BVHAccel::intersect()");
        let inv_dir = Vector3f::new(1.0 / ray.d.x, 1.0 / ray.d.y, 1.0 / ray.d.z);
        let dir_is_neg = [(inv_dir.x < 0.0) as usize, (inv_dir.y < 0.0) as usize, (inv_dir.z < 0.0) as usize];
        // Follow ray through BVH nodes to find primitive intersections
        let mut to_visit_offset: usize = 0;
        let mut current_node_index: usize = 0;
        let mut nodes_to_visit = [0_usize; 64];
        let mut si: Option<SurfaceInteraction> = None;
        loop {
            let node = &self.nodes[current_node_index];
            // Check ray against BVH node
            if node.bounds.intersect_p(ray, &inv_dir, dir_is_neg) {
                if node.n_primitives > 0 {
                    // Intersect ray with primitives in leaf BVH node
                    for i in 0..node.n_primitives as usize {
                        if let Some(isect) = self.primitives[node.offset as usize + i].intersect(ray) {
                            si = Some(isect);
                        }
                    }
                    if to_visit_offset == 0 {
                        break;
                    }
                    to_visit_offset -= 1;
                    current_node_index = nodes_to_visit[to_visit_offset];
                } else {
                    // Put far BVH node on _nodesToVisit_ stack, advance to near
                    // node
                    if dir_is_neg[node.axis as usize] != 0 {
                        nodes_to_visit[to_visit_offset] = current_node_index + 1;
                        to_visit_offset += 1;
                        current_node_index = node.offset as usize;
                    } else {
                        nodes_to_visit[to_visit_offset] = node.offset as usize;
                        current_node_index += 1;
                    }
                }
            } else {
                if to_visit_offset == 0 {
                    break;
                }
                to_visit_offset -= 1;
                current_node_index = nodes_to_visit[to_visit_offset];
            }
        }
        si
    }

    fn intersect_p(&self, ray: &Ray) -> bool {
        if self.nodes.len() == 0 {
            return false;
        }
        let _guard = Profiler::instance().profile("BVHAccel::intersect_p()");
        let inv_dir = Vector3f::new(1.0 / ray.d.x, 1.0 / ray.d.y, 1.0 / ray.d.z);
        let dir_is_neg = [(inv_dir.x < 0.0) as usize, (inv_dir.y < 0.0) as usize, (inv_dir.z < 0.0) as usize];
        // Follow ray through BVH nodes to find primitive intersections
        let mut to_visit_offset: usize = 0;
        let mut current_node_index: usize = 0;
        let mut nodes_to_visit = [0_usize; 64];
        loop {
            let node = &self.nodes[current_node_index];
            // Check ray against BVH node
            if node.bounds.intersect_p(ray, &inv_dir, dir_is_neg) {
                if node.n_primitives > 0 {
                    // Intersect ray with primitives in leaf BVH node
                    for i in 0..node.n_primitives as usize {
                        if self.primitives[node.offset as usize + i].intersect_p(ray) {
                            return true;
                        }
                    }
                    if to_visit_offset == 0 {
                        break;
                    }
                    to_visit_offset -= 1;
                    current_node_index = nodes_to_visit[to_visit_offset];
                } else {
                    // Put far BVH node on _nodesToVisit_ stack, advance to near
                    // node
                    if dir_is_neg[node.axis as usize] != 0 {
                        nodes_to_visit[to_visit_offset] = current_node_index + 1;
                        to_visit_offset += 1;
                        current_node_index = node.offset as usize;
                    } else {
                        nodes_to_visit[to_visit_offset] = node.offset as usize;
                        current_node_index += 1;
                    }
                }
            } else {
                if to_visit_offset == 0 {
                    break;
                }
                to_visit_offset -= 1;
                current_node_index = nodes_to_visit[to_visit_offset];
            }
        }
        false
    }
}

#[repr(C, align(64))]
#[derive(Debug, Default, Copy, Clone)]
pub struct LinearBVHNode {
    pub bounds: Bounds3f,
    pub offset: i32,  // both offests are the same type so the union is superflous.
    pub n_primitives: u16,  // 0 -> interior node
    pub axis: u8,   // interior node: xyz
    pub pad: u8 // ensure 32 byte total size
}

#[derive(Debug, Copy, Clone)]
pub struct BVHBuildNode<'a> {
    pub bounds: Bounds3f,
    pub child1: Option<&'a BVHBuildNode<'a>>,
    pub child2: Option<&'a BVHBuildNode<'a>>,
    pub split_axis: u8,
    pub first_prim_offset: usize,
    pub n_primitives: usize
}

impl<'a> BVHBuildNode<'a> {
    pub fn new() -> BVHBuildNode<'a> {
        BVHBuildNode{
            bounds: Bounds3f::default(),
            child1: None,
            child2: None,
            split_axis: 0,
            first_prim_offset: 0,
            n_primitives: 0
        }
    }

    pub fn init_leaf(&mut self, first: usize, n: usize, b: &Bounds3f) {
        self.first_prim_offset = first;
        self.n_primitives = n;
        self.bounds = *b;
        self.child1 = None;
        self.child2 = None;
        StatsAccumulator::instance().report_counter(String::from("BVH/Leaf nodes"), 1);
        StatsAccumulator::instance().report_ratio(String::from("BVH/Primitives per leaf node"), n as i64, 1);
    }

    pub fn init_interior(
        &mut self,
        axis: u8,
        c0: &'a mut BVHBuildNode<'a>,
        c1: &'a mut BVHBuildNode<'a>
    ) {
        self.n_primitives = 0;
        self.bounds = c0.bounds.union(&c1.bounds).aabb();
        self.split_axis = axis;
        self.child1 = Some(c0);
        self.child2 = Some(c1);
        StatsAccumulator::instance().report_counter(String::from("BVH/Interior nodes"), 1);
    }
}

pub struct BVHPrimitiveInfo {
    pub primitive_number: usize,
    pub bounds: Box<dyn Bounding3<Float>>,
    pub centroid: Point3f
}

unsafe impl Send for BVHPrimitiveInfo {}

impl BVHPrimitiveInfo {
    pub fn new(primitive_number: usize, bounds: Box<dyn Bounding3<Float>>) -> BVHPrimitiveInfo {
        BVHPrimitiveInfo {
            primitive_number,
            bounds,
            centroid: bounds.centroid()
        }
    }
}

impl Default for BVHPrimitiveInfo {
    fn default() -> BVHPrimitiveInfo {
        BVHPrimitiveInfo{
            primitive_number: 0,
            bounds: Box::new(Bounds3f::default()),
            centroid: Point3f::default()
        }
    }
}

#[derive(Debug, Default, Copy, Clone)]
pub struct MortonPrimitive {
    pub primitive_index: usize,
    pub morton_code: u32
}

#[derive(Debug, Default, Copy, Clone)]
struct BucketInfo {
    pub count: usize,
    pub bounds: Bounds3f
}

#[derive(Debug)]
struct LBVHTreelet<'a> {
    start_index: usize,
    n_primitives: usize,
    build_nodes: Vec<&'a mut BVHBuildNode<'a>>
}

const morton_bits: u32 = 10;
const morton_scale: u32 = 1 << morton_bits;

#[inline]
fn left_shift_3(mut x: u32) -> u32 {
    assert!(x <= morton_scale);
    if x == morton_scale {
        x -= 1;
    }
    x = (x | (x << 16)) & 0b00000011000000000000000011111111;
    // x = ---- --98 ---- ---- ---- ---- 7654 3210
    x = (x | (x << 8)) & 0b00000011000000001111000000001111;
    // x = ---- --98 ---- ---- 7654 ---- ---- 3210
    x = (x | (x << 4)) & 0b00000011000011000011000011000011;
    // x = ---- --98 ---- 76-- --54 ---- 32-- --10
    x = (x | (x << 2)) & 0b00001001001001001001001001001001;
    // x = ---- 9--8 --7- -6-- 5--4 --3- -2-- 1--0
    x
}

#[inline]
fn encode_morton_3(v: &Vector3f) -> u32  {
    assert!(v.x >= num::zero() && v.y >= num::zero() && v.z >= num::zero());
    (left_shift_3(v.z as u32) << 2) | (left_shift_3(v.y as u32) << 1) | left_shift_3(v.x as u32)
}

fn radix_sort(v: &mut Vec<MortonPrimitive>) {
    let mut temp_vector: Vec<MortonPrimitive> = Vec::with_capacity(v.len());
    const bits_per_pass: usize = 6;
    const n_bits: usize = 30;
    assert!(n_bits % bits_per_pass == 0, "Radix sort bits_per_pass must evenly divide n_bits");
    const n_passes: usize = n_bits / bits_per_pass;

    for pass in 0..n_passes {
        // Perform one pass of radix sort, sorting _bitsPerPass_ bits
        let low_bit = pass * bits_per_pass;

        // Set in and out vector pointers for radix sort pass
        let in_vec = if pass & 1 != 0 { &temp_vector } else { &v };
        let mut out_vec = if pass & 1 != 0 { &v } else { &temp_vector };

        // Count number of zero bits in array for current radix sort bit
        const n_buckets: usize = 1 << bits_per_pass;
        let mut bucket_count = [0_usize; n_buckets]; 
        const bit_mask: usize = (1 << bits_per_pass) - 1;
        for mp in in_vec {
            let bucket = (mp.morton_code as usize >> low_bit) & bit_mask;
            assert!(bucket >= 0 && bucket < n_buckets);
            bucket_count[bucket] += 1;
        }

        // Compute starting index in output array for each bucket
        let mut out_index = [0_usize; n_buckets];
        out_index[0] = 0;
        for i in 1..n_buckets {
            out_index[i] = out_index[i - 1] + bucket_count[i - 1];
        }

        // Store sorted values in output array
        for i in 0..in_vec.len() {
            let bucket = (in_vec[i].morton_code as usize >> low_bit) & bit_mask;
            unsafe {
                // Can't take two mutable loans from one vector, so instead just cast
                // them to their raw pointers to do the swap
                let pa: *mut MortonPrimitive = &mut in_vec[i];
                let pb: *mut MortonPrimitive = &mut out_vec[out_index[bucket]];
                ptr::swap(pa, pb);
            }
        }
    }
    // Copy final result from _tempVector_, if needed
    if n_passes & 1 != 0 {
        std::mem::swap(v, &mut temp_vector);
    }
}