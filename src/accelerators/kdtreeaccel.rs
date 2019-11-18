use crate::core::pbrt::{Float};
use crate::core::primitive::Primitive;
use crate::core::geometry::{Bounds3f, Bounding3, Vector3f, Ray};
use crate::core::profiler::Profiler;
use crate::core::light::Light;
use crate::core::material::{Material, TransportMode};
use crate::core::surface_interaction::SurfaceInteraction;
use obstack::Obstack;
use std::sync::Arc;

pub struct KdTreeAccel {
    isect_cost: i32,
    traversal_cost: i32,
    max_prims: u32,
    empty_bonus: Float,
    primitives: Vec<Arc<dyn Primitive>>,
    primitive_indices: Vec<u32>,
    nodes: Vec<KdAccelNode>,
    n_alloced_nodes: u32,
    next_free_node: u32,
    bounds: Box<Bounds3f>
}

impl KdTreeAccel {
    pub fn new(
        p: Vec<Arc<dyn Primitive>>,
        isect_cost: i32,
        traversal_cost: i32,
        empty_bonus: Float,
        max_prims: u32,
        mut max_depth: i32
    ) -> KdTreeAccel {
        let _guard = Profiler::instance().profile("Acceleration structure creation");
        if max_depth <= 0 {
            max_depth = (8.0 + 1.3 * (p.len() as f64).log2()).round() as i32;
        }

        let p_len = p.len();
        // Compute bounds for kd-tree construction
        let mut bounds: Bounds3f = Bounds3f::default();
        let mut prim_bounds: Vec<&Box<dyn Bounding3<Float>>> = Vec::with_capacity(p.len());
        for prim in &p {
            let b = prim.world_bound();
            bounds = bounds.union(&*b).aabb();
            prim_bounds.push(&b);
        }

        // Allocate working memory for kd-tree construction
        let mut edges: [Vec<BoundEdge>; 3] = [
            vec![BoundEdge::default(); 2 * p_len],
            vec![BoundEdge::default(); 2 * p_len],
            vec![BoundEdge::default(); 2 * p_len],
        ];
        let mut prims0 = vec![0_usize; p_len];
        let mut prims1 = vec![0_usize; (max_depth as usize + 1) * p_len];

        // Initialize _primNums_ for kd-tree construction
        let mut prim_nums: Vec<usize> = Vec::with_capacity(p_len);
        for i in 0..p_len {
            prim_nums.push(i);
        }
        // Start recursive construction of kd-tree
        let mut kd_tree = KdTreeAccel {
            bounds: Box::new(bounds),
            isect_cost,
            traversal_cost,
            max_prims,
            empty_bonus,
            primitives: p,
            primitive_indices: Vec::new(),
            nodes: Vec::new(),
            n_alloced_nodes: 0,
            next_free_node: 0
        };
        KdTreeAccel::build_tree(
            &mut kd_tree,
            0,
            bounds,
            prim_bounds,
            &prim_nums,
            p_len,
            max_depth as usize,
            &mut edges,
            &mut prims0,
            &mut prims1,
            0
        );
        kd_tree
    }

    pub fn build_tree(
        &mut self,
        node_num: usize,
        node_bounds: Bounds3f,
        all_prim_bounds: Vec<&Box<dyn Bounding3<Float>>>,
        prim_nums: &[usize],
        n_primitives: usize,
        depth: usize,
        edges: &mut [Vec<BoundEdge>],
        prims0: &mut [usize],
        prims1: &mut [usize],
        mut bad_refines: usize
    ) {
        assert_eq!(node_num, self.next_free_node as usize);
        // Get next free node from _nodes_ array
        if self.next_free_node == self.n_alloced_nodes {
            let n_new_alloc_nodes = (2 * self.n_alloced_nodes as usize).max(512);
            self.nodes.resize_with(n_new_alloc_nodes, || KdAccelNode::default());
            self.n_alloced_nodes = n_new_alloc_nodes as u32;
        }
        self.next_free_node += 1;

        // Initialize leaf node if termination criteria met
        if n_primitives <= self.max_prims as usize || depth == 0 {
            self.nodes[node_num].init_leaf(prim_nums, n_primitives, &mut self.primitive_indices);
            return;
        }

        // Initialize interior node and continue recursion

        // Choose split axis position for interior node
        let mut best_axis: i32 = -1;
        let mut best_offset: i32 = -1;
        let mut best_cost: Float = num::Float::infinity();
        let mut old_cost = (self.isect_cost as usize * n_primitives) as Float;
        let mut total_sa = node_bounds.surface_area();
        let mut inv_total_sa = 1.0 / total_sa;
        let mut d = Vector3f::from(node_bounds.max - node_bounds.min);

        // Choose which axis to split along
        let axis = node_bounds.maximum_extent() as usize;
        let mut retries = 0;
        loop {
            // Initialize edges for _axis_
            for i in 0..n_primitives {
                let pn = prim_nums[i];
                let bounds = all_prim_bounds[pn].aabb();
                edges[axis][2 * i] = BoundEdge::new(bounds.min[axis as u8], pn, true);
                edges[axis][2 * i + 1] = BoundEdge::new(bounds.max[axis as u8], pn, false);
            }

            // Sort _edges_ for _axis_
            edges[axis].sort_unstable_by(|e0,e1| {
                if e0.t == e1.t {
                    e0.edge_type.partial_cmp(&e1.edge_type).unwrap()
                } else {
                    e0.t.partial_cmp(&e1.t).unwrap()
                }
            });

            // Compute cost of all splits for _axis_ to find best
            let mut n_below = 0_usize;
            let mut n_above = n_primitives;
            for i in 0..2 * n_primitives {
                if edges[axis][i].edge_type == EdgeType::End {
                    n_above -= 1;
                }
                let edge_t = edges[axis][i].t;
                if edge_t > node_bounds.min[axis as u8] && edge_t < node_bounds.max[axis as u8] {
                    // Compute cost for split at _i_th edge

                    // Compute child surface areas for split at _edgeT_
                    let other_axis0 = (axis + 1) % 3;
                    let other_axis1 = (axis + 2) % 3;
                    let below_sa = 2.0 * (d[other_axis0] * d[other_axis1] + 
                        (edge_t - node_bounds.min[axis as u8]) * 
                        (d[other_axis0] + d[other_axis1]));
                    let above_sa = 2.0 * (d[other_axis0] * d[other_axis1] + 
                        (node_bounds.max[axis as u8] - edge_t) * 
                        (d[other_axis0] + d[other_axis1]));
                    let p_below = below_sa * inv_total_sa;
                    let p_above = above_sa * inv_total_sa;
                    let eb = if n_above == 0 || n_below == 0 { self.empty_bonus } else { 0.0 };
                    let cost = self.traversal_cost as Float + 
                        self.isect_cost as Float * (1.0 - eb) * (p_below * n_below as Float + p_above * n_above as Float);
                    // Update best split if this is lowest cost so far
                    if cost < best_cost {
                        best_cost = cost;
                        best_axis = axis as i32;
                        best_offset = i as i32;
                    }
                }
                if edges[axis][i].edge_type == EdgeType::Start {
                    n_below += 1;
                }
            }
            assert!(n_below == n_primitives && n_above == 0);

            // Create leaf if no good splits were found
            if best_axis == -1 && retries < 2 {
                retries += 1;
                axis = (axis + 1) % 3;
                // goto retrySplit
            } else {
                break;
            }
        }

        if best_cost > old_cost {
            bad_refines += 1;
        }
        if (best_cost > 4.0 * old_cost && n_primitives < 16) || best_axis == -1 ||
            bad_refines == 3 {
            self.nodes[node_num].init_leaf(prim_nums, n_primitives, &mut self.primitive_indices);
            return;
        }

        // Classify primitives with respect to split
        let mut n0 = 0_usize;
        let mut n1 = 0_usize;
        for i in 0..best_offset as usize {
            if edges[best_axis as usize][i].edge_type == EdgeType::Start {
                prims0[n0] = edges[best_axis as usize][i].prim_num;
                n0 += 1;
            }
        }
        for i in (best_offset + 1) as usize..2 * n_primitives {
            if edges[best_axis as usize][i].edge_type == EdgeType::End {
                prims1[n1] = edges[best_axis as usize][i].prim_num;
                n1 += 1;
            }
        }

        // Recursively initialize children nodes
        let t_split = edges[best_axis as usize][best_offset as usize].t;
        let mut bounds0 = node_bounds;
        let mut bounds1 = node_bounds;
        bounds0.max[best_axis as u8] = t_split;
        bounds1.min[best_axis as u8] = t_split;
        self.build_tree(node_num + 1, bounds0, all_prim_bounds, prims0, n0, depth - 1, edges, prims0, &mut prims1[n_primitives..], bad_refines);
        let above_child = self.next_free_node as usize;
        self.nodes[node_num].init_interior(best_axis as u32, above_child as u32, t_split);
        self.build_tree(above_child, bounds1, all_prim_bounds, prims1, n1, depth - 1, edges, prims0, &mut prims1[n_primitives..], bad_refines);
    }
}

impl Primitive for KdTreeAccel {
    fn get_area_light(&self) -> Option<Arc<dyn Light>> {
        error!("KdTreeAccel::get_area_light() called; should have gone to GeometricPrimitive.");
        None
    }

    fn get_material(&self) -> Option<Arc<dyn Material>> {
        error!("KdTreeAccel::get_material() called; should have gone to GeometricPrimitive.");
        None
    }

    fn compute_scattering_function(&self, arena: &Obstack, isect: &mut SurfaceInteraction, mode: TransportMode, allow_multiple_lobes: bool) {
        error!("KdTreeAccel::compute_scattering_function() called; should have gone to GeometricPrimitive.");
    }

    fn world_bound(&self) -> Box<dyn Bounding3<Float>> {
        self.bounds
    }

    fn intersect(&self, ray: &Ray) -> Option<SurfaceInteraction> {
        let _guard = Profiler::instance().profile("KdTreeAcell::intersect");
        // Compute initial parametric range of ray inside kd-tree extent
        let mut t_min: Float = 0.0;
        let mut t_max: Float = 0.0;
        if !self.bounds.intersect_ray(ray, &mut t_min, &mut t_max) {
            return None;
        }

        // Prepare to traverse kd-tree for ray
        let inv_dir = Vector3f::new(1.0 / ray.d.x, 1.0 / ray.d.y, 1.0 / ray.d.z);
        const max_todo: usize = 64;
        let todo = [KdToDo::default(); max_todo];
        let todo_pos = 0_usize;

        // Traverse kd-tree nodes in order for ray
        let mut hit: Option<SurfaceInteraction> = None;
        let mut node_idx = 0_usize;
        let mut node_opt: Option<&KdAccelNode> = Some(&self.nodes[0]);
        while let Some(node) = node_opt {
            // Bail out if we found a hit closer than the current node
            if ray.t_max < t_min {
                break;
            }
            if !node.is_leaf() {
                // Process kd-tree interior node

                // Compute parametric distance along ray to split plane
                let axis = node.split_axis() as u8;
                let t_plane = (node.split_pos() - ray.o[axis]) * inv_dir[axis];

                // Get node children pointers for ray
                let first_child_idx = 0_usize;
                let second_child_idx = 0_usize;
                let below_first = ray.o[axis] < node.split_pos() ||
                    (ray.o[axis] == node.split_pos() && ray.d[axis] <= 0.0);
                if below_first {
                    first_child_idx = node_idx + 1;
                    second_child_idx = node.above_child() as usize;
                } else {
                    first_child_idx = node.above_child() as usize;
                    second_child_idx = node_idx + 1;
                }
                let first_child = Some(&self.nodes[first_child_idx]);
                let second_child = Some(&self.nodes[second_child_idx]);

                // Advance to next child node, possibly enqueue other child
                if t_plane > t_max || t_plane <= 0.0 {
                    node_opt = first_child;
                    node_idx = first_child_idx;
                }
                else if t_plane < t_min {
                    node_opt = second_child;
                    node_idx = second_child_idx;
                } else {
                    // Enqueue _secondChild_ in todo list
                    todo[todo_pos].node = second_child;
                    todo[todo_pos].index = second_child_idx;
                    todo[todo_pos].t_min = t_plane;
                    todo[todo_pos].t_max = t_max;
                    todo_pos += 1;
                    node_opt = first_child;
                    node_idx = first_child_idx;
                    t_max = t_plane;
                }
            } else {
                // Check for intersections inside leaf node
                let n_primitives = node.n_primitives() as usize;
                if n_primitives == 1 {
                    let p = &self.primitives[node.one_primitive() as usize];
                    // Check one primitive inside leaf node
                    if let Some(si) = p.intersect(ray) {
                        hit = Some(si);
                    }
                } else {
                    for i in 0..n_primitives {
                        let index = self.primitive_indices[node.primitive_indices_offset() as usize + 1] as usize;
                        let p = &self.primitives[index];
                        // Check one primitive inside leaf node
                        if let Some(si) = p.intersect(ray) {
                            hit = Some(si);
                        }
                    }
                }

                // Grab next node to process from todo list
                if todo_pos > 0 {
                    todo_pos -= 1;
                    node_opt = todo[todo_pos].node;
                    node_idx = todo[todo_pos].index;
                    t_min = todo[todo_pos].t_min;
                    t_max = todo[todo_pos].t_max;
                } else {
                    break;
                }
            }
        }
        hit
    }

    fn intersect_p(&self, ray: &Ray) -> bool {
        let _guard = Profiler::instance().profile("KdTreeAcell::intersect_p");
        // Compute initial parametric range of ray inside kd-tree extent
        let mut t_min: Float = 0.0;
        let mut t_max: Float = 0.0;
        if !self.bounds.intersect_ray(ray, &mut t_min, &mut t_max) {
            return false;
        }

        // Prepare to traverse kd-tree for ray
        let inv_dir = Vector3f::new(1.0 / ray.d.x, 1.0 / ray.d.y, 1.0 / ray.d.z);
        const max_todo: usize = 64;
        let todo = [KdToDo::default(); max_todo];
        let todo_pos = 0_usize;

        // Traverse kd-tree nodes in order for ray
        let mut node_idx = 0_usize;
        let mut node_opt: Option<&KdAccelNode> = Some(&self.nodes[0]);
        while let Some(node) = node_opt {
            // Bail out if we found a hit closer than the current node
            if ray.t_max < t_min {
                break;
            }
            if !node.is_leaf() {
                // Process kd-tree interior node

                // Compute parametric distance along ray to split plane
                let axis = node.split_axis() as u8;
                let t_plane = (node.split_pos() - ray.o[axis]) * inv_dir[axis];

                // Get node children pointers for ray
                let first_child_idx = 0_usize;
                let second_child_idx = 0_usize;
                let below_first = ray.o[axis] < node.split_pos() ||
                    (ray.o[axis] == node.split_pos() && ray.d[axis] <= 0.0);
                if below_first {
                    first_child_idx = node_idx + 1;
                    second_child_idx = node.above_child() as usize;
                } else {
                    first_child_idx = node.above_child() as usize;
                    second_child_idx = node_idx + 1;
                }
                let first_child = Some(&self.nodes[first_child_idx]);
                let second_child = Some(&self.nodes[second_child_idx]);

                // Advance to next child node, possibly enqueue other child
                if t_plane > t_max || t_plane <= 0.0 {
                    node_opt = first_child;
                    node_idx = first_child_idx;
                }
                else if t_plane < t_min {
                    node_opt = second_child;
                    node_idx = second_child_idx;
                } else {
                    // Enqueue _secondChild_ in todo list
                    todo[todo_pos].node = second_child;
                    todo[todo_pos].index = second_child_idx;
                    todo[todo_pos].t_min = t_plane;
                    todo[todo_pos].t_max = t_max;
                    todo_pos += 1;
                    node_opt = first_child;
                    node_idx = first_child_idx;
                    t_max = t_plane;
                }
            } else {
                // Check for shadow ray intersections inside leaf node
                let n_primitives = node.n_primitives() as usize;
                if n_primitives == 1 {
                    let p = &self.primitives[node.one_primitive() as usize];
                    // Check one primitive inside leaf node
                    if p.intersect_p(ray) {
                        return true;
                    }
                } else {
                    for i in 0..n_primitives {
                        let index = self.primitive_indices[node.primitive_indices_offset() as usize + 1] as usize;
                        let p = &self.primitives[index];
                        // Check one primitive inside leaf node
                        if p.intersect_p(ray) {
                            return true;
                        }
                    }
                }

                // Grab next node to process from todo list
                if todo_pos > 0 {
                    todo_pos -= 1;
                    node_opt = todo[todo_pos].node;
                    node_idx = todo[todo_pos].index;
                    t_min = todo[todo_pos].t_min;
                    t_max = todo[todo_pos].t_max;
                } else {
                    break;
                }
            }
        }
        return false;
    }
}

#[repr(C)]
union KANPrivate {
    flags: u32,
    n_prims: u32,
    above_child: u32,
}

#[repr(C)]
pub union KANPublic {
    pub split: Float,
    pub one_primitive: u32,
    pub primitive_indices_offset: u32
}

pub struct KdAccelNode {
    pub pub_union: KANPublic,
    priv_union: KANPrivate
}

impl KdAccelNode {
    pub fn init_leaf(&mut self, prim_nums: &[usize], np: usize, primitive_indices: &mut Vec<u32>) {
        self.priv_union.flags = 3;
        self.priv_union.n_prims |= (np as u32) << 2;
        // Store primitive ids for leaf node
        match np {
            0 => self.pub_union.one_primitive = 0,
            1 => self.pub_union.one_primitive = prim_nums[0] as u32,
            _ => {
                self.pub_union.primitive_indices_offset = primitive_indices.len() as u32;
                for i in 0..np {
                    primitive_indices.push(prim_nums[i] as u32);
                }
            }
        }
    }

    pub fn init_interior(&mut self, axis: u32, ac: u32, s: Float) {
        self.pub_union.split = s;
        self.priv_union.flags = axis;
        self.priv_union.above_child |= ac << 2;
    }

    pub fn split_pos(&self) -> Float {
        unsafe {
            self.pub_union.split
        }
    }

    pub fn n_primitives(&self) -> u32 {
        unsafe {
            self.priv_union.n_prims >> 2
        }
    }

    pub fn split_axis(&self) -> u32 {
        unsafe {
            self.priv_union.flags & 3
        }
    }

    pub fn is_leaf(&self) -> bool {
        unsafe {
            self.priv_union.flags & 3 == 3
        }
    }

    pub fn above_child(&self) -> u32 {
        unsafe {
            self.priv_union.above_child >> 2
        }
    }

    pub fn one_primitive(&self) -> u32 {
        unsafe {
            self.pub_union.one_primitive
        }
    }

    pub fn split(&self) -> Float {
        unsafe {
            self.pub_union.split
        }
    }

    pub fn primitive_indices_offset(&self) -> u32 {
        unsafe {
            self.pub_union.primitive_indices_offset
        }
    }
}

impl Default for KdAccelNode {
    fn default() -> Self {
        KdAccelNode {
            pub_union: KANPublic { split: 0.0 },
            priv_union: KANPrivate { flags: 0 }
        }
    }
}

#[derive(Debug, PartialEq, PartialOrd, Clone)]
enum EdgeType {
    Start,
    End
}

#[derive(Debug, Clone)]
struct BoundEdge {
    pub t: Float,
    pub prim_num: usize,
    pub edge_type: EdgeType
}

impl BoundEdge {
    pub fn new(t: Float, prim_num: usize, starting: bool) -> BoundEdge {
        let edge_type = if starting { EdgeType::Start } else { EdgeType::End };
        BoundEdge {
            t,
            prim_num,
            edge_type
        }
    }
}

impl Default for BoundEdge {
    fn default() -> BoundEdge {
        BoundEdge {
            t: 0.0,
            prim_num: 0,
            edge_type: EdgeType::Start
        }
    }
}

#[derive(Copy, Clone, Default)]
struct KdToDo<'a> {
    node: Option<&'a KdAccelNode>,
    index: usize,
    t_min: Float,
    t_max: Float
}
