use crate::core::pbrt::{Float, consts::PI};
use crate::core::geometry::{Point3f, Vector3f, Normal3f};
use crate::core::transform::Transform;
use super::triangle::TriangleMesh;
use std::hash::{Hash, Hasher};
use smallvec::SmallVec;
use std::sync::Arc;
use std::cmp::Ordering;
use hashbrown::{HashSet, HashMap};

macro_rules! next {
    ($i: expr) => {
        ($i + 1) % 3
    };
}

macro_rules! prev {
    ($i: expr) => {
        ($i + 2) % 3
    };
}

/// An arena implemtation for subdivision objects.
#[derive(Default)]
struct Subdivision {
    vertices: Vec<Arc<SDVertex>>,
    faces: Vec<Arc<SDFace>>
}

impl Subdivision {
    pub fn new() -> Subdivision {
        Subdivision{
            vertices: Vec::new(),
            faces: Vec::new()
        }
    }

    pub fn with_capacity(vertices_length: usize, faces_length: usize) -> Self {
        Subdivision{
            vertices: Vec::with_capacity(vertices_length),
            faces: Vec::with_capacity(faces_length)
        }
    }

    pub fn add_vertex(&self, p: Point3f) -> Arc<SDVertex> {
        let index = self.vertices.len() as u32;
        let vertex = Arc::new(SDVertex::new(p, index));
        self.vertices.push(vertex);
        vertex
    }

    pub fn add_face(&self) -> Arc<SDFace> {
        let index = self.faces.len() as u32;
        let face = Arc::new(SDFace::new(index));
        self.faces.push(face);
        face
    }
}

struct SDVertex {
    p: Point3f,
    index: u32,
    start_face: Option<u32>,
    child: Option<u32>,
    regular: bool,
    boundary: bool
}

impl SDVertex {
    pub fn new(p: Point3f, index: u32) -> SDVertex {
        SDVertex {
            p,
            index,
            start_face: None,
            child: None,
            regular: false,
            boundary: false
        }
    }

    /// Returns the number of vertices adjacent to this vertex.
    pub fn valence(&self, sd: &Subdivision) -> i32 {
        let mut fi = self.start_face;
        if !self.boundary {
            // Compute valence of interior vertex
            let nf = 1_i32;
            while let Some(f) = fi {
                fi == sd.faces[f as usize].next_face(self);
                if fi == self.start_face {
                    break;
                }
                nf += 1;
            }
            nf
        }
        else {
            // Compute valence of boundary vertex
            let nf = 1_i32;
            while let Some(f) = fi {
                fi == sd.faces[f as usize].next_face(self);
                if fi == self.start_face {
                    break;
                }
                nf += 1;
            }
            fi = self.start_face;
            while let Some(f) = fi {
                fi == sd.faces[f as usize].prev_face(self);
                if fi == self.start_face {
                    break;
                }
                nf += 1;
            }
            nf + 1
        }
    }

    pub fn one_ring(&self, p: &mut SmallVec<[Point3f; 128]>, sd: &Subdivision) {
        let mut fi = self.start_face;
        let mut pi = 0_usize;
        if !self.boundary {
            // Get one-ring vertices for interior vertex
            while let Some(f) = fi{
                let face = sd.faces[f as usize];
                p[pi] = sd.vertices[face.next_vert(self).unwrap() as usize].p;
                pi += 1;
                fi = face.next_face(self);
                if fi == self.start_face {
                    break;
                }
            }
        }
        else {
            // Get one-ring vertices for boundary vertex
            while let Some(f) = fi {
                fi = sd.faces[f as usize].next_face(self);
            }
            p[pi] = sd.vertices[sd.faces[fi.unwrap() as usize].next_vert(self).unwrap() as usize].p;
            pi += 1;
            while let Some(f) = fi {
                let face = sd.faces[f as usize];
                p[pi] = sd.vertices[face.prev_vert(self).unwrap() as usize].p;
                pi += 1;
                fi = face.prev_face(self);
                if fi == self.start_face {
                    break;
                }
            }
        }
    }
}

struct SDFace {
    index: u32,
    v: [Option<u32>; 3],
    f: [Option<u32>; 3],
    children: [Option<u32>; 4]
}

impl SDFace {
    pub fn new(index: u32) -> SDFace {
        SDFace {
            index,
            v: [None; 3],
            f: [None; 3],
            children: [None; 4]
        }
    }

    pub fn vnum(&self, vert: &SDVertex) -> usize {
        for i in 0..3 {
            if let Some(v) = self.v[i] {
                if vert.index == v {
                    return i;
                }
            }
        }
        panic!("vert is not a vertex of SDFace");
    }

    pub fn vnum_index(&self, index: u32) -> usize {
        for i in 0..3 {
            if let Some(v) = self.v[i] {
                if index == v {
                    return i;
                }
            }
        }
        panic!("vert is not a vertex of SDFace");
    }

    pub fn next_face(&self, vert: &SDVertex) -> Option<u32> {
        self.f[self.vnum(vert)]
    }

    pub fn prev_face(&self, vert: &SDVertex) -> Option<u32> {
        self.f[prev!(self.vnum(vert))]
    }

    pub fn next_vert(&self, vert: &SDVertex) -> Option<u32> {
        self.v[next!(self.vnum(vert))]
    }

    pub fn prev_vert(&self, vert: &SDVertex) -> Option<u32> {
        self.v[prev!(self.vnum(vert))]
    }

    pub fn other_vert(&self, v0: u32, v1: u32) -> u32 {
        for i in 0..3 {
            let v = self.v[i].expect("Using SDFace with uninitialized values.");
            if v != v0 && v != v1 {
                return v;
            }
        }
        panic!("Basic logic error in SDVertex::otherVert()");
    }
}

struct SDEdge {
    v: [u32; 2],
    f: [Option<u32>; 2],
    f0_edge_num: i8
}

impl SDEdge {
    pub fn new(v0: u32, v1: u32) -> SDEdge {
        SDEdge {
            v: [v0, v1],
            f: [None; 2],
            f0_edge_num: -1
        }
    }
}

impl PartialEq for SDEdge {
    fn eq(&self, other: &SDEdge) -> bool {
        self.v[0] == other.v[0] && self.v[1] == other.v[1]
    }
}

impl Eq for SDEdge {}

impl PartialOrd for SDEdge {
    fn partial_cmp(&self, other: &SDEdge) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for SDEdge {
    fn cmp(&self, other: &SDEdge) -> Ordering {
        self.v[0]
            .cmp(&other.v[0])
            .then(self.v[1].cmp(&other.v[1]))
    }
}

impl Hash for SDEdge {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.v[0].hash(state);
        self.v[1].hash(state);
    }
}

pub fn loop_subdivide(
    object_to_world: &Transform,
    world_to_object: &Transform,
    reverse_orientation: bool,
    n_levels: u32,
    vertex_indices: &Vec<u32>,
    p: &Vec<Point3f>
) -> Arc<TriangleMesh> {
    let n_faces = p.len() / 3;
    let mut subdivision = Subdivision::with_capacity(p.len(), n_faces);
    // Allocate _LoopSubdiv_ vertices and faces
    for point in p {
        subdivision.add_vertex(*point);
    }

    for i in 0..n_faces {
        let mut face = subdivision.add_face();
        // Set face to vertex pointers
        for j in 0..3{
            let mut v = subdivision.vertices[vertex_indices[i * 3 + j] as usize];
            face.v[0] = Some(v.index);
            v.start_face = Some(face.index);
        }
    }

    // Set neighbor pointers in _faces_
    let edges: HashSet<SDEdge> = HashSet::new();
    for mut face in subdivision.faces {
        for edge_num in 0..3 {
            let v0 = edge_num;
            let v1 = next!(edge_num);
            let e = SDEdge::new(face.v[v0].unwrap(), face.v[v1].unwrap());
            if !edges.contains(&e) {
                // Handle new edge
                e.f[0] = Some(face.index);
                e.f0_edge_num = edge_num as i8;
                edges.insert(e);
            }
            else {
                // Handle previously seen edge
                let e = edges.take(&e).unwrap();
                subdivision.faces[e.f[0].unwrap() as usize].f[e.f0_edge_num as usize] = Some(face.index);
                face.f[edge_num] = e.f[0];
            }
        }
    }

    // Finish vertex initialization
    for v in subdivision.vertices {
        let mut fi = Some(subdivision.faces[v.start_face.unwrap() as usize].index);
        loop {
            fi = subdivision.faces[fi.unwrap() as usize].next_face(&v);
            match fi {
                Some(f) if f == v.start_face.unwrap() => break,
                None => break,
                _ => {}
            }
        }
        v.boundary = fi.is_none();
        let valence = v.valence(&subdivision);
        if !v.boundary && valence == 6 {
            v.regular = true;
        }
        else if v.boundary && valence == 4 {
            v.regular = true;
        }
        else {
            v.regular = false;
        }
    }

    // Refine _LoopSubdiv_ into triangles
    for i in 0..n_levels {
        let new_subdivision = Subdivision::new();
        // Allocate next level of children in mesh tree
        for vertex in subdivision.vertices {
            let child = new_subdivision.add_vertex(Point3f::default());
            vertex.child = Some(child.index);
            child.regular = vertex.regular;
            child.boundary = vertex.boundary;
        }
        for face in subdivision.faces {
            for k in 0..4 {
                let child = new_subdivision.add_face();
                face.children[k] = Some(child.index);
            }
        }

        // Update vertex positions and create new edge vertices

        // Update vertex positions for even vertices
        for vertex in subdivision.vertices {
            let child = new_subdivision.vertices[vertex.child.unwrap() as usize];
            if !vertex.boundary {
                // Apply one-ring rule for even vertex
                if vertex.regular {
                    child.p = weight_one_ring(vertex, 1.0 / 16.0, &subdivision);
                }
                else {
                    child.p = weight_one_ring(vertex, beta(vertex.valence(&subdivision)), &subdivision);
                }
            } else {
                // Apply boundary rule for even vertex
                child.p = weight_boundary(vertex, 1.0 / 8.0, &subdivision);
            }
        }

        // Compute new odd edge vertices
        let mut edge_vertices: HashMap<SDEdge, u32> = HashMap::new();
        for face in subdivision.faces {
            for k in 0..3 {
                // Compute odd vertex on _k_th edge
                let edge = SDEdge::new(face.v[k].unwrap(), face.v[next!(k)].unwrap());
                if !edge_vertices.contains_key(&edge) {
                    // Create and initialize new odd vertex
                    let vert = new_subdivision.add_vertex(Point3f::default());
                    vert.regular = true;
                    vert.boundary = face.f[k].is_none();
                    vert.start_face = face.children[3];
                    // Apply edge rules to compute new vertex position
                    if vert.boundary {
                        vert.p = 0.5 * subdivision.vertices[edge.v[0]as usize].p;
                        vert.p += 0.5 * subdivision.vertices[edge.v[1]as usize].p;
                    } else {
                        vert.p = 3.0 / 8.0 * subdivision.vertices[edge.v[0]as usize].p;
                        vert.p += 3.0 / 8.0 * subdivision.vertices[edge.v[1]as usize].p;
                        vert.p += 1.0 / 8.0 * subdivision.vertices[face.other_vert(edge.v[0], edge.v[1]) as usize].p;
                        let face_k = subdivision.faces[face.f[k].unwrap() as usize];
                        vert.p += 1.0 / 8.0 * subdivision.vertices[face_k.other_vert(edge.v[0], edge.v[1]) as usize].p;
                    }

                    edge_vertices.insert(edge, vert.index);
                }
            }
        }

        // Update new mesh topology

        // Update even vertex face pointers
        for vertex in subdivision.vertices {
            let start_face = subdivision.faces[vertex.start_face.unwrap() as usize];
            let vert_num = start_face.vnum(&vertex);
            let child = new_subdivision.vertices[vertex.child.unwrap() as usize];
            child.start_face = start_face.children[vert_num];
        }

        // Update face neighbor pointers
        for face in subdivision.faces {
            for j in 0..3 {
                // Update children _f_ pointers for siblings
                let child_3 = subdivision.faces[face.children[3].unwrap() as usize];
                child_3.f[j] = face.children[next!(j)];
                let child_j = subdivision.faces[face.children[j].unwrap() as usize];
                child_j.f[next!(j)] = face.children[3];

                // Update children _f_ pointers for neighbor children
                if face.f[j].is_some() {
                    let f2 = subdivision.faces[face.f[j].unwrap() as usize];
                    child_j.f[j] = f2.children[f2.vnum_index(face.v[j].unwrap())];
                }
                if face.f[prev!(j)].is_some() {
                    let f2 = subdivision.faces[face.f[prev!(j)].unwrap() as usize];
                    child_j.f[prev!(j)] = f2.children[f2.vnum_index(face.v[j].unwrap())];
                }
            }

            // Update face vertex pointers
            for face in subdivision.faces {
                for j in 0..3 {
                    // Update child vertex pointer to new even vertex
                    let child = subdivision.faces[face.children[j].unwrap() as usize];
                    let vertex = subdivision.vertices[face.v[j].unwrap() as usize];
                    child.v[j] = vertex.child;

                    // Update child vertex pointer to new odd vertex
                    let edge = SDEdge::new(face.v[j].unwrap(), face.v[next!(j)].unwrap());
                    if let Some(vert) = edge_vertices.get(&edge) {
                        let mut child = subdivision.faces[face.children[j].unwrap() as usize];
                        child.v[next!(j)] = Some(*vert);
                        child = subdivision.faces[face.children[next!(j)].unwrap() as usize];
                        child.v[j] = Some(*vert);
                        child = subdivision.faces[face.children[3].unwrap() as usize];
                        child.v[j] = Some(*vert);
                    }
                }
            }

            // Prepare for next level of subdivision
            subdivision = new_subdivision;
        }
    }

    // Push vertices to limit surface
    let p_limit: Vec<Point3f> = Vec::with_capacity(subdivision.vertices.len());
    for vertex in subdivision.vertices {
        let wb = if vertex.boundary {
            weight_boundary(vertex, 1.0 / 5.0, &subdivision)
        } else {
            weight_boundary(vertex, loop_gamma(vertex.valence(&subdivision)), &subdivision)
        };
        p_limit.push(wb);
    }
    for i in 0..subdivision.vertices.len() {
        subdivision.vertices[i].p = p_limit[i];
    }

    // Compute vertex tangents on limit surface
    let mut ns: Vec<Normal3f> = Vec::with_capacity(subdivision.vertices.len());
    let mut p_ring: SmallVec<[Point3f; 128]> = SmallVec::new();
    p_ring.resize(16, Point3f::default());
    for vertex in subdivision.vertices {
        let mut s = Vector3f::default();
        let mut t = Vector3f::default();
        let valence = vertex.valence(&subdivision) as usize;
        if valence > p_ring.len() {
            p_ring.resize(valence, Point3f::default());
        }
        vertex.one_ring(&mut p_ring, &subdivision);
        if !vertex.boundary {
            // Compute tangents of interior face
            for j in 0..valence {
                s += (2.0 * PI * j as Float / valence as Float).cos() * Vector3f::from(p_ring[j]);
                t += (2.0 * PI * j as Float / valence as Float).sin() * Vector3f::from(p_ring[j]);
            }
        } else {
            // Compute tangents of boundary face
            s = Vector3f::from(p_ring[valence - 1] - p_ring[0]);
            if valence == 2 {
                t = Vector3f::from(p_ring[0] + p_ring[1] - 2.0 * vertex.p);
            }
            else if valence == 3 {
                t = Vector3f::from(p_ring[1] - vertex.p);
            }
            else if valence == 4 { //regular
                t = Vector3f::from(-1.0 * p_ring[0] + 2.0 * p_ring[1] + 2.0 * p_ring[2] +
                                    -1.0 * p_ring[3] + -2.0 * vertex.p)
            } else {
                let theta = PI / (valence - 1) as Float;
                t = Vector3f::from(theta.sin() * (p_ring[0] + p_ring[valence + 1]));
                for k in 1..valence - 1 {
                    let wt = (2.0 * theta.cos() - 2.0) * (k as Float * theta).sin();
                    t += Vector3f::from(wt * p_ring[k]);
                }
                t = -t;
            }
        }
        ns.push(Normal3f::from(s.cross(&t)));
    }

    // Create triangle mesh from subdivision mesh
    let ntris = subdivision.faces.len();
    let verts: Vec<u32> = Vec::with_capacity(3 * ntris);
    let tot_verts = subdivision.vertices.len();
    for face in subdivision.faces {
        for j in 0..3 {
            verts.push(face.v[j].unwrap());
        }
    }

    Arc::new(TriangleMesh::new(
        *object_to_world,
        *world_to_object,
        reverse_orientation,
        ntris as u32,
        verts,
        p_limit,
        ns,
        Vec::new(),
        Vec::new(),
        None,
        None,
        Vec::new()
    ))
}

#[inline]
fn beta(valence: i32) -> Float {
    if valence == 3 {
        3.0 / 16.0
    }
    else {
        3.0 / (8.0 * valence as Float)
    }
}

#[inline]
fn loop_gamma(valence: i32) -> Float {
    1.0 / (valence as Float + 3.0 / (8.0 * beta(valence)))
}

fn weight_boundary(vert: Arc<SDVertex>, beta: Float, sd: &Subdivision) -> Point3f {
    let valence = vert.valence(sd);
    let mut p_ring: SmallVec<[Point3f; 128]> = SmallVec::with_capacity(valence as usize);
    vert.one_ring(&mut p_ring, sd);
    let mut p = (1.0 - 2.0 * beta) * vert.p;
    p += beta * p_ring[0];
    p += beta * p_ring[valence as usize - 1];
    p
}

fn weight_one_ring(vert: Arc<SDVertex>, beta: Float, sd: &Subdivision) -> Point3f {
    let valence = vert.valence(sd);
    let mut p_ring: SmallVec<[Point3f; 128]> = SmallVec::with_capacity(valence as usize);
    vert.one_ring(&mut p_ring, sd);
    let mut p = (1.0 - valence as Float * beta) * vert.p;
    for i in 0..valence as usize {
        p += beta * p_ring[i];
    }
    p
}