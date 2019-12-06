use super::pbrt::{Float, lerp, consts::{INV_PI, PI, INV_2_PI}};
use super::interaction::SurfaceInteraction;
use super::geometry::{Vector2f, Vector3f, Point2f, Point3f, spherical_theta, spherical_phi};
use super::transform::Transform;
use num::clamp;

pub trait Texture<T> {
    fn evaluate(&self, si: &SurfaceInteraction) -> T;
}

pub trait TextureMapping2D {
    fn map(&self, si: &SurfaceInteraction, dstdx: &mut Vector2f, dstdy: &mut Vector2f) -> Point2f;
}

pub struct UVMapping2D {
    su: Float,
    sv: Float,
    du: Float,
    dv: Float
}

impl UVMapping2D {
    pub fn new(su: Float, sv: Float, du: Float, dv: Float) -> UVMapping2D {
        UVMapping2D { su, sv, du, dv }
    }
}

impl Default for UVMapping2D {
    fn default() -> UVMapping2D {
        UVMapping2D { su: 1.0, sv: 1.0, du: 0.0, dv: 0.0 }
    }
}

impl TextureMapping2D for UVMapping2D {
    fn map(&self, si: &SurfaceInteraction, dstdx: &mut Vector2f, dstdy: &mut Vector2f) -> Point2f {
        // Compute texture differentials for 2D identity mapping
        *dstdx = Vector2f::new(self.su * *si.dudx.read().unwrap(), self.sv * *si.dvdx.read().unwrap());
        *dstdy = Vector2f::new(self.su * *si.dudy.read().unwrap(), self.sv * *si.dvdy.read().unwrap());
        Point2f::new(self.su * si.uv[0] + self.du, self.sv * si.uv[1] + self.dv)
    }
}

pub struct SphericalMapping2D {
    world_to_texture: Transform
}

impl SphericalMapping2D {
    pub fn new(world_to_texture: Transform) -> SphericalMapping2D {
        SphericalMapping2D {
            world_to_texture
        }
    }

    fn sphere(&self, p: &Point3f) -> Point2f {
        let vec = (self.world_to_texture.transform_point(p) - Point3f::new(0.0, 0.0, 0.0)).normalize();
        let theta = spherical_theta(&vec);
        let phi = spherical_phi(&vec);
        Point2f::new(theta * INV_PI, phi * INV_PI)
    }
}

impl TextureMapping2D for SphericalMapping2D {
    fn map(&self, si: &SurfaceInteraction, dstdx: &mut Vector2f, dstdy: &mut Vector2f) -> Point2f {
        let st = self.sphere(&si.p);
        // Compute texture coordinate differentials for sphere $(u,v)$ mapping
        const delta: Float = 0.1;
        let st_delta_x = self.sphere(&(si.p + delta * *si.dpdx.read().unwrap()));
        *dstdx = (st_delta_x - st) / delta;
        let st_delta_y = self.sphere(&(si.p + delta * *si.dpdy.read().unwrap()));
        *dstdy = (st_delta_y - st) / delta;

        // Handle sphere mapping discontinuity for coordinate differentials
        if dstdx[1] > 0.5 {
            dstdx[1] = 1.0 - dstdx[1];
        }
        else if dstdx[1] < -0.5 {
            dstdx[1] = -(dstdx[1] + 1.0);
        }
        if dstdy[1] > 0.5 {
            dstdy[1] = 1.0 - dstdy[1];
        }
        else if dstdy[1] < -0.5 {

            dstdy[1] = -(dstdy[1] + 1.0);
        }
        st
    }
}

pub struct CylindricalMapping2D {
    world_to_texture: Transform
}

impl CylindricalMapping2D {
    pub fn new(world_to_texture: Transform) -> CylindricalMapping2D {
        CylindricalMapping2D { world_to_texture }
    }

    fn cylinder(&self, p: &Point3f) -> Point2f {
        let vec = (self.world_to_texture.transform_point(p) - Point3f::default()).normalize();
        Point2f::new((PI + vec.y.atan2(vec.x)) * INV_2_PI, vec.z)
    }
}

impl TextureMapping2D for CylindricalMapping2D {
    fn map(&self, si: &SurfaceInteraction, dstdx: &mut Vector2f, dstdy: &mut Vector2f) -> Point2f {
        let st = self.cylinder(&si.p);
        // Compute texture coordinate differentials for cylinder $(u,v)$ mapping
        const delta: Float = 0.01;
        let st_delta_x = self.cylinder(&(si.p + delta * *si.dpdx.read().unwrap()));
        *dstdx = (st_delta_x - st) / delta;
        if dstdx[1] > 0.5 {
            dstdx[1] = 1.0 - dstdx[1];
        }
        else if dstdx[1] < -0.5 {
            dstdx[1] = -(dstdx[1] + 1.0);
        }
        let st_delta_y = self.cylinder(&(si.p + delta * *si.dpdy.read().unwrap()));
        *dstdy = (st_delta_y - st) / delta;
        if dstdy[1] > 0.5 {
            dstdy[1] = 1.0 - dstdy[1];
        }
        else if dstdy[1] < -0.5 {
            dstdy[1] = -(dstdy[1] + 1.0);
        }
        return st
    }
}

pub struct PlanarMapping2D {
    vs: Vector3f,
    vt: Vector3f,
    ds: Float,
    dt: Float
}

impl PlanarMapping2D {
    // default for ds = 0.0, dt = 0.0
    pub fn new(vs: Vector3f, vt: Vector3f, ds: Float, dt: Float) -> PlanarMapping2D {
        PlanarMapping2D { vs, vt, ds, dt }
    }
}

impl TextureMapping2D for PlanarMapping2D {
    fn map(&self, si: &SurfaceInteraction, dstdx: &mut Vector2f, dstdy: &mut Vector2f) -> Point2f {
        let vec = Vector3f::from(si.p);
        let dpdx = *si.dpdx.read().unwrap();
        let dpdy = *si.dpdy.read().unwrap();
        *dstdx = Vector2f::new(dpdx.dot(&self.vs), dpdx.dot(&self.vt));
        *dstdy = Vector2f::new(dpdy.dot(&self.vs), dpdy.dot(&self.vt));
        Point2f::new(self.ds + vec.dot(&self.vs), self.dt + vec.dot(&self.vt))
    }
}

pub trait TextureMapping3D {
    fn map(&self, si: &SurfaceInteraction, dpdx: &mut Vector3f, dpdy: &mut Vector3f) -> Point3f;
}

pub struct IdentityMapping3D {
    world_to_texture: Transform
}

impl IdentityMapping3D {
    pub fn new(world_to_texture: Transform) -> IdentityMapping3D {
        IdentityMapping3D { world_to_texture }
    }
}

impl TextureMapping3D for IdentityMapping3D {
    fn map(&self, si: &SurfaceInteraction, dpdx: &mut Vector3f, dpdy: &mut Vector3f) -> Point3f {
        *dpdx = self.world_to_texture.transform_vector(&*si.dpdx.read().unwrap());
        *dpdy = self.world_to_texture.transform_vector(&*si.dpdy.read().unwrap());
        self.world_to_texture.transform_point(&si.p)
    }
}

#[inline]
pub fn smooth_step(min: Float, max: Float, value: Float) -> Float {
    let v = clamp((value -min) / (max - min), 0.0, 1.0);
    v * v * (-2.0 * v + 3.0)
}

pub fn lanczos(mut x: Float, tau: Float) -> Float {
    x = x.abs();
    if x < 1e-5 {
        return 1.0;
    }
    if x > 1.0 {
        return 0.0;
    }
    x *= PI;
    let s = (x * tau).sin() / (x * tau);
    let lanczos_ = x.sin() / x;
    s * lanczos_
}

// defaults are y = 0.5, z = 0.5
pub fn noise(x: Float, y: Float, z: Float) -> Float {
    // Compute noise cell coordinates and offsets
    let mut ix = x.floor() as usize;
    let mut iy = y.floor() as usize;
    let mut iz = z.floor() as usize;
    let dx = x - ix as Float;
    let dy = y - ix as Float;
    let dz = z - ix as Float;

    // Compute gradient weights
    ix &= NOISE_PERM_SIZE - 1;
    iy &= NOISE_PERM_SIZE - 1;
    iz &= NOISE_PERM_SIZE - 1;
    let w000 = grad(ix, iy, iz, dx, dy, dz);
    let w100 = grad(ix + 1, iy, iz, dx - 1.0, dy, dz);
    let w010 = grad(ix, iy + 1, iz, dx, dy - 1.0, dz);
    let w110 = grad(ix + 1, iy + 1, iz, dx - 1.0, dy - 1.0, dz);
    let w001 = grad(ix, iy, iz + 1, dx, dy, dz - 1.0);
    let w101 = grad(ix + 1, iy, iz + 1, dx - 1.0, dy, dz - 1.0);
    let w011 = grad(ix, iy + 1, iz + 1, dx, dy - 1.0, dz - 1.0);
    let w111 = grad(ix + 1, iy + 1, iz + 1, dx - 1.0, dy - 1.0, dz - 1.0);

    // Compute trilinear interpolation of weights
    let wx = noise_weight(dx);
    let wy = noise_weight(dy);
    let wz = noise_weight(dz);
    let x00 = lerp(wx, w000, w100);
    let x10 = lerp(wx, w010, w110);
    let x01 = lerp(wx, w001, w101);
    let x11 = lerp(wx, w011, w111);
    let y0 = lerp(wy, x00, x10);
    let y1 = lerp(wy, x01, x11);
    lerp(wz, y0, y1)
}

pub fn noise_x(x: Float) -> Float {
    noise(x, 0.5, 0.5)
}

pub fn noise_xy(x: Float, y: Float) -> Float {
    noise(x, y, 0.5)
}

pub fn noise_from_point(p: &Point3f) -> Float {
    noise(p.x, p.y, p.z)
}

pub fn fbm(
    p: &Point3f,
    dpdx: &Vector3f,
    dpdy: &Vector3f,
    omega: Float,
    octaves: u32
) -> Float {
    // Compute number of octaves for antialiased FBm
    let len2 = dpdx.length_squared().max(dpdy.length_squared());
    let n = clamp(1.0 - 0.5 * len2.log2(), 0.0, octaves as Float);
    let n_int = n.floor() as i32;

    // Compute sum of octaves of noise for FBm
    let mut sum: Float = 0.0;
    let mut lambda: Float = 1.0;
    let mut o: Float = 1.0;
    for i in 0..n_int {
        sum += o * noise_from_point(&(lambda * *p));
        lambda *= 1.99;
        o *= omega;
    }
    let n_partial = n - n_int as Float;
    sum += o * smooth_step(0.3, 0.7, n_partial) * noise_from_point(&(lambda * *p));
    sum
}

pub fn turbulence(
    p: &Point3f,
    dpdx: &Vector3f,
    dpdy: &Vector3f,
    omega: Float,
    octaves: u32
) -> Float {
    // Compute number of octaves for antialiased FBm
    let len2 = dpdx.length_squared().max(dpdy.length_squared());
    let n = clamp(1.0 - 0.5 * len2.log2(), 0.0, octaves as Float);
    let n_int = n.floor() as i32;

    // Compute sum of octaves of noise for turbulence
    let mut sum: Float = 0.0;
    let mut lambda: Float = 1.0;
    let mut o: Float = 1.0;
    for i in 0..n_int {
        sum += o * noise_from_point(&(lambda * *p)).abs();
        lambda *= 1.99;
        o *= omega;
    }

    // Account for contributions of clamped octaves in turbulence
    let n_partial = n - n_int as Float;
    sum += o * lerp(smooth_step(0.3, 0.7, n_partial), 0.2,
        noise_from_point(&(lambda * *p)).abs());
    for i in n_int as u32..octaves {
        sum += o * 0.2;
        o *= omega;
    }
    sum
}

#[inline]
/// Returns the gradient vector for a particular integer lattice point.
pub fn grad(x: usize, y: usize, z: usize, dx: Float, dy: Float, dz: Float) -> Float {
    let h = NOISE_PERM[NOISE_PERM[NOISE_PERM[x] as usize + y] as usize + z];
    h &= 15;
    let u = if h < 8 || h == 12 || h == 13 { dx } else { dy };
    let v = if h < 4 || h == 12 || h == 13 { dy } else { dz };
    (if h & 1 != 0 { -u } else { u }) + (if h & 2 != 0 { -v } else { v })
}

/// A smothing function to ensure that the noise function has first- and second-derivative continuity as lookup points move between lattice cells.
#[inline]
pub fn noise_weight(t: Float) -> Float {
    let t3 = t * t * t;
    let t4 = t3 * t;
    6.0 * t4 * t - 15.0 * t4 + 10.0 * t3
}

const NOISE_PERM_SIZE: usize = 256;
const NOISE_PERM: [u8; 2 * NOISE_PERM_SIZE] = [
    151, 160, 137, 91, 90, 15, 131, 13, 201, 95, 96, 53, 194, 233, 7, 225, 140,
    36, 103, 30, 69, 142,
    // Remainder of the noise permutation table
    8, 99, 37, 240, 21, 10, 23, 190, 6, 148, 247, 120, 234, 75, 0, 26, 197, 62,
    94, 252, 219, 203, 117, 35, 11, 32, 57, 177, 33, 88, 237, 149, 56, 87, 174,
    20, 125, 136, 171, 168, 68, 175, 74, 165, 71, 134, 139, 48, 27, 166, 77,
    146, 158, 231, 83, 111, 229, 122, 60, 211, 133, 230, 220, 105, 92, 41, 55,
    46, 245, 40, 244, 102, 143, 54, 65, 25, 63, 161, 1, 216, 80, 73, 209, 76,
    132, 187, 208, 89, 18, 169, 200, 196, 135, 130, 116, 188, 159, 86, 164, 100,
    109, 198, 173, 186, 3, 64, 52, 217, 226, 250, 124, 123, 5, 202, 38, 147,
    118, 126, 255, 82, 85, 212, 207, 206, 59, 227, 47, 16, 58, 17, 182, 189, 28,
    42, 223, 183, 170, 213, 119, 248, 152, 2, 44, 154, 163, 70, 221, 153, 101,
    155, 167, 43, 172, 9, 129, 22, 39, 253, 19, 98, 108, 110, 79, 113, 224, 232,
    178, 185, 112, 104, 218, 246, 97, 228, 251, 34, 242, 193, 238, 210, 144, 12,
    191, 179, 162, 241, 81, 51, 145, 235, 249, 14, 239, 107, 49, 192, 214, 31,
    181, 199, 106, 157, 184, 84, 204, 176, 115, 121, 50, 45, 127, 4, 150, 254,
    138, 236, 205, 93, 222, 114, 67, 29, 24, 72, 243, 141, 128, 195, 78, 66,
    215, 61, 156, 180, 151, 160, 137, 91, 90, 15, 131, 13, 201, 95, 96, 53, 194,
    233, 7, 225, 140, 36, 103, 30, 69, 142, 8, 99, 37, 240, 21, 10, 23, 190, 6,
    148, 247, 120, 234, 75, 0, 26, 197, 62, 94, 252, 219, 203, 117, 35, 11, 32,
    57, 177, 33, 88, 237, 149, 56, 87, 174, 20, 125, 136, 171, 168, 68, 175, 74,
    165, 71, 134, 139, 48, 27, 166, 77, 146, 158, 231, 83, 111, 229, 122, 60,
    211, 133, 230, 220, 105, 92, 41, 55, 46, 245, 40, 244, 102, 143, 54, 65, 25,
    63, 161, 1, 216, 80, 73, 209, 76, 132, 187, 208, 89, 18, 169, 200, 196, 135,
    130, 116, 188, 159, 86, 164, 100, 109, 198, 173, 186, 3, 64, 52, 217, 226,
    250, 124, 123, 5, 202, 38, 147, 118, 126, 255, 82, 85, 212, 207, 206, 59,
    227, 47, 16, 58, 17, 182, 189, 28, 42, 223, 183, 170, 213, 119, 248, 152, 2,
    44, 154, 163, 70, 221, 153, 101, 155, 167, 43, 172, 9, 129, 22, 39, 253, 19,
    98, 108, 110, 79, 113, 224, 232, 178, 185, 112, 104, 218, 246, 97, 228, 251,
    34, 242, 193, 238, 210, 144, 12, 191, 179, 162, 241, 81, 51, 145, 235, 249,
    14, 239, 107, 49, 192, 214, 31, 181, 199, 106, 157, 184, 84, 204, 176, 115,
    121, 50, 45, 127, 4, 150, 254, 138, 236, 205, 93, 222, 114, 67, 29, 24, 72,
    243, 141, 128, 195, 78, 66, 215, 61, 156, 180
]; 