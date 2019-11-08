use super::pbrt::Float;

#[derive(Debug, Copy, Clone)]
pub struct Matrix4x4 {
    pub m: [[Float;4]; 4]
}

impl Matrix4x4 {
    pub fn new(
        t00: Float,
        t01: Float,
        t02: Float,
        t03: Float,
        t10: Float,
        t11: Float,
        t12: Float,
        t13: Float,
        t20: Float,
        t21: Float,
        t22: Float,
        t23: Float,
        t30: Float,
        t31: Float,
        t32: Float,
        t33: Float
    ) -> Matrix4x4 {
        Matrix4x4{
            m: [
                [t00, t01, t02, t03],
                [t10, t11, t12, t13],
                [t20, t21, t22, t23],
                [t30, t31, t32, t33]
            ]
        }
    }

    /// Transpose the Matrix4x4
    #[inline]
    pub fn transpose(&self) -> Matrix4x4 {
        Matrix4x4 {
            m: [
                [self.m[0][0], self.m[1][0], self.m[2][0], self.m[3][0]],
                [self.m[0][1], self.m[1][1], self.m[2][1], self.m[3][1]],
                [self.m[0][2], self.m[1][2], self.m[2][2], self.m[3][2]],
                [self.m[0][3], self.m[1][3], self.m[2][3], self.m[3][3]],
            ]
        }
    }

    /// Multiply two Matrix4x4
    pub fn mul(&self, m2: &Matrix4x4) -> Matrix4x4 {
        // This is the unrolled version of http://www.pbr-book.org/3ed-2018/Utilities/Mathematical_Routines.html#fragment-Matrix4x4PublicMethods-1
        Matrix4x4{
            m: [
                [
                    self.m[0][0] * m2.m[0][0] + self.m[0][1] * m2.m[0][0] + self.m[0][2] * m2.m[2][0] + self.m[0][3] * m2.m[3][0],
                    self.m[0][0] * m2.m[0][1] + self.m[0][1] * m2.m[0][1] + self.m[0][2] * m2.m[2][1] + self.m[0][3] * m2.m[3][1],
                    self.m[0][0] * m2.m[0][2] + self.m[0][1] * m2.m[0][2] + self.m[0][2] * m2.m[2][2] + self.m[0][3] * m2.m[3][2],
                    self.m[0][0] * m2.m[0][3] + self.m[0][1] * m2.m[0][3] + self.m[0][2] * m2.m[2][3] + self.m[0][3] * m2.m[3][3],
                ],
                [
                    self.m[1][0] * m2.m[0][0] + self.m[1][1] * m2.m[1][0] + self.m[1][2] * m2.m[2][0] + self.m[1][3] * m2.m[3][0],
                    self.m[1][0] * m2.m[0][1] + self.m[1][1] * m2.m[1][1] + self.m[1][2] * m2.m[2][1] + self.m[1][3] * m2.m[3][1],
                    self.m[1][0] * m2.m[0][2] + self.m[1][1] * m2.m[1][2] + self.m[1][2] * m2.m[2][2] + self.m[1][3] * m2.m[3][2],
                    self.m[1][0] * m2.m[0][3] + self.m[1][1] * m2.m[1][3] + self.m[1][2] * m2.m[2][3] + self.m[1][3] * m2.m[3][3],
                ],
                [
                    self.m[2][0] * m2.m[0][0] + self.m[2][1] * m2.m[2][0] + self.m[2][2] * m2.m[2][0] + self.m[2][3] * m2.m[3][0],
                    self.m[2][0] * m2.m[0][1] + self.m[2][1] * m2.m[2][1] + self.m[2][2] * m2.m[2][1] + self.m[2][3] * m2.m[3][1],
                    self.m[2][0] * m2.m[0][2] + self.m[2][1] * m2.m[2][2] + self.m[2][2] * m2.m[2][2] + self.m[2][3] * m2.m[3][2],
                    self.m[2][0] * m2.m[0][3] + self.m[2][1] * m2.m[2][3] + self.m[2][2] * m2.m[2][3] + self.m[2][3] * m2.m[3][3],
                ],
                [
                    self.m[3][0] * m2.m[0][0] + self.m[3][1] * m2.m[3][0] + self.m[3][2] * m2.m[2][0] + self.m[3][3] * m2.m[3][0],
                    self.m[3][0] * m2.m[0][1] + self.m[3][1] * m2.m[3][1] + self.m[3][2] * m2.m[2][1] + self.m[3][3] * m2.m[3][1],
                    self.m[3][0] * m2.m[0][2] + self.m[3][1] * m2.m[3][2] + self.m[3][2] * m2.m[2][2] + self.m[3][3] * m2.m[3][2],
                    self.m[3][0] * m2.m[0][3] + self.m[3][1] * m2.m[3][3] + self.m[3][2] * m2.m[2][3] + self.m[3][3] * m2.m[3][3],
                ]
            ]
        }
    }

    /// Return the inverse of the Matrix4x4
    pub fn inverse(&self) -> Matrix4x4 {
        let mut indxc = vec![0; 4];
        let mut indxr = vec![0; 4];
        let mut ipiv = vec![0; 4];
        let mut minv: Matrix4x4 = Matrix4x4::new(
            self.m[0][0], self.m[0][1], self.m[0][2], self.m[0][3], self.m[1][0], self.m[1][1], self.m[1][2], self.m[1][3],
            self.m[2][0], self.m[2][1], self.m[2][2], self.m[2][3], self.m[3][0], self.m[3][1], self.m[3][2], self.m[3][3],
        );
        for i in 0..4 {
            let mut irow = 0;
            let mut icol = 0;
            let mut big: Float = 0.0;
            // choose pivot
            for j in 0..4 {
                if ipiv[j] != 1 {
                    for k in 0..4 {
                        if ipiv[k] == 0 {
                            let abs: Float = (minv.m[j][k]).abs();
                            if abs >= big {
                                big = abs;
                                irow = j;
                                icol = k;
                            }
                        } else {
                            if ipiv[k] > 1 {
                                println!("Singular matrix in MatrixInvert");
                            }
                        }
                    }
                }
            }
            ipiv[icol] = ipiv[icol] + 1;
            // swap rows _irow_ and _icol_ for pivot
            if irow != icol {
                for k in 0..4 {
                    // C++: std::swap(minv[irow][k], minv[icol][k]);
                    let swap = minv.m[irow][k];
                    minv.m[irow][k] = minv.m[icol][k];
                    minv.m[icol][k] = swap;
                }
            }
            indxr[i] = irow;
            indxc[i] = icol;
            if minv.m[icol][icol] == 0.0 {
                println!("Singular matrix in MatrixInvert");
            }
            // set $m[icol][icol]$ to one by scaling row _icol_ appropriately
            let pivinv: Float = 1.0 / minv.m[icol][icol];
            minv.m[icol][icol] = 1.0;
            for j in 0..4 {
                minv.m[icol][j] = minv.m[icol][j] * pivinv;
            }
            // subtract this row from others to zero out their columns
            for j in 0..4 {
                if j != icol {
                    let save: Float = minv.m[j][icol];
                    minv.m[j][icol] = 0.0;
                    for k in 0..4 {
                        minv.m[j][k] = minv.m[j][k] - (minv.m[icol][k] * save);
                    }
                }
            }
        }
        // swap columns to reflect permutation
        for i in 0..4 {
            let j = 3 - i;
            if indxr[j] != indxc[j] {
                for k in 0..4 {
                    // C++: std::swap(minv[k][indxr[j]], minv[k][indxc[j]]);
                    let swap = minv.m[k][indxr[j]];
                    minv.m[k][indxr[j]] = minv.m[k][indxc[j]];
                    minv.m[k][indxc[j]] = swap;
                }
            }
        }
        minv
    }
}

/// Default is the identity matrix.
impl Default for Matrix4x4 {
    fn default() -> Self {
        Matrix4x4{
            m: [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0]
            ]
        }
    }
}

impl PartialEq for Matrix4x4 {
    fn eq(&self, rhs: &Matrix4x4) -> bool {
        self.m[0][0] == rhs.m[0][0] &&
        self.m[0][1] == rhs.m[0][1] &&
        self.m[0][2] == rhs.m[0][2] &&
        self.m[0][3] == rhs.m[0][3] &&
        self.m[1][0] == rhs.m[1][0] &&
        self.m[1][1] == rhs.m[1][1] &&
        self.m[1][2] == rhs.m[1][2] &&
        self.m[1][3] == rhs.m[1][3] &&
        self.m[2][0] == rhs.m[2][0] &&
        self.m[2][1] == rhs.m[2][1] &&
        self.m[2][2] == rhs.m[2][2] &&
        self.m[2][3] == rhs.m[2][3] &&
        self.m[3][0] == rhs.m[3][0] &&
        self.m[3][1] == rhs.m[3][1] &&
        self.m[3][2] == rhs.m[3][2] &&
        self.m[3][3] == rhs.m[3][3]
    }
}