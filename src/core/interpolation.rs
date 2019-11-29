use super::pbrt::{Float, find_interval};

// Spline Interpolation functions
pub fn catmull_rom(nodes: &[Float], values: &[Float], x: Float) -> Float {
    if !(x >= nodes[0] && x <= nodes[nodes.len() - 1]) {
        return 0.0;
    }
    let idx = find_interval(nodes.len(), |i| nodes[i] <= x);
    let x0 = nodes[idx];
    let x1 = nodes[idx + 1];
    let f0 = values[idx];
    let f1 = values[idx + 1];
    let width = x1 - x0;
    let mut d0: Float = 0.0;
    let mut d1: Float = 0.0;
    if idx > 0 {
        d0 = width * (f1 - values[idx - 1]) / (x1 - nodes[idx - 1]);
    } else {
        d0 = f1 - f0;
    }

    if idx + 2 < nodes.len() {
        d1 = width * (values[idx + 2] - f0) / (nodes[idx + 2] - x0);
    } else {
        d1 = f1 - f0;
    }

    let t = (x - x0) / (x1 - x0);
    let t2 = t * t;
    let t3 = t2 * t;
    (2.0 * t3 - 3.0 * t2 + 1.0) * f0 + (-2.0 * t3 + 3.0 * t2) * f1 +
           (t3 - 2.0 * t2 + t) * d0 + (t3 - t2) * d1
}

pub fn catmull_rom_weights(nodes: &[Float], x: Float, offset: &mut usize, weights: &mut [Float; 4]) -> bool {
    // Return _false_ if _x_ is out of bounds
    if !(x >= nodes[0] && x <= nodes[nodes.len() - 1]) {
        return false;
    }

    // Search for the interval _idx_ containing _x_
    let idx = find_interval(nodes.len(), |i| nodes[i] <= x);
    *offset = idx - 1;
    let x0 = nodes[idx];
    let x1 = nodes[idx + 1];

    // Compute the $t$ parameter and powers
    let t = (x - x0) / (x1 - x0);
    let t2 = t * t;
    let t3 = t2 * t;

    // Compute initial node weights $w_1$ and $w_2$
    weights[1] = 2.0 * t3 - 3.0 * t2 + 1.0;
    weights[2] = -2.0 * t3 + 3.0 * t2;

    // Compute first node weight $w_0$
    if idx > 0 {
        let w0 = (t3 - 2.0 * t2 + t) * (x1 - x0) / (x1 - nodes[idx - 1]);
        weights[0] = -w0;
        weights[2] += w0;
    } else {
        let w0 = t3 - 2.0 * t2 + t;
        weights[0] = 0.0;
        weights[1] -= w0;
        weights[2] += w0;
    }

    // Compute last node weight $w_3$
    if idx + 2 < nodes.len() {
        let w3 = (t3 - t2) * (x1 - x0) / (nodes[idx + 2] - x0);
        weights[1] -= w3;
        weights[3] = w3;
    } else {
        let w3 = t3 - t2;
        weights[1] -= w3;
        weights[2] += w3;
        weights[3] = 0.0;
    }
    true
}

pub fn sample_catmull_rom(
    n: usize,
    x: &[Float],
    f: &[Float],
    nodes: &[Float],
    u: Float,
    fval: Option<&mut Float>,
    pdf: Option<&mut Float>
) -> Float {
    // Map _u_ to a spline interval by inverting _F_
    u *= nodes[n - 1];
    let i = find_interval(nodes.len(), |i| nodes[i] <= u);

    // Look up $x_i$ and function values of spline segment _i_
    let x0 = x[i];
    let x1 = x[i + 1];
    let f0 = f[i];
    let f1 = f[i + 1];
    let width = x1 - x0;

    // Approximate derivatives using finite differences
    let mut d0: Float = 0.0;
    let mut d1: Float = 0.0;
    if i > 0 {
        d0 = width * (f1 - f[i - 1]) / (x1 - x[i - 1]);
    } else {
        d0 = f1 - f0;
    }
    if i + 2 < n {
        d1 = width * (f[i + 2] - f0) / (x[i + 2] - x0);
    } else {
        d1 = f1 - f0;
    }

    // Re-scale _u_ for continous spline sampling step
    u = (u - nodes[i]) / width;

    // Invert definite integral over spline segment and solution

    // Set initial guess for $t$ by importance sampling a linear interpolant
    let mut t: Float = 0.0;
    if f0 != f1 {
        t = (f0 * f0 + 2.0 * u * (f1 - f0)).max(0.0).sqrt() / (f0 - f1);
    } else {
        t = u / f0;
    }
    let mut a: Float = 0.0;
    let mut b: Float = 1.0;
    let mut nodes_hat: Float = 0.0;
    let mut fhat: Float = 0.0;
    loop {
        // Fall back to a bisection step when _t_ is out of bounds
        if !(t > a && t < b) {
            t = 0.5 * (a + b);
        }

        // Evaluate target function and its derivative in Horner form
        nodes_hat = t * (f0 +
                    t * (0.5 * d0 +
                        t * ((1.0 / 3.0) * (-2.0 * d0 - d1) + f1 - f0 +
                            t * (0.25 * (d0 + d1) + 0.5 * (f0 - f1)))));
        fhat = f0 +
            t * (d0 +
                    t * (-2.0 * d0 - d1 + 3.0 * (f1 - f0) +
                        t * (d0 + d1 + 2.0 * (f0 - f1))));

        // Stop the iteration if converged
        if (nodes_hat - u).abs() < 1e-6 || b - a < 1e-6 {
            break;
        }

        // Update bisection bounds using updated _t_
        if nodes_hat - u < 0.0 {
            a = t;
        } else {
            b = t;
        }

        // Perform a Newton step
        t -= (nodes_hat - u) / fhat;
    }

    // Return the sample position and function value
    if let Some(fval) = fval {
        *fval = fhat;
    }
    if let Some(pdf) = pdf {
        *pdf = fhat / nodes[nodes.len() - 1];
    }
    x0 + width * t
}

pub fn sample_catmull_rom_2d(
    nodes1: &[Float],
    nodes2: &[Float],
    values: &[Float],
    cdf: &[Float],
    alpha: Float,
    mut u: Float,
    fval: Option<&mut Float>,
    pdf: Option<&mut Float>
) -> Float {
    // Determine offset and coefficients for the _alpha_ parameter
    let mut offset: usize = 0;
    let mut weights: [Float; 4] = [0.0; 4];
    if !catmull_rom_weights(nodes1, alpha, &mut offset, &mut weights) {
        return 0.0;
    }

    // Define a lambda function to interpolate table entries
    let interpolate = |array: &[Float], idx: usize|{
        let mut value: Float = 0.0;
        for i in 0..4 {
            if weights[i] != 0.0 {
                value += array[(offset + i) * nodes2.len() + idx] * weights[i];
            }
        }
        value
    };

    // Map _u_ to a spline interval by inverting the interpolated _cdf_
    let maximum = interpolate(cdf, nodes2.len() - 1);
    u *= maximum;
    let idx =
        find_interval(nodes2.len(), |i| interpolate(cdf, i) <= u);

    // Look up node positions and interpolated function values
    let f0 = interpolate(values, idx);
    let f1 = interpolate(values, idx + 1);
    let x0 = nodes2[idx];
    let x1 = nodes2[idx + 1];
    let width = x1 - x0;
    let mut d0: Float = 0.0;
    let mut d1: Float = 0.0;

    // Re-scale _u_ using the interpolated _cdf_
    u = (u - interpolate(cdf, idx)) / width;

    // Approximate derivatives using finite differences of the interpolant
    if idx > 0 {
        d0 = width * (f1 - interpolate(values, idx - 1)) /
             (x1 - nodes2[idx - 1]);
    } else {
        d0 = f1 - f0;
    }
    if idx + 2 < nodes2.len() {
        d1 = width * (interpolate(values, idx + 2) - f0) /
             (nodes2[idx + 2] - x0);
    } else {
        d1 = f1 - f0;
    }

    // Invert definite integral over spline segment and return solution

    // Set initial guess for $t$ by importance sampling a linear interpolant
    let mut t: Float = 0.0;
    if f0 != f1 {
        t = f0 - (f0 * f0 + 2.0 * u * (f1 - f0)).max(0.0).sqrt() / (f0 - f1);
    } else {
        t = u / f0;
    }
    let mut a: Float = 0.0;
    let mut b: Float = 1.0;
    let mut nodes_hat: Float = 0.0;
    let mut fhat: Float = 0.0;
    loop {
        // Fall back to a bisection step when _t_ is out of bounds
        if !(t >= a && t <= b) {
            t = 0.5 * (a + b);
        }

        // Evaluate target function and its derivative in Horner form
        nodes_hat = t * (f0 +
                    t * (0.5 * d0 +
                         t * ((1.0 / 3.0) * (-2.0 * d0 - d1) + f1 - f0 +
                              t * (0.25 * (d0 + d1) + 0.5 * (f0 - f1)))));
        fhat = f0 +
               t * (d0 +
                    t * (-2.0 * d0 - d1 + 3.0 * (f1 - f0) +
                         t * (d0 + d1 + 2.0 * (f0 - f1))));

        // Stop the iteration if converged
        if (nodes_hat - u).abs() < 1e-6 || b - a < 1e-6 {
            break
        };

        // Update bisection bounds using updated _t_
        if nodes_hat - u < 0.0 {
            a = t;
        } else {
            b = t;
        }

        // Perform a Newton step
        t -= (nodes_hat - u) / fhat;
    }

    // Return the sample position and function value
    if let Some(fval) = fval {
        *fval = fhat;
    }
    if let Some(pdf) = pdf {
        *pdf = fhat / maximum;
    }
    x0 + width * t
}

pub fn integrate_catmull_rom(
    x: &[Float],
    values: &[Float],
    cdf: &mut [Float]
) -> Float {
    let mut sum: Float = 0.0;
    cdf[0] = 0.0;
    for i in 0..x.len() - 1 {
        // Look up $x_i$ and function values of spline segment _i_
        let x0 = x[i];
        let x1 = x[i + 1];
        let f0 = values[i];
        let f1 = values[i + 1];
        let width = x1 - x0;

        // Approximate derivatives using finite differences
        let mut d0: Float = 0.0;
        let mut d1: Float = 0.0;
        if i > 0 {
            d0 = width * (f1 - values[i - 1]) / (x1 - x[i - 1]);
        } else {
            d0 = f1 - f0;
        }
        if i + 2 < x.len() {
            d1 = width * (values[i + 2] - f0) / (x[i + 2] - x0);
        } else {
            d1 = f1 - f0;
        }

        // Keep a running sum and build a cumulative distribution function
        sum += ((d0 - d1) * (1.0 / 12.0) + (f0 + f1) * 0.5) * width;
        cdf[i + 1] = sum;
    }
    sum
}

pub fn invert_catmull_rom(
    x: &[Float],
    values: &[Float],
    u: Float
) -> Float {
    let n = x.len();
    // Stop when _u_ is out of bounds
    if !(u > values[0]) {
        return x[0];
    } else if !(u < values[n - 1]) {
        return x[n - 1];
    }

    // Map _u_ to a spline interval by inverting _values_
    let i = find_interval(n, |i| values[i] <= u);

    // Look up $x_i$ and function values of spline segment _i_
    let x0 = x[i];
    let x1 = x[i + 1];
    let f0 = values[i];
    let f1 = values[i + 1];
    let width = x1 - x0;

    // Approximate derivatives using finite differences
    let mut d0: Float = 0.0;
    let mut d1: Float = 0.0;
    if i > 0 {
        d0 = width * (f1 - values[i - 1]) / (x1 - x[i - 1]);
    } else {
        d0 = f1 - f0;
    }
    if i + 2 < n {
        d1 = width * (values[i + 2] - f0) / (x[i + 2] - x0);
    } else {
        d1 = f1 - f0;
    }

    // Invert the spline interpolant using Newton-Bisection
    let a: Float = 0.0;
    let b: Float = 1.0;
    let t: Float = 0.5;
    let mut f_hat: Float = 0.0;
    let mut fhat: Float = 0.0;
    loop {
        // Fall back to a bisection step when _t_ is out of bounds
        if !(t > a && t < b) {
            t = 0.5 * (a + b);
        }

        // Compute powers of _t_
        let t2 = t * t;
        let t3 = t2 * t;

        // Set _Fhat_ using Equation (8.27)
        f_hat = (2.0 * t3 - 3.0 * t2 + 1.0) * f0 + (-2.0 * t3 + 3.0 * t2) * f1 +
               (t3 - 2.0 * t2 + t) * d0 + (t3 - t2) * d1;

        // Set _fhat_ using Equation (not present)
        fhat = (6.0 * t2 - 6.0 * t) * f0 + (-6.0 * t2 + 6.0 * t) * f1 +
               (3.0 * t2 - 4.0 * t + 1.0) * d0 + (3.0 * t2 - 2.0 * t) * d1;

        // Stop the iteration if converged
        if (f_hat - u).abs() < 1e-6 || b - a < 1e-6  {
            break;
        }

        // Update bisection bounds using updated _t_
        if f_hat - u < 0.0 {
            a = t;
        } else {
            b = t;
        }

        // Perform a Newton step
        t -= (f_hat - u) / fhat;
    }
    x0 + t * width

}

// Fourier Interpolation
pub fn fourier(a: &[Float], m: usize, cos_phi: f64) -> Float {
    let mut value: f64 = 0.0;
    // Initialize cosine iterates
    let mut cos_k_minus_one_phi: f64 = cos_phi;
    let mut cos_k_phi: f64 = 1.0;
    for k in 0..m {
        // Add the current summand and update the cosine iterates
        value += a[k] as f64 * cos_k_phi;
        let cos_k_plus_one_phi = 2.0 * cos_phi * cos_k_phi - cos_k_minus_one_phi;
        cos_k_minus_one_phi = cos_k_phi;
        cos_k_phi = cos_k_plus_one_phi;
    }
    value as f32
}

pub fn sample_fourier(
    ak: &[Float],
    recip: &[Float],
    m: usize,
    mut u: Float,
    pdf: &mut Float,
    phi_ptr: &mut Float
) -> Float {
    
    // Pick a side and declare bisection variables
    let flip = u >= 0.5;
    if flip {
        u = 1.0 - 2.0 * (u - 0.5);
    } else {
        u *= 2.0;
    }
    let mut a: f64 = 0.0;
    let mut b: f64 = std::f64::consts::PI;
    let mut phi: f64 = std::f64::consts::PI;
    let mut F: f64 = 0.0;
    let mut f: f64 = 0.0;
    loop {
        // Evaluate $F(\phi)$ and its derivative $f(\phi)$

        // Initialize sine and cosine iterates
        let cos_phi: f64 = phi.cos();
        let sin_phi: f64 = (1.0 - cos_phi * cos_phi).max(0.0).sqrt();
        let mut cos_phi_prev: f64 = cos_phi;
        let mut cos_phi_cur: f64 = 1.0;
        let mut sin_phi_prev: f64 = -sin_phi;
        let mut sin_phi_cur: f64 = 0.0;

        // Initialize _F_ and _f_ with the first series term
        let mut c_f: f64 = ak[0] as f64 * phi;
        let mut f: f64 = ak[0] as f64;
        for k in 1..m {
            // Compute next sine and cosine iterates
            let sin_phi_next: f64 = 2.0 * cos_phi * sin_phi_cur - sin_phi_prev;
            let cos_phi_next: f64 = 2.0 * cos_phi * cos_phi_cur - cos_phi_prev;
            sin_phi_prev = sin_phi_cur;
            sin_phi_cur = sin_phi_next;
            cos_phi_prev = cos_phi_cur;
            cos_phi_cur = cos_phi_next;

            // Add the next series term to _F_ and _f_
            c_f += ak[k] as f64 * recip[k] as f64 * sin_phi_next;
            f += ak[k] as f64 * cos_phi_next;
        }
        c_f -= u as f64 * ak[0] as f64 * std::f64::consts::PI;

        // Update bisection bounds using updated $\phi$
        if c_f > 0.0 {
            b = phi;
        } else {
            a = phi;
        }

        // Stop the Fourier bisection iteration if converged
        if c_f.abs() < 1e-6 || b - a < 1e-6 {
            break
        };

        // Perform a Newton step given $f(\phi)$ and $F(\phi)$
        phi -= c_f / f;

        // Fall back to a bisection step when $\phi$ is out of bounds
        if !(phi > a && phi < b) {
            phi = 0.5 * (a + b);
        }
    }
    // Potentially flip $\phi$ and return the result
    if flip {
        phi = 2.0 * std::f64::consts::PI - phi;
    }
    *pdf = (1.0 / std::f64::consts::PI * 2.0 * f / ak[0] as f64) as Float;
    *phi_ptr = phi as Float;
    f as Float
}