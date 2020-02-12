use super::pbrt::{Float, Spectrum};
use super::geometry::{Point2f, Vector2f, Point3f, Vector3f, Normal3f};
use super::spectrum::{blackbody_normalized, CIE_lambda, SpectrumType};
use super::floatfile::read_float_file;
use super::utils::float_to_string_general;
use std::sync::Arc;
use hashbrown::HashMap;
use std::fmt;
use num::cast;
use std::path::Path;

#[derive(Debug, Default, Clone)]
pub struct ParamSetItem<T> {
    pub name: String,
    pub values: Vec<T>,
    pub looked_up: bool
}

impl<T> ParamSetItem<T> {
    pub fn new(name: String, val: T) -> ParamSetItem<T> {
        ParamSetItem {
            name: name,
            values: vec![val],
            looked_up: false
        }
    }

    pub fn from_vec(name: String, val: Vec<T>) -> ParamSetItem<T>
    where
    T: Clone
    {
        ParamSetItem {
            name: name,
            values: val.clone(),
            looked_up: false
        }
    }
}

#[derive(Debug, Default, Clone)]
pub struct ParamSet {
    pub bools: Vec<Arc<ParamSetItem<bool>>>,
    pub ints: Vec<Arc<ParamSetItem<i32>>>,
    pub floats: Vec<Arc<ParamSetItem<Float>>>,
    pub point2fs: Vec<Arc<ParamSetItem<Point2f>>>,
    pub vector2fs: Vec<Arc<ParamSetItem<Vector2f>>>,
    pub point3fs: Vec<Arc<ParamSetItem<Point3f>>>,
    pub vector3fs: Vec<Arc<ParamSetItem<Vector3f>>>,
    pub normals: Vec<Arc<ParamSetItem<Normal3f>>>,
    pub spectra: Vec<Arc<ParamSetItem<Spectrum>>>,
    pub strings: Vec<Arc<ParamSetItem<String>>>,
    pub textures: Vec<Arc<ParamSetItem<String>>>,
    cached_spectra: HashMap<String, Spectrum>
}

impl ParamSet {
    pub fn add_float(&mut self, name: String, values: Vec<Float>) {
        self.erase_float(name);
        self.floats.push(Arc::new(ParamSetItem::from_vec(name, values)));
    }

    pub fn add_int(&mut self, name: String, values: Vec<i32>) {
        self.erase_int(name);
        self.ints.push(Arc::new(ParamSetItem::from_vec(name, values)));
    }

    pub fn add_bool(&mut self, name: String, values: Vec<bool>) {
        self.erase_bool(name);
        self.bools.push(Arc::new(ParamSetItem::from_vec(name, values)));
    }

    pub fn add_point2f(&mut self, name: String, values: Vec<Point2f>) {
        self.erase_point2f(name);
        self.point2fs.push(Arc::new(ParamSetItem::from_vec(name, values)));
    }

    pub fn add_vector2f(&mut self, name: String, values: Vec<Vector2f>) {
        self.erase_vector2f(name);
        self.vector2fs.push(Arc::new(ParamSetItem::from_vec(name, values)));
    }

    pub fn add_point3f(&mut self, name: String, values: Vec<Point3f>) {
        self.erase_point3f(name);
        self.point3fs.push(Arc::new(ParamSetItem::from_vec(name, values)));
    }

    pub fn add_vector3f(&mut self, name: String, values: Vec<Vector3f>) {
        self.erase_vector3f(name);
        self.vector3fs.push(Arc::new(ParamSetItem::from_vec(name, values)));
    }

    pub fn add_normal3f(&mut self, name: String, values: Vec<Normal3f>) {
        self.erase_normal3f(name);
        self.normals.push(Arc::new(ParamSetItem::from_vec(name, values)));
    }

    pub fn add_rgb_spectrum(&mut self, name: String, values: Vec<Float>) {
        self.erase_spectrum(name);
        assert_eq!(values.len() % 3, 0);
        let n_values = values.len() / 3;
        let mut s: Vec<Spectrum> = Vec::with_capacity(n_values);
        for i in 0..n_values {
            let rgb = [values[3 * n_values], values[3 * n_values + 1], values[3 * n_values + 2]];
            s.push(Spectrum::from_rgb(&rgb));
        }
        self.spectra.push(Arc::new(ParamSetItem::from_vec(name, s)));
    }

    pub fn add_xyz_spectrum(&mut self, name: String, values: Vec<Float>) {
        self.erase_spectrum(name);
        assert_eq!(values.len() % 3, 0);
        let n_values = values.len() / 3;
        let mut s: Vec<Spectrum> = Vec::with_capacity(n_values);
        for i in 0..n_values {
            let xyz = [values[3 * n_values], values[3 * n_values + 1], values[3 * n_values + 2]];
            s.push(Spectrum::from_xyz(&xyz, SpectrumType::Reflectance));
        }
        self.spectra.push(Arc::new(ParamSetItem::from_vec(name, s)));
    }

    pub fn add_blackbody_spectrum(&mut self, name: String, values: Vec<Float>) {
        self.erase_spectrum(name);
        assert_eq!(values.len() % 2, 0);    // temperature (K), scale, ...
        let n_values = values.len() / 2;
        let mut s: Vec<Spectrum> = Vec::with_capacity(n_values);
        let mut v: Vec<Float> = vec![0.0; n_values];
        for i in 0..n_values {
            blackbody_normalized(&CIE_lambda, values[2 * i], &mut v);
            let rgb = [values[3 * n_values], values[3 * n_values + 1], values[3 * n_values + 2]];
            s.push(Spectrum::from_sampled(&CIE_lambda, &v) * values[2 * i + 1]);
        }
        self.spectra.push(Arc::new(ParamSetItem::from_vec(name, s)));
    }

    pub fn add_sampled_spectrum(&mut self, name: String, values: Vec<Float>) {
        self.erase_spectrum(name);
        assert_eq!(values.len() % 2, 0);
        let n_values = values.len() / 2;
        let mut wl: Vec<Float> = Vec::with_capacity(n_values);
        let mut v: Vec<Float> = Vec::with_capacity(n_values);
        for i in 0..n_values {
            wl.push(values[2 * i]);
            v.push(values[2 * i + 1]);
        }
        let s = Spectrum::from_sampled(&wl, &v);
        self.spectra.push(Arc::new(ParamSetItem::new(name, s)));
    }

    pub fn add_sampled_spectrum_files(&mut self, name: String, names: Vec<String>) {
        self.erase_spectrum(name);
        let mut s: Vec<Spectrum> = Vec::with_capacity(names.len());
        for i in 0..names.len() {
            let path = Path::new(names[i].as_str());
            let path_string = String::from(path.to_str().unwrap());
            if let Some(spectra) = self.cached_spectra.get(&path_string) {
                s.push(*spectra);
                continue;
            }

            match read_float_file(path_string.as_str()) {
                Ok(vals) => {
                    if vals.len() % 2 != 0 {
                        warn!("Extra value found in spectrum file \"{}\". Ignoring it.", path_string);
                    }
                    let mut wls: Vec<Float> = Vec::new();
                    let mut v: Vec<Float> = Vec::new();
                    for j in 0..vals.len() / 2 {
                        wls.push(vals[2 * j]);
                        v.push(vals[2 * j + 1]);
                    }
                    s.push(Spectrum::from_sampled(&wls, &v));
                }
                Err(_) => {
                    warn!("Unable to read SPD file \"{}\".  Using black distribution.", path_string);
                    s.push(Spectrum::new(0.0));
                }
            }
            self.cached_spectra.insert(path_string, s[i]);
        }
        self.spectra.push(Arc::new(ParamSetItem::from_vec(name, s)));
    }

    pub fn add_string(&mut self, name: String, values: Vec<String>) {
        self.erase_string(name);
        self.strings.push(Arc::new(ParamSetItem::from_vec(name, values)));
    }

    pub fn add_texture(&mut self, name: String, value: String) {
        self.erase_texture(name);
        self.textures.push(Arc::new(ParamSetItem::new(name, value)));
    }

    pub fn erase_int(&mut self, n: String) -> bool {
        for i in 0..self.ints.len() {
            if self.ints[i].name == n {
                self.ints.remove(i);
                return true;
            }
        }
        false
    }

    pub fn erase_bool(&mut self, n: String) -> bool {
        for i in 0..self.bools.len() {
            if self.bools[i].name == n {
                self.bools.remove(i);
                return true;
            }
        }
        false
    }

    pub fn erase_float(&mut self, n: String) -> bool {
        for i in 0..self.floats.len() {
            if self.floats[i].name == n {
                self.floats.remove(i);
                return true;
            }
        }
        false
    }

    pub fn erase_point2f(&mut self, n: String) -> bool {
        for i in 0..self.point2fs.len() {
            if self.point2fs[i].name == n {
                self.point2fs.remove(i);
                return true;
            }
        }
        false
    }

    pub fn erase_vector2f(&mut self, n: String) -> bool {
        for i in 0..self.vector2fs.len() {
            if self.vector2fs[i].name == n {
                self.vector2fs.remove(i);
                return true;
            }
        }
        false
    }

    pub fn erase_point3f(&mut self, n: String) -> bool {
        for i in 0..self.point3fs.len() {
            if self.point3fs[i].name == n {
                self.point3fs.remove(i);
                return true;
            }
        }
        false
    }

    pub fn erase_vector3f(&mut self, n: String) -> bool {
        for i in 0..self.vector3fs.len() {
            if self.vector3fs[i].name == n {
                self.vector3fs.remove(i);
                return true;
            }
        }
        false
    }

    pub fn erase_normal3f(&mut self, n: String) -> bool {
        for i in 0..self.normals.len() {
            if self.normals[i].name == n {
                self.normals.remove(i);
                return true;
            }
        }
        false
    }

    pub fn erase_spectrum(&mut self, n: String) -> bool {
        for i in 0..self.spectra.len() {
            if self.spectra[i].name == n {
                self.spectra.remove(i);
                return true;
            }
        }
        false
    }

    pub fn erase_string(&mut self, n: String) -> bool {
        for i in 0..self.strings.len() {
            if self.strings[i].name == n {
                self.strings.remove(i);
                return true;
            }
        }
        false
    }

    pub fn erase_texture(&mut self, n: String) -> bool {
        for i in 0..self.textures.len() {
            if self.textures[i].name == n {
                self.textures.remove(i);
                return true;
            }
        }
        false
    }

    pub fn find_one_float(&mut self, name: &str, d: Float) -> Float {
        for mut f in &self.floats {
            if f.name == name && f.values.len() == 1 {
                f.looked_up = true;
                return f.values[0];
            }
        }
        d
    }

    pub fn find_one_int(&mut self, name: &str, d: i32) -> i32 {
        for mut v in &self.ints {
            if v.name == name && v.values.len() == 1 {
                v.looked_up = true;
                return v.values[0];
            }
        }
        d
    }

    pub fn find_one_bool(&mut self, name: &str, d: bool) -> bool {
        for mut v in &self.bools {
            if v.name == name && v.values.len() == 1 {
                v.looked_up = true;
                return v.values[0];
            }
        }
        d
    }

    pub fn find_one_point2f(&mut self, name: &str, d: Point2f) -> Point2f {
        for mut v in &self.point2fs {
            if v.name == name && v.values.len() == 1 {
                v.looked_up = true;
                return v.values[0];
            }
        }
        d
    }

    pub fn find_one_vector2f(&mut self, name: &str, d: Vector2f) -> Vector2f {
        for mut v in &self.vector2fs {
            if v.name == name && v.values.len() == 1 {
                v.looked_up = true;
                return v.values[0];
            }
        }
        d
    }

    pub fn find_one_point3f(&mut self, name: &str, d: Point3f) -> Point3f {
        for mut v in &self.point3fs {
            if v.name == name && v.values.len() == 1 {
                v.looked_up = true;
                return v.values[0];
            }
        }
        d
    }

    pub fn find_one_vector3f(&mut self, name: &str, d: Vector3f) -> Vector3f {
        for mut v in &self.vector3fs {
            if v.name == name && v.values.len() == 1 {
                v.looked_up = true;
                return v.values[0];
            }
        }
        d
    }

    pub fn find_one_normal3f(&mut self, name: &str, d: Normal3f) -> Normal3f {
        for mut v in &self.normals {
            if v.name == name && v.values.len() == 1 {
                v.looked_up = true;
                return v.values[0];
            }
        }
        d
    }

    pub fn find_one_spectrum(&mut self, name: &str, d: Spectrum) -> Spectrum {
        for mut v in &self.spectra {
            if v.name == name && v.values.len() == 1 {
                v.looked_up = true;
                return v.values[0];
            }
        }
        d
    }

    pub fn find_one_string(&mut self, name: &str, d: String) -> String {
        for mut v in &self.strings {
            if v.name == name && v.values.len() == 1 {
                v.looked_up = true;
                return v.values[0];
            }
        }
        d
    }

    pub fn find_one_filename(&mut self, name: &str, d: String) -> String {
        for mut v in &self.strings {
            if v.name == name && v.values.len() == 1 {
                v.looked_up = true;
                return if v.values[0] != "" { v.values[0] } else { d };
            }
        }
        d
    }

    pub fn find_float(&mut self, name: &str) -> Option<Vec<Float>> {
        for mut f in &self.floats {
            if f.name == name && f.values.len() == 1 {
                f.looked_up = true;
                return Some(f.values.clone());
            }
        }
        None
    }

    pub fn find_int(&mut self, name: &str) -> Option<Vec<i32>> {
        for mut f in &self.ints {
            if f.name == name && f.values.len() == 1 {
                f.looked_up = true;
                return Some(f.values.clone());
            }
        }
        None
    }

    pub fn find_bool(&mut self, name: &str) -> Option<Vec<bool>> {
        for mut f in &self.bools {
            if f.name == name && f.values.len() == 1 {
                f.looked_up = true;
                return Some(f.values.clone());
            }
        }
        None
    }

    pub fn find_point2f(&mut self, name: &str) -> Option<Vec<Point2f>> {
        for mut f in &self.point2fs {
            if f.name == name && f.values.len() == 1 {
                f.looked_up = true;
                return Some(f.values.clone());
            }
        }
        None
    }

    pub fn find_vector2f(&mut self, name: &str) -> Option<Vec<Vector2f>> {
        for mut f in &self.vector2fs {
            if f.name == name && f.values.len() == 1 {
                f.looked_up = true;
                return Some(f.values.clone());
            }
        }
        None
    }

    pub fn find_point3f(&mut self, name: &str) -> Option<Vec<Point3f>> {
        for mut f in &self.point3fs {
            if f.name == name && f.values.len() == 1 {
                f.looked_up = true;
                return Some(f.values.clone());
            }
        }
        None
    }

    pub fn find_vector3f(&mut self, name: &str) -> Option<Vec<Vector3f>> {
        for mut f in &self.vector3fs {
            if f.name == name && f.values.len() == 1 {
                f.looked_up = true;
                return Some(f.values.clone());
            }
        }
        None
    }

    pub fn find_normal3f(&mut self, name: &str) -> Option<Vec<Normal3f>> {
        for mut f in &self.normals {
            if f.name == name && f.values.len() == 1 {
                f.looked_up = true;
                return Some(f.values.clone());
            }
        }
        None
    }

    pub fn find_spectrum(&mut self, name: &str) -> Option<Vec<Spectrum>> {
        for mut f in &self.spectra {
            if f.name == name && f.values.len() == 1 {
                f.looked_up = true;
                return Some(f.values.clone());
            }
        }
        None
    }

    pub fn find_string(&mut self, name: &str) -> Option<Vec<String>> {
        for mut f in &self.strings {
            if f.name == name && f.values.len() == 1 {
                f.looked_up = true;
                return Some(f.values.clone());
            }
        }
        None
    }

    pub fn report_unused(&self) {
        for v in self.ints {
            if !v.looked_up {
                warn!("Parameter \"{}\" not used", v.name);
            }
        }
        for v in self.bools {
            if !v.looked_up {
                warn!("Parameter \"{}\" not used", v.name);
            }
        }
        for v in self.floats {
            if !v.looked_up {
                warn!("Parameter \"{}\" not used", v.name);
            }
        }
        for v in self.point2fs {
            if !v.looked_up {
                warn!("Parameter \"{}\" not used", v.name);
            }
        }
        for v in self.vector2fs {
            if !v.looked_up {
                warn!("Parameter \"{}\" not used", v.name);
            }
        }
        for v in self.point3fs {
            if !v.looked_up {
                warn!("Parameter \"{}\" not used", v.name);
            }
        }
        for v in self.vector3fs {
            if !v.looked_up {
                warn!("Parameter \"{}\" not used", v.name);
            }
        }
        for v in self.normals {
            if !v.looked_up {
                warn!("Parameter \"{}\" not used", v.name);
            }
        }
        for v in self.spectra {
            if !v.looked_up {
                warn!("Parameter \"{}\" not used", v.name);
            }
        }
        for v in self.strings {
            if !v.looked_up {
                warn!("Parameter \"{}\" not used", v.name);
            }
        }
        for v in self.textures {
            if !v.looked_up {
                warn!("Parameter \"{}\" not used", v.name);
            }
        }
    }

    pub fn clear(&mut self) {
        self.ints.clear();
        self.ints.shrink_to_fit();
        self.bools.clear();
        self.bools.shrink_to_fit();
        self.floats.clear();
        self.floats.shrink_to_fit();
        self.point2fs.clear();
        self.point2fs.shrink_to_fit();
        self.vector2fs.clear();
        self.vector2fs.shrink_to_fit();
        self.point3fs.clear();
        self.point3fs.shrink_to_fit();
        self.vector3fs.clear();
        self.vector3fs.shrink_to_fit();
        self.normals.clear();
        self.normals.shrink_to_fit();
        self.spectra.clear();
        self.spectra.shrink_to_fit();
        self.strings.clear();
        self.strings.shrink_to_fit();
        self.textures.clear();
        self.textures.shrink_to_fit();
        // not in C++ but also clearing the spectra cache
        self.cached_spectra.clear();
        self.cached_spectra.shrink_to_fit();
    }

    pub fn print(&self, ident: usize) {
        print_items("integer", ident, &self.ints);
        print_items("bool", ident, &self.bools);
        print_items("float", ident, &self.floats);
        print_items("point2", ident, &self.point2fs);
        print_items("vector2", ident, &self.vector2fs);
        print_items("point", ident, &self.point3fs);
        print_items("vector", ident, &self.vector3fs);
        print_items("normal", ident, &self.normals);
        print_items("string", ident, &self.strings);
        print_items("texture", ident, &self.textures);
        print_items("rgb", ident, &self.spectra);
    }
}

impl fmt::Display for ParamSet {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for item in &self.ints {
            write!(f, "\"integer {}\" [{}] ", item.name,
                item.values.iter().map(|v| v.to_string()).collect::<Vec<String>>().join(" "))?;
        }
        for item in &self.bools {
            write!(f, "\"bool {}\" [{}] ", item.name,
                item.values.iter().map(|v| v.to_string()).collect::<Vec<String>>().join(" "))?;
        }
        for item in &self.floats {
            write!(f, "\"float {}\" [{}] ", item.name,
                item.values.iter().map(|v| float_to_string_general(*v, 8)).collect::<Vec<String>>().join(" "))?;
        }
        for item in &self.point2fs {
            write!(f, "\"point2 {}\" [{}] ", item.name,
                item.values.iter().map(|v| format!("{} {}", float_to_string_general(v.x, 8), float_to_string_general(v.y, 8))).collect::<Vec<String>>().join(" "))?;
        }
        for item in &self.vector2fs {
            write!(f, "\"vector2 {}\" [{}] ", item.name,
                item.values.iter().map(|v| format!("{} {}", float_to_string_general(v.x, 8), float_to_string_general(v.y, 8))).collect::<Vec<String>>().join(" "))?;
        }
        for item in &self.point3fs {
            write!(f, "\"point3 {}\" [{}] ", item.name,
                item.values.iter().map(|v| format!("{} {} {}", float_to_string_general(v.x, 8), float_to_string_general(v.y, 8), float_to_string_general(v.z, 8))).collect::<Vec<String>>().join(" "))?;
        }
        for item in &self.vector3fs {
            write!(f, "\"vector3 {}\" [{}] ", item.name,
                item.values.iter().map(|v| format!("{} {} {}", float_to_string_general(v.x, 8), float_to_string_general(v.y, 8), float_to_string_general(v.z, 8))).collect::<Vec<String>>().join(" "))?;
        }
        for item in &self.normals {
            write!(f, "\"normal {}\" [{}] ", item.name,
                item.values.iter().map(|v| format!("{} {} {}", float_to_string_general(v.x, 8), float_to_string_general(v.y, 8), float_to_string_general(v.z, 8))).collect::<Vec<String>>().join(" "))?;
        }
        for item in &self.strings {
            write!(f, "\"string {}\" [{}] ", item.name,
                item.values.iter().map(|v| format!("\"{}\"", v)).collect::<Vec<String>>().join(" "))?;
        }
        for item in &self.textures {
            write!(f, "\"texture {}\" [{}] ", item.name,
                item.values.iter().map(|v| format!("\"{}\"", v)).collect::<Vec<String>>().join(" "))?;
        }
        for item in &self.spectra {
            write!(f, "\"color {}\" [{}] ", item.name,
                item.values.iter().map(|v| {
                    let mut rgb = [0.0 as Float; 3];
                    v.to_rgb(&mut rgb);
                    format!("{} {} {}", float_to_string_general(rgb[0], 8), float_to_string_general(rgb[1], 8), float_to_string_general(rgb[2], 8))
                }).collect::<Vec<String>>().join(" "))?;
        }

        Ok(())
    }
}

fn print_items<T>(type_name: &str, indent: usize, items: &Vec<Arc<ParamSetItem<T>>>)
where
T: Printer
{
    for item in items {
        let s = format!("{:3$}\"{} {}\" [ ", "", type_name, item.name, indent + 8);
        print!("\n{}", s);
        let mut np = s.len();
        for i in 0..item.values.len() {
            np += Printer::print(&item.values[i]);
            if np > 90 && i < item.values.len() - 1 {
                let s = format!("{:1$}", "", indent + 8);
                print!("\n{}", s);
                np = s.len();
            }
        }
        print!("] ");
    }
}

/// This trait is used for functionality to emulate the printf function from C, that returns the number of characters printed.
trait Printer {
    fn print(&self) -> usize;
}

impl Printer for i32 {
    fn print(&self) -> usize {
        let s = format!("{} ", self);
        print!("{}", s);
        s.len()
    }
}

impl Printer for bool {
    fn print(&self) -> usize {
        let s = format!("{} ", self);
        print!("{}", s);
        s.len()
    }
}

impl Printer for Float {
    fn print(&self) -> usize {
        if self.trunc() == *self {
            if let Some(v) = cast::<Float, u32>(*self) {
                let s = format!("{}", v);
                print!("{} ", s);
                return s.len() + 1;
            }
        }
        let s = float_to_string_general(*self, 9);
        print!("{} ", s);
        s.len() + 1
    }
}

impl Printer for Point2f {
    fn print(&self) -> usize {
        let mut np = Printer::print(&self.x);
        np + Printer::print(&self.y)
    }
}

impl Printer for Vector2f {
    fn print(&self) -> usize {
        let mut np = Printer::print(&self.x);
        np + Printer::print(&self.y)
    }
}

impl Printer for Point3f {
    fn print(&self) -> usize {
        let mut np = Printer::print(&self.x);
        np += Printer::print(&self.y);
        np + Printer::print(&self.z)
    }
}

impl Printer for Vector3f {
    fn print(&self) -> usize {
        let mut np = Printer::print(&self.x);
        np += Printer::print(&self.y);
        np + Printer::print(&self.z)
    }
}

impl Printer for Normal3f {
    fn print(&self) -> usize {
        let mut np = Printer::print(&self.x);
        np += Printer::print(&self.y);
        np + Printer::print(&self.z)
    }
}

impl Printer for String {
    fn print(&self) -> usize {
        print!("\"{}\" ", self);
        self.len() + 3
    }
}

impl Printer for Spectrum {
    fn print(&self) -> usize {
        let mut rgb = [0.0 as Float; 3];
        self.to_rgb(&mut rgb);
        let mut np = Printer::print(&rgb[0]);
        np += Printer::print(&rgb[1]);
        np + Printer::print(&rgb[2])
    }
}