use super::pbrt::{Float, gamma_correct};
use super::geometry::{Point2i, Bounds2i, Bounding2};
use super::spectrum::RGBSpectrum;
use std::sync::Arc;
use std::path::Path;
use image::{GenericImageView};
use num::{cast, clamp};
use byteorder::{ReadBytesExt, WriteBytesExt, LittleEndian, BigEndian, NativeEndian};
use std::fs::{File, OpenOptions};
use std::io::{Read, Write};

pub fn read_image(name: &str, resolution: &mut Point2i) -> Option<Arc<Vec<RGBSpectrum>>> {
    let path = Path::new(name);
    if let Some(extension) = path.extension() {
        match extension.to_str().unwrap().to_lowercase().as_str() {
            "exr" => read_image_exr(name, resolution),
            "tga" | "png" | "bmp" | "hdr" => read_image_image(name, resolution),
            "pfm" => read_image_pfm(name, resolution),
            _ => {
                error!("Unable to load image stored in format \"{}\" for filename \"{}\"", extension.to_str().unwrap(), name);
                return None;
            }
        }
    } else {
        error!("Unable to load image stored in unknown format for filename \"{}\"", name);
        None
    }
}

pub fn write_image(
    name: &str,
    rgb: &[Float],
    output_bounds: &Bounds2i,
    total_resolution: &Point2i
) {
    let resolution = output_bounds.diagonal();
    let path = Path::new(name);
    if let Some(extension) = path.extension() {
        match extension.to_str().unwrap().to_lowercase().as_str() {
            "exr" => write_image_exr(name, rgb, resolution.x, resolution.y, total_resolution.x,
                total_resolution.y, output_bounds.min.x, output_bounds.min.y),
            "tga" =>  {
                // 8-bit formats; apply gamma
                let rgb: Vec<u8> = rgb.into_iter().map(|p| 
                    clamp(255.0 * gamma_correct(*p) + 0.5, 0.0, 255.0) as u8
                ).collect();
                write_image_tga(name, &rgb, resolution.x, resolution.y,
                    total_resolution.x, total_resolution.y, output_bounds.min.x, output_bounds.min.y);
            }
            "png" | "bmp" => {
                // 8-bit formats; apply gamma
                let rgb: Vec<u8> = rgb.into_iter().map(|p| 
                    clamp(255.0 * gamma_correct(*p) + 0.5, 0.0, 255.0) as u8
                ).collect();
                write_image_image(name, &rgb, resolution.x, resolution.y,
                    total_resolution.x, total_resolution.y,
                    output_bounds.min.x, output_bounds.min.y)
            },
            "pfm" => { write_image_pfm(name, rgb, resolution.x as u32, resolution.y as u32); },
            _ => {
                error!("Unable to load image stored in format \"{}\" for filename \"{}\"", extension.to_str().unwrap(), name);
            }
        }
    } else {
        error!("Can't determine image file type from suffix of filename \"{}\"", name);
    }
}

fn read_image_image(name: &str, dimensions: &mut Point2i) -> Option<Arc<Vec<RGBSpectrum>>> {
    if let Ok(img) = image::open(Path::new(name)) {
        dimensions.x = img.width() as i32;
        dimensions.y = img.height() as i32;
        let texels: Vec<RGBSpectrum> = img.to_rgb().pixels().map(|p| {
            let r: Float = cast(p[0]).unwrap() / 255.0;
            let g: Float = cast(p[1]).unwrap() / 255.0;
            let b: Float = cast(p[2]).unwrap() / 255.0;
            RGBSpectrum::from_rgb(&[r, g, b])
        }).collect();
        Some(Arc::new(texels))
    } else {
        error!("Error loading image from filename \"{}\"", name);
        None
    }
}

fn write_image_image(
    name: &str,
    pixels: &[u8],
    x_res: i32,
    y_res: i32,
    total_x_res: i32,
    total_y_res: i32,
    x_offset: i32,
    y_offset: i32
) {
    image::save_buffer(Path::new(name), pixels, x_res as u32, y_res as u32, image::RGB(8));
}

fn read_image_exr(name: &str, resolution: &mut Point2i) -> Option<Arc<Vec<RGBSpectrum>>> {
    // TODO: implement EXR read
    error!("OpenEXR support not yet implemented");
    None
}

fn write_image_exr(
    name: &str,
    pixels: &[Float],
    x_res: i32,
    y_res: i32,
    total_x_res: i32,
    total_y_res: i32,
    x_offset: i32,
    y_offset: i32
) {
    // TODO: implement EXR write
    error!("OpenEXR support not yet implemented"); 
}

fn read_image_pfm(name: &str, resolution: &mut Point2i) -> Option<Arc<Vec<RGBSpectrum>>> {
    let texels: Vec<RGBSpectrum> = Vec::new();
    let path = Path::new(name);
    let mut file = match File::open(path) {
        Ok(f) => f,
        Err(e) => {
            error!("Unable to open image file \"{}\"", name);
            return None;
        }
    };

    // read either "Pf" or "PF"
    let mut header = [0_u8; 2];
    if file.read_exact(&mut header).is_err() {
        error!("PFM file \"{}\" has an incompatible file format or version.", name);
        return None;
    }
    let n_channels: usize = if header != [b'P', b'f'] {
        1
    } else if header != [b'P', b'F']{
        3
    } else {
        error!("PFM file \"{}\" has an incompatible file format or version.", name);
        return None;
    };

    // read the rest of the header
    // read width
    let width: usize = match read_word(&file){
        Ok(buf) => match parse_token(&buf, "width") {
            Ok(v) => v,
            Err(e) => {
                error!("PFM file \"{}\" {}", name, e);
                return None;
            }
        },
        Err(_) => {
            error!("PFM file \"{}\" Error reading field width", name);
            return None;
        }
    };

    // read height
    let height: usize = match read_word(&file){
        Ok(buf) => match parse_token(&buf, "height") {
            Ok(v) => v,
            Err(e) => {
                error!("PFM file \"{}\" {}", name, e);
                return None;
            }
        },
        Err(_) => {
            error!("PFM file \"{}\" Error reading field height", name);
            return None;
        }
    };

    // read scale
    let scale: f32 = match read_word(&file){
        Ok(buf) => match parse_token(&buf, "scale") {
            Ok(v) => v,
            Err(e) => {
                error!("PFM file \"{}\" {}", name, e);
                return None;
            }
        },
        Err(_) => {
            error!("PFM file \"{}\" Error reading field scale", name);
            return None;
        }
    };

    // read the data
    let n_floats = n_channels * width * height;
    let mut data = vec![0.0_f32; n_floats];
    let mut idx = 0;
    // flip in Y as P*M has the origin at the lower left.
    for y in (0..height).rev() {
        if scale < 0.0 {
            data[idx] = match file.read_f32::<LittleEndian>(){
                Ok(v) => v,
                Err(_) => {
                    error!("Error reading PFM file \"{}\"", name);
                    return None;
                }
            };
        } else {
            data[idx] = match file.read_f32::<BigEndian>(){
                Ok(v) => v,
                Err(_) => {
                    error!("Error reading PFM file \"{}\"", name);
                    return None;
                }
            };
        }
        idx += 1;
    }

    if scale.abs() != 1.0 {
        for i in 0..n_floats {
            data[i] *= scale.abs();
        }
    }

    // create the RGBs ...
    let mut rgb: Vec<RGBSpectrum> = Vec::with_capacity(width * height);
    if n_channels == 1 {
        for i in 0..width * height {
            rgb.push(RGBSpectrum::new(data[i]));
        }
    } else {
        for i in 0..width * height {
            let frgb = [data[3 * i], data[3 * i + 1], data[3 * i + 2]];
            rgb.push(RGBSpectrum::from_rgb(&frgb));
        }
    }

    info!("Read PFM image {} ({} x {})", name, width, height);
    resolution.x = width as i32;
    resolution.y = height as i32;
    Some(Arc::new(rgb))
}

fn parse_token<T>(buffer: &[u8], field: &str) -> Result<T, &'static str>
where
T: std::str::FromStr
{
    match std::str::from_utf8(buffer) {
        Ok(s) => match s.parse() {
            Ok(v) => Ok(v),
            Err(_) => Err(format!("Error reading field {}", field).as_str())
        },
        Err(_) => Err(format!("Error reading field {}", field).as_str())
    }
}

fn read_word(fp: &File) -> std::io::Result<Vec<u8>> {
    let mut buffer: Vec<u8> = Vec::new();
    let mut c: u8 = fp.read_u8()?;
    while (c as char).is_ascii_whitespace() {
        buffer.push(c);
    }
    Ok(buffer)
}

fn write_image_pfm(filename: &str, rgb: &[Float], width: u32, height: u32) -> bool {
    let mut file = match OpenOptions::new().write(true).create(true).open(filename) {
        Ok(f) => f,
        Err(e) => {
            error!("Unable to open output PFM file \"{}\"", filename);
            return false;
        }
    };

    // only write 3 channel PFMs here...
    if file.write_all("PF\n".as_bytes()).is_err() {
        error!("Error writing PFM file \"{}\"", filename);
        return false;
    }

    // write the width and height, which must be positive
    if file.write_all(format!("{} {}\n", width, height).as_bytes()).is_err() {
        error!("Error writing PFM file \"{}\"", filename);
        return false;
    }

    // write the scale, which encodes endianness
    #[cfg(target_endian = "big")]
    let scale: f32 = 1.0;
    #[cfg(not(target_endian = "big"))]
    let scale: f32 = -1.0;
    if file.write_all(format!("{}\n", scale).as_bytes()).is_err() {
        error!("Error writing PFM file \"{}\"", filename);
        return false;
    }

    // write the data from bottom left to upper right as specified by
    // http://netpbm.sourceforge.net/doc/pfm.html
    // The raster is a sequence of pixels, packed one after another, with no
    // delimiters of any kind. They are grouped by row, with the pixels in each
    // row ordered left to right and the rows ordered bottom to top.
    for y in (0..height as usize).rev() {
        for x in 0..3 * width as usize {
            if file.write_f32::<NativeEndian>(rgb[y * width as usize * 3 + x]).is_err() {
                error!("Error writing PFM file \"{}\"", filename);
                return false;
            }
        }
    }

    true
}

fn write_image_tga(
    name: &str,
    pixels: &[u8],
    x_res: i32,
    y_res: i32,
    total_x_res: i32,
    total_y_res: i32,
    x_offset: i32,
    y_offset: i32
) {
    // TODO: implement TGA write
    error!("Writing TGA not yet implemented");
}