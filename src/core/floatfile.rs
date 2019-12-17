use super::pbrt::Float;
use std::path::Path;
use std::fs::File;
use std::io::{BufRead, BufReader, Result as IOResult, Error, ErrorKind};

pub fn read_float_file(filename: &str) -> IOResult<Vec<Float>> {
    let f = match File::open(Path::new(filename)) {
        Ok(f) => f,
        Err(e) => {
            error!("Unable to open file \"{}\"", filename);
            return Err(e);
        }
    };

    let result: Vec<Float> = Vec::new();
    let reader = BufReader::new(f);
    for (line_number, line) in reader.lines().enumerate() {
        if let Ok(line) = line {
            if line.is_empty() {
                continue;
            }
            for token in line.split_whitespace() {
                if token.chars().next() == Some('#') {
                    continue;
                }

                match token.parse::<Float>() {
                    Ok(v) => result.push(v),
                    Err(_) => error!("Unexpected text found at line {} of float file \"{}\"", line_number, filename)
                }
            }
        } else {
            error!("Error reading float file \"{}\" at line {}", filename, line_number);
            return Err(Error::new(ErrorKind::InvalidData, format!("Error reading float file \"{}\" at line {}", filename, line_number)));
        }
    }

    Ok(result)
}