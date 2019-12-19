//! File utilities.
//! functions from C++ that mimic the rust std library are ommited

use std::path::Path;

static mut SEARCH_DIRECTORY: &'static str = "";

pub fn set_search_directory(dir_name: &'static str) {
    unsafe {
        SEARCH_DIRECTORY = dir_name;
    }
}

pub fn resolve_filename(filename: &str) -> &Path {
    unsafe {
        if SEARCH_DIRECTORY.is_empty() || filename.is_empty() {
            return Path::new(filename);
        } else if Path::new(filename).is_absolute() {
            return Path::new(filename);
        }

        &*Path::new(SEARCH_DIRECTORY).join(filename)
    }
}

pub fn directory_containing(filename: &str) -> &str {
    let path = Path::new(filename);
    if path.is_dir() {
        path.to_str().unwrap()
    } else {
        path.parent().unwrap().to_str().unwrap()
    }
}
