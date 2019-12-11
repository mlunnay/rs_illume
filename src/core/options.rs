use super::pbrt::Float;
use parking_lot::Once;
use std::mem::transmute;

#[derive(Debug, Default, Clone)]
pub struct PbrtOptions {
    pub n_threads: u32,
    pub quick_render: bool,
    pub quiet: bool,
    pub cat: bool,
    pub to_ply: bool,
    pub image_file: String,
    pub crop_window: [[Float; 2]; 2]
}

impl PbrtOptions {
    pub fn instance() -> &'static mut PbrtOptions {
        static mut INSTANCE: *mut PbrtOptions = 0 as *mut PbrtOptions;
        static ONCE: Once = Once::new();

        unsafe {
            ONCE.call_once(|| {
                let instance = PbrtOptions::default();
                // Put it in the heap so it can outlive this call
                INSTANCE = transmute(Box::new(instance));
            });

            &mut *INSTANCE
        }
    }
}