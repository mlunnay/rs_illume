pub struct ProgressReporter {
    
}

impl ProgressReporter {
    pub fn new(total_work: u64, title: &str) -> ProgressReporter {
        unimplemented!();
    }

    pub fn update(&mut self, num: u64) {
        unimplemented!();
    }

    pub fn done(&mut self) {
        unimplemented!();
    }
}

unsafe impl Sync for ProgressReporter {}
unsafe impl Send for ProgressReporter {}