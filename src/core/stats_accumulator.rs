use crossbeam_channel::{unbounded, Receiver, Sender, Select};
use hashbrown::HashMap;
use std::sync::atomic::{AtomicUsize, Ordering};
use parking_lot::Once;
use std::mem::transmute;
use std::io::Write;

pub struct StatsGuard<'a> {
    inner: &'a StatsAccumulator
}

pub struct StatsAccumulator {
    r_counter: Receiver<(String, i64)>,
    s_counter: Sender<(String, i64)>,
    r_memory_counter: Receiver<(String, i64)>,
    s_memory_counter: Sender<(String, i64)>,
    r_int_distribution: Receiver<(String, i64)>,
    s_int_distribution: Sender<(String, i64)>,
    r_float_distribution: Receiver<(String, f64)>,
    s_float_distribution: Sender<(String, f64)>,
    r_percentage: Receiver<(String, i64, i64)>,
    s_percentage: Sender<(String, i64, i64)>,
    r_ratio: Receiver<(String, i64, i64)>,
    s_ratio: Sender<(String, i64, i64)>,

    counters: HashMap<String, i64>,
    memory_counters: HashMap<String, i64>,
    // value is (sum, count, min, max)
    int_distributions: HashMap<String, (i64, i64, i64, i64)>,
    // value is (sum, count, min, max)
    float_distributions: HashMap<String, (f64, i64, f64, f64)>,
    percentages: HashMap<String, (i64, i64)>,
    ratios: HashMap<String, (i64, i64)>,

    thread_counter: AtomicUsize
}

impl StatsAccumulator {
    pub fn new() -> StatsAccumulator {
        let (s_counter, r_counter) = unbounded::<(String, i64)>();
        let (s_memory_counter, r_memory_counter) = unbounded::<(String, i64)>();
        let (s_int_distribution, r_int_distribution) = unbounded::<(String, i64)>();
        let (s_float_distribution, r_float_distribution) = unbounded::<(String, f64)>();
        let (s_percentage, r_percentage) = unbounded::<(String, i64, i64)>();
        let (s_ratio, r_ratio) = unbounded::<(String, i64, i64)>();

        StatsAccumulator{
            s_counter,
            r_counter,
            s_memory_counter,
            r_memory_counter,
            s_int_distribution,
            r_int_distribution,
            s_float_distribution,
            r_float_distribution,
            s_percentage,
            r_percentage,
            s_ratio,
            r_ratio,
            counters: HashMap::new(),
            memory_counters: HashMap::new(),
            int_distributions: HashMap::new(),
            float_distributions: HashMap::new(),
            percentages: HashMap::new(),
            ratios: HashMap::new(),

            thread_counter: AtomicUsize::new(0)
        }
    }

    /// Returns a singleton instance of the StatsAccumulator.
    pub fn instance() -> &'static mut StatsAccumulator {
        static mut INSTANCE: *mut StatsAccumulator = 0 as *mut StatsAccumulator;
        static ONCE: Once = Once::new();

        unsafe {
            ONCE.call_once(|| {
                let instance = StatsAccumulator::new();
                // Put it in the heap so it can outlive this call
                INSTANCE = transmute(Box::new(instance));
            });

            &mut *INSTANCE
        }
    }

    pub fn report_counter(&self, name: String, val: i64) {
        self.s_counter.send((name, val)).unwrap();
    }

    pub fn report_memory_counter(&self, name: String, val: i64) {
        self.s_memory_counter.send((name, val)).unwrap();
    }

    pub fn report_int_distribution(&self, name: String, val: i64) {
        self.s_int_distribution.send((name, val)).unwrap();
    }

    pub fn report_float_distribution(&self, name: String, val: f64) {
        self.s_float_distribution.send((name, val)).unwrap();
    }

    pub fn report_percentage(&self, name: String, num: i64, denom: i64) {
        self.s_percentage.send((name, num, denom)).unwrap();
    }

    pub fn report_ratio(&self, name: String, num: i64, denom: i64) {
        self.s_ratio.send((name, num, denom)).unwrap();
    }

    /// loop over the accumulation recievers.
    pub fn accumulate(&mut self) {
        let mut select = Select::new();
        let counter = select.recv(&self.r_counter);
        let memory_counter = select.recv(&self.r_memory_counter);
        let int_distribution = select.recv(&self.r_int_distribution);
        let float_distribution = select.recv(&self.r_float_distribution);
        let percentage = select.recv(&self.r_percentage);
        let ratio = select.recv(&self.r_ratio);
        
        loop {
            let index = match select.try_ready() {
                Err(_) => break,
                Ok(i) => i
            };

            if index == counter {
                let res = self.r_counter.try_recv();
                match res {
                    Err(e) => {
                        if e.is_disconnected() {
                            select.remove(counter);
                        }
                    }
                    Ok((name, val)) => {
                        let distrubution = self.counters.entry(name).or_default();
                        *distrubution += val;
                    }
                }
            }
            else if index == memory_counter {
                let res = self.r_memory_counter.try_recv();
                match res {
                    Err(e) => {
                        if e.is_disconnected() {
                            select.remove(memory_counter);
                        }
                    }
                    Ok((name, val)) => {
                        let distrubution = self.memory_counters.entry(name).or_default();
                        *distrubution += val;
                    }
                }
            }
            else if index == int_distribution {
                let res = self.r_int_distribution.try_recv();
                match res {
                    Err(e) => {
                        if e.is_disconnected() {
                            select.remove(int_distribution);
                        }
                    }
                    Ok((name, val)) => {
                        let distrubution = self.int_distributions.entry(name).or_default();
                        distrubution.0 += val;
                        distrubution.1 += 1;
                        distrubution.2 = val.min(distrubution.2);
                        distrubution.3 = val.max(distrubution.3);
                    }
                }

            }
            else if index == float_distribution {
                let res = self.r_float_distribution.try_recv();
                match res {
                    Err(e) => {
                        if e.is_disconnected() {
                            select.remove(float_distribution);
                        }
                    }
                    Ok((name, val)) => {
                        let distrubution = self.float_distributions.entry(name).or_default();
                        distrubution.0 += val;
                        distrubution.1 += 1;
                        distrubution.2 = val.min(distrubution.2);
                        distrubution.3 = val.max(distrubution.3);
                    }
                }

            }
            else if index == percentage {
                let res = self.r_percentage.try_recv();
                match res {
                    Err(e) => {
                        if e.is_disconnected() {
                            select.remove(percentage);
                        }
                    }
                    Ok((name, val, denom)) => {
                        let distrubution = self.percentages.entry(name).or_default();
                        distrubution.0 += val;
                        distrubution.1 += denom;
                    }
                }

            }
            else if index == ratio {
                let res = self.r_ratio.try_recv();
                match res {
                    Err(e) => {
                        if e.is_disconnected() {
                            select.remove(ratio);
                        }
                    }
                    Ok((name, val, denom)) => {
                        let distrubution = self.ratios.entry(name).or_default();
                        distrubution.0 += val;
                        distrubution.1 += denom;
                    }
                }

            }
        }
    }  

    /// Returns an RAII guard that signals a thread is using the StatsAccumulator.
    pub fn using(&self) -> StatsGuard {
        StatsGuard::new(self)
    }

    /// Called at the beggining of a thread to increment the usage count.
    pub fn connect(&self) {
        let old_count = self.thread_counter.fetch_add(1, Ordering::SeqCst);

        if old_count > std::isize::MAX as usize {
            panic!("StatsAccumulator maximum connection count exceeded.");
        }
    }

    /// Called at the end of a thread to decrement the usage count and if needed disconnect all of the accumulation senders.
    pub fn disconnect(&self) {
        if self.thread_counter.fetch_sub(1, Ordering::SeqCst) == 1 {
            drop(&self.s_counter);
            drop(&self.s_memory_counter);
            drop(&self.s_int_distribution);
            drop(&self.s_float_distribution);
            drop(&self.s_percentage);
            drop(&self.s_ratio);
        }
    }

    pub fn print<T: Write>(&self, dest: &mut T) {
        let mut to_print: HashMap<String, Vec<String>> = HashMap::default();

        for (counter, value) in &self.counters {
            let (category, title) = get_category_and_title(&counter);
            to_print.entry(category.into()).or_default().push(format!("{:<42}               {:12}", title, value));
        }

        for (counter, value) in &self.memory_counters {
            let (category, title) = get_category_and_title(&counter);
            let mut value = *value as f64 / 1024.0;
            let value_string = if value < 1024.0 {
                "kB"
            }
            else {
                value /= 1024.0;
                if value < 1024.0 {
                    "MiB"
                }
                else {
                    value /= 1024.0;
                    "GiB"
                }
            };
            to_print.entry(category.into()).or_default().push(format!("{:<42}               {:9.2} {}", title, value, value_string));
        }

        for (counter, value) in &self.int_distributions {
            if value.1 == 0_i64 {
                continue;
            }
            let (category, title) = get_category_and_title(&counter);
            let avg = value.0 as f64 / value.1 as f64;
            to_print.entry(category.into()).or_default().push(format!("{:<42}                      {:.3} avg [range {} - {}]", title, avg, value.2, value.3));
        }

        for (counter, value) in &self.float_distributions {
            if value.1 == 0_i64 {
                continue;
            }
            let (category, title) = get_category_and_title(&counter);
            let avg = value.0 / value.1 as f64;
            to_print.entry(category.into()).or_default().push(format!("{:<42}                      {:.3} avg [range {} - {}]", title, avg, value.2, value.3));
        }

        for (counter, value) in &self.percentages {
            if value.1 == 0 {
                continue;
            }
            let (category, title) = get_category_and_title(&counter);
            let percent = value.0 as f64 * 100.0 / value.1 as f64;
            to_print.entry(category.into()).or_default().push(format!("{:<42}{:12} {:12} ({:.2}%)", title, value.0, value.1, percent));
        }

        for (counter, value) in &self.ratios {
            if value.1 == 0 {
                continue;
            }
            let (category, title) = get_category_and_title(&counter);
            let ratio = value.0 as f64 / value.1 as f64;
            to_print.entry(category.into()).or_default().push(format!("{:<42}{:12} {:12} ({:.2}%)", title, value.0, value.1, ratio));
        }

        for (category, items) in to_print {
            dest.write(format!("  {}\n", category).as_bytes()).unwrap();
            for item in items {
                dest.write(format!("    {}\n", item).as_bytes()).unwrap();
            }
        }
    }
}

unsafe impl Send for StatsAccumulator {}
unsafe impl Sync for StatsAccumulator {}

/// A RAII guard for threads to singal their usage of StatsAccumulator.
impl<'a> StatsGuard<'a> {
    pub fn new(accumulator: &'a StatsAccumulator) -> Self {
        accumulator.connect();
        StatsGuard{inner: accumulator}
    }
}

impl<'a> Drop for StatsGuard<'a> {
    fn drop(&mut self) {
        self.inner.disconnect();
    }
}

impl<'a> Clone for StatsGuard<'a> {
    fn clone(&self) -> Self {
        StatsGuard::new(self.inner.clone())
    }
}

#[inline]
pub fn get_category_and_title<'a>(s: &'a String) -> (&'a str, &'a str) {
    let split: Vec<&str> = s.splitn(2, '/').collect();
    if split.len() == 1 {
        ("", split[0])
    }
    else {
        (split[0], split[1])
    }
}