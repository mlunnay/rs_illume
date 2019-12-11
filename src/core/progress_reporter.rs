use super::pbrt::Float;
use std::sync::{Arc, atomic::{AtomicU64, AtomicBool, Ordering}};
use std::time::{Instant, Duration};
use std::thread::{JoinHandle, spawn, sleep};
use super::options::PbrtOptions;
use terminal_size::{Width, terminal_size};
use std::io::Write;

pub struct ProgressReporter {
    total_work: u64,
    title: String,
    start_time: Instant,
    work_done: Arc<AtomicU64>,
    exit_thread: Arc<AtomicBool>,
    update_thread: Option<Arc<JoinHandle<()>>>
}

impl ProgressReporter {
    pub fn new(total_work: u64, title: &'static str) -> ProgressReporter {
        let start_time = Instant::now();
        let mut reporter = ProgressReporter {
            total_work,
            title: String::from(title),
            start_time,
            work_done: Arc::new(AtomicU64::new(0)),
            exit_thread: Arc::new(AtomicBool::new(false)),
            update_thread: None
        };
        // Launch thread to periodically update progress bar
        if !PbrtOptions::instance().quiet {
            let work_done = reporter.work_done.clone();
            let exit_thread = reporter.exit_thread.clone();
            // this contains the logic of ProgressReporter::PrintBar()
            if let Some((Width(w), _)) = terminal_size() {
                reporter.update_thread = Some(Arc::new(spawn(move || {
                    let bar_length = w as usize - 28;
                    let total_plusses = 2.max(bar_length - title.len());
                    let mut plusses_printed: usize = 0;

                    //Initialize progress string
                    // let buf_len = title.len() + total_plusses + 64;
                    let mut buf: String = format!("\r{}: [", title);
                    let plusses_position = buf.len();
                    for _ in 0..total_plusses {
                        buf.push(' ');
                    }
                    buf.push_str("] ");
                    print!("{}", buf);
                    std::io::stdout().flush().unwrap();

                    let mut sleep_duration = Duration::from_millis(250);
                    let mut iter_count = 0;
                    while !exit_thread.load(Ordering::SeqCst) {
                        sleep(sleep_duration);

                        // Periodically increase sleepDuration to reduce overhead of
                        // updates.
                        iter_count += 1;
                        if iter_count == 10 {
                            // Up to 0.5s after ~2.5s elapsed
                            sleep_duration *= 2;
                        } else if iter_count == 70 {
                            // Up to 1s after an additional ~30s have elapsed.
                            sleep_duration *= 2;
                        } else if iter_count == 520 {
                            // After 15m, jump up to 5s intervals
                            sleep_duration *= 5;
                        }

                        let percent_done = work_done.load(Ordering::SeqCst) as Float / total_work as Float;
                        let plusses_needed = (total_plusses as Float * percent_done).round() as usize;
                        let mut cur_space = plusses_position;
                        while plusses_printed < plusses_needed {
                            buf.replace_range(cur_space..cur_space + 1, "+");
                            cur_space += 1;
                            plusses_printed += 1;
                        }
                        print!("{}", buf);

                        // Update elapsed time and estimated time to completion
                        let seconds = ((Instant::now() - start_time).as_millis() as f64 / 1000.0) as Float;
                        let est_remaining = seconds / percent_done - seconds;
                        if percent_done == 1.0 {
                            print!(" ({:.1}s)       ", seconds);
                        } else if !est_remaining.is_infinite() {
                            print!(" ({:.1}s|{:.1}s)  ", seconds, est_remaining.max(0.0));
                        } else {
                            print!(" ({:.1}s|?s)  ", seconds);
                        }
                        std::io::stdout().flush().unwrap();
                    }
                })));
            } else {
                error!("Unable to get terminal width");
            }
            
        }
        reporter
    }

    pub fn update(&mut self, num: u64) {
        if num == 0 || PbrtOptions::instance().quiet {
            return;
        }
        self.work_done.fetch_add(num, Ordering::SeqCst);
    }

    pub fn done(&mut self) {
        self.work_done.store(self.total_work, Ordering::SeqCst);
    }

    pub fn elapsed_ms(&self) -> u128 {
        (Instant::now() - self.start_time).as_millis()
    }
}

unsafe impl Sync for ProgressReporter {}
unsafe impl Send for ProgressReporter {}

impl Drop for ProgressReporter {
    fn drop(&mut self) {
        if !PbrtOptions::instance().quiet {
            self.work_done.store(self.total_work, Ordering::SeqCst);
            self.exit_thread.store(true, Ordering::SeqCst);
            if self.update_thread.is_some() {
                self.update_thread.take().unwrap()
                    .join().expect("Tried to join a non running thread in ProgressReporter::drop()");
            }
            println!("");
        }
    }
}