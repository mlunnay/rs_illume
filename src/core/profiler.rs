use crossbeam_channel::{unbounded, Receiver, Sender};
use hashbrown::HashMap;
use std::sync::atomic::{AtomicUsize, AtomicBool, Ordering};
use parking_lot::Once;
use std::mem::transmute;
use std::io::Write;
use std::time::{Instant, Duration};
use std::thread::{ThreadId, self};
use std::hash::{Hash, Hasher};

#[derive(Debug)]
pub enum ProfileAction {
    Start(ThreadId, String),
    End(ThreadId, Duration)
}

pub struct Profiler {
    suspend_count: AtomicUsize,
    pub reciever: Receiver<ProfileAction>,
    pub sender: Sender<ProfileAction>,
    pub profiles: HashMap<String, Profile>,
    profile_hierarchies: HashMap<ThreadId, Vec<String>>,
    guard_count: AtomicUsize,
    finished: AtomicBool
}

impl Profiler {
    pub fn new() -> Self {
        let (sender, reciever) = unbounded::<ProfileAction>();
        Profiler{
            suspend_count: AtomicUsize::new(0),
            reciever,
            sender,
            profiles: HashMap::new(),
            profile_hierarchies: HashMap::new(),
            guard_count: AtomicUsize::new(0),
            finished: AtomicBool::new(false)
        }
    }

    /// Returns a singleton instance of Profiler.
    pub fn instance() -> &'static mut Self {
        static mut INSTANCE: *mut Profiler = 0 as *mut Profiler;
        static ONCE: Once = Once::new();

        unsafe {
            ONCE.call_once(|| {
                let instance = Profiler::new();
                // Put it in the heap so it can outlive this call
                INSTANCE = transmute(Box::new(instance));
            });

            &mut *INSTANCE
        }
    }

    /// Returns a ProfilePhase RAII Guard that communicates to this profiler its start and its duration when it goes out of scope.
    pub fn profile(&self, name: &str) -> ProfilePhase {
        ProfilePhase::new(self, name)
    }

    /// Recieve data on the communication channel.
    pub fn run(&mut self) {
        loop {
            let res = self.reciever.try_recv();
            match res {
                Ok(action) => {
                    println!("{:?}", action);
                    match action {
                        ProfileAction::Start(thread_id, name) => {
                            let path = self.profile_hierarchies.entry(thread_id).or_default();
                            path.push(name);
                        }
                        ProfileAction::End(thread_id, duration) => {
                            let path = self.profile_hierarchies.entry(thread_id).or_default();
                            let root_profile = self.profiles.entry(path[0].clone()).or_insert(Profile::new(path[0].clone()));
                            if path.len() == 1 {
                                root_profile.duration += duration;
                            }
                            else {
                                root_profile.get_or_create(&path[1..]).duration += duration;
                            }
                            path.pop();
                        }
                    }
                }
                Err(e) => {
                    // never seems to be disconnected even after drop.
                    if e.is_disconnected() {
                        break;
                    }
                }
            }
            // needed as channel does not seem to disconnect.
            if self.reciever.is_empty() && self.finished.load(Ordering::Acquire) == true {
                break;
            }
            thread::yield_now();
        }
    }

    /// Returns an RAII guard that signals a thread is using this Profiler.
    pub fn using(&self) -> ProfileGuard {
        ProfileGuard::new(self)
    }

    fn add_guard(&self) {
        self.guard_count.fetch_add(1, Ordering::SeqCst);
    }

    fn remove_guard(&self) {
        if self.guard_count.fetch_sub(1, Ordering::SeqCst) == 1 {
            drop(&self.reciever);
            self.finished.store(true, Ordering::SeqCst);
        }
    }

    pub fn suspend(&self) {
        self.suspend_count.fetch_add(1, Ordering::SeqCst);
    }

    pub fn resume(&self) {
        self.suspend_count.fetch_sub(1, Ordering::SeqCst);
    }

    pub fn print<T: Write>(&self, dest: &mut T) {
        let total_seconds = self.profiles.values().map(|p| p.duration).sum::<Duration>().as_secs_f64();
        
        for profile in self.profiles.values() {
            print_profile(&profile, dest, total_seconds, 0)
        }
    }
}

unsafe impl Send for Profiler {}
unsafe impl Sync for Profiler {}

/// An RAII guard for thread access to the Profiler
pub struct ProfileGuard<'a> {
    profiler: &'a Profiler
}

impl<'a> ProfileGuard<'a> {
    pub fn new(profiler: &'a Profiler) -> Self {
        profiler.add_guard();
        ProfileGuard{
            profiler
        }
    }
}

impl<'a> Drop for ProfileGuard<'a> {
    fn drop(&mut self) {
        self.profiler.remove_guard();
    }
}

/// An RAII object to time the duration from when its created to when it is dropped,
/// then sends it to its parent Profiler.
pub struct ProfilePhase<'a> {
    profiler: &'a Profiler,
    start: Instant,
    // profiler was not suspended when this ProfilePhase was created.
    active: bool
}

impl<'a> ProfilePhase<'a> {
    pub fn new(profiler: &'a Profiler, name: &str) -> Self {
        let active = profiler.suspend_count.load(Ordering::Acquire) == 0;
        if active {
            profiler.sender.send(ProfileAction::Start(thread::current().id(), name.into())).unwrap();
        }
        ProfilePhase {
            profiler,
            start: Instant::now(),
            active
        }
    }
}

impl<'a> Drop for ProfilePhase<'a> {
    fn drop(&mut self) {
        if self.active {
            self.profiler.sender.send(ProfileAction::End(thread::current().id(), self.start.elapsed())).unwrap();
        }
    }
}

impl <'a> Clone for ProfilePhase<'a> {
    fn clone(&self) -> Self {
        ProfilePhase {
            profiler: self.profiler.clone(),
            start: self.start.clone(),
            active: self.active
        }
    }
}

#[derive(Debug)]
pub struct Profile {
    pub parent_path: Vec<String>,
    pub name: String,
    pub duration: Duration,
    pub children: Vec<Profile>
}

impl Profile {
    pub fn new(name: String) -> Self {
        Profile {
            parent_path: Vec::new(),
            name,
            duration: Default::default(),
            children: Vec::new()
        }
    }

    /// Get a profile from the tree with the given path, creating it and any needed predecessors as needed.
    pub fn get_or_create(&mut self, path: &[String]) -> &mut Profile {
        if path.len() == 0 {
            return self;
        }
        let child;
         child = (0..self.children.len()).find(|&i| self.children[i].name == path[0]);
        
        let child = match child {
            Some(i) => &mut self.children[i],
            None => {
                let mut p = Profile::new(path[0].clone());
                let mut parent_path = self.parent_path.clone();
                parent_path.push(self.name.clone());
                p.parent_path = parent_path;
                self.children.push(p);
                self.children.last_mut().unwrap()
            }
        };
        if path.len() == 1 {
            return child;
        }
        return child.get_or_create(&path[1..]);
    }

    /// Flatten a Profiles path into a string seperated by /.
    pub fn path_as_string(&self) -> String {
        format!("{}/{}", self.parent_path.join("/"), self.name)
    }
}

impl Hash for Profile {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.name.hash(state);
    }
}

impl PartialEq for Profile {
    fn eq(&self, other: &Self) -> bool {
        self.name == other.name
    }
}

// recursie profile printer
fn print_profile<T: Write>(profile: &Profile, dest: &mut T, total: f64, indent: usize) {
    let pct = 100.0 * profile.duration.as_secs_f64() / total;
    dest.write(format!("{:>indent$}{:<trunc$} {:5.2}% ({})\n", "", profile.name, pct, time_string(&profile.duration), indent=indent, trunc=67-indent).as_bytes()).unwrap();
    for child in &profile.children {
        print_profile(child, dest, total, indent + 2);
    }
}

const SECONDS_IN_HOUR: f64 = 3600.0;
const SECONDS_IN_MINUTE: f64 = 60.0;
// convert a duration into a human readable string.
pub fn time_string(duration: &Duration) -> String {
    let mut seconds = duration.as_secs_f64() as f64;
    let hours = f64::floor(seconds / SECONDS_IN_HOUR) as u32;
    seconds -= hours as f64 * SECONDS_IN_HOUR;
    let minutes = f64::floor(seconds / SECONDS_IN_MINUTE) as u32;
    seconds -= minutes as f64 * SECONDS_IN_MINUTE;
    format!("{:4}:{:0<2}:{:4.2}", hours, minutes, seconds)
}