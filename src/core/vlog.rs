//! A verbosity log for controlled output logging.
//! This will log any log messages that are less than or equal to the logging level.
//! Log level 0 is considered off and can not be logged to.
//! This always logs at the info level of logging using the log crate.
#[macro_export]

pub static mut log_level: usize = 0;

/// Logs to the Verbosity Log at the given level.
macro_rules! vlog {
    ($level: expr, $($arg:tt)*) => {
        if $level <= $crate::core::vlog::log_level {
            info!($($arg)*);
        }
    };
}

pub fn set_log_level(level: usize) {
    unsafe {
        log_level = level;
    }
}

pub fn get_log_level() -> usize {
    unsafe {
        log_level
    }
}
