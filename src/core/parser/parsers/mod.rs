mod utils;
pub use utils::*;
mod values;
pub use values::*;
mod transforms;
pub use transforms::*;
mod parameters;
pub use parameters::*;
mod directives;
pub use directives::*;

#[cfg(test)]
mod tests;

use crate::core::parser::file_span::FileSpan;
use nom::{IResult, error::VerboseError};


pub type ParseResult<T> = IResult<FileSpan, T, VerboseError<FileSpan>>;