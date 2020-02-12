pub mod parsers;
pub use parsers::*;
pub mod ast;
pub mod file_span;
pub mod visitor;

use ast::Directive;
use file_span::FileSpan;

use std::env::{set_current_dir};
use std::path::Path;
use std::io::prelude::*;
use std::fs::File;
use std::io;
use std::rc::Rc;

/// Parse a file and return the AST.
/// This function sets the current directory to the parent path of the file.
pub fn parse_to_ast(filename: &str) -> io::Result<Vec<Directive>> {
    let path = Path::new(filename);

    set_current_dir(path.canonicalize()?)?;

    let mut data = String::new();
    let mut file = File::open(path)?;
    file.read_to_string(&mut data)?;
    let span = FileSpan::new(Rc::new(data), Rc::new(String::from(filename)));

    match root(span){
        Ok((_, d)) => Ok(d),
        Err(e) => {
            // TODO: print parsing errors to console

            Err(io::Error::from(io::ErrorKind::Other))
        }
    }
}

/// Parse the given file and set the parser states.
// TODO: add parameters for parser state
pub fn parse(filename: &str) -> io::Result<()> {
    let ast = parse_to_ast(filename)?;

    // TODO: visit ast and set the parser states

    Ok(())
}