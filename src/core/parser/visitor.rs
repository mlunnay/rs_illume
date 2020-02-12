//! # AST visitor
//! This module provides the Visitor trait and implimentations for
//! the PBRT API visitor for instantiating a pbrt scene. 

/// AST Visitor
/// # Example
/// ```
/// impl Visitor for Scale {
///     type Output = ();
///     fn visit(&mut self) -> Result<Self::Output, ()> {
///         set_scale(self.x.value, self.y.value, self.z.value);
///     }
/// }
/// ```
pub trait Visitor {
    type Output;

    /// Visitor method for implementations.
    /// Should return OK() to continue or Err(()) to stop further traversal.
    /// This allows easy error propigatio so internally this should be called like:
    /// ```
    /// let child = self.child.visit()?;
    /// ```
    fn visit(&mut self) -> Result<Self::Output,()>;
}

/// Visitor start point.
pub fn visit<T, O>(ast: &mut Vec<T>) -> Result<Vec<O>, ()>
where
T: Visitor<Output=O>,
{
    let mut res: Vec<O> = Vec::new();
    for child in ast {
        match child.visit() {
            Ok(o) => res.push(o),
            Err(()) => return Err(())
        }
    }
    Ok(res)
}