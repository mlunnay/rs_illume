use std::sync::Arc;
use std::fmt;

pub trait Medium: fmt::Debug {

}

#[derive(Debug, Default, Clone)]
pub struct MediumInterface {
    pub inside: Option<Arc<dyn Medium>>,
    pub outside: Option<Arc<dyn Medium>>
}

impl MediumInterface {
    pub fn new(
        inside: Option<Arc<dyn Medium>>,
        outside: Option<Arc<dyn Medium>>
    ) -> MediumInterface {
        MediumInterface{
            inside,
            outside
        }
    }

    pub fn is_medium_transition(&self) -> bool {
        self.inside != self.outside
    }
}

impl<T> From<T> for MediumInterface
where
T: Medium + 'static
{
    fn from(medium: T) -> MediumInterface {
        MediumInterface{
            inside: Some(Arc::new(medium)),
            outside: Some(Arc::new(medium))
        }
    }
}

impl From<Arc<dyn Medium>> for MediumInterface {
    fn from(medium: Arc<dyn Medium>) -> MediumInterface {
        MediumInterface {
            inside: Some(medium.clone()),
            outside: Some(medium.clone())
        }
    }
}