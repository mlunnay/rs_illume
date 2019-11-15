use std::sync::Arc;

pub trait Medium{

}

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