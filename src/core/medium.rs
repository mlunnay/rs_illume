use std::sync::Arc;

pub trait Medium{

}

pub struct MediumInterface {
    pub inside: Option<Arc<dyn Medium>>,
    pub outside: Option<Arc<dyn Medium>>
}