use super::ParseResult;
use crate::core::parser::ast::*;
use crate::core::parser::file_span::FileSpan;
use nom::{
    bytes::complete::{tag},
    character::complete::{one_of, digit1},
    sequence::{pair},
    combinator::{opt, recognize, map},
    number::complete::double,
    ParseTo, do_parse, recognize, escaped, one_of, tag, take_until
};

pub fn parse_number(i: FileSpan) -> ParseResult<NumberValue> {
    map(recognize(double), |span: FileSpan| NumberValue {
        span: span.clone(),
        value: (*span).parse::<f64>().unwrap()
    })(i)
    
}

pub fn parse_int(i: FileSpan) -> ParseResult<NumberValue> {
    map(recognize(pair(opt(tag("-")), digit1)), |span: FileSpan| NumberValue {
        span: span.clone(),
        value: (*span).parse::<f64>().unwrap()
    })(i)
    
}

pub fn parse_ident(i: FileSpan) -> ParseResult<IdentValue> { 
    do_parse!(i,
        span: recognize!(
            escaped!(
            one_of!("_|-.abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789"),
            '\\', one_of("\"ntr\\")
        )) >>
        (IdentValue {
            span: span.clone(),
            value: span.parse_to().unwrap()
        })
    )
}

pub fn parse_quoted_string(i: FileSpan) -> ParseResult<IdentValue> {
    do_parse!(i,
        start: tag!("\"") >>
        span: take_until!("\"") >>
        end: tag!("\"") >>
        (IdentValue {
            span: FileSpan::merge(&start, &end),
            value: span.parse_to().unwrap()
        })
    )
}