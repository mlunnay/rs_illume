use super::ParseResult;
use crate::core::parser::file_span::FileSpan;
use nom::{
    bytes::complete::{take_while},
    error::{ParseError, VerboseError, ErrorKind},
    branch::{alt},
    character::complete::{line_ending, not_line_ending, multispace0, char},
    sequence::{preceded, terminated},
    combinator::{opt},
    multi::many0,
    InputLength, Err
};

pub fn sp(i: FileSpan) -> ParseResult<FileSpan>
{
    let chars = " \t\r\n";
  
    // nom combinators like `take_while` return a function. That function is the
    // parser,to which we can pass the input
    take_while(move |c| chars.contains(c))(i)
}

/// End of input parser
pub fn eoi(i: FileSpan) -> ParseResult<FileSpan> {
    if i.input_len() == 0 {
        Ok((i.clone(), i.clone()))
    } else {
        Err(Err::Error(VerboseError::from_error_kind(i, ErrorKind::Eof)))
    }
}

pub fn eol(i: FileSpan) -> ParseResult<FileSpan> {
    alt((
        eoi,
        line_ending
    ))(i)
}

pub fn parse_comment(i: FileSpan) -> ParseResult<FileSpan> {
    preceded(opt(sp),
        preceded(char('#'),
            not_line_ending)
    )(i)
}

pub fn ws_and_comment(i: FileSpan) -> ParseResult<Vec<FileSpan>> {
    preceded(
        multispace0,
        many0(terminated(
            parse_comment,
            multispace0
        ))
    )(i)
}