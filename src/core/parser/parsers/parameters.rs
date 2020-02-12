use super::ParseResult;
use crate::core::parser::ast::*;
use crate::core::parser::file_span::FileSpan;
use super::values::*;
use super::utils::*;

use nom::{
    bytes::complete::{tag},
    character::complete::{
        one_of, char
    },
    error::VerboseError,
    sequence::{
        preceded, pair, separated_pair, tuple
    },
    combinator::{recognize, map, complete},
    multi::{separated_list, count, many1, many0},
    branch::{alt, Alt},
};

pub fn parse_named_parameter(i: FileSpan, type_str: &str) -> ParseResult<(IdentValue, FileSpan)> 
{
    pair(recognize(char('\"')),
        pair(
            preceded(tag(type_str),
                preceded(sp,
                    parse_ident)
            ),
            recognize(char('\"'))
        )
    )(i).map(|(i, (start, (name, end)))|
        (i, (name, FileSpan::merge(&start, &end)))
    )
}

pub fn parse_alt_named_parameter<'a, List: Alt<FileSpan, FileSpan, VerboseError<FileSpan>>>(i: FileSpan, l: List) -> ParseResult<(IdentValue, FileSpan)> {
    pair(recognize(char('\"')),
        pair(
            preceded(alt(l),
                preceded(sp,
                    parse_ident)
            ),
            recognize(char('\"'))
        )
    )(i).map(|(i, (start, (name, end)))|
        (i, (name, FileSpan::merge(&start, &end)))
    )
}

macro_rules! parameter_array {
    ($parser: expr) => {
        map(pair(recognize(char('[')),
            preceded(sp, pair(
                separated_list(many1(one_of(" \t\r\n")), $parser),
                preceded(sp, recognize(char(']')))
            ))
        ), |(start, (v, end))|
        (v, FileSpan::merge(&start, &end)))
    };
}

pub fn parse_integer_parameters(i: FileSpan) -> ParseResult<Parameter> {
    pair(
        |i| parse_named_parameter(i, "integer"),
        preceded(sp,
            alt((
                map(parse_int, |v| (vec![v.clone()], v.span.clone())),
                parameter_array!(parse_int)
            ))
        )
    )(i).map(|(i, ((name, start), (values, end)))| {
        (i, Parameter::Integer(IntegerParameter{
            span: FileSpan::merge(&start, &end),
            name: name,
            values: values
        }))
    })
}

pub fn parse_float_parameters(i: FileSpan) -> ParseResult<Parameter> {
    pair(
        |i| parse_named_parameter(i, "float"),
        preceded(sp, 
            alt((
                map(parse_number, |v| (vec![v.clone()], v.span.clone())),
                parameter_array!(parse_number)
            ))
        )
    )(i).map(|(i, ((name, start), (values, end)))| {
        (i, Parameter::Float(FloatParameter{
            span: FileSpan::merge(&start, &end),
            name: name,
            values: values
        }))
    })
}

pub fn parse_point2_parameter(i: FileSpan) -> ParseResult<Parameter> {
    pair(
        |i| parse_named_parameter(i, "point2"),
        preceded(sp, parameter_array!(
            separated_pair(parse_number, sp, parse_number)
        ))
    )(i).map(|(i, ((name, start), (values, end)))| 
        (i, Parameter::Point2(Point2Parameter{
            span: FileSpan::merge(&start, &end),
            name: name,
            values: values
        }))
    )
}

pub fn parse_vector2_parameter(i: FileSpan) -> ParseResult<Parameter> {
    pair(
        |i| parse_named_parameter(i, "vector2"),
        preceded(sp, parameter_array!(
            pair(parse_number,
                preceded(sp, parse_number)
            )))
    )(i).map(|(i, ((name, start), (values, end)))| 
        (i, Parameter::Vector2(Vector2Parameter{
            span: FileSpan::merge(&start, &end),
            name: name,
            values: values
        }))
    )
}

pub fn parse_point3_parameter(i: FileSpan) -> ParseResult<Parameter> {
    pair(
        |i| parse_alt_named_parameter(i, (tag("point3"), tag("point"))),
        preceded(sp, parameter_array!(
            count(preceded(sp, parse_number), 3)
        ))
    )(i).map(|(i, ((name, start), (values, end)))| {
        let values = values.iter().map(|v| (v[0].clone(), v[1].clone(), v[2].clone())).collect();
        (i, Parameter::Point3(Point3Parameter{
            span: FileSpan::merge(&start, &end),
            name: name,
            values: values
        }))
    })
}

pub fn parse_vector3_parameter(i: FileSpan) -> ParseResult<Parameter> {
    pair(
        |i| parse_alt_named_parameter(i, (tag("vector3"), tag("vector"))),
        preceded(sp, parameter_array!(
            count(preceded(sp, parse_number), 3)
        ))
    )(i).map(|(i, ((name, start), (values, end)))| {
        let values = values.iter().map(|v| (v[0].clone(), v[1].clone(), v[2].clone())).collect();
        (i, Parameter::Vector3(Vector3Parameter{
            span: FileSpan::merge(&start, &end),
            name: name,
            values: values
        }))
    })
}

pub fn parse_normal3_parameter(i: FileSpan) -> ParseResult<Parameter> {
    pair(
        |i| parse_alt_named_parameter(i, (tag("normal3"), tag("normal"))),
        preceded(sp, parameter_array!(
            count(preceded(sp, parse_number), 3)
        ))
    )(i).map(|(i, ((name, start), (values, end)))| {
        let values = values.iter().map(|v| (v[0].clone(), v[1].clone(), v[2].clone())).collect();
        (i, Parameter::Normal3(Normal3Parameter{
            span: FileSpan::merge(&start, &end),
            name: name,
            values: values
        }))
    })
}

pub fn parse_rgb(i: FileSpan) -> ParseResult<Parameter> {
    pair(
        |i| parse_alt_named_parameter(i, (tag("rgb"), tag("color"))),
        preceded(sp, parameter_array!(
            count(preceded(sp, parse_number), 3)
        ))
    )(i).map(|(i, ((name, start), (values, end)))| {
        (i, Parameter::Spectrum(SpectrumParameter{
            span: FileSpan::merge(&start, &end),
            name: name,
            spectrum_type: SpectrumType::RGB,
            values: values
        }))
    })
}

pub fn parse_xyz(i: FileSpan) -> ParseResult<Parameter> {
    pair(
        |i| parse_named_parameter(i, "xyz"),
        preceded(sp, parameter_array!(
            count(preceded(sp, parse_number), 3)
        ))
    )(i).map(|(i, ((name, start), (values, end)))| {
        (i, Parameter::Spectrum(SpectrumParameter{
            span: FileSpan::merge(&start, &end),
            name: name,
            spectrum_type: SpectrumType::XYZ,
            values: values
        }))
    })
}

pub fn parse_spectrum(i: FileSpan) -> ParseResult<Parameter> {
    pair(
        |i| parse_named_parameter(i, "spectrum"),
        preceded(sp, parameter_array!(
            parse_number
        ))
    )(i).map(|(i, ((name, start), (values, end)))| {
        (i, Parameter::Spectrum(SpectrumParameter{
            span: FileSpan::merge(&start, &end),
            name: name,
            spectrum_type: SpectrumType::Sampled,
            values: vec![values]
        }))
    })
}

pub fn parse_blackbody(i: FileSpan) -> ParseResult<Parameter> {
    pair(
        |i| parse_named_parameter(i, "blackbody"),
        preceded(sp, parameter_array!(
            count(preceded(sp, parse_number), 2)
        ))
    )(i).map(|(i, ((name, start), (values, end)))| {
        (i, Parameter::Spectrum(SpectrumParameter{
            span: FileSpan::merge(&start, &end),
            name: name,
            spectrum_type: SpectrumType::Blackbody,
            values: values
        }))
    })
}

pub fn parse_spectrum_file(i: FileSpan) -> ParseResult<Parameter> {
    pair(
        |i| parse_named_parameter(i, "spectrum"),
        preceded(sp, parse_quoted_string)
    )(i).map(|(i, ((name, start), path))| {
        (i, Parameter::SpectrumPath(SpectrumPathParameter{
            span: FileSpan::merge(&start, &path.span),
            name: name,
            path: path
        }))
    })
}

pub fn parse_spectrum_parameter(i: FileSpan) -> ParseResult<Parameter> {
    alt((
        parse_rgb,
        parse_xyz,
        parse_spectrum,
        parse_spectrum_file,
        parse_blackbody
    ))(i)
}

pub fn parse_string_parameter(i: FileSpan) -> ParseResult<Parameter> {
    pair(
        |i| parse_named_parameter(i, "string"),
        preceded(sp, parse_quoted_string)
    )(i).map(|(i, ((name, start), value))| {
        (i, Parameter::String(StringParameter{
            span: FileSpan::merge(&start, &value.span),
            name,
            value
        }))
    })
}

pub fn parse_boolean_parameter(i: FileSpan) -> ParseResult<Parameter> {
    pair(
        |i| parse_named_parameter(i, "bool"),
        preceded(sp, 
            map(tuple((
                recognize(char('\"')), 
                alt((
                    tag("true"),
                    tag("false")
                )),
                recognize(char('\"'))
            )), |(start, value, end)| BooleanValue {
                    span: FileSpan::merge(&start, &end),
                    value: match &*value {
                        "true" => true,
                        "false" => false,
                        _ => panic!("parse_boolean_parameter never should have reached here")
                    }
        }))
    )(i).map(|(i, ((name, start), value))| {
        (i, Parameter::Boolean(BooleanParameter{
            span: FileSpan::merge(&start, &value.span),
            name,
            value
        }))
    })
}

pub fn parse_texture_parameter(i: FileSpan) -> ParseResult<Parameter> {
    pair(
        |i| parse_named_parameter(i, "texture"),
        preceded(sp, parse_quoted_string)
    )(i).map(|(i, ((name, start), value))| {
        (i, Parameter::Texture(TextureParameter{
            span: FileSpan::merge(&start, &value.span),
            name,
            value
        }))
    })
}

pub fn parameter_list(i: FileSpan) -> ParseResult<Vec<Parameter>> {
    many0(preceded(ws_and_comment,
        alt((
            complete(parse_integer_parameters),
            complete(parse_float_parameters),
            complete(parse_point2_parameter),
            complete(parse_vector2_parameter),
            complete(parse_point3_parameter),
            complete(parse_vector3_parameter),
            complete(parse_normal3_parameter),
            complete(parse_spectrum_parameter),
            complete(parse_boolean_parameter),
            complete(parse_string_parameter),
            complete(parse_texture_parameter)
        ))
    ))(i)
}

pub fn parameter(i: FileSpan) -> ParseResult<Parameter> {
    alt((
        complete(parse_integer_parameters),
        complete(parse_float_parameters),
        complete(parse_point2_parameter),
        complete(parse_vector2_parameter),
        complete(parse_point3_parameter),
        complete(parse_vector3_parameter),
        complete(parse_normal3_parameter),
        complete(parse_spectrum_parameter),
        complete(parse_boolean_parameter),
        complete(parse_string_parameter),
        complete(parse_texture_parameter)
    ))(i)
}