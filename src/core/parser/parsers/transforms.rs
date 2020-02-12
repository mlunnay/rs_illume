use super::ParseResult;
use crate::core::parser::ast::*;
use crate::core::parser::file_span::FileSpan;
use super::values::*;
use super::utils::*;
use nom::{
    bytes::complete::{tag},
    sequence::{preceded},
    character::complete::space1,
    multi::many1,
    branch::{alt},
    combinator::complete,
    do_parse, tag, count, alt
};

pub fn parse_lookat(i: FileSpan) -> ParseResult<Transformation> {
    do_parse!(i.clone(),
        start: tag!("LookAt") >>
        values: count!(preceded(ws_and_comment, parse_number), 9) >>
        (Transformation::Lookat(Lookat{
            span: FileSpan::merge(&start, &values.last().unwrap().get_span()),
            eye_x: values[0].clone(),
            eye_y: values[1].clone(),
            eye_z: values[2].clone(),
            look_x: values[3].clone(),
            look_y: values[4].clone(),
            look_z: values[5].clone(),
            up_x: values[6].clone(),
            up_y: values[7].clone(),
            up_z: values[8].clone(),
        }))
    )
}

pub fn parse_identity(i: FileSpan) -> ParseResult<Transformation> {
    do_parse!(i,
        span: tag!("Identity") >>
        (Transformation::Identity(Identity {
            span: span.clone()
        }))
    )
}

pub fn parse_translate(i: FileSpan) -> ParseResult<Transformation> {
    do_parse!(i,
        start: tag!("Translate") >>
        space1 >>
        x: parse_number >>
        space1 >>
        y: parse_number >>
        space1 >>
        z: parse_number >>
        (Transformation::Translate(Translate {
            span: FileSpan::merge(&start, &z.get_span()),
            x,
            y,
            z
        }))
    )
}

pub fn parse_scale(i: FileSpan) -> ParseResult<Transformation> {
    do_parse!(i,
        start: tag!("Scale") >>
        space1 >>
        x: parse_number >>
        space1 >>
        y: parse_number >>
        space1 >>
        z: parse_number >>
        (Transformation::Scale(Scale {
            span: FileSpan::merge(&start, &z.get_span()),
            x,
            y,
            z
        }))
    )
}

pub fn parse_rotate(i: FileSpan) -> ParseResult<Transformation> {
    do_parse!(i,
        start: tag!("Rotate") >>
        space1 >>
        angle: parse_number >>
        space1 >>
        x: parse_number >>
        space1 >>
        y: parse_number >>
        space1 >>
        z: parse_number >>
        (Transformation::Rotate(Rotate {
            span: FileSpan::merge(&start, &z.get_span()),
            angle,
            x,
            y,
            z
        }))
    )
}

pub fn parse_coordinate_system(i: FileSpan) -> ParseResult<Transformation> {
    do_parse!(i,
        start: tag!("CoordinateSystem") >>
        space1 >>
        identity: parse_quoted_string >>
        (Transformation::CoordinateSystem(CoordinateSystem {
            span: FileSpan::merge(&start, &identity.get_span()),
            identity
        }))
    )
}

pub fn parse_coord_sys_transform(i: FileSpan) -> ParseResult<Transformation> {
    do_parse!(i,
        start: tag!("CoordSysTransform") >>
        space1 >>
        identity: parse_quoted_string >>
        (Transformation::CoordSysTransform(CoordSysTransform {
            span: FileSpan::merge(&start, &identity.get_span()),
            identity
        }))
    )
}

pub fn parse_transform(i: FileSpan) -> ParseResult<Transformation> {
    do_parse!(i,
        start: tag!("Transform") >>
        space1 >>
        m00: parse_number >>
        space1 >>
        m01: parse_number >>
        space1 >>
        m02: parse_number >>
        space1 >>
        m03: parse_number >>
        space1 >>
        m10: parse_number >>
        space1 >>
        m11: parse_number >>
        space1 >>
        m12: parse_number >>
        space1 >>
        m13: parse_number >>
        space1 >>
        m20: parse_number >>
        space1 >>
        m21: parse_number >>
        space1 >>
        m22: parse_number >>
        space1 >>
        m23: parse_number >>
        space1 >>
        m30: parse_number >>
        space1 >>
        m31: parse_number >>
        space1 >>
        m32: parse_number >>
        space1 >>
        m33: parse_number >>
        (Transformation::Transform(Transform {
            span: FileSpan::merge(&start, &m33.get_span()),
            m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31, m32, m33
        }))
    )
}

pub fn parse_concat_transform(i: FileSpan) -> ParseResult<Transformation> {
    do_parse!(i,
        start: tag!("ConcatTransform") >>
        space1 >>
        m00: parse_number >>
        space1 >>
        m01: parse_number >>
        space1 >>
        m02: parse_number >>
        space1 >>
        m03: parse_number >>
        space1 >>
        m10: parse_number >>
        space1 >>
        m11: parse_number >>
        space1 >>
        m12: parse_number >>
        space1 >>
        m13: parse_number >>
        space1 >>
        m20: parse_number >>
        space1 >>
        m21: parse_number >>
        space1 >>
        m22: parse_number >>
        space1 >>
        m23: parse_number >>
        space1 >>
        m30: parse_number >>
        space1 >>
        m31: parse_number >>
        space1 >>
        m32: parse_number >>
        space1 >>
        m33: parse_number >>
        (Transformation::ConcatTransform(ConcatTransform {
            span: FileSpan::merge(&start, &m33.get_span()),
            m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31, m32, m33
        }))
    )
}

pub fn parse_active_transform(i: FileSpan) -> ParseResult<Transformation> {
    do_parse!(i,
        start: tag!("ActiveTransform") >>
        space1 >>
        transform_type: alt!(tag!("StartTime") | tag!("EndTime") | tag!("All")) >>
        (Transformation::ActiveTransform(ActiveTransform {
            span: FileSpan::merge(&start, &transform_type),
            transform_type: match &*transform_type {
                "StartTime" => ActiveTransformType::StartTime,
                "EndTime" => ActiveTransformType::EndTime,
                "All" => ActiveTransformType::All,
                _ => panic!()
            }
        }))
    )
}

pub fn parse_reverse_orientation(i: FileSpan) -> ParseResult<Transformation> {
    tag("ReverseOrientation")(i).map(|(i, s)| (i, Transformation::ReverseOrientation(ReverseOrientation{ span: s.clone() })))
}

pub fn transform_list(i: FileSpan) -> ParseResult<Vec<Transformation>> {
    many1(preceded(ws_and_comment,
        complete(alt((
            parse_identity,
            parse_translate,
            parse_scale,
            parse_rotate,
            parse_lookat,
            parse_coordinate_system,
            parse_coord_sys_transform,
            parse_transform,
            parse_concat_transform,
            parse_active_transform,
            parse_reverse_orientation
        )))
    ))(i)
}

pub fn parse_transform_times(i: FileSpan) -> ParseResult<TransformTimes> {
    do_parse!(i,
        s: tag!("TransformTimes") >>
        space1 >>
        start: parse_number >>
        space1 >>
        end: parse_number >>
        (TransformTimes {
            span: FileSpan::merge(&s, &end.get_span()),
            start,
            end
        })
    )
}