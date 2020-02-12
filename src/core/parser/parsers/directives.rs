use super::ParseResult;
use crate::core::parser::ast::*;
use crate::core::parser::file_span::FileSpan;
use super::values::*;
use super::utils::*;
use super::parameters::*;
use super::transforms::*;
use nom::{
    bytes::complete::{tag},
    error::{
        ParseError, VerboseError, ErrorKind
    },
    sequence::preceded,
    combinator::{map, complete},
    multi::{many0},
    branch::{alt},
    Err, IResult
};
use std::rc::Rc;
use std::io::prelude::*;
use std::path::Path;
use std::fs::File;
use std::error::Error;


pub fn parse_include(i: FileSpan) -> ParseResult<IdentValue> {
    let (i, start) = tag("Include")(i)?;
    preceded(ws_and_comment, parse_quoted_string)(i)
}

pub fn parse_camera(i: FileSpan) -> ParseResult<Directive> {
    let (i, start) = tag("Camera")(i)?;
    let (i, type_name) = preceded(ws_and_comment, parse_quoted_string)(i)?;
    let (i, parameters) = parameter_list(i)?;
    let end = if parameters.len() != 0 {
        parameters.last().unwrap().get_span()
    } else {
        type_name.span.clone()
    };
    Ok((i, Directive::Camera(CameraDirective {
        span: FileSpan::merge(&start, &end),
        type_name,
        parameters
    })))
}

pub fn parse_sampler(i: FileSpan) -> ParseResult<Directive> {
    let (i, start) = tag("Sampler")(i)?;
    let (i, type_name) = preceded(ws_and_comment, parse_quoted_string)(i)?;
    let (i, parameters) = parameter_list(i)?;
    let end = if parameters.len() != 0 {
        parameters.last().unwrap().get_span()
    } else {
        type_name.span.clone()
    };
    Ok((i, Directive::Sampler(SamplerDirective {
        span: FileSpan::merge(&start, &end),
        type_name,
        parameters
    })))
}

pub fn parse_film(i: FileSpan) -> ParseResult<Directive> {
    let (i, start) = tag("Film")(i)?;
    let (i, type_name) = preceded(ws_and_comment, parse_quoted_string)(i)?;
    let (i, parameters) = parameter_list(i)?;
    let end = if parameters.len() != 0 {
        parameters.last().unwrap().get_span()
    } else {
        type_name.span.clone()
    };
    Ok((i, Directive::Film(FilmDirective {
        span: FileSpan::merge(&start, &end),
        type_name,
        parameters
    })))
}

pub fn parse_pixel_filter(i: FileSpan) -> ParseResult<Directive> {
    let (i, start) = tag("PixelFilter")(i)?;
    let (i, type_name) = preceded(ws_and_comment, parse_quoted_string)(i)?;
    let (i, parameters) = parameter_list(i)?;
    let end = if parameters.len() != 0 {
        parameters.last().unwrap().get_span()
    } else {
        type_name.span.clone()
    };
    Ok((i, Directive::Filter(PixelFilterDirective {
        span: FileSpan::merge(&start, &end),
        type_name,
        parameters
    })))
}

pub fn parse_integrator(i: FileSpan) -> ParseResult<Directive> {
    let (i, start) = tag("Integrator")(i)?;
    let (i, type_name) = preceded(ws_and_comment, parse_quoted_string)(i)?;
    let (i, parameters) = parameter_list(i)?;
    let end = if parameters.len() != 0 {
        parameters.last().unwrap().get_span()
    } else {
        type_name.span.clone()
    };
    Ok((i, Directive::Integrator(IntegratorDirective {
        span: FileSpan::merge(&start, &end),
        type_name,
        parameters
    })))
}

pub fn parse_accelerator(i: FileSpan) -> ParseResult<Directive> {
    let (i, start) = tag("Accelerator")(i)?;
    let (i, type_name) = preceded(ws_and_comment, parse_quoted_string)(i)?;
    let (i, parameters) = parameter_list(i)?;
    let end = if parameters.len() != 0 {
        parameters.last().unwrap().get_span()
    } else {
        type_name.span.clone()
    };
    Ok((i, Directive::Accelerator(AcceleratorDirective {
        span: FileSpan::merge(&start, &end),
        type_name,
        parameters
    })))
}

pub fn parse_shape(i: FileSpan) -> ParseResult<Directive> {
    let (i, start) = tag("Shape")(i)?;
    let (i, type_name) = preceded(ws_and_comment, parse_quoted_string)(i)?;
    let (i, parameters) = parameter_list(i)?;
    let end = if parameters.len() != 0 {
        parameters.last().unwrap().get_span()
    } else {
        type_name.span.clone()
    };
    Ok((i, Directive::Shape(ShapeDirective {
        span: FileSpan::merge(&start, &end),
        type_name,
        parameters
    })))
}

pub fn parse_object_instance(i: FileSpan) -> ParseResult<Directive> {
    let (i, start) = tag("ObjectInstance")(i)?;
    let (i, type_name) = preceded(ws_and_comment, parse_quoted_string)(i)?;
    Ok((i, Directive::ObjectInstace(ObjectInstanceDirective {
        span: FileSpan::merge(&start, &type_name.span),
        type_name
    })))
}

pub fn parse_light_source(i: FileSpan) -> ParseResult<Directive> {
    let (i, start) = tag("LightSource")(i)?;
    let (i, type_name) = preceded(ws_and_comment, parse_quoted_string)(i)?;
    let (i, parameters) = parameter_list(i)?;
    let end = if parameters.len() != 0 {
        parameters.last().unwrap().get_span()
    } else {
        type_name.span.clone()
    };
    Ok((i, Directive::LightSource(LightSourceDirective {
        span: FileSpan::merge(&start, &end),
        type_name,
        parameters
    })))
}

pub fn parse_area_light_source(i: FileSpan) -> ParseResult<Directive> {
    let (i, start) = tag("AreaLightSource")(i)?;
    let (i, type_name) = preceded(ws_and_comment, parse_quoted_string)(i)?;
    let (i, parameters) = parameter_list(i)?;
    let end = if parameters.len() != 0 {
        parameters.last().unwrap().get_span()
    } else {
        type_name.span.clone()
    };
    Ok((i, Directive::AreaLightSource(AreaLightSourceDirective {
        span: FileSpan::merge(&start, &end),
        type_name,
        parameters
    })))
}

pub fn parse_material(i: FileSpan) -> ParseResult<Directive> {
    let (i, start) = tag("Material")(i)?;
    let (i, type_name) = preceded(ws_and_comment, parse_quoted_string)(i)?;
    let (i, parameters) = parameter_list(i)?;
    let end = if parameters.len() != 0 {
        parameters.last().unwrap().get_span()
    } else {
        type_name.span.clone()
    };
    Ok((i, Directive::Material(MaterialDirective {
        span: FileSpan::merge(&start, &end),
        type_name,
        parameters
    })))
}

pub fn parse_texture(i: FileSpan) -> ParseResult<Directive> {
    let (i, start) = tag("Texture")(i)?;
    let (i, name) = preceded(ws_and_comment, parse_quoted_string)(i)?;
    let (i, type_name) = preceded(ws_and_comment, parse_quoted_string)(i)?;
    let (i, class) = preceded(ws_and_comment, parse_quoted_string)(i)?;
    let (i, parameters) = parameter_list(i)?;
    let end = if parameters.len() != 0 {
        parameters.last().unwrap().get_span()
    } else {
        class.span.clone()
    };
    Ok((i, Directive::Texture(TextureDirective {
        span: FileSpan::merge(&start, &end),
        name,
        type_name,
        class,
        parameters
    })))
}

pub fn parse_named_medium(i: FileSpan) -> ParseResult<Directive> {
    let (i, start) = tag("MakeNamedMedium")(i)?;
    let (i, type_name) = preceded(ws_and_comment, parse_quoted_string)(i)?;
    let (i, parameters) = parameter_list(i)?;
    let end = if parameters.len() != 0 {
        parameters.last().unwrap().get_span()
    } else {
        type_name.span.clone()
    };
    Ok((i, Directive::NamedMedium(NamedMediumDirective {
        span: FileSpan::merge(&start, &end),
        type_name,
        parameters
    })))
}

pub fn parse_medium_interface(i: FileSpan) -> ParseResult<Directive> {
    let (i, start) = tag("MediumInterface")(i)?;
    let (i, interior) = preceded(ws_and_comment, parse_quoted_string)(i)?;
    let (i, exterior) = preceded(ws_and_comment, parse_quoted_string)(i)?;
    Ok((i, Directive::MediumInterface(MediumInterfaceDirective {
        span: FileSpan::merge(&start, &exterior.span),
        interior,
        exterior
    })))
}

/// Load the given file and parse it with the given parser.
pub fn do_include<O, F>(filename: &str, f: F) -> ParseResult<Vec<O>>
where
F: Fn(FileSpan) -> ParseResult<O>,
O: Clone
{
    let path = Path::new(filename);
    let mut data = String::new();
    match File::open(path) {
        Ok(mut f) => {
            if let Err(e) = f.read_to_string(&mut data) {
                data = e.description().into();
            }
        },
        Err(e) => {
            let span = FileSpan::new(Rc::new(String::from(e.description())), Rc::new(String::from(filename)));
            return Err(Err::Error(VerboseError::from_error_kind(span, ErrorKind::Tag)));
        }
    };
    let span = FileSpan::new(Rc::new(data), Rc::new(String::from(filename)));
    many0(preceded(ws_and_comment, f))(span)
}

pub fn many_with_include<O, F>(f: F) -> impl Fn(FileSpan) -> ParseResult<Vec<O>>
where
F: Fn(FileSpan) -> ParseResult<O> + Copy,
O: Clone + std::fmt::Debug
{
    move |i: FileSpan| {
        let mut i = i.clone();
        let mut acc: Vec<O> = Vec::with_capacity(4);
        
        loop {
            let test: ParseResult<IdentValue> = preceded(tag("Include"), preceded(ws_and_comment, parse_quoted_string))(i.clone());
            match test {
                Ok((i2, path)) => {
                    if let Ok((_i, directives)) = &do_include(&path.value, f) {
                        for i in 0..directives.len() {
                            acc.push(directives[i].clone());
                        }
                        i = i2;
                        continue;
                    }
                }
                _ => ()
            }
            match preceded(ws_and_comment, f)(i.clone()) {
                Err(Err::Error(_)) => {
                    return Ok((i, acc));
                },
                Err(e) => return Err(e),
                Ok((i1, o)) => {
                    if i1 == i {
                        return Err(Err::Error(VerboseError::from_error_kind(i, ErrorKind::Many0)));
                    }

                    i = i1.clone();
                    acc.push(o);
                }
            }
        }
    }
}

pub fn parse_object_block(i: FileSpan) -> ParseResult<Directive> {
    let (i, start) = tag("ObjectBegin")(i)?;
    let (i, name) = preceded(ws_and_comment, parse_quoted_string)(i)?;
    let (i, shapes) = many_with_include(parse_shape)(i)?;
    let res: IResult<FileSpan, FileSpan, VerboseError<FileSpan>> = preceded(sp, eoi)(i.clone());
    match res {
        Ok(r) => println!("End of Input: {:?}", r),
        _ => println!("Not End of Input")
    }
    let (i, end) = preceded(ws_and_comment, tag("ObjectEnd"))(i.clone())?;
    Ok((i, Directive::Object(ObjectBlock{
        span: FileSpan::merge(&start, &end),
        name,
        children: shapes.into_iter().map(move |s| match s { Directive::Shape(s) => s, _ => unreachable!() }).collect()
    })))
}

pub fn parse_attribute_block(i: FileSpan) -> ParseResult<Directive> {
    let (i, start) = tag("AttributeBegin")(i)?;
    let (i, children) = many_with_include(|i| 
        complete(alt((
            map(alt((
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
            )), |t| Directive::Transformation(t)),
            parse_shape,
            parse_object_instance,
            parse_light_source,
            parse_area_light_source,
            parse_material,
            parse_texture,
            parse_medium_interface
        )))(i)
    )(i)?;
    let (i, end) = preceded(ws_and_comment, tag("AttributeEnd"))(i)?;
    Ok((i, Directive::Attribute(AttributeBlock{
        span: FileSpan::merge(&start, &end),
        children
    })))
}

pub fn parse_transform_block(i: FileSpan) -> ParseResult<Directive> {
    let (i, start) = tag("TransformBegin")(i)?;
    let (i, children) = many_with_include(|i| 
        complete(alt((
            map(alt((
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
            )), |t| Directive::Transformation(t)),
            parse_shape,
            parse_object_instance,
            parse_light_source,
            // parse_area_light_source,
            parse_material,
            parse_texture,
            parse_medium_interface
        )))(i)
    )(i)?;
    let (i, end) = preceded(ws_and_comment, tag("TransformEnd"))(i)?;
    Ok((i, Directive::Transform(TransformBlock{
        span: FileSpan::merge(&start, &end),
        children
    })))
}

pub fn world_directive(i: FileSpan) -> ParseResult<Directive> {
    let (i, start) = tag("WorldBegin")(i)?;
    let (i, children) = many_with_include(|i|
        complete(alt((
            map(alt((
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
            )), |t| Directive::Transformation(t)),
            parse_shape,
            parse_object_instance,
            parse_light_source,
            parse_area_light_source,
            parse_material,
            parse_texture,
            parse_medium_interface,
            parse_attribute_block,
            parse_transform_block,
            parse_object_block
        )))(i)
    )(i)?;
    let (i, end) = preceded(ws_and_comment, tag("WorldEnd"))(i)?;
    Ok((i, Directive::World(WorldBlock{
        span: FileSpan::merge(&start, &end),
        children
    })))
}

pub fn root(i: FileSpan) -> ParseResult<Vec<Directive>> {
    let (i, mut directives) = preceded(ws_and_comment, 
        many_with_include(|i|
            alt((
                map(alt((
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
                )), |t| Directive::Transformation(t)),
                parse_camera,
                parse_sampler,
                parse_film,
                parse_pixel_filter,
                parse_integrator,
                parse_accelerator,
                parse_named_medium,
                parse_medium_interface
        ))(i)
    ))(i)?;
    let (i, world) = preceded(ws_and_comment, world_directive)(i)?;
    directives.push(world);
    Ok((i, directives))
}