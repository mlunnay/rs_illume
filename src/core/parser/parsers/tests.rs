use nom::sequence::terminated;
use super::*;
use crate::core::parser::file_span::FileSpan;
use crate::core::parser::ast::*;
use std::rc::Rc;
use nom::Err;
use std::path::Path;

macro_rules! parse_with_cast {
    ($parser: ident, $cast: path, $input: expr) => {
        match $parser(FileSpan::new(Rc::new($input.into()), Rc::new(String::from("input")))) {
            Ok(r) => {
                match r.1 {
                    $cast(i) => i,
                    _ => panic!()
                }
            }
            Err(Err::Error(e)) | Err(Err::Failure(e)) => {
                println!("** parsing error:");
                println!("{}", convert_error_span($input, e));
                panic!();
            }
            Err(e) => {
                println!("{:?}", e);
                panic!();
            }
        };
    };
}

macro_rules! parse {
    ($parser: ident, $input: expr) => {
        match $parser(FileSpan::new(Rc::new($input.into()), Rc::new(String::from("input")))) {
            Ok(r) => r.1,
            Err(Err::Error(e)) | Err(Err::Failure(e)) => {
                println!("** parsing error (kind: {:?}):", e.errors.len()/* [0].1 */);
                println!("{}", convert_error_span($input, e));
                panic!();
            }
            Err(e) => {
                println!("{:?}", e);
                panic!();
            }
        };
    };
}

#[test]
fn test_ws_and_comment() {
    let comment = parse!(ws_and_comment, " \t#test\n  ");
    assert_eq!(&*comment[0], "test");
    assert_eq!(parse!(ws_and_comment, " \t\n  ").len(), 0);
    assert_eq!(parse!(ws_and_comment, "#test\n#test\n").len(), 2);
}

fn convert_error_span(input: &str, e: VerboseError<FileSpan>) -> String {
    let errors = e.errors.iter().map(|v| (&*v.0, v.1.clone())).collect();
    let e = VerboseError {
        errors
    };
    nom::error::convert_error(input, e)
}

#[test]
fn test_parse_number() {
    assert_eq!(parse_number(FileSpan::new(Rc::new(String::from("3.14")), Rc::new(String::from("input")))).unwrap().1.value, 3.14);
    assert_eq!(parse_number(FileSpan::new(Rc::new(String::from("-3.14")), Rc::new(String::from("input")))).unwrap().1.value, -3.14);
    assert_eq!(parse_number(FileSpan::new(Rc::new(String::from("3e10")), Rc::new(String::from("input")))).unwrap().1.value, 3e10);
    assert_eq!(parse_number(FileSpan::new(Rc::new(String::from("3E10")), Rc::new(String::from("input")))).unwrap().1.value, 3e10);
    assert_eq!(parse_number(FileSpan::new(Rc::new(String::from("3e-10")), Rc::new(String::from("input")))).unwrap().1.value, 3e-10);
    assert_eq!(parse_number(FileSpan::new(Rc::new(String::from("0.5")), Rc::new(String::from("input")))).unwrap().1.value,0.5);
    assert_eq!(parse_number(FileSpan::new(Rc::new(String::from(".5")), Rc::new(String::from("input")))).unwrap().1.value, 0.5);
}

#[test]
fn test_parse_int() {
    assert_eq!(parse_int(FileSpan::new(Rc::new(String::from("4")), Rc::new(String::from("input")))).unwrap().1.value, 4.0);
    assert_eq!(parse_int(FileSpan::new(Rc::new(String::from("4350")), Rc::new(String::from("input")))).unwrap().1.value, 4350.0);
    assert_eq!(parse_int(FileSpan::new(Rc::new(String::from("-1")), Rc::new(String::from("input")))).unwrap().1.value, -1.0);
    assert_eq!(parse_int(FileSpan::new(Rc::new(String::from("0")), Rc::new(String::from("input")))).unwrap().1.value, 0.0);
}

#[test]
fn test_parse_ident() {
    assert_eq!(parse_ident(FileSpan::new(Rc::new(String::from("test")), Rc::new(String::from("input")))).unwrap().1, IdentValue{
        span: FileSpan {
            data: Rc::new(String::from("test")),
            filename: Rc::new(String::from("input")),
            offset: 0,
            length: "test".len(),
            line: 1
        },
        value: String::from("test")
    });
    assert_eq!(parse_ident(FileSpan::new(Rc::new(String::from("te\\nst")), Rc::new(String::from("input")))).unwrap().1, IdentValue{
        span: FileSpan {
            data: Rc::new(String::from("te\\nst")),
            filename: Rc::new(String::from("input")),
            offset: 0,
            length: "te\\nst".len(),
            line: 1,
        },
        value: String::from("te\\nst")
    });
}

#[test]
fn test_parse_quoted_string() {
    assert_eq!(parse_quoted_string(FileSpan::new(Rc::new(String::from("\"test\"")), Rc::new(String::from("input")))).unwrap().1, IdentValue{
        span: FileSpan {
            data: Rc::new(String::from("\"test\"")),
            filename: Rc::new(String::from("input")),
            offset: 0,
            length: "\"test\"".len(),
            line: 1,
        },
        value: String::from("test")
    });
}

#[test]
fn test_parse_lookat() {
    // let lookat = parse_lookat(FileSpan::new(Rc::new(String::from("LookAt 0 0 0 10 10 10 0 1 0")), Rc::new(String::from("input")))).unwrap().1;
    let lookat = match parse_lookat(FileSpan::new(Rc::new(String::from("LookAt 0 0 0 10 10 10 0 1 0")), Rc::new(String::from("input")))).unwrap().1 {
        Transformation::Lookat(l) => l,
        _ => panic!()
    };
    // println!("{:?}", lookat);
    assert_eq!(lookat.eye_x.value, 0.0);
    assert_eq!(lookat.eye_y.value, 0.0);
    assert_eq!(lookat.eye_z.value, 0.0);
    assert_eq!(lookat.look_x.value, 10.0);
    assert_eq!(lookat.look_y.value, 10.0);
    assert_eq!(lookat.look_z.value, 10.0);
    assert_eq!(lookat.up_x.value, 0.0);
    assert_eq!(lookat.up_y.value, 1.0);
    assert_eq!(lookat.up_z.value, 0.0);

    let input = "LookAt 3 4 1.5  # eye
    .5 .5 0  # look at point
    0 0 1    # up vector";
    let lookat = parse_with_cast!(parse_lookat, Transformation::Lookat, input);
    assert_eq!(lookat.eye_x.value, 3.0);
    assert_eq!(lookat.eye_y.value, 4.0);
    assert_eq!(lookat.eye_z.value, 1.5);
    assert_eq!(lookat.look_x.value, 0.5);
    assert_eq!(lookat.look_y.value, 0.5);
    assert_eq!(lookat.look_z.value, 0.0);
    assert_eq!(lookat.up_x.value, 0.0);
    assert_eq!(lookat.up_y.value, 0.0);
    assert_eq!(lookat.up_z.value, 1.0);
}

#[test]
fn test_parse_translate() {
    let translate = match parse_translate(FileSpan::new(Rc::new(String::from("Translate 4 5.5 1.0")), Rc::new(String::from("input")))).unwrap().1 {
        Transformation::Translate(t) => t,
        _ => panic!()
    };
    assert_eq!(translate.x.value, 4.0);
    assert_eq!(translate.y.value, 5.5);
    assert_eq!(translate.z.value, 1.0);
}

#[test]
fn test_parse_scale() {
    let scale = match parse_scale(FileSpan::new(Rc::new(String::from("Scale 4 5.5 1.0")), Rc::new(String::from("input")))).unwrap().1 {
        Transformation::Scale(s) => s,
        _ => panic!()
    };
    assert_eq!(scale.x.value, 4.0);
    assert_eq!(scale.y.value, 5.5);
    assert_eq!(scale.z.value, 1.0);
}

#[test]
fn test_parse_rotate() {
    let rotate = match parse_rotate(FileSpan::new(Rc::new(String::from("Rotate 45 4 5.5 1.0")), Rc::new(String::from("input")))).unwrap().1 {
        Transformation::Rotate(r) => r,
        _ => panic!()
    };
    assert_eq!(rotate.angle.value, 45.0);
    assert_eq!(rotate.x.value, 4.0);
    assert_eq!(rotate.y.value, 5.5);
    assert_eq!(rotate.z.value, 1.0);
}

#[test]
fn test_parse_coordinate_system() {
    let cs = match parse_coordinate_system(FileSpan::new(Rc::new(String::from("CoordinateSystem \"test\"")), Rc::new(String::from("input")))).unwrap().1 {
        Transformation::CoordinateSystem(c) => c,
        _ => panic!()
    };
    assert_eq!(cs.identity.value, String::from("test"));
}

#[test]
fn test_parse_coord_sys_transform() {
    let cs = match parse_coord_sys_transform(FileSpan::new(Rc::new(String::from("CoordSysTransform \"test\"")), Rc::new(String::from("input")))).unwrap().1 {
        Transformation::CoordSysTransform(c) => c,
        _ => panic!()
    };
    assert_eq!(cs.identity.value, String::from("test"));
}

#[test]
fn test_parse_transform() {
    let transform = match parse_transform(FileSpan::new(Rc::new(String::from("Transform 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16")), Rc::new(String::from("input")))).unwrap().1 {
        Transformation::Transform(t) => t,
        _ => panic!()
    };
    assert_eq!(transform.m00.value, 1.0);
    assert_eq!(transform.m01.value, 2.0);
    assert_eq!(transform.m02.value, 3.0);
    assert_eq!(transform.m03.value, 4.0);
    assert_eq!(transform.m10.value, 5.0);
    assert_eq!(transform.m11.value, 6.0);
    assert_eq!(transform.m12.value, 7.0);
    assert_eq!(transform.m13.value, 8.0);
    assert_eq!(transform.m20.value, 9.0);
    assert_eq!(transform.m21.value, 10.0);
    assert_eq!(transform.m22.value, 11.0);
    assert_eq!(transform.m23.value, 12.0);
    assert_eq!(transform.m30.value, 13.0);
    assert_eq!(transform.m31.value, 14.0);
    assert_eq!(transform.m32.value, 15.0);
    assert_eq!(transform.m33.value, 16.0);
}

#[test]
fn test_parse_concat_transform() {
    let transform = match parse_concat_transform(FileSpan::new(Rc::new(String::from("ConcatTransform 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16")), Rc::new(String::from("input")))).unwrap().1 {
        Transformation::ConcatTransform(t) => t,
        _ => panic!()
    };
    assert_eq!(transform.m00.value, 1.0);
    assert_eq!(transform.m01.value, 2.0);
    assert_eq!(transform.m02.value, 3.0);
    assert_eq!(transform.m03.value, 4.0);
    assert_eq!(transform.m10.value, 5.0);
    assert_eq!(transform.m11.value, 6.0);
    assert_eq!(transform.m12.value, 7.0);
    assert_eq!(transform.m13.value, 8.0);
    assert_eq!(transform.m20.value, 9.0);
    assert_eq!(transform.m21.value, 10.0);
    assert_eq!(transform.m22.value, 11.0);
    assert_eq!(transform.m23.value, 12.0);
    assert_eq!(transform.m30.value, 13.0);
    assert_eq!(transform.m31.value, 14.0);
    assert_eq!(transform.m32.value, 15.0);
    assert_eq!(transform.m33.value, 16.0);
}

#[test]
fn test_parse_active_transform() {
    let transform = match parse_active_transform(FileSpan::new(Rc::new(String::from("ActiveTransform StartTime")), Rc::new(String::from("input")))).unwrap().1 {
        Transformation::ActiveTransform(t) => t,
        _ => panic!()
    };
    assert_eq!(transform.transform_type, ActiveTransformType::StartTime);
    let transform = match parse_active_transform(FileSpan::new(Rc::new(String::from("ActiveTransform EndTime")), Rc::new(String::from("input")))).unwrap().1 {
        Transformation::ActiveTransform(t) => t,
        _ => panic!()
    };
    assert_eq!(transform.transform_type, ActiveTransformType::EndTime);
    let transform = match parse_active_transform(FileSpan::new(Rc::new(String::from("ActiveTransform All")), Rc::new(String::from("input")))).unwrap().1 {
        Transformation::ActiveTransform(t) => t,
        _ => panic!()
    };
    assert_eq!(transform.transform_type, ActiveTransformType::All);
}

#[test]
fn test_parse_transform_times() {
    let transform = parse_transform_times(FileSpan::new(Rc::new(String::from("TransformTimes 100 250.6")), Rc::new(String::from("input")))).unwrap().1;
    assert_eq!(transform.start.value, 100.0);
    assert_eq!(transform.end.value, 250.6);
}

#[test]
fn test_parse_integer_parameters() {
    // let params = match parse_integer_parameters(FileSpan::new(Rc::new(String::from("\"integer test\" 20")), Rc::new(String::from("input")))).unwrap().1 {
    //     Parameter::Integer(i) => i,
    //     _ => panic!()
    // };
    let params = match parse_integer_parameters(FileSpan::new(Rc::new(String::from("\"integer test\" 20")), Rc::new(String::from("input")))) {
        Ok(p) => { match p.1 {
            Parameter::Integer(i) => i,
            _ => panic!()
        }}
        Err(Err::Error(e)) | Err(Err::Failure(e)) => {
            println!("error parsing:\n{}", convert_error_span("\"integer test\" 20", e));
            panic!();
        }
        _ => panic!()
    };
    assert_eq!(params.name.value, String::from("test"));
    assert_eq!(params.values.len(), 1);
    assert_eq!(params.values[0].value, 20.0);

    // let params = match parse_integer_parameters(FileSpan::new(Rc::new(String::from("\"integer test\" [20]")), Rc::new(String::from("input")))).unwrap().1 {
    //     Parameter::Integer(i) => i,
    //     _ => panic!()
    // };
    let params = match parse_integer_parameters(FileSpan::new(Rc::new(String::from("\"integer test\" [20]")), Rc::new(String::from("input")))) {
        Ok(p) => { match p.1 {
            Parameter::Integer(i) => i,
            _ => panic!()
        }}
        Err(Err::Error(e)) | Err(Err::Failure(e)) => {
            println!("error parsing:\n{}", convert_error_span("\"integer test\" [20]", e));
            panic!();
        }
        _ => panic!()
    };

    assert_eq!(params.values.len(), 1);
    assert_eq!(params.values[0].value, 20.0);

    // let params = match parse_integer_parameters(FileSpan::new(Rc::new(String::from("\"integer test\" [20 42 36]")), Rc::new(String::from("input")))).unwrap().1 {
    //     Parameter::Integer(i) => i,
    //     _ => panic!()
    // };
    let params = match parse_integer_parameters(FileSpan::new(Rc::new(String::from("\"integer test\" [20 42 36]")), Rc::new(String::from("input")))) {
        Ok(p) => { match p.1 {
            Parameter::Integer(i) => i,
            _ => panic!()
        }}
        Err(Err::Error(e)) | Err(Err::Failure(e)) => {
            println!("error parsing:\n{}", convert_error_span("\"integer test\" [20 42 36]", e));
            panic!();
        }
        _ => panic!()
    };
    assert_eq!(params.values.len(), 3);
    assert_eq!(params.values[0].value, 20.0);
    assert_eq!(params.values[1].value, 42.0);
    assert_eq!(params.values[2].value, 36.0);
}

#[test]
fn test_parse_float_parameters() {
    let params = match parse_float_parameters(FileSpan::new(Rc::new(String::from("\"float test\" 20.5")), Rc::new(String::from("input")))) {
        Ok(r) => {
            match r.1 {
                Parameter::Float(i) => i,
                _ => panic!()
            }
        }
        Err(Err::Error(e)) | Err(Err::Failure(e)) => {
            println!("** parsing error:");
            println!("{:?}", convert_error_span("\"float test\" 20.5", e));
            panic!();
        }
        _ => panic!()
    };
    assert_eq!(params.name.value, String::from("test"));
    assert_eq!(params.values.len(), 1);
    assert_eq!(params.values[0].value, 20.5);

    let params = match parse_float_parameters(FileSpan::new(Rc::new(String::from("\"float test\" [20.5]")), Rc::new(String::from("input")))).unwrap().1 {
        Parameter::Float(i) => i,
        _ => panic!()
    };
    assert_eq!(params.values.len(), 1);
    assert_eq!(params.values[0].value, 20.5);

    let params = match parse_float_parameters(FileSpan::new(Rc::new(String::from("\"float test\" [20.5 42 36.0]")), Rc::new(String::from("input")))).unwrap().1 {
        Parameter::Float(i) => i,
        _ => panic!()
    };
    assert_eq!(params.values.len(), 3);
    assert_eq!(params.values[0].value, 20.5);
    assert_eq!(params.values[1].value, 42.0);
    assert_eq!(params.values[2].value, 36.0);
}

#[test]
fn test_named_parameter() {
    let (name, _) = parse_named_parameter(FileSpan::new(Rc::new(String::from("\"float test\"")), Rc::new(String::from("input"))), "float").unwrap().1;
    assert_eq!(name.value, String::from("test"));
    let (name, _) = parse_named_parameter(FileSpan::new(Rc::new(String::from("\"int test2\"")), Rc::new(String::from("input"))), "int").unwrap().1;
    assert_eq!(name.value, String::from("test2"));
    assert!(parse_named_parameter(FileSpan::new(Rc::new(String::from("\"float test\"")), Rc::new(String::from("input"))), "string").is_err());
}

#[test]
fn test_parse_point2_parameter() {
    let params = match parse_point2_parameter(FileSpan::new(Rc::new(String::from("\"point2 test\" [20.5 10]")), Rc::new(String::from("input")))).unwrap().1 {
        Parameter::Point2(i) => i,
        _ => panic!()
    };
    assert_eq!(params.values.len(), 1);
    assert_eq!((params.values[0].0.value, params.values[0].1.value), (20.5, 10.0));
}

#[test]
fn test_parse_vector2_parameter() {
    let params = match parse_vector2_parameter(FileSpan::new(Rc::new(String::from("\"vector2 test\" [20.5 10]")), Rc::new(String::from("input")))).unwrap().1 {
        Parameter::Vector2(i) => i,
        _ => panic!()
    };
    assert_eq!(params.values.len(), 1);
    assert_eq!((params.values[0].0.value, params.values[0].1.value), (20.5, 10.0));
}

#[test]
fn test_parse_point3_parameter() {
    let params = match parse_point3_parameter(FileSpan::new(Rc::new(String::from("\"point3 test\" [20.5 10 0]")), Rc::new(String::from("input")))).unwrap().1 {
        Parameter::Point3(i) => i,
        _ => panic!()
    };
    assert_eq!(params.values.len(), 1);
    assert_eq!((params.values[0].0.value, params.values[0].1.value, params.values[0].2.value), (20.5, 10.0, 0.0));

    let params = match parse_point3_parameter(FileSpan::new(Rc::new(String::from("\"point test\" [20.5 10 0]")), Rc::new(String::from("input")))).unwrap().1 {
        Parameter::Point3(i) => i,
        _ => panic!()
    };
    assert_eq!(params.values.len(), 1);
    assert_eq!((params.values[0].0.value, params.values[0].1.value, params.values[0].2.value), (20.5, 10.0, 0.0));
}

#[test]
fn test_parse_vector3_parameter() {
    let params = match parse_vector3_parameter(FileSpan::new(Rc::new(String::from("\"vector3 test\" [20.5 10 0]")), Rc::new(String::from("input")))).unwrap().1 {
        Parameter::Vector3(i) => i,
        _ => panic!()
    };
    assert_eq!(params.values.len(), 1);
    assert_eq!((params.values[0].0.value, params.values[0].1.value, params.values[0].2.value), (20.5, 10.0, 0.0));

    let params = match parse_vector3_parameter(FileSpan::new(Rc::new(String::from("\"vector test\" [20.5 10 0]")), Rc::new(String::from("input")))).unwrap().1 {
        Parameter::Vector3(i) => i,
        _ => panic!()
    };
    assert_eq!(params.values.len(), 1);
    assert_eq!((params.values[0].0.value, params.values[0].1.value, params.values[0].2.value), (20.5, 10.0, 0.0));
}

#[test]
fn test_parse_normal3_parameter() {
    let params = match parse_normal3_parameter(FileSpan::new(Rc::new(String::from("\"normal3 test\" [20.5 10 0]")), Rc::new(String::from("input")))).unwrap().1 {
        Parameter::Normal3(i) => i,
        _ => panic!()
    };
    assert_eq!(params.values.len(), 1);
    assert_eq!((params.values[0].0.value, params.values[0].1.value, params.values[0].2.value), (20.5, 10.0, 0.0));

    let params = match parse_normal3_parameter(FileSpan::new(Rc::new(String::from("\"normal test\" [20.5 10 0]")), Rc::new(String::from("input")))).unwrap().1 {
        Parameter::Normal3(i) => i,
        _ => panic!()
    };
    assert_eq!(params.values.len(), 1);
    assert_eq!((params.values[0].0.value, params.values[0].1.value, params.values[0].2.value), (20.5, 10.0, 0.0));
}

#[test]
fn test_parse_spectrum() {
    let params = match parse_spectrum(FileSpan::new(Rc::new(String::from("\"spectrum test\" [300 .3 400 .6   410 .65  415 .8  500 .2  600 .1]")), Rc::new(String::from("input")))) {
        Ok(r) => {
            match r.1 {
                Parameter::Spectrum(i) => i,
                _ => panic!()
            }
        }
        Err(Err::Error(e)) | Err(Err::Failure(e)) => {
            println!("** parsing error:");
            println!("{}", convert_error_span("\"spectrum test\" [300 .3 400 .6   410 .65  415 .8  500 .2  600 .1]", e));
            panic!();
        }
        _ => panic!()
    };
    assert_eq!(params.name.value, "test");
    assert_eq!(params.spectrum_type, SpectrumType::Sampled);
    assert_eq!(params.values.len(), 1);
    assert_eq!(params.values[0].len(), 12);
    assert_eq!(params.values[0][0].value, 300.0);
    assert_eq!(params.values[0][1].value, 0.3);
}

#[test]
fn test_parse_spectrum_parameter() {
    let params = match parse_spectrum_parameter(FileSpan::new(Rc::new(String::from("\"rgb test\" [.2 .5 .3]")), Rc::new(String::from("input")))).unwrap().1 {
        Parameter::Spectrum(i) => i,
        _ => panic!()
    };
    assert_eq!(params.name.value, "test");
    assert_eq!(params.spectrum_type, SpectrumType::RGB);
    assert_eq!(params.values.len(), 1);
    assert_eq!(params.values[0][0].value, 0.2);
    assert_eq!(params.values[0][1].value, 0.5);
    assert_eq!(params.values[0][2].value, 0.3);

    let params = match parse_spectrum_parameter(FileSpan::new(Rc::new(String::from("\"color test\" [.2 .5 .3]")), Rc::new(String::from("input")))).unwrap().1 {
        Parameter::Spectrum(i) => i,
        _ => panic!()
    };
    assert_eq!(params.name.value, "test");
    assert_eq!(params.spectrum_type, SpectrumType::RGB);
    assert_eq!(params.values.len(), 1);
    assert_eq!(params.values[0][0].value, 0.2);
    assert_eq!(params.values[0][1].value, 0.5);
    assert_eq!(params.values[0][2].value, 0.3);

    let params = match parse_spectrum_parameter(FileSpan::new(Rc::new(String::from("\"xyz test\" [.4 .5 .6]")), Rc::new(String::from("input")))) {
        Ok(r) => {
            match r.1 {
                Parameter::Spectrum(i) => i,
                _ => panic!()
            }
        }
        Err(Err::Error(e)) | Err(Err::Failure(e)) => {
            println!("** parsing error:");
            println!("{}", convert_error_span("\"xyz test\" [.4 .5 .6]", e));
            panic!();
        }
        _ => panic!()
    };
    assert_eq!(params.name.value, "test");
    assert_eq!(params.spectrum_type, SpectrumType::XYZ);
    assert_eq!(params.values.len(), 1);
    assert_eq!(params.values[0][0].value, 0.4);
    assert_eq!(params.values[0][1].value, 0.5);
    assert_eq!(params.values[0][2].value, 0.6);

    let params = match parse_spectrum_parameter(FileSpan::new(Rc::new(String::from("\"spectrum test\" [300 .3  400 .6   410 .65  415 .8  500 .2  600 .1]")), Rc::new(String::from("input")))) {
        Ok(r) => {
            match r.1 {
                Parameter::Spectrum(i) => i,
                _ => panic!()
            }
        }
        Err(Err::Error(e)) | Err(Err::Failure(e)) => {
            println!("** parsing error:");
            println!("{}", convert_error_span("\"spectrum test\" [300 .3  400 .6   410 .65  415 .8  500 .2  600 .1]", e));
            panic!();
        }
        _ => panic!()
    };
    assert_eq!(params.name.value, "test");
    assert_eq!(params.spectrum_type, SpectrumType::Sampled);
    assert_eq!(params.values.len(), 1);
    assert_eq!(params.values[0].len(), 12);
    assert_eq!(params.values[0][0].value, 300.0);
    assert_eq!(params.values[0][1].value, 0.3);

    let params = match parse_spectrum_parameter(FileSpan::new(Rc::new(String::from("\"blackbody test\" [.4 .5]")), Rc::new(String::from("input")))) {
        Ok(r) => {
            match r.1 {
                Parameter::Spectrum(i) => i,
                _ => panic!()
            }
        }
        Err(Err::Error(e)) | Err(Err::Failure(e)) => {
            println!("** parsing error:");
            println!("{}", convert_error_span("\"blackbody test\" [.4 .5]", e));
            panic!();
        }
        _ => panic!()
    };
    assert_eq!(params.name.value, "test");
    assert_eq!(params.spectrum_type, SpectrumType::Blackbody);
    assert_eq!(params.values.len(), 1);
    assert_eq!(params.values[0][0].value, 0.4);
    assert_eq!(params.values[0][1].value, 0.5);

    let params = match parse_spectrum_parameter(FileSpan::new(Rc::new(String::from("\"spectrum test\" \"filename.test\"")), Rc::new(String::from("input")))) {
        Ok(r) => {
            match r.1 {
                Parameter::SpectrumPath(i) => i,
                _ => panic!()
            }
        }
        Err(Err::Error(e)) | Err(Err::Failure(e)) => {
            println!("** parsing error:");
            println!("{}", convert_error_span("\"spectrum test\" \"filename.test\"", e));
            panic!();
        }
        _ => panic!()
    };

    assert_eq!(params.name.value, "test");
    assert_eq!(params.path.value, "filename.test");
}

#[test]
fn test_string_parameter() {
    let params = match parse_string_parameter(FileSpan::new(Rc::new(String::from("\"string test\" \"test value\"")), Rc::new(String::from("input")))) {
        Ok(r) => {
            match r.1 {
                Parameter::String(i) => i,
                _ => panic!()
            }
        }
        Err(Err::Error(e)) | Err(Err::Failure(e)) => {
            println!("** parsing error:");
            println!("{}", convert_error_span("\"string test\" \"test value\"", e));
            panic!();
        }
        _ => panic!()
    };

    assert_eq!(params.name.value, "test");
    assert_eq!(params.value.value, "test value");
}

#[test]
fn test_boolean_parameter() {
    let params = match parse_boolean_parameter(FileSpan::new(Rc::new(String::from("\"bool test\" \"true\"")), Rc::new(String::from("input")))) {
        Ok(r) => {
            match r.1 {
                Parameter::Boolean(i) => i,
                _ => panic!()
            }
        }
        Err(Err::Error(e)) | Err(Err::Failure(e)) => {
            println!("** parsing error:");
            println!("{}", convert_error_span("\"bool test\" \"true\"", e));
            panic!();
        }
        _ => panic!()
    };

    assert_eq!(params.name.value, "test");
    assert_eq!(params.value.value, true);

    let params = match parse_boolean_parameter(FileSpan::new(Rc::new(String::from("\"bool test\" \"false\"")), Rc::new(String::from("input")))) {
        Ok(r) => {
            match r.1 {
                Parameter::Boolean(i) => i,
                _ => panic!()
            }
        }
        Err(Err::Error(e)) | Err(Err::Failure(e)) => {
            println!("** parsing error:");
            println!("{}", convert_error_span("\"bool test\" \"false\"", e));
            panic!();
        }
        _ => panic!()
    };

    assert_eq!(params.name.value, "test");
    assert_eq!(params.value.value, false);
}

#[test]
fn test_parse_comment() {
    let comment = parse_comment(FileSpan::new(Rc::new(String::from(" \t#test\n")), Rc::new(String::from("input")))).unwrap().1;
    assert_eq!(&*comment, "test");
}

#[test]
fn test_parameter_list() {
    let res: IResult<FileSpan, Vec<Parameter>, VerboseError<FileSpan>> = terminated(parameter_list, eoi)(FileSpan::new(Rc::new(String::from("\"bool test\" \"true\"")), Rc::new(String::from("input"))));
    let parameters = res.unwrap().1;
    // let parameters = parse!(parameter_list, eoi), "\"bool test\" \"true\" ");
    assert_eq!(parameters.len(), 1);
    match &parameters[0] {
        Parameter::Boolean(p) => {
            assert_eq!(p.name.value, "test");
            assert_eq!(p.value.value, true);
        },
        _ => panic!("parameter is not boolean")
    }

    let parameters = parse!(parameter_list, "\"bool test\" \"true\" #test\n#test\n\"string test2\" \"test string\"");
    // println!("{:?}", parameters);
    assert_eq!(parameters.len(), 2);
    match &parameters[0] {
        Parameter::Boolean(p) => {
            assert_eq!(p.name.value, "test");
            assert_eq!(p.value.value, true);
        },
        _ => panic!("parameter 0 is not boolean")
    }
    match &parameters[1] {
        Parameter::String(p) => {
            assert_eq!(p.name.value, "test2");
            assert_eq!(p.value.value, "test string");
        },
        _ => panic!("parameter 1 is not string")
    }
}

#[test]
fn test_transforms() {
    let transforms = parse!(transform_list, "Scale 1 2 3#test\nTranslate 1 2 3");
    assert_eq!(transforms.len(), 2);
    match &transforms[0] {
        Transformation::Scale(s) => {
            assert_eq!(s.x.value, 1.0);
        },
        _ => panic!("transformation 0 is not a Scale")
    }match &transforms[1] {
        Transformation::Translate(s) => {
            assert_eq!(s.x.value, 1.0);
        },
        _ => panic!("transformation 1 is not a Scale")
    }
}

#[test]
fn test_camera() {
    let camera = parse_with_cast!(parse_camera, Directive::Camera, "Camera \"realistic\"");
    assert_eq!(camera.type_name.value, "realistic");

    let camera = parse_with_cast!(parse_camera, Directive::Camera, "Camera \"realistic\" \"string test\" \"test\"");
    assert_eq!(camera.type_name.value, "realistic");
    assert_eq!(camera.parameters.len(), 1);
    match &camera.parameters[0] {
        Parameter::String(p) => {
            assert_eq!(p.name.value, "test");
            assert_eq!(p.value.value, "test");
        },
        _ => panic!("parameter 0 is not string")
    }
}

#[test]
fn test_object_block() {
    let object = parse_with_cast!(parse_object_block, Directive::Object, "ObjectBegin \"shape1\"
            Shape \"sphere\" \"float radius\" 1
            Shape \"trianglemesh\"
                \"integer indices\" [0 1 2 0 2 3]
                \"point P\" [ -20 -20 0   20 -20 0   20 20 0   -20 20 0 ]
                \"float st\" [ 0 0   1 0    1 1   0 1 ]
        ObjectEnd");
    assert_eq!(object.name.value, "shape1");
    assert_eq!(object.children.len(), 2);
}

#[test]
fn test_attribute_block() {
    let object = parse_with_cast!(parse_attribute_block, Directive::Attribute, "AttributeBegin
    Texture \"checks\" \"spectrum\" \"checkerboard\"
            \"float uscale\" [8] \"float vscale\" [8]
            \"rgb tex1\" [.1 .1 .1] \"rgb tex2\" [.8 .8 .8]
    Material \"matte\" \"texture Kd\" \"checks\"
    Translate 0 0 -1
    Shape \"trianglemesh\"
        \"integer indices\" [0 1 2 0 2 3]
        \"point P\" [ -20 -20 0   20 -20 0   20 20 0   -20 20 0 ]
        \"float st\" [ 0 0   1 0    1 1   0 1 ]
    AttributeEnd");
    assert_eq!(object.children.len(), 4);
}

#[test]
fn test_transform_block() {
    let object = parse_with_cast!(parse_transform_block, Directive::Transform, "TransformBegin
        Translate 1 0 1
        Shape \"sphere\"
    TransformEnd");
    assert_eq!(object.children.len(), 2);
}

#[test]
fn test_world_directive() {
    let world = parse_with_cast!(world_directive, Directive::World, "WorldBegin

        # uniform blue-ish illumination from all directions
        LightSource \"infinite\" \"rgb L\" [.4 .45 .5]
        
        # approximate the sun
        LightSource \"distant\"  \"point from\" [ -30 40  100 ]
        \"blackbody L\" [3000 1.5]
        
        AttributeBegin
        Material \"glass\"
        Shape \"sphere\" \"float radius\" 1
        AttributeEnd
        
        AttributeBegin
        Texture \"checks\" \"spectrum\" \"checkerboard\"
                \"float uscale\" [8] \"float vscale\" [8]
                \"rgb tex1\" [.1 .1 .1] \"rgb tex2\" [.8 .8 .8]
        Material \"matte\" \"texture Kd\" \"checks\"
        Translate 0 0 -1
        Shape \"trianglemesh\"
            \"integer indices\" [0 1 2 0 2 3]
            \"point P\" [ -20 -20 0   20 -20 0   20 20 0   -20 20 0 ]
            \"float st\" [ 0 0   1 0    1 1   0 1 ]
        AttributeEnd
    
    WorldEnd");
    assert_eq!(world.children.len(), 4);
}

#[test]
fn test_many_with_include() {
    use std::io::Write;
    let test_include = "Shape \"sphere\" \"float radius\" 1
    Shape \"trianglemesh\"
        \"integer indices\" [0 1 2 0 2 3]
        \"point P\" [ -20 -20 0   20 -20 0   20 20 0   -20 20 0 ]
        \"float st\" [ 0 0   1 0    1 1   0 1 ]";
    let mut path = std::env::temp_dir();
    path.push(Path::new("shapes.pbrt"));
    {
        std::env::set_current_dir(std::env::temp_dir()).unwrap();
        
        let mut file = std::fs::File::create(&path).unwrap();
        file.write(test_include.as_bytes()).unwrap();
        file.flush().unwrap();

        let parser =  many_with_include(parse_shape);
        let ast = parse!(parser, "Include \"shapes.pbrt\"\nShape \"sphere\" \"float radius\" 1");
        assert_eq!(ast.len(), 3);
    }
    
    std::fs::remove_file(&path).unwrap();
}

#[test]
fn test_root() {
    // testing with the example prbt file from the file format docs at https://pbrt.org/fileformat-v3.html#example
    let ast = parse!(root, "LookAt 3 4 1.5  # eye
    .5 .5 0  # look at point
    0 0 1    # up vector
Camera \"perspective\" \"float fov\" 45

Sampler \"halton\" \"integer pixelsamples\" 128
Integrator \"path\"
Film \"image\" \"string filename\" \"simple.png\"
    \"integer xresolution\" [400] \"integer yresolution\" [400]

WorldBegin

# uniform blue-ish illumination from all directions
LightSource \"infinite\" \"rgb L\" [.4 .45 .5]

# approximate the sun
LightSource \"distant\"  \"point from\" [ -30 40  100 ]
\"blackbody L\" [3000 1.5]

AttributeBegin
Material \"glass\"
Shape \"sphere\" \"float radius\" 1
AttributeEnd

AttributeBegin
Texture \"checks\" \"spectrum\" \"checkerboard\"
        \"float uscale\" [8] \"float vscale\" [8]
        \"rgb tex1\" [.1 .1 .1] \"rgb tex2\" [.8 .8 .8]
Material \"matte\" \"texture Kd\" \"checks\"
Translate 0 0 -1
Shape \"trianglemesh\"
    \"integer indices\" [0 1 2 0 2 3]
    \"point P\" [ -20 -20 0   20 -20 0   20 20 0   -20 20 0 ]
    \"float st\" [ 0 0   1 0    1 1   0 1 ]
AttributeEnd

WorldEnd");
    assert_eq!(ast.len(), 6);
}