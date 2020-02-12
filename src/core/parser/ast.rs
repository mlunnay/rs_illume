use super::file_span::FileSpan;

pub trait GetSpan {
    fn get_span(&self) -> FileSpan;
}

macro_rules! impl_get_span {
    ($t: ty) => {
        impl GetSpan for $t {
            fn get_span(&self) -> FileSpan {
                self.span.clone()
            }
        }
    };
}

pub trait GetValue {
    type Output;
    fn get_value(&self) -> &Self::Output;
}

macro_rules! impl_get_value {
    ($t: ty, $r: ty) => {
        impl GetValue for $t {
            type Output = $r;
            fn get_value(&self) -> &$r {
                &self.value
            }
        }
    };
}

#[derive(Debug, Clone, PartialEq)]
pub struct NumberValue {
    pub span: FileSpan,
    pub value: f64
}

impl_get_span!(NumberValue);
impl_get_value!(NumberValue, f64);

#[derive(Debug, PartialEq, Clone)]
pub struct IdentValue {
    pub span: FileSpan,
    pub value: String
}

impl_get_span!(IdentValue);
impl_get_value!(IdentValue, String);

#[derive(Debug, PartialEq, Clone)]
pub struct BooleanValue {
    pub span: FileSpan,
    pub value: bool
}

impl_get_span!(BooleanValue);
impl_get_value!(BooleanValue, bool);

#[derive(Debug, PartialEq, Clone)]
pub enum Value {
    Number(NumberValue),
    Ident(IdentValue),
    Boolean(BooleanValue)
}

impl Value {
    pub fn get_span(&self) -> FileSpan {
        match self {
            Value::Number(n) => n.span.clone(),
            Value::Ident(s) => s.span.clone(),
            Value::Boolean(b) => b.span.clone()
        }
    }
}

#[derive(Debug, PartialEq, Clone)]
pub struct Identity {
    pub span: FileSpan
}

impl_get_span!(Identity);

#[derive(Debug, PartialEq, Clone)]
pub struct Lookat {
    pub span: FileSpan,
    pub eye_x: NumberValue,
    pub eye_y: NumberValue,
    pub eye_z: NumberValue,
    pub look_x: NumberValue,
    pub look_y: NumberValue,
    pub look_z: NumberValue,
    pub up_x: NumberValue,
    pub up_y: NumberValue,
    pub up_z: NumberValue
}

impl_get_span!(Lookat);

#[derive(Debug, PartialEq, Clone)]
pub struct Translate {
    pub span: FileSpan,
    pub x: NumberValue,
    pub y: NumberValue,
    pub z: NumberValue
}

impl_get_span!(Translate);

#[derive(Debug, PartialEq, Clone)]
pub struct Scale {
    pub span: FileSpan,
    pub x: NumberValue,
    pub y: NumberValue,
    pub z: NumberValue
}

impl_get_span!(Scale);

#[derive(Debug, PartialEq, Clone)]
pub struct Rotate {
    pub span: FileSpan,
    pub angle: NumberValue,
    pub x: NumberValue,
    pub y: NumberValue,
    pub z: NumberValue
}

impl_get_span!(Rotate);

#[derive(Debug, PartialEq, Clone)]
pub struct CoordinateSystem {
    pub span: FileSpan,
    pub identity: IdentValue
}

impl_get_span!(CoordinateSystem);

#[derive(Debug, PartialEq, Clone)]
pub struct CoordSysTransform {
    pub span: FileSpan,
    pub identity: IdentValue
}

impl_get_span!(CoordSysTransform);

#[derive(Debug, PartialEq, Clone)]
pub struct Transform {
    pub span: FileSpan,
    pub m00: NumberValue,
    pub m01: NumberValue,
    pub m02: NumberValue,
    pub m03: NumberValue,
    pub m10: NumberValue,
    pub m11: NumberValue,
    pub m12: NumberValue,
    pub m13: NumberValue,
    pub m20: NumberValue,
    pub m21: NumberValue,
    pub m22: NumberValue,
    pub m23: NumberValue,
    pub m30: NumberValue,
    pub m31: NumberValue,
    pub m32: NumberValue,
    pub m33: NumberValue,
}

impl_get_span!(Transform);

#[derive(Debug, PartialEq, Clone)]
pub struct ConcatTransform {
    pub span: FileSpan,
    pub m00: NumberValue,
    pub m01: NumberValue,
    pub m02: NumberValue,
    pub m03: NumberValue,
    pub m10: NumberValue,
    pub m11: NumberValue,
    pub m12: NumberValue,
    pub m13: NumberValue,
    pub m20: NumberValue,
    pub m21: NumberValue,
    pub m22: NumberValue,
    pub m23: NumberValue,
    pub m30: NumberValue,
    pub m31: NumberValue,
    pub m32: NumberValue,
    pub m33: NumberValue,
}

impl_get_span!(ConcatTransform);

#[derive(Debug, PartialEq, Clone)]
pub enum ActiveTransformType {
    StartTime,
    EndTime,
    All
}

#[derive(Debug, PartialEq, Clone)]
pub struct ActiveTransform {
    pub span: FileSpan,
    pub transform_type: ActiveTransformType
}

impl_get_span!(ActiveTransform);

#[derive(Debug, PartialEq, Clone)]
pub struct TransformTimes {
    pub span: FileSpan,
    pub start: NumberValue,
    pub end: NumberValue
}

impl_get_span!(TransformTimes);

#[derive(Debug, PartialEq, Clone)]
pub struct ReverseOrientation {
    pub span: FileSpan
}

impl_get_span!(ReverseOrientation);

#[derive(Debug, PartialEq, Clone)]
pub enum Transformation {
    Identity(Identity),
    Lookat(Lookat),
    Translate(Translate),
    Scale(Scale),
    Rotate(Rotate),
    CoordinateSystem(CoordinateSystem),
    CoordSysTransform(CoordSysTransform),
    Transform(Transform),
    ConcatTransform(ConcatTransform),
    ActiveTransform(ActiveTransform),
    ReverseOrientation(ReverseOrientation)
}

impl Transformation {
    pub fn get_span(&self) -> FileSpan {
        match self {
            Transformation::Identity(s) => s.span.clone(),
            Transformation::Lookat(s) => s.span.clone(),
            Transformation::Translate(s) => s.span.clone(),
            Transformation::Scale(s) => s.span.clone(),
            Transformation::Rotate(s) => s.span.clone(),
            Transformation::CoordinateSystem(s) => s.span.clone(),
            Transformation::CoordSysTransform(s) => s.span.clone(),
            Transformation::Transform(s) => s.span.clone(),
            Transformation::ConcatTransform(s) => s.span.clone(),
            Transformation::ActiveTransform(s) => s.span.clone(),
            Transformation::ReverseOrientation(s) => s.span.clone()
        }
    }
}

#[derive(Debug, PartialEq, Clone)]
pub struct IntegerParameter {
    pub span: FileSpan,
    pub name: IdentValue,
    pub values: Vec<NumberValue>
}

#[derive(Debug, PartialEq, Clone)]
pub struct FloatParameter {
    pub span: FileSpan,
    pub name: IdentValue,
    pub values: Vec<NumberValue>
}

#[derive(Debug, PartialEq, Clone)]
pub struct Point2Parameter {
    pub span: FileSpan,
    pub name: IdentValue,
    pub values: Vec<(NumberValue, NumberValue)>
}

#[derive(Debug, PartialEq, Clone)]
pub struct Vector2Parameter {
    pub span: FileSpan,
    pub name: IdentValue,
    pub values: Vec<(NumberValue, NumberValue)>
}

#[derive(Debug, PartialEq, Clone)]
pub struct Point3Parameter {
    pub span: FileSpan,
    pub name: IdentValue,
    pub values: Vec<(NumberValue, NumberValue, NumberValue)>
}

#[derive(Debug, PartialEq, Clone)]
pub struct Vector3Parameter {
    pub span: FileSpan,
    pub name: IdentValue,
    pub values: Vec<(NumberValue, NumberValue, NumberValue)>
}

#[derive(Debug, PartialEq, Clone)]
pub struct Normal3Parameter {
    pub span: FileSpan,
    pub name: IdentValue,
    pub values: Vec<(NumberValue, NumberValue, NumberValue)>
}

#[derive(Debug, PartialEq, Clone)]
pub enum SpectrumType {
    RGB,
    XYZ,
    Sampled,
    Blackbody
}

#[derive(Debug, PartialEq, Clone)]
pub struct SpectrumParameter {
    pub span: FileSpan,
    pub name: IdentValue,
    pub spectrum_type: SpectrumType,
    pub values: Vec<Vec<NumberValue>>
}

#[derive(Debug, PartialEq, Clone)]
pub struct SpectrumPathParameter {
    pub span: FileSpan,
    pub name: IdentValue,
    pub path: IdentValue
}

#[derive(Debug, PartialEq, Clone)]
pub struct BooleanParameter {
    pub span: FileSpan,
    pub name: IdentValue,
    pub value: BooleanValue
}

#[derive(Debug, PartialEq, Clone)]
pub struct StringParameter {
    pub span: FileSpan,
    pub name: IdentValue,
    pub value: IdentValue
}

#[derive(Debug, PartialEq, Clone)]
pub struct TextureParameter {
    pub span: FileSpan,
    pub name: IdentValue,
    pub value: IdentValue
}

macro_rules! impl_parameter {
    ($($enumvariant: ident($value: ty),)*) => {
        #[derive(Debug, PartialEq, Clone)]
        pub enum Parameter {
            $($enumvariant($value),)*
        }

        impl Parameter {
            pub fn get_span(&self) -> FileSpan {
                match self {
                    $(Parameter::$enumvariant(v) => v.span.clone(),)*
                }
            }

            pub fn get_name(&self) -> &String {
                match self {
                    $(Parameter::$enumvariant(v) => &v.name.value,)*
                }
            }
        }
    };
}

impl_parameter!{
    Integer(IntegerParameter),
    Float(FloatParameter),
    Point2(Point2Parameter),
    Vector2(Vector2Parameter),
    Point3(Point3Parameter),
    Vector3(Vector3Parameter),
    Normal3(Normal3Parameter),
    Spectrum(SpectrumParameter),
    SpectrumPath(SpectrumPathParameter),
    Boolean(BooleanParameter),
    String(StringParameter),
    Texture(TextureParameter),
}

// #[derive(Debug, PartialEq, Clone)]
// pub struct GeneralDirective {
//     pub span: FileSpan,
//     pub identifier: IdentValue,
//     pub type_name: IdentValue,
//     pub parameters: Vec<Parameter>
// }

#[derive(Debug, PartialEq, Clone)]
pub struct CameraDirective {
    pub span: FileSpan,
    pub type_name: IdentValue,
    pub parameters: Vec<Parameter>
}

#[derive(Debug, PartialEq, Clone)]
pub struct SamplerDirective {
    pub span: FileSpan,
    pub type_name: IdentValue,
    pub parameters: Vec<Parameter>
}

#[derive(Debug, PartialEq, Clone)]
pub struct FilmDirective {
    pub span: FileSpan,
    pub type_name: IdentValue,
    pub parameters: Vec<Parameter>
}

#[derive(Debug, PartialEq, Clone)]
pub struct PixelFilterDirective {
    pub span: FileSpan,
    pub type_name: IdentValue,
    pub parameters: Vec<Parameter>
}

#[derive(Debug, PartialEq, Clone)]
pub struct IntegratorDirective {
    pub span: FileSpan,
    pub type_name: IdentValue,
    pub parameters: Vec<Parameter>
}

#[derive(Debug, PartialEq, Clone)]
pub struct AcceleratorDirective {
    pub span: FileSpan,
    pub type_name: IdentValue,
    pub parameters: Vec<Parameter>
}

#[derive(Debug, PartialEq, Clone)]
pub struct ShapeDirective {
    pub span: FileSpan,
    pub type_name: IdentValue,
    pub parameters: Vec<Parameter>
}

#[derive(Debug, PartialEq, Clone)]
pub struct ObjectInstanceDirective {
    pub span: FileSpan,
    pub type_name: IdentValue
}

#[derive(Debug, PartialEq, Clone)]
pub struct LightSourceDirective {
    pub span: FileSpan,
    pub type_name: IdentValue,
    pub parameters: Vec<Parameter>
}

#[derive(Debug, PartialEq, Clone)]
pub struct AreaLightSourceDirective {
    pub span: FileSpan,
    pub type_name: IdentValue,
    pub parameters: Vec<Parameter>
}

#[derive(Debug, PartialEq, Clone)]
pub struct MaterialDirective {
    pub span: FileSpan,
    pub type_name: IdentValue,
    pub parameters: Vec<Parameter>
}

#[derive(Debug, PartialEq, Clone)]
pub struct TextureDirective {
    pub span: FileSpan,
    pub name: IdentValue,
    pub type_name: IdentValue,
    pub class: IdentValue,
    pub parameters: Vec<Parameter>
}

#[derive(Debug, PartialEq, Clone)]
pub struct NamedMediumDirective {
    pub span: FileSpan,
    pub type_name: IdentValue,
    pub parameters: Vec<Parameter>
}

#[derive(Debug, PartialEq, Clone)]
pub struct MediumInterfaceDirective {
    pub span: FileSpan,
    pub interior: IdentValue,
    pub exterior: IdentValue
}

// #[derive(Debug, PartialEq, Clone)]
// pub struct IncludeDirective {
//     pub span: FileSpan,
//     pub path: IdentValue
// }

#[derive(Debug, PartialEq, Clone)]
pub struct WorldBlock {
    pub span: FileSpan,
    pub children: Vec<Directive>
}

#[derive(Debug, PartialEq, Clone)]
pub struct AttributeBlock {
    pub span: FileSpan,
    pub children: Vec<Directive>
}

#[derive(Debug, PartialEq, Clone)]
pub struct TransformBlock {
    pub span: FileSpan,
    pub children: Vec<Directive>
}

#[derive(Debug, PartialEq, Clone)]
pub struct ObjectBlock {
    pub span: FileSpan,
    pub name: IdentValue,
    pub children: Vec<ShapeDirective>
}

#[derive(Debug, PartialEq, Clone)]
pub enum Directive {
    // Include(IncludeDirective),
    Camera(CameraDirective),
    Sampler(SamplerDirective),
    Film(FilmDirective),
    Filter(PixelFilterDirective),
    Integrator(IntegratorDirective),
    Accelerator(AcceleratorDirective),
    Shape(ShapeDirective),
    ObjectInstace(ObjectInstanceDirective),
    LightSource(LightSourceDirective),
    AreaLightSource(AreaLightSourceDirective),
    Material(MaterialDirective),
    Texture(TextureDirective),
    NamedMedium(NamedMediumDirective),
    MediumInterface(MediumInterfaceDirective),
    Transformation(Transformation),
    TransformTimes(TransformTimes),
    Attribute(AttributeBlock),
    Transform(TransformBlock),
    World(WorldBlock),
    Object(ObjectBlock)
}

impl Directive {
    pub fn get_span(&self) -> FileSpan {
        match self {
            // Directive::Include(v) => v.span.clone(),
            Directive::Camera(v) => v.span.clone(),
            Directive::Sampler(v) => v.span.clone(),
            Directive::Film(v) => v.span.clone(),
            Directive::Filter(v) => v.span.clone(),
            Directive::Integrator(v) => v.span.clone(),
            Directive::Accelerator(v) => v.span.clone(),
            Directive::Shape(v) => v.span.clone(),
            Directive::ObjectInstace(v) => v.span.clone(),
            Directive::LightSource(v) => v.span.clone(),
            Directive::AreaLightSource(v) => v.span.clone(),
            Directive::Material(v) => v.span.clone(),
            Directive::Texture(v) => v.span.clone(),
            Directive::NamedMedium(v) => v.span.clone(),
            Directive::MediumInterface(v) => v.span.clone(),
            Directive::Transformation(v) => v.get_span(),
            Directive::TransformTimes(v) => v.span.clone(),
            Directive::Attribute(v) => v.span.clone(),
            Directive::Transform(v) => v.span.clone(),
            Directive::World(v) => v.span.clone(),
            Directive::Object(v) => v.span.clone(),
        }
    }
}