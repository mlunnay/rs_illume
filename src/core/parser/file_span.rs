use std::iter::Enumerate;
use std::rc::Rc;
pub use std::ops::{Range, RangeFrom, RangeFull, RangeTo};
use nom::ExtendInto;
use nom::{
    error::{ErrorKind, ParseError},
    AsBytes, Compare, CompareResult, Err, FindSubstring, FindToken, IResult, InputIter,
    InputLength, InputTake, InputTakeAtPosition, Offset, ParseTo, Slice,
};
use memchr::Memchr;
use std::str::FromStr;

#[derive(Default, Clone)]
pub struct FileSpan {
    pub data: Rc<String>,
    pub filename: Rc<String>,
    pub offset: usize,
    pub length: usize,
    pub line: usize
}

impl FileSpan {
    pub fn new(data: Rc<String>, filename: Rc<String>) -> FileSpan {
        FileSpan {
            data: data.clone(),
            filename: filename.clone(),
            offset: 0,
            length: data.len(),
            line: 1
        }
    }

    pub fn as_str(&self) -> &str {
        self
    }

    /// Returns the column number that this span starts at.
    pub fn get_column(&self) -> usize {
        match self.data[..self.offset].rfind('\n') {
            Some(pos) => self.offset - pos,
            None => self.offset + 1
        }
    }

    /// Returns the column number that this span starts at for UTF8 text.
    pub fn get_utf8_column(&self) -> usize {
        let before = match self.data[..self.offset].rfind('\n') {
            Some(pos) => &self.data[pos + 1..self.offset],
            None => &self.data[..self.offset]
        };
        before.as_bytes().iter().filter(|&&byte| (byte >> 6) != 0b10).count() + 1
    }

    pub fn merge(&self, other: &FileSpan) -> FileSpan {
        FileSpan {
            data: self.data.clone(),
            filename: self.filename.clone(),
            offset: self.offset,
            length: other.offset - self.offset + other.length,
            line: self.line
        }
    }
}

impl std::fmt::Debug for FileSpan {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "FileSpan {{ data: {:?}, filename: {:?}, offset: {}, length: {}, line: {} }}",
            &self.data[self.offset..self.offset + self.length], self.filename, self.offset, self.length, self.line)
    }
}

impl PartialEq for FileSpan {
    fn eq(&self, other: &Self) -> bool {
        self.filename == other.filename && self.data == other.data && self.offset == other.offset && self.length == other.length
    }
}

impl Eq for FileSpan {}

impl std::ops::Deref for FileSpan {
    type Target = str;

    #[inline]
    fn deref(&self) -> &str {
        &self.data[self.offset..self.offset + self.length]
    }
}

impl AsBytes for FileSpan {
    fn as_bytes(&self) -> &[u8] {
        self.data[self.offset..self.offset + self.length].as_bytes()
    }
}

impl InputLength for FileSpan {
    fn input_len(&self) -> usize {
        self.length
    }
}

impl InputTake for FileSpan {
    fn take(&self, count: usize) -> Self {
        self.slice(..count)
    }

    fn take_split(&self, count: usize) -> (Self, Self) {
        (self.slice(count..), self.slice(..count))
    }
}

impl InputTakeAtPosition for FileSpan {
    type Item = char;

    fn split_at_position<P, E: ParseError<Self>>(
        &self,
        predicate: P
    ) -> IResult<Self, Self, E> 
    where
    P: Fn(Self::Item) -> bool
    {
        match (&self.data[self.offset..self.offset + self.length]).position(predicate) {
            Some(n) => Ok(self.take_split(n)),
            None => Err(Err::Incomplete(nom::Needed::Size(1)))
        }
    }

    fn split_at_position1<P, E: ParseError<Self>>(
        &self,
        predicate: P,
        e: ErrorKind
    ) -> IResult<Self, Self, E>
    where
    P: Fn(Self::Item) -> bool
    {
        match (&self.data[self.offset..self.offset + self.length]).position(predicate) {
            Some(0) => Err(Err::Error(E::from_error_kind(self.clone(), e))),
            Some(n) => Ok(self.take_split(n)),
            None => Err(Err::Incomplete(nom::Needed::Size(1))),
        }
    }

    fn split_at_position_complete<P, E: ParseError<Self>>(
        &self,
        predicate: P,
    ) -> IResult<Self, Self, E>
    where
        P: Fn(Self::Item) -> bool,
    {
        match self.split_at_position(predicate) {
            Err(Err::Incomplete(_)) => Ok(self.take_split(self.input_len())),
            res => res,
        }
    }

    fn split_at_position1_complete<P, E: ParseError<Self>>(
        &self,
        predicate: P,
        e: ErrorKind,
    ) -> IResult<Self, Self, E>
    where
        P: Fn(Self::Item) -> bool,
    {
        match (&self.data[self.offset..self.offset + self.length]).position(predicate) {
            Some(0) => Err(Err::Error(E::from_error_kind(self.clone(), e))),
            Some(n) => Ok(self.take_split(n)),
            None => {
                if self.length == 0 {
                    Err(Err::Error(E::from_error_kind(self.clone(), e)))
                } else {
                    Ok(self.take_split(self.input_len()))
                }
            }
        }
    }
}

impl InputIter for FileSpan {
    type Item = char;
    type Iter = Enumerate<Self::IterElem>;
    type IterElem = FileSpanIterator;

    #[inline]
    fn iter_indices(&self) -> Self::Iter {
        self.iter_elements().enumerate()
    }
    #[inline]
    fn iter_elements(&self) -> Self::IterElem {
        FileSpanIterator{
            data: self.data.clone(),
            current: self.offset,
            end: self.offset + self.length
        }
    }
    #[inline]
    fn position<P>(&self, predicate: P) -> Option<usize>
    where
        P: Fn(Self::Item) -> bool,
    {
        (&self.data[self.offset..self.offset + self.length]).position(predicate)
    }
    #[inline]
    fn slice_index(&self, count: usize) -> Option<usize> {
        (&self.data[self.offset..self.offset + self.length]).slice_index(count)
    }
}

impl Compare<FileSpan> for FileSpan {
    #[inline(always)]
    fn compare(&self, t: FileSpan) -> CompareResult {
        (&self.data[self.offset..self.offset + self.length]).compare(&*t)
    }

    #[inline(always)]
    fn compare_no_case(&self, t: FileSpan) -> CompareResult {
        (&self.data[self.offset..self.offset + self.length]).compare_no_case(&*t)
    }
}

impl<'a> Compare<&'a str> for FileSpan {
    #[inline(always)]
    fn compare(&self, t: &'a str) -> CompareResult {
        (&self.data[self.offset..self.offset + self.length]).compare(t)
    }

    #[inline(always)]
    fn compare_no_case(&self, t: &'a str) -> CompareResult {
        (&self.data[self.offset..self.offset + self.length]).compare_no_case(t)
    }
}

macro_rules! impl_slice_range {
    ( $range_type:ty, $can_return_self:expr ) => {
        impl Slice<$range_type> for FileSpan 
        // where
        // str: Slice<RangeTo<usize>> + Slice<Range<usize>> + Slice<RangeFrom<usize>> + Slice<RangeFull>
        {
            fn slice(&self, range: $range_type) -> Self {
                if $can_return_self(&range) {
                    return self.clone();
                }
                let next_fragment = (&self.data[self.offset..self.offset + self.length]).slice(range);
                let consumed_len = (&self.data[self.offset..self.offset + self.length]).offset(&next_fragment);
                if consumed_len == 0 {
                    return FileSpan {
                        data: self.data.clone(),
                        filename: self.filename.clone(),
                        line: self.line,
                        offset: self.offset,
                        length: next_fragment.len()
                    };
                }

                let consumed = (&self.data[self.offset..self.offset + self.length]).slice(..consumed_len);
                let next_offset = self.offset + consumed_len;

                let consumed_as_bytes = consumed.as_bytes();
                let iter = Memchr::new(b'\n', consumed_as_bytes);
                let number_of_lines = iter.count();
                let next_line = self.line + number_of_lines;

                FileSpan {
                    data: self.data.clone(),
                    filename: self.filename.clone(),
                    line: next_line,
                    offset: next_offset,
                    length: next_fragment.len()
                }
            }
        }
    };
}

impl_slice_range!(Range<usize>, |_| false);
impl_slice_range!(RangeTo<usize>, |_| false);
impl_slice_range!(RangeFrom<usize>, |range:&RangeFrom<usize>| range.start == 0);
impl_slice_range!(RangeFull, |_| false);

impl FindToken<u8> for FileSpan {
    fn find_token(&self, token: u8) -> bool {
        (&self.data[self.offset..self.offset + self.length]).find_token(token)
    }
}

impl<'a> FindToken<&'a u8> for FileSpan {
    fn find_token(&self, token: &'a u8) -> bool {
        (&self.data[self.offset..self.offset + self.length]).find_token(token)
    }
}

impl FindToken<char> for FileSpan {
    fn find_token(&self, token: char) -> bool {
        (&self.data[self.offset..self.offset + self.length]).find_token(token)
    }
}

impl<'a> FindSubstring<&'a str> for FileSpan {
    #[inline]
    fn find_substring(&self, substr: &'a str) -> Option<usize> {
        (&self.data[self.offset..self.offset + self.length]).find_substring(substr)
    }
}

impl<R: FromStr> ParseTo<R> for FileSpan {
    #[inline]
    fn parse_to(&self) -> Option<R> {
        (&self.data[self.offset..self.offset + self.length]).parse_to()
    }
}

impl Offset for FileSpan {
    fn offset(&self, second: &Self) -> usize {
        second.offset - self.offset
    }
}

impl ExtendInto for FileSpan {
    type Item = char;
    type Extender = String;

    #[inline]
    fn new_builder(&self) -> Self::Extender {
        (&self.data[self.offset..self.offset + self.length]).new_builder()
    }

    #[inline]
    fn extend_into(&self, acc: &mut Self::Extender) {
        (&self.data[self.offset..self.offset + self.length]).extend_into(acc)
    }
}

impl nom::HexDisplay for FileSpan {
    fn to_hex(&self, chunk_size: usize) -> String {
        (&self.data[self.offset..self.offset + self.length]).to_hex(chunk_size)
    }

    fn to_hex_from(&self, chunk_size: usize, from: usize) -> String {
        (&self.data[self.offset..self.offset + self.length]).to_hex_from(chunk_size, from)
    }
}

/// Character iterator over a FileSpan.
pub struct FileSpanIterator {
    data: Rc<String>,
    current: usize,
    end: usize
}

impl Iterator for FileSpanIterator {
    type Item = char;

    fn next(&mut self) -> Option<char> {
        if self.current == self.end {
            return None
        }

        let res = self.data.chars().nth(self.current);
        self.current += 1;
        res
    }
}

/// Capture the position of the current fragment

#[macro_export]
macro_rules! position {
    ($input:expr,) => {
        tag!($input, "")
    };
}

/// Capture the position of the current fragment
pub fn position<T, E>(s: T) -> IResult<T, T, E>
where
    E: ParseError<T>,
    T: InputIter + InputTake,
{
    nom::bytes::complete::take(0usize)(s)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn filespan_should_slice() {
        let span = FileSpan::new(Rc::new(String::from("testing")), Rc::new(String::from("input")));
        assert_eq!(span.slice(..4), FileSpan{
            data: Rc::new(String::from("testing")),
            filename: Rc::new(String::from("input")),
            length: 4,
            offset: 0,
            line: 1
        });
        assert_eq!(span.slice(1..4), FileSpan{
            data: Rc::new(String::from("testing")),
            filename: Rc::new(String::from("input")),
            length: 3,
            offset: 1,
            line: 1
        });
        assert_eq!(span.slice(4..), FileSpan{
            data: Rc::new(String::from("testing")),
            filename: Rc::new(String::from("input")),
            length: 3,
            offset: 4,
            line: 1
        });
        assert_eq!(span.slice(..), FileSpan{
            data: Rc::new(String::from("testing")),
            filename: Rc::new(String::from("input")),
            length: 7,
            offset: 0,
            line: 1
        });
    }

    #[test]
    fn filespan_should_deref_to_str() {
        let span = FileSpan::new(Rc::new(String::from("testing")), Rc::new(String::from("input")));
        assert_eq!(&*span, "testing");
        assert_eq!(&*span.slice(..4), "test");
    }

    #[test]
    fn filespan_should_calulate_columns() {
        let span = FileSpan::new(Rc::new(String::from("foo
        bar")), Rc::new(String::from("input")));
        let index = span.find_substring("bar").unwrap();
        assert_eq!(span.slice(index..).get_column(), 9);
        assert_eq!(span.slice(index..).get_utf8_column(), 9);
    }

    #[test]
    fn filespan_should_calulate_utf8_columns() {
        let span = FileSpan::new(Rc::new(String::from("メカジキ")), Rc::new(String::from("input")));
        assert_eq!(span.slice(6..).get_utf8_column(), 3);

        // also check it works for non utf8 input
        let span = FileSpan::new(Rc::new(String::from("foo
        bar")), Rc::new(String::from("input")));
        let index = span.find_substring("bar").unwrap();
        assert_eq!(span.slice(index..).get_utf8_column(), 9);
    }

    #[test]
    fn filespan_should_iterate_indices() {
        let span = FileSpan::new(Rc::new(String::from("testing")), Rc::new(String::from("input")));
        assert_eq!((&span).iter_indices().collect::<Vec<(usize, char)>>(),
        vec![(0, 't'), (1, 'e'), (2, 's'), (3, 't'), (4, 'i'), (5, 'n'), (6, 'g')]);
    }

    #[test]
    fn filespan_should_iterate_elements() {
        let span = FileSpan::new(Rc::new(String::from("testing")), Rc::new(String::from("input")));
        assert_eq!((&span).iter_elements().collect::<Vec<char>>(),
        vec!['t', 'e', 's', 't', 'i', 'n', 'g']);
    }

    #[test]
    fn filespan_should_get_position_of_char() {
        let span = FileSpan::new(Rc::new(String::from("testing")), Rc::new(String::from("input")));
        assert_eq!(span.position(|c| c == 'e'), Some(1));
        assert_eq!(span.position(|c| c == 'a'), None);
        // test that position on a slice gives the correct position
        assert_eq!(span.slice(4..).position(|c| c == 'n'), Some(1));
    }

    #[test]
    fn filespan_should_compare() {
        let span = FileSpan::new(Rc::new(String::from("testing")), Rc::new(String::from("input")));
        assert_eq!(span.compare("test"), CompareResult::Ok);
        assert_eq!(span.compare("testing"), CompareResult::Ok);
        assert_eq!(span.compare("foo"), CompareResult::Error);
        assert_eq!(span.compare("testing123"), CompareResult::Incomplete);
        assert_eq!(span.compare_no_case("Testing"), CompareResult::Ok);
    }

    #[test]
    fn filespan_should_find_token() {
        let span = FileSpan::new(Rc::new(String::from("testing")), Rc::new(String::from("input")));
        assert!(span.find_token(b't'));
        assert!(span.find_token('t'));
        assert!(span.find_token(&(b't')));
        assert!(!span.find_token(b'a'));
        assert!(!span.find_token('a'));
        assert!(!span.find_token(&(b'a')));
        assert!(span.slice(4..).find_token(b'i'));
        assert!(span.slice(4..).find_token('i'));
        assert!(span.slice(4..).find_token(&(b'i')));
        assert!(!span.slice(4..).find_token(b't'));
        assert!(!span.slice(4..).find_token('t'));
        assert!(!span.slice(4..).find_token(&(b't')));
    }

    #[test]
    fn filespan_should_find_substring() {
        let span = FileSpan::new(Rc::new(String::from("testing")), Rc::new(String::from("input")));
        assert_eq!(span.find_substring("ing"), Some(4));
        assert_eq!(span.find_substring("foo"), None);
    }

    #[test]
    fn filespan_should_parse_to_string() {
        let span = FileSpan::new(Rc::new(String::from("testing")), Rc::new(String::from("input")));
        assert_eq!(span.parse_to(), Some("testing".to_string()));
    }

    #[test]
    fn filespan_should_calculate_offset() {
        let span = FileSpan::new(Rc::new(String::from("testing")), Rc::new(String::from("input")));
        let slice = span.slice(4..);
        assert_eq!(span.offset(&slice), 4);
    }

    #[test]
    fn filespan_should_take_split() {
        let span = FileSpan::new(Rc::new(String::from("testing")), Rc::new(String::from("input")));
        let (first, second) = span.take_split(4);
        assert_eq!(first.offset, 4);
        assert_eq!(first.length, 3);
        assert_eq!(&*first, "ing");
        assert_eq!(second.offset, 0);
        assert_eq!(second.length, 4);
        assert_eq!(&*second, "test");
    }

    type TestError = (FileSpan, ErrorKind);

    #[test]
    fn filespan_should_split_at_position() {
        let span = FileSpan::new(Rc::new(String::from("testing")), Rc::new(String::from("input")));
        assert_eq!(span.split_at_position::<_, TestError>(|c| c == 'i'),
            Ok((
                FileSpan{
                    data: Rc::new(String::from("testing")),
                    filename: Rc::new(String::from("input")),
                    offset: 4,
                    length: 3,
                    line: 1
                },
                FileSpan{
                    data: Rc::new(String::from("testing")),
                    filename: Rc::new(String::from("input")),
                    offset: 0,
                    length: 4,
                    line: 1
                }
            ))
        );

        assert_eq!(span.split_at_position::<_, TestError>(|c| c == 'a'),
            Err(Err::Incomplete(nom::Needed::Size(1))));
    }

    #[test]
    fn filespan_should_slit_at_position1() {
        let span = FileSpan::new(Rc::new(String::from("testing")), Rc::new(String::from("input")));
        assert_eq!(
            span.split_at_position1::<_, TestError>(|c| { c == 'i' }, ErrorKind::Alpha),
            span.split_at_position::<_, TestError>(|c| { c == 'i' }),
        );
    }

    #[test]
    fn should_capture_position() {
        use super::position;
        use nom::bytes::complete::{tag, take_until};
        use nom::IResult;

        fn parser<'a>(s: FileSpan) -> IResult<FileSpan, (FileSpan, String)> {
            let (s, _) = take_until("def")(s)?;
            let (s, p) = position(s)?;
            let (s, t) = tag("def")(s)?;
            Ok((s, (p, t.parse_to().unwrap())))
        }

        let s = FileSpan::new(Rc::new(String::from("abc\ndefghij")), Rc::new(String::from("input")));
        let (_, (s, t)) = parser(s).unwrap();
        assert_eq!(s.offset, 4);
        assert_eq!(s.line, 2);
        assert_eq!(t, "def");
    }
}