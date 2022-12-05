use defmt::Format;
use heapless::String;

#[derive(Debug, PartialEq, Format)]
pub enum UnescapeError<const N: usize> {
    Overflow(String<N>),
    UnexpectedEnd(String<N>),
    BadEscape(char),
    BadHexUnicode,
}

pub fn unescape<const N: usize>(s: &str) -> Result<String<N>, UnescapeError<N>> {
    let mut result = String::new();
    let mut state = State::Text;
    for c in s.chars() {
        let r = match state {
            State::Text => {
                if c == '\\' {
                    Ok(State::Escape)
                } else {
                    result.push(c).map(|_| State::Text)
                }
            }
            State::Escape => match c {
                '"' => result.push('"').map(|_| State::Text),
                '\\' => result.push('\\').map(|_| State::Text),
                '/' => result.push('/').map(|_| State::Text),
                'b' => result.push('\x08').map(|_| State::Text),
                'f' => result.push('\x0c').map(|_| State::Text),
                'n' => result.push('\n').map(|_| State::Text),
                'r' => result.push('\r').map(|_| State::Text),
                't' => result.push('\t').map(|_| State::Text),
                'u' => Ok(State::HexDigit(0, 0)),
                _ => {
                    return Err(UnescapeError::BadEscape(c));
                }
            },
            State::HexDigit(n, v) => {
                let new_state = if c.is_digit(16) {
                    State::HexDigit(n + 1, v * 16 + c.to_digit(16).unwrap())
                } else {
                    return Err(UnescapeError::BadHexUnicode);
                };
                match new_state {
                    State::HexDigit(4, v) if v >= 0xd800 && v <= 0xdbff => {
                        Ok(State::SurrogateEscape(v))
                    }
                    State::HexDigit(4, v) => {
                        if let Some(uc) = char::from_u32(v) {
                            result.push(uc).map(|_| State::Text)
                        } else {
                            return Err(UnescapeError::BadHexUnicode);
                        }
                    }
                    State::HexDigit(_, _) => Ok(new_state),
                    _ => unreachable!(),
                }
            }
            State::SurrogateEscape(v) => {
                if c == '\\' {
                    Ok(State::SurrogateU(v))
                } else {
                    return Err(UnescapeError::BadHexUnicode);
                }
            }
            State::SurrogateU(v) => {
                if c == 'u' {
                    Ok(State::SurrogateHexDigit(v, 0, 0))
                } else {
                    return Err(UnescapeError::BadHexUnicode);
                }
            }
            State::SurrogateHexDigit(hi, n, v) => {
                let new_state = if c.is_digit(16) {
                    State::SurrogateHexDigit(hi, n + 1, v * 16 + c.to_digit(16).unwrap())
                } else {
                    return Err(UnescapeError::BadHexUnicode);
                };
                match new_state {
                    State::SurrogateHexDigit(hi, 4, lo) if lo >= 0xdc00 && lo <= 0xdfff => {
                        let codepoint = ((hi - 0xd800) << 10) + (lo - 0xdc00) + 0x10000;
                        if let Some(uc) = char::from_u32(codepoint) {
                            result.push(uc).map(|_| State::Text)
                        } else {
                            return Err(UnescapeError::BadHexUnicode);
                        }
                    }
                    State::SurrogateHexDigit(_, 4, _) => return Err(UnescapeError::BadHexUnicode),
                    State::SurrogateHexDigit(_, _, _) => Ok(new_state),
                    _ => unreachable!(),
                }
            }
        };
        state = match r {
            Ok(s) => s,
            Err(()) => return Err(UnescapeError::Overflow(result)),
        };
    }

    if let State::Text = state {
        Ok(result)
    } else {
        return Err(UnescapeError::UnexpectedEnd(result));
    }
}

#[derive(Copy, Clone)]
enum State {
    Text,
    Escape,
    HexDigit(u16, u32),
    SurrogateEscape(u32), // expecting backslash after first utf-16 value
    SurrogateU(u32),      // expecting u
    SurrogateHexDigit(u32, u16, u32),
}

#[cfg(test)]
mod tests {
    use crate::unescape::UnescapeError;

    use super::unescape;

    #[test]
    fn test_unescape() {
        assert!(matches!(unescape::<16>("hello"), Ok(s) if s == "hello"));
        assert!(matches!(unescape::<16>("hel\\nlo"), Ok(s) if s == "hel\nlo"));
        assert!(matches!(unescape::<16>("hel\\\"lo"), Ok(s) if s == "hel\"lo"));
        assert!(matches!(unescape::<16>("hel\\u1234lo"), Ok(s) if s == "hel\u{1234}lo"));
        assert_eq!(
            unescape::<16>("hell\\o w"),
            Err(UnescapeError::BadEscape('o'))
        );
    }
}
