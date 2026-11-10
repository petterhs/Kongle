//! InfiniTime-compatible media state and events, without hardware dependencies.
use heapless::String;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Command {
    Play = 0x00,
    Pause = 0x01,
    Next = 0x03,
    Previous = 0x04,
    Open = 0xe0,
}

#[derive(Clone, Copy)]
pub struct Request {
    pub session: u32,
    pub command: Command,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct State {
    pub session: u32,
    pub playing: bool,
    pub title: String<40>,
    pub artist: String<40>,
    pub album: String<40>,
    pub position: u32,
    pub duration: u32,
    pub track: u32,
    pub tracks: u32,
    pub speed: u32,
    pub repeat: u8,
    pub shuffle: u8,
}
impl State {
    pub const fn new() -> Self {
        Self {
            session: 0,
            playing: false,
            title: String::new(),
            artist: String::new(),
            album: String::new(),
            position: 0,
            duration: 0,
            track: 0,
            tracks: 0,
            speed: 100,
            repeat: 0,
            shuffle: 0,
        }
    }
    pub fn available(&self) -> bool {
        self.session != 0 && (self.playing || !self.title.is_empty() || !self.artist.is_empty())
    }
    pub fn apply(&mut self, field: u8, data: &[u8]) -> Result<(), ()> {
        match field {
            2 | 11 | 12 => {
                if data.len() != 1 || data[0] > 1 {
                    return Err(());
                }
                match field {
                    2 => self.playing = data[0] != 0,
                    11 => self.repeat = data[0],
                    _ => self.shuffle = data[0],
                }
            }
            3..=5 => {
                if data.len() > 40 || core::str::from_utf8(data).is_err() {
                    return Err(());
                }
                let text = crate::notifications::display_text(data);
                match field {
                    3 => self.artist = text,
                    4 => self.title = text,
                    _ => self.album = text,
                }
            }
            6..=10 => {
                let value = u32::from_be_bytes(data.try_into().map_err(|_| ())?);
                match field {
                    6 => self.position = value,
                    7 => self.duration = value,
                    8 => self.track = value,
                    9 => self.tracks = value,
                    _ => self.speed = value,
                }
            }
            _ => return Err(()),
        }
        Ok(())
    }
}
