use base64::{Engine as _, engine::general_purpose};
use chrono::prelude::*;
use crate::InputChanges;
use protocol::{device::ControllerButtons};
use std::time::Instant;
use std::io::{self, Write, BufRead};


#[derive(Debug)]
pub enum ReplayError {
   InvalidCodePath, 
   IoError(io::Error),
   ParseError(String),
   MaxTimeExceeded,
}


impl From<io::Error> for ReplayError {
    fn from(err: io::Error) -> Self {
        ReplayError::IoError(err)
    }
}

#[derive(Debug, PartialEq)]
pub enum Recorder {
    Off,
    Waiting(Instant),
    Recording{ start: Instant, events: Vec<(u32, InputChanges)> }
}

impl Recorder {
    pub fn new() -> Self {
        Self::Off
    }

    pub fn toggle(&mut self) -> Result<(), ReplayError> {
        match self {
            Self::Off => {
                *self = Self::Waiting(Instant::now());
            },
            Self::Waiting(_) => {
                *self = Self::Off;
            },
            Self::Recording{start: _, events} => {
               Self::write_events(events)?;
               *self = Self::Off;
            },
        }
        Ok(())
    }

    pub fn take_event(&mut self, changes: &InputChanges) -> Result<(), ReplayError> {
        match self {
            Self::Off => {},
            Self::Waiting(ref start) | Self::Recording{ref start, ..} => {
                if !changes.changes() {
                    return Ok(());
                }

                let elapsed = start.elapsed().as_millis();

                if elapsed > u32::MAX as u128 {
                    *self = Self::Off;
                    log::error!("recording exceeded maximum time (~1193 hours).");
                    return Err(ReplayError::MaxTimeExceeded);
                }

                match self {
                    Self::Waiting(_) => {
                        *self = Self::Recording{start: *start, events: vec![(elapsed as u32, *changes)]};
                    }
                    Self::Recording{events, ..} => {
                        events.push((elapsed as u32, *changes));
                    }
                    _ => {
                        log::error!("invalid state in take_event reached, this is a bug.");
                        return Err(ReplayError::InvalidCodePath);
                    },
                }
            },
        }
        Ok(())
    }


    fn write_events(events: &[(u32, InputChanges)]) -> Result<(), ReplayError> {
        let filename = Local::now().format("%Y-%m-%d_%H:%M:%S.replay").to_string();

        let mut file = std::fs::File::create(filename)?;

        for (timestamp, changes) in events {
            write!(file, "{timestamp},{},{}", changes.pressed.bits(), changes.released.bits()).unwrap();
            if let Some(axes) = changes.axes {
                write!(file, ",{},{},{},{}\n", axes[0], axes[1], axes[2], axes[3])
            } else {
                write!(file, "\n")
            }.unwrap();
        }
        Ok(())
    }
}

#[derive(Debug)]
pub enum Player {
    Off(Vec<(u32, InputChanges)>),
    Playing{start: Instant, events: Vec<(u32, InputChanges)>}
}

impl Player {
    pub fn from_file(filename: &str) -> Result<Self, ReplayError> {
        let file = std::fs::File::open(filename)?;

        let mut reader = std::io::BufReader::new(file);

        let mut events = Vec::new();

        let mut last_time = 0;
        let mut line = 1;
        loop {
            let mut changes = InputChanges::NOCHANGE;
            let mut string = String::new();
            if reader.read_line(&mut string).is_err() { break; }
            let things = string.trim().split(',').collect::<Vec<_>>();
            let length = things.len();
            if string.is_empty() {
                break;
            }

            if length != 3 && length != 7 {
                return Err(ReplayError::ParseError(format!("line {line} has an invalid number of items: {length}")));
            }

            let time: u32 = things[0].parse().unwrap();
            if time < last_time {
                return Err(ReplayError::ParseError(format!("line {line} went backwards in time ({time}ms < {last_time}ms)")));
            }
            //add error
            changes.pressed = ControllerButtons::from_bits(things[1].parse::<u16>().unwrap()).unwrap();
            changes.released = ControllerButtons::from_bits(things[2].parse::<u16>().unwrap()).unwrap();

            if length == 7 {
            //add error
                let lx: i8 = things[3].parse().unwrap();
                let ly: i8 = things[4].parse().unwrap();
                let rx: i8 = things[5].parse().unwrap();
                let ry: i8 = things[6].parse().unwrap();
                changes.axes = Some([lx, ly, rx, ry]);
            }
            last_time = time;
            events.push((time, changes));
            line += 1;
        }
        Ok(Self::Off(events))
    }

    pub fn play(self) -> Self {
        match self {
            Self::Off(events) => {
                Self::Playing{start: Instant::now(), events }
            }
            Self::Playing{..} => {
                log::warn!("Player::play was called while in state Playing.");
                self
            }
        }
    }
    pub fn get_events(&mut self) -> Vec<(u32, InputChanges)> {
        match self {
            Self::Off(_) => {
                log::warn!("Player::get_events was called while in state Off.");
                return vec![];
            }
            Self::Playing{start, events} => {
                let elapsed = start.elapsed().as_millis();
                if elapsed > u32::MAX as u128 {
                    log::error!("recording exceeded maximum time (~1193 hours), resetting.");
                }
                let elapsed = elapsed as u32;

                let index = events.partition_point(|x| x.0 < elapsed);

                let mut passed = events.split_off(index);

                std::mem::swap(events, &mut passed);

                passed
            }
        }
    }

    pub fn is_playing(&self) -> bool {
        if let Self::Playing{..} = self {
            return true;
        }
        false
    }
}
