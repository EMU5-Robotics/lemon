use crate::InputChanges;
use chrono::prelude::*;
use protocol::device::ControllerButtons;
use std::io::{self, BufRead, Write};
use std::{fmt, time::Instant};

#[derive(Debug)]
pub enum ReplayError {
	InvalidCodePath,
	IoError(io::Error),
	ParseError(String),
	MaxTimeExceeded,
}

impl fmt::Display for ReplayError {
	fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
		match self {
			Self::InvalidCodePath => write!(f, "Invalid Code Path"),
			Self::IoError(e) => write!(f, "{e}"),
			Self::ParseError(e) => write!(f, "{e}"),
			Self::MaxTimeExceeded => write!(f, "Max Time Exceeded"),
		}
	}
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
	Recording {
		last: Instant,
		events: Vec<(u32, InputChanges)>,
	},
}

impl Recorder {
	pub fn new() -> Self {
		Self::Off
	}

	pub fn toggle(&mut self) -> Result<(), ReplayError> {
		match self {
			Self::Off => {
				*self = Self::Waiting(Instant::now());
			}
			Self::Waiting(_) => {
				*self = Self::Off;
			}
			Self::Recording { last: _, events } => {
				Self::write_events(events)?;
				*self = Self::Off;
			}
		}
		Ok(())
	}

	pub fn take_event(&mut self, changes: &InputChanges) -> Result<(), ReplayError> {
		match self {
			Self::Off => {}
			Self::Waiting(ref last) | Self::Recording { ref last, .. } => {
				if !changes.changes() {
					return Ok(());
				}

				let elapsed = last.elapsed().as_micros() / 100;

				if elapsed > u32::MAX as u128 {
					*self = Self::Off;
					log::error!("recording exceeded maximum time between events (~119 hours).");
					return Err(ReplayError::MaxTimeExceeded);
				}

				match self {
					Self::Waiting(_) => {
						*self = Self::Recording {
							last: Instant::now(),
							events: vec![(elapsed as u32, *changes)],
						};
					}
					Self::Recording { events, last, .. } => {
						*last = Instant::now();
						events.push((elapsed as u32, *changes));
					}
					_ => {
						log::error!("invalid state in take_event reached, this is a bug.");
						return Err(ReplayError::InvalidCodePath);
					}
				}
			}
		}
		Ok(())
	}

	fn write_events(events: &[(u32, InputChanges)]) -> Result<(), ReplayError> {
		let filename = Local::now().format("%Y-%m-%d_%H:%M:%S.replay").to_string();

		let mut file = std::fs::File::create(filename)?;

		for (diff, changes) in events {
			write!(
				file,
				"{diff},{},{}",
				changes.pressed.bits(),
				changes.released.bits()
			)?;
			if let Some(axes) = changes.axes {
				writeln!(file, ",{},{},{},{}", axes[0], axes[1], axes[2], axes[3])
			} else {
				writeln!(file)
			}?;
		}
		Ok(())
	}
}

#[derive(Debug)]
pub enum Player {
	Off(Vec<(u32, InputChanges)>),
	Playing {
		last: Instant,
		events: Vec<(u32, InputChanges)>,
		cursor: usize,
	},
}

impl Default for Player {
	fn default() -> Self {
		Player::Off(Vec::new())
	}
}

impl Player {
	pub fn from_file(filename: &str) -> Result<Self, ReplayError> {
		let file = std::fs::File::open(filename)?;

		let mut reader = std::io::BufReader::new(file);

		let mut events = Vec::new();

		let mut line = 1;
		loop {
			let mut changes = InputChanges::NOCHANGE;
			let mut string = String::new();
			if reader.read_line(&mut string).is_err() {
				break;
			}
			let things = string.trim().split(',').collect::<Vec<_>>();
			let length = things.len();
			if string.is_empty() {
				break;
			}

			if length != 3 && length != 7 {
				return Err(ReplayError::ParseError(format!(
					"line {line} has an invalid number of items: {length}"
				)));
			}

			let Ok(diff_time) = things[0].parse() else {
				return Err(ReplayError::ParseError(format!(
					"line {line} has an invalid diff time: {}",
					things[0]
				)));
			};

			let Ok(Some(pressed)) = things[1].parse::<u16>().map(ControllerButtons::from_bits)
			else {
				return Err(ReplayError::ParseError(format!(
					"line {line} has invalid pressed bitfield: {}",
					things[1]
				)));
			};
			changes.pressed = pressed;

			let Ok(Some(released)) = things[2].parse::<u16>().map(ControllerButtons::from_bits)
			else {
				return Err(ReplayError::ParseError(format!(
					"line {line} has invalid released bitfield: {}",
					things[2]
				)));
			};
			changes.released = released;

			if length == 7 {
				let Ok(lx) = things[3].parse::<i8>() else {
					return Err(ReplayError::ParseError(format!(
						"line {line} has invalid lx value: {}",
						things[3]
					)));
				};
				let Ok(ly) = things[4].parse::<i8>() else {
					return Err(ReplayError::ParseError(format!(
						"line {line} has invalid ly value: {}",
						things[4]
					)));
				};
				let Ok(rx) = things[5].parse::<i8>() else {
					return Err(ReplayError::ParseError(format!(
						"line {line} has invalid rx value: {}",
						things[5]
					)));
				};
				let Ok(ry) = things[6].parse::<i8>() else {
					return Err(ReplayError::ParseError(format!(
						"line {line} has invalid rx value: {}",
						things[6]
					)));
				};
				changes.axes = Some([lx, ly, rx, ry]);
			}
			events.push((diff_time, changes));
			line += 1;
		}
		Ok(Self::Off(events))
	}

	pub fn play(self) -> Self {
		match self {
			Self::Off(events) => {
				log::info!("Player started");
				Self::Playing {
					last: Instant::now(),
					events,
					cursor: 0,
				}
			}
			Self::Playing { .. } => {
				log::warn!("Player::play was called while in state Playing.");
				self
			}
		}
	}

	pub fn get_events(&mut self) -> &[(u32, InputChanges)] {
		let (reset, range) = match self {
			Self::Off(_) => {
				log::warn!("Player::get_events was called while in state Off.");
				return &[];
			}
			Self::Playing {
				last,
				events,
				cursor,
			} => {
				let start_index = *cursor;
				let mut event_sum = 0;
				loop {
					// end of playback reached
					if *cursor >= events.len() {
						log::info!("playback ended!");
						if *cursor - start_index > 1 {
							log::warn!(
								"more than one event returned! (is robot processing slowly?)."
							);
						}
						break (true, start_index..*cursor);
					}

					// time since last_update
					let elapsed = last.elapsed().as_micros() / 100;
					if elapsed > u32::MAX as u128 {
						log::error!("recording exceeded maximum time between events (~119 hours), resetting.");
						break (true, 0..0);
					}
					let elapsed = elapsed as u32;

					event_sum += events[*cursor].0;
					if elapsed < event_sum {
						if start_index == *cursor {
							return &[];
						}
						*last = Instant::now();
						break (false, start_index..*cursor);
					}
					*cursor += 1;
				}
			}
		};

		if reset {
			self.reset();
		}

		&self.events()[range]
	}
	fn events(&self) -> &[(u32, InputChanges)] {
		match self {
			Self::Off(events) => events,
			Self::Playing { events, .. } => events,
		}
	}
	pub fn reset(&mut self) {
		let events = if let Self::Playing { events, .. } = self {
			let mut tmp = Vec::new();
			std::mem::swap(&mut tmp, events);
			tmp
		} else {
			return;
		};
		*self = Self::Off(events);
	}
	pub fn is_playing(&self) -> bool {
		if let Self::Playing { .. } = self {
			return true;
		}
		false
	}
}
