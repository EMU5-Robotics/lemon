#![allow(dead_code)]
#![allow(unused_imports)]

use std::{
	thread,
	time::{Duration, Instant},
};

use protocol::device::{ControllerButtons, Gearbox};

use crate::{
	logging::*,
	state::{InputChanges, Motor, RerunLogger},
};

pub struct Loader {
	motors: [Motor; 2],
	state: LoaderState,
	primed_pos: i32,
	loaded_pos: i32,

	hold_time: Duration,
	load_time: Duration,
	speed: f32,
	pos_threshold: u32,
	pub fold_up: bool,
	pub fold_out: bool,
	pub hold_load: bool,
	reset_time: Instant,
	logger: RerunLogger,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum LoaderState {
	Primed(Instant),
	Loading,
	Loaded(Instant),
	Reseting,
	Idle,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LoaderPosState {
	Primed,
	Folded,
	Other,
}

impl Loader {
	pub fn new(logger: RerunLogger, motors: [Motor; 2]) -> Self {
		Self {
			motors,
			state: LoaderState::Idle,
			// primed_pos: 46_000,
			// loaded_pos: 7_500,
			primed_pos: 0,
			loaded_pos: 38_500,
			hold_time: Duration::from_millis(100),
			load_time: Duration::from_millis(1000),
			speed: 0.5,
			pos_threshold: 1000,
			fold_up: false,
			fold_out: false,
			hold_load: false,
			reset_time: Instant::now(),
			logger,
		}
	}

	pub fn transition(&mut self) {
		match self.state {
			LoaderState::Primed(at) => {
				self.set_power(0.0);
				// If elapsed hold time then change state, else just wait
				if Instant::now() > at + self.load_time {
					self.state = LoaderState::Loading;
				}
			}
			LoaderState::Loading => {
				self.set_power(1.0);
				// If we are close the loaded position then transition state
				let pos = self.get_position();
				if pos > self.loaded_pos || pos.abs_diff(self.loaded_pos) < self.pos_threshold {
					self.state = LoaderState::Loaded(Instant::now());
				}
			}
			LoaderState::Loaded(at) => {
				self.set_power(0.0);
				if self.fold_up {
					self.state = LoaderState::Idle;
					self.fold_up = false;
				}
				if Instant::now() > at + self.hold_time && !self.hold_load {
					self.state = LoaderState::Reseting;
					self.reset_time = Instant::now();
				}
			}
			LoaderState::Reseting => {
				self.set_power(-1.0);
				// If we are close the primed position then transition state
				let pos = self.get_position();
				if pos < self.primed_pos || pos.abs_diff(self.primed_pos) < self.pos_threshold {
					if self.fold_out {
						self.state = LoaderState::Idle;
						self.fold_out = false;
					} else {
						self.state = LoaderState::Primed(Instant::now());
					}
				}
			}
			LoaderState::Idle => {
				self.set_power(0.0);
			}
		}
	}

	pub fn is_ready_to_fire(&self) -> bool {
		let now = Instant::now();
		matches!(self.state, LoaderState::Reseting)
			&& now > self.reset_time + Duration::from_millis(250)
			&& now < self.reset_time + Duration::from_millis(270)
	}

	pub fn start_primed(&mut self) {
		self.state = LoaderState::Primed(Instant::now() - self.load_time);
	}

	pub fn start_folded(&mut self) {
		self.state = LoaderState::Loaded(Instant::now() - self.load_time);
	}

	pub fn reset(&mut self) {
		self.state = LoaderState::Reseting;
	}

	pub fn state_pos(&self) -> LoaderPosState {
		let pos = self.get_position();
		if pos < self.primed_pos || pos.abs_diff(self.primed_pos) < 2000 {
			LoaderPosState::Primed
		} else if pos > self.loaded_pos || pos.abs_diff(self.loaded_pos) < 2000 {
			LoaderPosState::Folded
		} else {
			LoaderPosState::Other
		}
	}

	fn set_power(&mut self, power: f32) {
		let vel = power.clamp(-1.0, 1.0) * self.speed * 200.0;
		for motor in &self.motors {
			motor.velocity(vel as i16);
		}
	}

	fn get_position(&self) -> i32 {
		let mut sum = 0;
		let mut count = 0;
		for motor in &self.motors {
			if motor.is_connected() {
				sum += motor.position() * motor.reversed_factor() as i32;
				count += 1;
			}
		}
		match count {
			0 => 0,
			n => sum / n,
		}
	}

	pub fn get_gearboxes(&self) -> impl IntoIterator<Item = (u8, Gearbox)> + '_ {
		[
			(self.motors[0].port(), Gearbox::Green),
			(self.motors[1].port(), Gearbox::Green),
		]
	}
}

pub struct Catapult {
	motors: [Motor; 2],
	state: CatapultState,
	speed_mv: i16,
	prime_timeout: Duration,
	prime_power: f32,
	prime_dist: u32,
	quat_dist: u32,
	cycle: usize,
	start_pos: Option<i32>,

	logger: RerunLogger,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CatapultState {
	Priming,
	Primed(Instant),
	Fire,
	Idle,
	// Calibration states
	Calibration(CatapultCalibrationState),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CatapultCalibrationState {
	Unknown(usize, [i32; 10], Instant),
	Wait(Instant),
	MoveForward(Instant),
	Rest(Instant),
}

impl Catapult {
	pub fn new(logger: RerunLogger, motors: [Motor; 2]) -> Self {
		Self {
			motors,
			state: CatapultState::Idle,
			speed_mv: 12_000,
			prime_timeout: Duration::from_secs(2),
			prime_power: 0.2,
			prime_dist: 125_000,
			// prime_dist: 125_000,
			quat_dist: 135_000,
			cycle: 0,
			start_pos: None,

			logger,
		}
	}

	pub fn transition(&mut self) {
		match self.state {
			CatapultState::Priming => {
				self.set_power(1.0);
				if self.get_position() >= self.prime_dist as i32 + self.cycle() {
					self.state = CatapultState::Primed(Instant::now());
				}
			}
			CatapultState::Primed(at) => {
				self.set_power(self.prime_power);
				if Instant::now() > at + self.prime_timeout {
					self.state = CatapultState::Idle;
				}
			}
			CatapultState::Fire => {
				self.set_power(1.0);
				if self.get_position()
					>= self.prime_dist as i32 + self.cycle() + self.quat_dist as i32
				{
					self.cycle += 1;
					self.state = CatapultState::Priming;
				}
			}
			CatapultState::Idle => {
				self.set_power(0.0);
			}
			CatapultState::Calibration(state) => self.transition_calibration(state),
		}
	}

	fn transition_calibration(&mut self, mut state: CatapultCalibrationState) {
		match state {
			CatapultCalibrationState::Unknown(mut cursor, mut last, start) => {
				self.set_power(1.0);

				// Average last 10 samples of current
				let velocity = self.get_actual_velocity();
				last[cursor % last.len()] = velocity;
				cursor += 1;
				let avg = last.iter().sum::<i32>() / last.len() as i32;

				// If the current current is noticeably less we have probably stopped meshing, we
				// now know we are in a toothless section
				if velocity > avg - 10 && Instant::now() > start + Duration::from_millis(1000) {
					state = CatapultCalibrationState::Wait(Instant::now());
				} else {
					state = CatapultCalibrationState::Unknown(cursor, last, start);
				}
			}
			CatapultCalibrationState::Wait(at) => {
				self.set_power(0.0);
				if Instant::now() > at + Duration::from_millis(300) {
					state = CatapultCalibrationState::MoveForward(Instant::now());
				}
			}
			CatapultCalibrationState::MoveForward(at) => {
				self.set_power(0.05);
				if Instant::now() > at + Duration::from_millis(300) {
					state = CatapultCalibrationState::Rest(Instant::now());
				}
			}
			CatapultCalibrationState::Rest(at) => {
				self.set_power(0.0);
				if Instant::now() > at + Duration::from_millis(300) {
					self.start_pos = Some(self.get_position() + self.start_pos.unwrap_or(0));
					self.state = CatapultState::Idle;
					return;
				}
			}
		};
		self.state = CatapultState::Calibration(state);
	}

	pub fn is_primed(&self) -> bool {
		matches!(self.state, CatapultState::Primed(_))
	}

	pub fn is_idle(&self) -> bool {
		matches!(self.state, CatapultState::Idle)
	}

	pub fn is_calibrated(&self) -> bool {
		!matches!(self.state, CatapultState::Calibration(_))
	}

	pub fn prime(&mut self) {
		if matches!(self.state, CatapultState::Idle) {
			self.state = CatapultState::Priming;
		}
	}

	pub fn fire(&mut self) {
		if matches!(self.state, CatapultState::Primed(_)) {
			self.state = CatapultState::Fire;
		}
	}

	pub fn reset(&mut self) {
		self.state = CatapultState::Calibration(CatapultCalibrationState::Unknown(
			0,
			[0; 10],
			Instant::now(),
		));
		self.cycle = 0;
	}

	pub fn count(&self) -> usize {
		self.cycle
	}

	fn cycle(&self) -> i32 {
		self.cycle as i32 * self.quat_dist as i32 * 2
	}

	fn set_power(&mut self, power: f32) {
		let mv = power.clamp(-1.0, 1.0) * self.speed_mv as f32;
		for motor in &self.motors {
			motor.voltage(mv as i16);
		}
	}

	fn set_halt(&mut self) {
		for motor in &self.motors {
			motor.velocity(0);
		}
	}

	fn get_position(&self) -> i32 {
		let mut sum = 0;
		let mut count = 0;
		for motor in &self.motors {
			if motor.is_connected() {
				sum += motor.position() * motor.reversed_factor() as i32;
				count += 1;
			}
		}
		let sum = match count {
			0 => 0,
			n => sum / n,
		};
		match self.start_pos {
			Some(start) => sum - start,
			None => sum,
		}
	}

	fn get_actual_velocity(&self) -> i32 {
		let mut sum = 0;
		let mut count = 0;
		for motor in &self.motors {
			if motor.is_connected() {
				sum += motor.actual_velocity() as i32;
				count += 1;
			}
		}
		match count {
			0 => 0,
			n => sum / n,
		}
	}

	pub fn get_gearboxes(&self) -> impl IntoIterator<Item = (u8, Gearbox)> {
		[
			(self.motors[0].port(), Gearbox::Red),
			(self.motors[1].port(), Gearbox::Red),
		]
	}
}
