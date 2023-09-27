use client::coprocessor::serial::{find_v5_port, Serial, SerialData};
use protocol::{device::ControllerButtons, ControlPkt, StatusPkt};
use std::time::Duration;

mod motion_profile;
mod odom;
mod replay;
mod units;

use replay::{Player, Recorder};
use units::*;

fn main() {
	common::create_logger();

	let ports = find_v5_port().unwrap();
	let serial = Serial::open(&ports.0.port_name).unwrap();
	let data = serial.spawn_threaded();

	let mut last = get_input(&data);
	let mut current;

	let drive = Drive::new(
		[
			Motor::new(10, true),
			Motor::new(2, true),
			Motor::new(1, false),
		],
		[
			Motor::new(20, false),
			Motor::new(12, false),
			Motor::new(11, true),
		],
		0.8,
	);

	let mut odom = odom::DriveOdom::new(meter!(0.1285), &drive);

	let mut recorder = Recorder::new();
	let mut player = Player::from_file("test.replay").unwrap_or_default();

	let mut axes = [0, 0, 0, 0];

	loop {
		current = get_input(&data);
		let mut output = get_last_output(&data);

		let mut input_changes = InputChanges::from_pair(&last, &current);

		// check for player independent events
		if input_changes.released(ControllerButtons::A)
			|| (input_changes.pressed(ControllerButtons::A) && recorder != Recorder::Off)
		{
			if let Err(e) = recorder.toggle() {
				log::error!("recorder toggle failed with: {e}");
			}
		}

		if input_changes.released(ControllerButtons::B) {
			player = player.play();
		}

		if let Err(e) = recorder.take_event(&input_changes) {
			log::error!("recorder failed to take event with: {e}");
		}

		if player.is_playing() {
			// play replay
			let events = player.get_events();
			if !events.is_empty() {
				// override input changes event
				input_changes = events[0].1;
			}
		}
		drive.side_encoders(&current);

		// update axes if detected change
		if let Some(new_axes) = input_changes.axes {
			axes = new_axes;
		}

		let d_power = axes[1] as f32 / 127.0;
		let t_power = axes[2] as f32 / 127.0;
		drive.drive(d_power, t_power, &mut output);

		odom.update(&current);

		send_data(&data, output);
		std::mem::swap(&mut last, &mut current);
		std::thread::sleep(Duration::from_millis(1));
	}
}

#[derive(Debug, PartialEq, Copy, Clone)]
pub struct InputChanges {
	pressed: ControllerButtons,
	released: ControllerButtons,
	axes: Option<[i8; 4]>,
}

impl InputChanges {
	pub const NOCHANGE: Self = InputChanges {
		pressed: ControllerButtons::empty(),
		released: ControllerButtons::empty(),
		axes: None,
	};

	pub fn from_pair(last: &StatusPkt, current: &StatusPkt) -> Self {
		let pressed = current.controller_buttons & !last.controller_buttons;

		let released = !current.controller_buttons & last.controller_buttons;

		let axes = if current.controller_axes == last.controller_axes {
			None
		} else {
			Some(current.controller_axes)
		};

		Self {
			pressed,
			released,
			axes,
		}
	}
	pub fn changes(&self) -> bool {
		self.pressed != ControllerButtons::empty()
			|| self.released != ControllerButtons::empty()
			|| self.axes.is_some()
	}
	pub fn released(&self, buttons: ControllerButtons) -> bool {
		self.released & buttons != ControllerButtons::empty()
	}
	pub fn pressed(&self, buttons: ControllerButtons) -> bool {
		self.pressed & buttons != ControllerButtons::empty()
	}
}

pub struct Motor {
	port: u8,
	reversed: bool,
}

impl Motor {
	pub fn new(port: u8, reversed: bool) -> Self {
		Self { port, reversed }
	}
}

pub struct Drive {
	left: [Motor; 3],
	right: [Motor; 3],
	turn_rate: f32,
}

const MAX_MILLIVOLT: f32 = 4000.0;

impl Drive {
	pub fn new(left: [Motor; 3], right: [Motor; 3], turn_rate: f32) -> Self {
		for l_motor in &left {
			if !(1..=20).contains(&l_motor.port) {
				panic!("Invalid motor port");
			}
		}
		for r_motor in &right {
			if !(1..=20).contains(&r_motor.port) {
				panic!("Invalid motor port");
			}
		}
		if !(0.0..1.0).contains(&turn_rate) {
			panic!("Invalid turn rate");
		}
		Self {
			left,
			right,
			turn_rate,
		}
	}

	pub fn drive(&self, fpower: f32, tpower: f32, output: &mut ControlPkt) {
		let lpower = ((fpower + self.turn_rate * tpower).clamp(-1.0, 1.0) * MAX_MILLIVOLT) as i16;
		let rpower = ((fpower - self.turn_rate * tpower).clamp(-1.0, 1.0) * MAX_MILLIVOLT) as i16;

		for motor in &self.left {
			output.set_power(
				motor.port as usize,
				if motor.reversed { -lpower } else { lpower },
			);
		}

		for motor in &self.right {
			output.set_power(
				motor.port as usize,
				if motor.reversed { -rpower } else { rpower },
			);
		}
	}
	pub fn side_encoders(&self, pkt: &StatusPkt) -> Option<(Length, Length)> {
		let (l, r) = (
			pkt.get_encoder(1)? as f64 * 0.006,
			pkt.get_encoder(20)? as f64 * 0.006,
		);
		Some((meter!(l), meter!(r)))
	}
}

fn get_input(data: &SerialData) -> StatusPkt {
	data.recv_pkt_lock().unwrap().clone()
}

fn get_last_output(data: &SerialData) -> ControlPkt {
	data.send_pkt_lock().unwrap().clone()
}

fn send_data(data: &SerialData, packet: ControlPkt) {
	*data.send_pkt_lock().unwrap() = packet;
}
