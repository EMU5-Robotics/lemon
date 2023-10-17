use client::coprocessor::serial::{find_v5_port, Serial, SerialData};
use protocol::{device::ControllerButtons, ControlPkt, StatusPkt};
use rerun::{default_server_addr, RecordingStream, RecordingStreamBuilder};
use std::{
	error::Error,
	time::{Duration, Instant},
};

mod motion_profile;
mod odom;
mod path;
mod pid;
mod replay;
mod units;

use crate::pid::*;
use replay::{Player, Recorder};
use units::*;

use once_cell::sync::Lazy;

pub static RERUN_REC: Lazy<RecordingStream> = Lazy::new(|| {
	RecordingStreamBuilder::new("lemon")
		//.default_enabled(false)
		.connect(
			"192.168.240.179:9876".parse().unwrap(),
			Some(Duration::from_millis(2_000)),
		)
		.unwrap_or(
			RecordingStreamBuilder::new("lemon (file fallback)")
				.save("recording.rrd")
				.unwrap(),
		)
});

pub static PROGRAM_START: Lazy<Instant> = Lazy::new(|| Instant::now());

fn main() -> Result<(), Box<dyn Error>> {
	once_cell::sync::Lazy::<Instant>::force(&PROGRAM_START);

	rerun::Logger::new(RERUN_REC.clone())
		.with_path_prefix("logs")
		.with_filter(rerun::default_log_filter())
		.init()?;

	let ports = find_v5_port()?;
	let serial = Serial::open(&ports.0.port_name)?;
	let data = serial.spawn_threaded();

	let mut last = get_input(&data);
	let mut current;

	let mut drive = Drive::new(
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

	let mut odom = odom::DriveOdom::new(meter!(0.1285));

	let _velocity_pid = VelocityPid::new(0.3, 0.3, 0.3, meter_per_second!(1.0));
	let _right_velocity_pid = VelocityPid::new(0.3, 0.3, 0.3, meter_per_second!(1.0));
	let _turning_pid = AnglePid::new(0.3, 0.3, 0.3, radian!(1.0));

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

		odom.update(&current, &mut drive);

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
	last_encoder: (Length, Length),
	last_update: Instant,
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
			last_encoder: (meter!(0.0), meter!(0.0)),
			last_update: Instant::now(),
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
	pub fn side_encoders(&mut self, pkt: &StatusPkt) -> Option<(Length, Length)> {
		let smooth_window: Time = millisecond!(40.0);

		let (mut l, mut r) = self.raw_side_encoders(pkt)?;

		// smooth encoder values to avoid aliasing issues
		let elapsed = second!(self.last_update.elapsed().as_secs_f64());
		let t = (elapsed / smooth_window).value;

		if t < 1.0 {
			l = (1.0 - t) * self.last_encoder.0 + t * l;
			r = (1.0 - t) * self.last_encoder.1 + t * r;
		}

		self.last_update = Instant::now();
		self.last_encoder = (l, r);

		Some((l, r))
	}
	pub fn raw_side_encoders(&mut self, pkt: &StatusPkt) -> Option<(Length, Length)> {
		Some((
			meter!(pkt.get_encoder(1)? as f64 * 0.006),
			meter!(pkt.get_encoder(20)? as f64 * 0.006),
		))
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
