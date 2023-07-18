use client::coprocessor::{Port, SerialData};
use protocol::{Packet3, Packet4};
use std::time::Duration;

use fern::colors::{Color, ColoredLevelConfig};

fn main() {
	create_logger();

	let port = Port::open("/dev/ttyACM1").unwrap();
	let data = port.spawn_threaded();

	#[allow(unused_assignments)]
	let (mut last, mut current) = initialise(&data);

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

	loop {
		current = get_input(&data);
		let mut output = get_last_output(&data);

		// check controller input
		let controller = current.controller_axes;
		let d_power = controller[1] as f32 / 127.0;
		let t_power = controller[2] as f32 / 127.0;

		drive.drive(d_power, t_power, &mut output);

		send_data(&data, output);
		std::mem::swap(&mut last, &mut current);
		std::thread::sleep(Duration::from_millis(10));
	}
}

struct Motor {
	port: u8,
	reversed: bool,
}

impl Motor {
	pub fn new(port: u8, reversed: bool) -> Self {
		Self { port, reversed }
	}
}

struct Drive {
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
	pub fn drive(&self, fpower: f32, tpower: f32, output: &mut Packet4) {
		let lpower = ((fpower + self.turn_rate * tpower).clamp(-1.0, 1.0) * MAX_MILLIVOLT) as i16;
		let rpower = ((fpower - self.turn_rate * tpower).clamp(-1.0, 1.0) * MAX_MILLIVOLT) as i16;

		for motor in &self.left {
			output.set_motor(
				motor.port as usize - 1,
				if motor.reversed { -lpower } else { lpower },
			);
		}

		for motor in &self.right {
			output.set_motor(
				motor.port as usize - 1,
				if motor.reversed { -rpower } else { rpower },
			);
		}
	}
}

fn get_input(data: &SerialData) -> Packet3 {
	{
		data.recv_pkt_lock().unwrap().clone()
	}
}

fn get_last_output(data: &SerialData) -> Packet4 {
	{
		data.send_pkt_lock().unwrap().clone()
	}
}

fn initialise(data: &SerialData) -> (Packet3, Packet3) {
	let last = { data.recv_pkt_lock().unwrap().clone() };
	let current = { data.recv_pkt_lock().unwrap().clone() };
	(last, current)
}

fn send_data(data: &SerialData, packet: Packet4) {
	*data.send_pkt_lock().unwrap() = packet;
}

pub fn create_logger() {
	let colors = ColoredLevelConfig::new()
		.error(Color::Red)
		.warn(Color::Yellow)
		.info(Color::Cyan)
		.debug(Color::Magenta);

	fern::Dispatch::new()
		.format(move |out, message, record| {
			out.finish(format_args!(
				"{} {} [{}] {}",
				chrono::Local::now().format("%H:%M:%S"),
				colors.color(record.level()),
				record.target(),
				message
			))
		})
		.level(log::LevelFilter::Debug)
		.chain(std::io::stderr())
		.apply()
		.unwrap();
}
