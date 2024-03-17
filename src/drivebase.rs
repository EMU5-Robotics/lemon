use protocol::device::Gearbox;

use crate::{
	brain::Brain,
	motor::{self, Motor},
};

pub struct Tankdrive<const SIDE_N: usize> {
	left: [(Motor, bool); SIDE_N],
	right: [(Motor, bool); SIDE_N],
}

impl<const SIDE_N: usize> Tankdrive<SIDE_N> {
	pub fn new(
		left: [(u8, bool); SIDE_N],
		right: [(u8, bool); SIDE_N],
		gearbox: Gearbox,
		brain: &mut Brain,
	) -> Self {
		let to_motor_array = |v: [(u8, bool); SIDE_N]| v.map(|e| (brain.get_motor(e.0), e.1));
		let s = Self {
			left: to_motor_array(left),
			right: to_motor_array(right),
		};
		brain.set_gearboxes(
			gearbox,
			s.left.iter().chain(s.right.iter()).map(|(m, _)| m.port()),
		);
		s
	}
	pub fn set_side_percent_voltage(&mut self, left: f64, right: f64) {
		if left.abs() > 1.0 || right.abs() > 1.0 {
			log::warn!("Tankdrive::set_side_percent_voltage recieved values outside of [-1, 1]: (left: {left}, right: {right}). Values will be clamped");
		}

		let map_val = |v: f64, rev: bool| {
			let mut v = v.clamp(-1.0, 1.0);
			if rev {
				v = -v;
			}
			v
		};

		for (motor, rev) in &mut self.left {
			motor.set_target(motor::Target::PercentVoltage(map_val(left, *rev)));
		}
		for (motor, rev) in &mut self.right {
			motor.set_target(motor::Target::PercentVoltage(map_val(right, *rev)));
		}
	}
	pub fn set_side_percent_max_rpm(&mut self, left: f64, right: f64, max_rpm: f64) {
		if left.abs() > 1.0 || right.abs() > 1.0 {
			log::warn!("Tankdrive::set_side_percent_max_rpm recieved values outside of [-1, 1]: (left: {left}, right: {right}). Values will be clamped");
		}
		if max_rpm < 0.0 {
			log::warn!("Tankdrive::set_side_percent_max_rpm recieved a negative max_rpm: {max_rpm} rpm. Value will be made positive");
		}

		let map_val = |v: f64, rev: bool| {
			let mut v = v.clamp(-1.0, 1.0);
			if rev {
				v = -v;
			}
			(v * max_rpm) as i16
		};

		for (motor, rev) in &mut self.left {
			motor.set_target(motor::Target::RotationalVelocity(map_val(left, *rev)));
		}
		for (motor, rev) in &mut self.right {
			motor.set_target(motor::Target::RotationalVelocity(map_val(right, *rev)));
		}
	}
}
