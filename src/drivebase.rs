use crate::{
	brain::Brain,
	motor::{self, Motor},
};

pub struct Tankdrive<const SIDE_N: usize> {
	left: [(Motor, bool); SIDE_N],
	right: [(Motor, bool); SIDE_N],
}

impl<const SIDE_N: usize> Tankdrive<SIDE_N> {
	pub fn new(left: [(u8, bool); SIDE_N], right: [(u8, bool); SIDE_N], brain: &Brain) -> Self {
		let to_motor_array = |v: [(u8, bool); SIDE_N]| v.map(|e| (brain.get_motor(e.0), e.1));
		Self {
			left: to_motor_array(left),
			right: to_motor_array(right),
		}
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
}
