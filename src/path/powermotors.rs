use crate::{odom::DriveImuOdom, pid::*, state::Motor, units::*};

use uom::ConstZero;

use super::{IntoSeg, PathSegment};

pub struct PowerMotors {
	voltages: Vec<i16>,
	motors: Vec<Motor>,
}

impl IntoSeg for PowerMotors {}

impl PowerMotors {
	#[allow(dead_code)]
	pub fn new(voltages: Vec<i16>, motors: Vec<Motor>) -> Self {
		Self { voltages, motors }
	}
}

impl PathSegment for PowerMotors {
	fn start_follow(&mut self, _: &DriveImuOdom, _: &mut AnglePid) {
		for (m, &v) in self.motors.iter_mut().zip(self.voltages.iter()) {
			m.voltage(v);
		}
	}
	fn follow(&mut self, _: &DriveImuOdom, _: &mut AnglePid) -> Option<(Velocity, Velocity)> {
		for (m, &v) in self.motors.iter_mut().zip(self.voltages.iter()) {
			m.voltage(v);
		}
		Some((ConstZero::ZERO, ConstZero::ZERO))
	}
	fn end_follow(&mut self) {
		for m in &self.motors {
			m.voltage(0);
		}
	}
}
