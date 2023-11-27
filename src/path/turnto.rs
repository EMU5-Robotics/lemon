use std::f64::consts::{PI, TAU};

use crate::{
	odom::{DriveImuOdom, Odometry},
	pid::*,
	units::*,
};

use super::{IntoSeg, PathSegment};

use uom::ConstZero;

// in global space
pub struct TurnTo {
	target: Angle,
	offset: Angle,
}

impl IntoSeg for TurnTo {}
impl IntoSeg for RelativeTurnTo {}

impl TurnTo {
	pub fn new(angle: Angle) -> Self {
		Self {
			target: angle,
			offset: ConstZero::ZERO,
		}
	}
}

impl PathSegment for TurnTo {
	// transform target into local space and set offset
	fn start_follow(&mut self, odom: &DriveImuOdom, turn_pid: &mut AnglePid) {
		turn_pid.reset();

		// transform angle into closest angle in global space
		let map_to_closest_global = |target: Angle| -> Angle {
			let angle = odom.angle().value;

			// convert angle to [-PI, PI]
			let target = {
				// map angle to [-TAU, TAU]
				let mut target = target.value % TAU;
				// map angle to [0, TAU];
				target += TAU;
				target %= TAU;
				// map angle to [-PI, PI]
				if target > PI {
					target -= TAU;
				}
				target
			};

			let num_angles = (angle / TAU).floor();
			let res = num_angles * TAU + target;
			let mapped_angle = if (res - angle).abs() > PI {
				(angle / TAU).ceil() * TAU + target
			} else {
				res
			};

			radian!(mapped_angle)
		};

		// set target angle as relative angle to said transformed angle
		// and store offset
		self.offset = -odom.angle();
		self.target = map_to_closest_global(self.target) + self.offset;
		turn_pid.change_target(self.target);
	}
	fn follow(
		&mut self,
		odom: &DriveImuOdom,
		turn_pid: &mut AnglePid,
	) -> Option<(Velocity, Velocity)> {
		let angle = odom.angle();
		let angular_vel = turn_pid.step(angle + self.offset);

		let side_spacing: Length = meter!(0.14); // 28cm between sides?
		let mut side_vel = angular_vel * side_spacing;

		// scale velocity to stay within velocity limits
		let max_vel: Velocity = meter_per_second!(2.6);
		let scale = max_vel / side_vel.abs();
		if scale.value < 1.0 && !scale.value.is_nan() {
			side_vel = side_vel * scale;
		}

		if (angle + self.offset - self.target).abs() < degree!(5.0) && side_vel.abs().value < 0.01 {
			return None;
		}
		Some((-side_vel, side_vel))
	}
	fn end_follow(&mut self) {}
}

impl RelativeTurnTo {
	pub fn new(angle: Angle) -> Self {
		Self {
			target: angle,
			offset: ConstZero::ZERO,
		}
	}
}

pub struct RelativeTurnTo {
	target: Angle,
	offset: Angle,
}

impl PathSegment for RelativeTurnTo {
	// transform target into local space and set offset
	fn start_follow(&mut self, odom: &DriveImuOdom, turn_pid: &mut AnglePid) {
		turn_pid.reset();

		// offset angle by current position
		self.offset = -odom.angle();
		turn_pid.change_target(self.target);
	}
	fn follow(
		&mut self,
		odom: &DriveImuOdom,
		turn_pid: &mut AnglePid,
	) -> Option<(Velocity, Velocity)> {
		let angle = odom.angle();
		let angular_vel = turn_pid.step(angle + self.offset);

		let side_spacing: Length = meter!(0.14); // 28cm between sides?
		let mut side_vel = angular_vel * side_spacing;

		// scale velocity to stay within velocity limits
		let max_vel: Velocity = meter_per_second!(2.6);
		let scale = max_vel / side_vel.abs();
		if scale.value < 1.0 && !scale.value.is_nan() {
			side_vel = side_vel * scale;
		}

		if (angle + self.offset - self.target).abs() < degree!(5.0) && side_vel.abs().value < 0.01 {
			return None;
		}
		Some((-side_vel, side_vel))
	}
	fn end_follow(&mut self) {}
}
