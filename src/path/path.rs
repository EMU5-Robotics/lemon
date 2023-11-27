use robot_algorithms::prelude::{Pos2, Vec2};

use crate::{
	odom::{DriveImuOdom, Odometry},
	pid::*,
	units::*,
};

use super::{IntoSeg, PathSegment};

// ANGLE IS NOT USED CURRENTLY DUE TO NO PURE PURSUIT
// SO REVERSED DOES NOT CHANGE HEADING
// THIS WILL NEED TO BE CHANGED LATER

pub struct PathSeg {
	points: Vec<Vec2>,
	velocities: Vec<Velocity>,
	reversed: bool,
}

impl IntoSeg for PathSeg {}
impl IntoSeg for RelativePath {}

impl PathSeg {
	pub fn line(start: Vec2, end: Vec2, reversed: bool) -> Self {
		const NUM_POINTS: usize = 64;

		let mut points = Vec::new();
		let mut velocities = Vec::new();

		let max_vel = meter_per_second!(2.6);
		let till_max = second!(0.1); // time taken to accelerate to max speed
		let max_accel = max_vel / till_max;

		let distance = meter!((end - start).magnitude());

		// figure out time for triangle profile
		// half_area_tri = (1/2) * v * half_t (note t is half the time)
		// distance / 2 = (1/2) * max_accel * half_t * half_t
		// distance  = max_accel * half_t^2
		// half_t = sqrt(distance / max_accel)
		let half_t = (distance / max_accel).sqrt();

		// not enough time to reach max_accel
		// use triangle profile
		// for constant acceleration:
		// distance = t * v * (1/2)
		// v = t * max_accel
		// distance = (1/2) * t^2 * max_accel
		// t = sqrt(2 * distance / max_accel)
		if half_t < till_max {
			// create points and velocities
			for i in 0..NUM_POINTS {
				let mut ratio = i as f64 / (NUM_POINTS - 1) as f64;
				let real_ratio = ratio;
				if ratio > 0.5 {
					ratio = 1.0 - ratio;
				}

				let cur_dist = ratio * distance;
				let mut tv = (2.0 * cur_dist / max_accel).sqrt();
				if i < NUM_POINTS / 2 {
					tv += (half_t * 0.05).min(half_t);
				}

				let v = tv * max_accel;
				let pos = start + real_ratio * (end - start);
				velocities.push(v);
				points.push(pos);
			}
			return Self {
				points,
				velocities,
				reversed,
			};
		}

		// use trapezoid profile
		for i in 0..NUM_POINTS {
			let mut ratio = i as f64 / (NUM_POINTS - 1) as f64;
			let real_ratio = ratio;
			if ratio > 0.5 {
				ratio = 1.0 - ratio;
			}
			let cur_dist = ratio * distance;
			let mut tv = (2.0 * cur_dist / max_accel).sqrt();

			if tv < till_max && i < NUM_POINTS / 2 {
				tv += (0.05 * till_max).min(till_max);
			}

			let v = tv * max_accel;
			let pos = start + real_ratio * (end - start);
			velocities.push(v);
			points.push(pos);
		}

		Self {
			points,
			velocities,
			reversed,
		}
	}
	pub fn into_relative(self) -> RelativePath {
		RelativePath {
			points: self.points,
			velocities: self.velocities,
			reversed: self.reversed,
			offset: Default::default(),
		}
	}
}

impl PathSegment for PathSeg {
	fn follow(&mut self, odom: &DriveImuOdom, _: &mut AnglePid) -> Option<(Velocity, Velocity)> {
		let current_pos = odom.pos();
		let current_pos = Pos2::new(current_pos.0.value, current_pos.1.value); // meters

		// calculate target velocity from motion profile
		let vel = crate::motion_profile::get_profile_velocity(
			(&self.velocities, &self.points),
			current_pos,
		);

		let mut left_vel = vel;
		let mut right_vel = vel;

		// scale velocities to stay within velocity limits
		let max_vel: Velocity = meter_per_second!(1.0);
		let scale = max_vel / left_vel.abs().max(right_vel.abs());
		if scale.value < 1.0 && !scale.value.is_nan() {
			left_vel = left_vel * scale;
			right_vel = right_vel * scale;
		}

		// exit cond dictated by motion_profile.rs
		if left_vel.value == 0.0 && right_vel.value == 0.0 {
			return None;
		}

		if self.reversed {
			left_vel = -left_vel;
			right_vel = -right_vel;
		}

		Some((left_vel, right_vel))
	}
}
pub struct RelativePath {
	points: Vec<Vec2>,
	velocities: Vec<Velocity>,
	offset: (Vec2, Angle),
	reversed: bool,
}

impl PathSegment for RelativePath {
	fn start_follow(&mut self, odom: &DriveImuOdom, _: &mut AnglePid) {
		let pos = odom.pos();
		self.offset.0 = Vec2::new(-pos.0.value, -pos.1.value);
	}
	fn follow(&mut self, odom: &DriveImuOdom, _: &mut AnglePid) -> Option<(Velocity, Velocity)> {
		let current_pos = odom.pos();
		let current_pos = Pos2::new(current_pos.0.value, current_pos.1.value) + self.offset.0; // meters

		// calculate target velocity from motion profile
		let vel = crate::motion_profile::get_profile_velocity(
			(&self.velocities, &self.points),
			current_pos,
		);

		let mut left_vel = vel;
		let mut right_vel = vel;

		// scale velocities to stay within velocity limits
		let max_vel: Velocity = meter_per_second!(1.0);
		let scale = max_vel / left_vel.abs().max(right_vel.abs());
		if scale.value < 1.0 && !scale.value.is_nan() {
			left_vel = left_vel * scale;
			right_vel = right_vel * scale;
		}

		// exit cond dictated by motion_profile.rs
		if left_vel.value == 0.0 && right_vel.value == 0.0 {
			return None;
		}

		if self.reversed {
			left_vel = -left_vel;
			right_vel = -right_vel;
		}

		Some((left_vel, right_vel))
	}
}
