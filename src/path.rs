use std::{
	collections::VecDeque,
	f64::consts::{PI, TAU},
};

use robot_algorithms::prelude::{Pos2, Vec2};

use crate::{
	odom::{DriveImuOdom, Odometry},
	pid::*,
	units::*,
};

use uom::ConstZero;

enum FollowState {
	Following,
	Stopping,
	Turning,
	SegmentInit,
}

pub struct Path {
	segments: VecDeque<PathSegment>,
	current_segment: Option<PathSegment>,
}

impl Path {
	pub fn new(segments: Vec<PathSegment>) -> Self {
		Self {
			segments: segments.into(),
			current_segment: None,
		}
	}
	pub fn follow(
		&mut self,
		odom: &DriveImuOdom,
		turn_pid: &mut AnglePid,
	) -> Option<(Velocity, Velocity)> {
		// check if there is an active segment
		if let Some(seg) = &self.current_segment {
			match seg.follow(odom, turn_pid) {
				Some(vels) => return Some(vels),
				None => {} // segment ended
			}
		}

		// try get new active segment
		self.current_segment = self.segments.pop_front();

		// new segment exists, start Following
		// and then return follow from recursive call
		if let Some(ref mut seg) = &mut self.current_segment {
			seg.start_follow(odom, turn_pid);
			return self.follow(odom, turn_pid);
		}

		// new segment does not exist return None
		None
	}
}

pub enum PathSegment {
	TurnTo {
		target: Angle,
		offset: Angle,
	},
	Path {
		points: Vec<Vec2>,
		velocities: Vec<Velocity>,
		reversed: bool,
	},
}

impl PathSegment {
	pub fn start_follow(&mut self, odom: &DriveImuOdom, turn_pid: &mut AnglePid) {
		turn_pid.reset();

		// transform target into local space and set offset
		if let Self::TurnTo {
			ref mut target,
			ref mut offset,
		} = self
		{
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
			*offset = -odom.angle();
			*target = map_to_closest_global(*target) + *offset;
		}
	}
	pub fn follow(
		&self,
		odom: &DriveImuOdom,
		turn_pid: &mut AnglePid,
	) -> Option<(Velocity, Velocity)> {
		match self {
			Self::TurnTo { target, offset } => {
				let angle = odom.angle();
				let angular_vel = turn_pid.step(angle + *offset);

				let side_spacing: Length = meter!(0.14); // 28cm between sides?
				let mut side_vel = angular_vel * side_spacing;

				// scale velocity to stay within velocity limits
				let max_vel: Velocity = meter_per_second!(2.6);
				let scale = max_vel / side_vel.abs();
				if scale.value < 1.0 && !scale.value.is_nan() {
					side_vel = side_vel * scale;
				}

				if (angle + *offset - *target).abs() < degree!(5.0) && side_vel.abs().value < 0.01 {
					return None;
				}
				Some((-side_vel, side_vel))
			}
			Self::Path {
				points,
				velocities,
				reversed: _, // todo: reversing not handled yet
			} => {
				let current_pos = odom.pos();
				let current_pos = Pos2::new(current_pos.0.value, current_pos.1.value); // meters

				// calculate target velocity from motion profile
				let vel =
					crate::motion_profile::get_profile_velocity((velocities, points), current_pos);

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
					println!("exited");
					return None;
				}

				Some((left_vel, right_vel))
			}
		}
	}
	pub fn rotate_abs(angle: Angle) -> Self {
		Self::TurnTo {
			target: angle,
			offset: ConstZero::ZERO,
		}
	}
	pub fn line(start: Vec2, end: Vec2) -> Self {
		const NUM_POINTS: usize = 64;

		let mut points = Vec::new();
		let mut velocities = Vec::new();

		let max_vel = meter_per_second!(2.6);
		let till_max = second!(0.8); // time taken to accelerate to max speed
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
			return Self::Path {
				points,
				velocities,
				reversed: false,
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

		Self::Path {
			points,
			velocities,
			reversed: false,
		}
	}
}
