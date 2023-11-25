use std::{
	collections::VecDeque,
	f64::consts::{PI, TAU},
};

use robot_algorithms::{
	path_tracking::pure_pursuit,
	prelude::{Pos2, Ray, Vec2},
};

use crate::{
	odom::{DriveImuOdom, Odometry},
	pid::*,
	units::*,
};

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
				Some(powers) => return Some(powers),
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
			Self::TurnTo { target: _, offset } => {
				let angle = odom.angle();
				let angular_vel = turn_pid.step(angle + *offset);

				let side_spacing: Length = meter!(0.14); // 28cm between sides?
				let mut side_vel = angular_vel * side_spacing;

				// scale velocity to stay within velocity limits
				let max_vel: Velocity = meter_per_second!(2.6);
				let scale = max_vel / side_vel.abs();
				if scale.value < 1.0 {
					side_vel = side_vel * scale;
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

				// check if near end of path (disable angular velocity PID)
				let mut curve_radius = meter!(0.0);
				if true {
					let pos = Ray::new(current_pos, odom.angle().value); // radians
					let lookahead_sq: Length = meter!(0.01);

					// calculate target curvature from pure pursuit
					curve_radius =
						match pure_pursuit::get_curvature(points, &pos, lookahead_sq.value) {
							Ok(curve) => meter!(1.0 / curve),
							Err(e) => {
								log::error!("pure pursuit failed with: {e:?}");
								return None;
							}
						};
				}

				// convert velocity and curve radius into side velocities
				let side_spacing: Length = meter!(0.14); // 28cm between sides?
				let ratio = (curve_radius - side_spacing) / (curve_radius + side_spacing);
				let mut left_vel = ratio * vel;
				let mut right_vel = vel / ratio;

				// scale velocities to stay within velocity limits
				let max_vel: Velocity = meter_per_second!(1.0);
				let scale = max_vel / left_vel.abs().max(right_vel.abs());
				if scale.value < 1.0 {
					left_vel = left_vel * scale;
					right_vel = right_vel * scale;
				}

				Some((left_vel, right_vel))
			}
		}
	}
	pub fn line(start: Vec2, end: Vec2) -> Self {
		const NUM_POINTS: usize = 16;

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
				let cur_dist = (i as f64 / (NUM_POINTS - 1) as f64) * distance;
				let mut tv = (2.0 * cur_dist / max_accel).sqrt();
				if tv > half_t {
					tv = 2.0 * half_t - tv;
				}

				let v = tv * max_accel;
				let pos = start + (cur_dist / distance).value * (end - start);
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
		// distance = h(a+b)/2
		// a = a (time at max_vel)
		// b = a + 2.0 * till_max (time at max_vel + acceleration time)
		// h = max_vel
		// distance = max_vel(a + a + 2.0 * till_max)/2
		// distance = max_vel(a+till_max)
		// distance / max_vel = a + till_max
		// distance / max_vel - till_max = a
		let t_at_max = distance / max_vel - till_max;
		let t_total = t_at_max + 2.0 * till_max;
		for i in 0..NUM_POINTS {
			let cur_dist = (i as f64 / (NUM_POINTS - 1) as f64) * distance;
			let mut tv = (2.0 * cur_dist / max_accel).sqrt();

			if tv < till_max { // accel
				 // no change
			} else if tv > till_max {
				// constant speed
				tv = till_max;
			} else {
				// decel
				tv = t_total - tv;
			}

			let v = tv * max_accel;
			let pos = start + (cur_dist / distance).value * (end - start);
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
