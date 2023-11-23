use std::{
	collections::VecDeque,
	f64::consts::{PI, TAU},
};

use robot_algorithms::{
	path_tracking::pure_pursuit,
	prelude::{Pos2, Ray, Vec2},
};

use crate::{
	odom::{DriveOdom, Odometry},
	pid::*,
	units::*,
};

enum FollowState {
	Following,
	Stopping,
	Turning,
	SegmentInit,
}

struct Path {
	segments: VecDeque<PathSegment>,
}

enum PathSegment {
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
	pub fn start_follow(
		&mut self,
		odom: &DriveOdom,
		turn_pid: &mut AnglePid,
		left_vel_pid: &mut VelocityPid,
		right_vel_pid: &mut VelocityPid,
	) {
		turn_pid.reset();
		left_vel_pid.reset();
		right_vel_pid.reset();

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
		odom: &DriveOdom,
		turn_pid: &mut AnglePid,
		left_vel_pid: &mut VelocityPid,
		right_vel_pid: &mut VelocityPid,
	) -> Option<(Power, Power)> {
		match self {
			Self::TurnTo { target: _, offset } => {
				// todo: change output from ratio to side velocity or angular velocity
				let angular_velocity = turn_pid.step(odom.angle() + *offset);

				let side_spacing: Length = meter!(0.1); // 20cm between sides?
				let mut side_vel = angular_velocity * side_spacing;

				// scale velocity to stay within velocity limits
				let max_vel: Velocity = meter_per_second!(1.0);
				let scale = max_vel / side_vel;
				if scale.value < 1.0 {
					side_vel = side_vel * scale;
				}

				// set PID targets
				left_vel_pid.change_target(-side_vel);
				right_vel_pid.change_target(side_vel);

				let cur_side_vel = odom.side_vel();

				// todo: double check coordinate system
				// calculate motor powers with PIDs
				let left_power = left_vel_pid.step(cur_side_vel.0);
				let right_power = right_vel_pid.step(cur_side_vel.1);

				Some((left_power, right_power))
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
				let side_spacing: Length = meter!(0.1); // 20cm between sides?
				let ratio = (curve_radius - side_spacing) / (curve_radius + side_spacing);
				let mut left_vel = ratio * vel;
				let mut right_vel = vel / ratio;

				// scale velocities to stay within velocity limits
				let max_vel: Velocity = meter_per_second!(1.0);
				let scale = max_vel / left_vel.max(right_vel);
				if scale.value < 1.0 {
					left_vel = left_vel * scale;
					right_vel = right_vel * scale;
				}

				// set PID targets
				left_vel_pid.change_target(left_vel);
				right_vel_pid.change_target(right_vel);

				let side_vel = odom.side_vel();

				// calculate motor powers with PIDs
				let left_power = left_vel_pid.step(side_vel.0);
				let right_power = right_vel_pid.step(side_vel.1);

				Some((left_power, right_power))
			}
		}
	}
}
