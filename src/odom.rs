use std::time::{Duration, Instant};

use crate::{logging::*, parts::drive::Drive, parts::imu::*, state::RerunLogger, units::*};

pub trait Odometry {
	fn pos(&self) -> (Length, Length);
	fn angle(&self) -> Angle;
	fn vel(&self) -> (Velocity, Velocity);
	fn side_vel(&self) -> (Velocity, Velocity);
}

pub struct DriveImuOdom {
	pos: (Length, Length),
	angle: Angle,
	vel: (Velocity, Velocity),
	side_vel: (Velocity, Velocity),

	dist: (Length, Length),
	log_update: Instant,
	update: Instant,

	imu: Imu,
	logger: RerunLogger,
}

impl DriveImuOdom {
	pub fn new(logger: RerunLogger) -> anyhow::Result<Self> {
		let imu = Imu::new();

		Ok(Self {
			pos: (ConstZero::ZERO, ConstZero::ZERO),
			angle: ConstZero::ZERO,
			vel: (ConstZero::ZERO, ConstZero::ZERO),
			side_vel: (ConstZero::ZERO, ConstZero::ZERO),
			dist: (ConstZero::ZERO, ConstZero::ZERO),
			log_update: Instant::now(),
			update: Instant::now(),
			imu,
			logger,
		})
	}

	pub fn update(&mut self, drive: &mut Drive) {
		// get displacement and change in displacement
		// relative to the sides on the robot
		let (dist_l, dist_r) = match drive.get_encoders() {
			Some(d) => d,
			None => return,
		};
		let (diff_l, diff_r) = (dist_l - self.dist.0, dist_r - self.dist.1);

		let time = second!(self.update.elapsed().as_secs_f64());
		if let Some(vel) = drive.get_actual_velocity() {
			self.side_vel = vel;
		}

		// calculate displacement and change in angle for center of the robot
		let diff_dist = 0.5 * (diff_l + diff_r);
		let diff_angle = self.imu.angle_difference().unwrap_or(degree!(0.0));

		// calculate displacement in global space
		let tmp = self.angle + 0.5 * diff_angle;
		let (dx, dy) = tmp.sin_cos();
		let (dx, dy) = (diff_dist * dx, diff_dist * dy);

		// apply global displacement and rotation
		self.pos.0 -= dx;
		self.pos.1 += dy;
		self.angle += diff_angle;

		// update state
		self.dist = (dist_l, dist_r);
		self.update = Instant::now();

		self.vel = (dx / time, dy / time);

		if self.log_update.elapsed() > Duration::from_millis(3) {
			self.logger.with(|rerun, start| {
				rerun.set_time_seconds("odom", start.elapsed().as_secs_f64());

				// timeseries_colour(rerun, "odom/pos/x", self.pos.0.value, [127, 0, 127]);
				// timeseries_colour(rerun, "odom/pos/y", self.pos.1.value, [0, 127, 127]);
				timeseries_colour(
					rerun,
					"odom/angle",
					self.angle.value.to_degrees(),
					[127, 127, 0],
				);
				//set_robot_offset(rerun, self.pos);
			});
			self.log_update = Instant::now();
		}
	}
}

impl Odometry for DriveImuOdom {
	fn pos(&self) -> (Length, Length) {
		self.pos
	}

	fn angle(&self) -> Angle {
		self.angle
	}

	fn vel(&self) -> (Velocity, Velocity) {
		self.vel
	}

	fn side_vel(&self) -> (Velocity, Velocity) {
		self.side_vel
	}
}
