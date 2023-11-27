use std::time::{Duration, Instant};

use bno055::{BNO055OperationMode, Bno055};
use rppal::{hal::Delay, i2c::I2c};
use uom::ConstZero;

use crate::{logging::*, parts::drive::Drive, state::RerunLogger, units::*};

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

	last_difference: Angle,
	dist: (Length, Length),
	imu_update: Instant,
	log_update: Instant,
	update: Instant,

	imu: Bno055<I2c>,
	logger: RerunLogger,
}

impl DriveImuOdom {
	pub fn new(logger: RerunLogger) -> anyhow::Result<Self> {
		let i2c = I2c::new()?;
		let mut delay = Delay::new();
		let mut imu = Bno055::new(i2c).with_alternative_address();
		imu.init(&mut delay)?;
		imu.set_mode(BNO055OperationMode::GYRO_ONLY, &mut delay)?;
		imu.set_external_crystal(true, &mut delay)?;

		Ok(Self {
			pos: (ConstZero::ZERO, ConstZero::ZERO),
			angle: ConstZero::ZERO,
			vel: (ConstZero::ZERO, ConstZero::ZERO),
			side_vel: (ConstZero::ZERO, ConstZero::ZERO),
			last_difference: ConstZero::ZERO,
			dist: (ConstZero::ZERO, ConstZero::ZERO),
			imu_update: Instant::now(),
			log_update: Instant::now(),
			update: Instant::now(),
			imu,
			logger,
		})
	}

	pub fn get_angle_difference(&mut self) -> Option<Angle> {
		let elapsed = self.imu_update.elapsed();
		if elapsed < Duration::from_micros(10_500) {
			return None;
		}

		let gyro: AngularVelocity = match self.imu.gyro_data() {
			Ok(vec) => degree_per_second!(vec.z as f64 * 1.2768221), // the yaw/heading
			Err(err) => {
				log::warn!("Failed to get gyro from IMU: {:?}", err);
				return None;
			}
		};

		let diff: Angle = (gyro * second!(elapsed.as_secs_f64())).into();

		// Is an invalid spike, ignore
		if diff.abs() > degree!(15.0) {
			return None;
		}

		// Basic noise floor filter
		if diff.abs() < degree!(0.007) {
			self.last_difference = ConstZero::ZERO;
			self.imu_update = Instant::now();
			None
		} else {
			self.last_difference = diff;
			self.imu_update = Instant::now();
			Some(diff)
		}
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
		let diff_angle = match self.get_angle_difference() {
			Some(d) => d,
			None => degree!(0.0),
		};

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
