use std::{
	f64::consts::PI,
	time::{Duration, Instant},
};

use bno055::Bno055;
use ringbuffer::RingBuffer;
use rppal::i2c::I2c;
use uom::ConstZero;

use crate::{logging::*, parts::drive::Drive, state::RerunLogger, units::*};

pub trait Odometry {
	fn pos(&self) -> (Length, Length);
	fn angle(&self) -> Angle;
	fn vel(&self) -> (Velocity, Velocity);
	fn side_vel(&self) -> (Velocity, Velocity);
}

pub struct DriveOdom {
	dist: (Length, Length),
	angle: Angle,
	dw: Length,
	pos: (Length, Length),
	update: Instant,
	vel: (Velocity, Velocity),
	side_vels: (util::RingBuf<Velocity, 5>, util::RingBuf<Velocity, 5>),
	last_log: Instant,
	logger: RerunLogger,
}

impl DriveOdom {
	pub fn new(logger: RerunLogger, dw: Length) -> Self {
		Self {
			dist: (ConstZero::ZERO, ConstZero::ZERO),
			dw,
			angle: ConstZero::ZERO,
			pos: (ConstZero::ZERO, ConstZero::ZERO),
			update: Instant::now(),
			vel: (ConstZero::ZERO, ConstZero::ZERO),
			side_vels: (util::RingBuf::new(), util::RingBuf::new()),
			last_log: Instant::now(),
			logger,
		}
	}

	pub fn update(&mut self, drive: &mut Drive) {
		if self.update.elapsed() < Duration::from_micros(5200) {
			return;
		}

		// get displacement and change in displacement
		// relative to the sides on the robot
		let (dist_l, dist_r) = match drive.get_encoders() {
			Some(d) => d,
			None => return,
		};
		let (diff_l, diff_r) = (dist_l - self.dist.0, dist_r - self.dist.1);

		let time = second!(self.update.elapsed().as_secs_f64());
		self.side_vels.0.push(diff_l / time);
		self.side_vels.1.push(diff_r / time);

		// calculate displacement and change in angle for center of the robot
		let diff_dist = 0.5 * (diff_l + diff_r);
		let diff_angle: Angle = (0.5 * (diff_r - diff_l) / self.dw).into();

		// calculate displacement in global space
		let tmp = self.angle + 0.5 * diff_angle;
		let (dx, dy) = tmp.sin_cos();
		let (dx, dy) = (diff_dist * dx, diff_dist * dy);

		// apply global displacement and rotation
		self.pos.0 -= dx;
		self.pos.1 += dy;
		self.angle += diff_angle;

		// update state and calculate running averages
		self.dist = (dist_l, dist_r);
		self.update = Instant::now();

		self.vel = (dx / time, dy / time);

		if self.last_log.elapsed() > Duration::from_millis(0) {
			self.last_log = Instant::now();
			self.logger.with(|rerun, start| {
				rerun.set_time_seconds("odom", start.elapsed().as_secs_f64());

				timeseries(
					rerun,
					"odom/side_vel/x",
					util::get_vel(&self.side_vels.0).value,
				);
				timeseries(
					rerun,
					"odom/side_vel/y",
					util::get_vel(&self.side_vels.1).value,
				);

				// encoders
				timeseries_colour(rerun, "odom/encoders/left", dist_l.value, [255, 0, 0]);
				timeseries_colour(rerun, "odom/encoders/right", dist_r.value, [0, 255, 0]);

				// encoders (raw)
				let (dist_l, dist_r) = drive
					.get_encoders_raw()
					.unwrap_or((meter!(0.0), meter!(0.0)));
				timeseries_colour(rerun, "odom/raw/encoders/left", dist_l.value, [127, 0, 0]);
				timeseries_colour(rerun, "odom/raw/encoders/right", dist_r.value, [0, 127, 0]);

				// pos
				timeseries_colour(rerun, "odom/pos/x", self.pos.0.value, [127, 0, 127]);
				timeseries_colour(rerun, "odom/pos/y", self.pos.1.value, [0, 127, 127]);
				timeseries_colour(
					rerun,
					"odom/angle",
					self.angle.value * 180.0 / PI,
					[127, 127, 127],
				);
			});
		}
	}
}

impl Odometry for DriveOdom {
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
		(
			util::get_vel(&self.side_vels.0),
			util::get_vel(&self.side_vels.1),
		)
	}
}

pub struct DriveImuOdom {
	pos: (Length, Length),
	angles: util::RingBuf<Angle, 4>,
	vel: (Velocity, Velocity),
	side_vel: (util::RingBuf<Velocity, 5>, util::RingBuf<Velocity, 5>),

	headings: util::RingBuf<f32, 4>,
	last_difference: Angle,
	dist: (Length, Length),
	update: Instant,

	imu: Bno055<I2c>,
	logger: RerunLogger,
}

impl DriveImuOdom {
	pub fn new(logger: RerunLogger, imu: Bno055<I2c>) -> Self {
		Self {
			pos: (ConstZero::ZERO, ConstZero::ZERO),
			angles: (&[ConstZero::ZERO][..]).into(),
			vel: (ConstZero::ZERO, ConstZero::ZERO),
			side_vel: (util::RingBuf::new(), util::RingBuf::new()),
			last_difference: ConstZero::ZERO,
			headings: (&[0.0][..]).into(),
			dist: (ConstZero::ZERO, ConstZero::ZERO),
			update: Instant::now(),
			imu,
			logger,
		}
	}

	fn get_angle_difference(&mut self) -> Option<Angle> {
		let raw_heading = match self.imu.euler_angles() {
			Ok(angles) => angles.c, // the yaw/heading
			Err(err) => {
				log::warn!("Failed to get Euler angles from IMU: {:?}", err);
				return None;
			}
		};
		let last_heading = *self.headings.back().unwrap();

		// Check if out of bounds???
		if raw_heading < 0.0 || raw_heading > 368.00 {
			return None;
		}

		// Check if we likely wrapped around
		let diff = degree!(if (raw_heading - last_heading).abs() > 160.0 {
			if raw_heading < last_heading {
				(raw_heading + 360.0) - last_heading
			} else {
				(raw_heading - 360.0) - last_heading
			}
		} else {
			raw_heading - last_heading
		} as _);
		self.headings.push(raw_heading);
		self.last_difference = diff;

		// self.logger.with(|rerun, _| {
		// 	use crate::logging::*;
		// 	rerun.set_time_seconds("", start.elapsed().as_secs_f64());
		// 	timeseries(rerun, "heading", raw_heading as _);
		// 	timeseries(rerun, "diff", diff as _);
		// });

		Some(diff)
	}

	pub fn update(&mut self, drive: &mut Drive) {
		if self.update.elapsed() < Duration::from_micros(15_000) {
			return;
		}

		// get displacement and change in displacement
		// relative to the sides on the robot
		let (dist_l, dist_r) = match drive.get_encoders() {
			Some(d) => d,
			None => return,
		};
		let (diff_l, diff_r) = (dist_l - self.dist.0, dist_r - self.dist.1);

		let time = second!(self.update.elapsed().as_secs_f64());
		self.side_vel.0.push(diff_l / time);
		self.side_vel.1.push(diff_r / time);

		// calculate displacement and change in angle for center of the robot
		let diff_dist = 0.5 * (diff_l + diff_r);
		let diff_angle = match self.get_angle_difference() {
			Some(d) => d,
			None => self.last_difference,
		};

		// calculate displacement in global space
		let tmp = util::get_angle(&self.angles) + 0.5 * diff_angle;
		let (dx, dy) = tmp.sin_cos();
		let (dx, dy) = (diff_dist * dx, diff_dist * dy);

		// apply global displacement and rotation
		self.pos.0 -= dx;
		self.pos.1 += dy;
		self.angles.push(*self.angles.back().unwrap() + diff_angle);

		// update state and calculate running averages
		self.dist = (dist_l, dist_r);
		self.update = Instant::now();

		self.vel = (dx / time, dy / time);

		self.logger.with(|rerun, start| {
			rerun.set_time_seconds("odom", start.elapsed().as_secs_f64());

			// timeseries(rerun, "odom/side_vel/x", get_vel(&self.side_vels.0).value);
			// timeseries(rerun, "odom/side_vel/y", get_vel(&self.side_vels.1).value);

			// encoders
			// timeseries_colour(rerun, "odom/encoders/left", dist_l.value, [255, 0, 0]);
			// timeseries_colour(rerun, "odom/encoders/right", dist_r.value, [0, 255, 0]);

			// encoders (raw)
			// let (dist_l, dist_r) = drive
			// .get_encoders_raw()
			// .unwrap_or((meter!(0.0), meter!(0.0)));
			// timeseries_colour(rerun, "odom/raw/encoders/left", dist_l.value, [127, 0, 0]);
			// timeseries_colour(rerun, "odom/raw/encoders/right", dist_r.value, [0, 127, 0]);

			// pos
			timeseries_colour(rerun, "odom/pos/x", self.pos.0.value, [127, 0, 127]);
			timeseries_colour(rerun, "odom/pos/y", self.pos.1.value, [0, 127, 127]);
			timeseries_colour(
				rerun,
				"odom/angle",
				self.angles.back().unwrap().value.to_degrees(),
				[127, 127, 127],
			);
		});
	}
}

impl Odometry for DriveImuOdom {
	fn pos(&self) -> (Length, Length) {
		self.pos
	}

	fn angle(&self) -> Angle {
		util::get_angle(&self.angles)
	}

	fn vel(&self) -> (Velocity, Velocity) {
		self.vel
	}

	fn side_vel(&self) -> (Velocity, Velocity) {
		(
			util::get_vel(&self.side_vel.0),
			util::get_vel(&self.side_vel.1),
		)
	}
}

mod util {
	use crate::units::*;
	use ringbuffer::{ConstGenericRingBuffer, RingBuffer};
	use uom::ConstZero;

	pub type RingBuf<T, const N: usize> = ConstGenericRingBuffer<T, N>;

	pub fn get_vel<const N: usize>(rb: &RingBuf<Velocity, N>) -> Velocity {
		if rb.len() == 0 {
			return ConstZero::ZERO;
		}
		rb.iter().copied().reduce(|a, b| a + b).unwrap() / rb.len() as f64
	}

	pub fn get_angle<const N: usize>(rb: &RingBuf<Angle, N>) -> Angle {
		if rb.len() == 0 {
			return ConstZero::ZERO;
		}
		rb.iter().copied().reduce(|a, b| a + b).unwrap() / rb.len() as f64
	}
}
