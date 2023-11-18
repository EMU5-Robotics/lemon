use std::{
	f64::consts::PI,
	time::{Duration, Instant},
};

use rerun::archetypes::TimeSeriesScalar;
use uom::ConstZero;

use crate::{parts::drive::Drive, state::RerunLogger, units::*};

pub struct DriveOdom {
	dist: (Length, Length),
	angle: Angle,
	dw: Length,
	pos: (Length, Length),
	update: Instant,
	vel: (Velocity, Velocity),
	side_vel: (Velocity, Velocity),
	accel: (Acceleration, Acceleration),
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
			side_vel: (ConstZero::ZERO, ConstZero::ZERO),
			accel: (ConstZero::ZERO, ConstZero::ZERO),
			last_log: Instant::now(),
			logger,
		}
	}

	pub fn update(&mut self, drive: &mut Drive) -> Option<()> {
		// get displacement and change in displacement
		// relative to the sides on the robot
		let (dist_l, dist_r) = drive.get_encoders()?;
		let (diff_l, diff_r) = (dist_l - self.dist.0, dist_r - self.dist.1);

		let time = second!(self.update.elapsed().as_secs_f64());
		self.side_vel = (diff_l / time, diff_r / time);

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

		let vel = (dx / time, dy / time);
		let diff_vel = (vel.0 - self.vel.0, vel.1 - self.vel.1);

		let accel = (diff_vel.0 / time, diff_vel.1 / time);
		self.accel = accel;

		self.vel = vel;
		if self.last_log.elapsed() > Duration::from_millis(0) {
			self.last_log = Instant::now();
			self.logger.with(|rerun, start| {
				rerun.set_time_seconds("odom", start.elapsed().as_secs_f64());
				rerun
					.log("odom/diff/l", &TimeSeriesScalar::new(diff_l.value))
					.unwrap();
				rerun
					.log("odom/diff/r", &TimeSeriesScalar::new(diff_r.value))
					.unwrap();
				rerun
					.log("odom/diff/angle", &TimeSeriesScalar::new(diff_angle.value))
					.unwrap();
				rerun
					.log("odom/diff/dist", &TimeSeriesScalar::new(diff_dist.value))
					.unwrap();
				rerun
					.log("odom/vel/x", &TimeSeriesScalar::new(self.vel.0.value))
					.unwrap();
				rerun
					.log("odom/vel/y", &TimeSeriesScalar::new(self.vel.1.value))
					.unwrap();
				rerun
					.log(
						"odom/side_vel/x",
						&TimeSeriesScalar::new(self.side_vel.0.value),
					)
					.unwrap();
				rerun
					.log(
						"odom/side_vel/y",
						&TimeSeriesScalar::new(self.side_vel.1.value),
					)
					.unwrap();
				rerun
					.log(
						"odom/encoders/left",
						&TimeSeriesScalar::new(dist_l.value).with_color([255, 0, 0]),
					)
					.unwrap();
				rerun
					.log(
						"odom/encoders/right",
						&TimeSeriesScalar::new(dist_r.value).with_color([0, 255, 0]),
					)
					.unwrap();
				rerun
					.log(
						"odom/encoders/delta",
						&TimeSeriesScalar::new(dist_r.value - dist_l.value).with_color([0, 0, 255]),
					)
					.unwrap();

				let (dist_l, dist_r) = drive
					.get_encoders_raw()
					.unwrap_or((meter!(0.0), meter!(0.0)));
				rerun
					.log(
						"odom/raw/encoders/left",
						&TimeSeriesScalar::new(dist_l.value).with_color([127, 0, 0]),
					)
					.unwrap();
				rerun
					.log(
						"odom/raw/encoders/right",
						&TimeSeriesScalar::new(dist_r.value).with_color([0, 127, 0]),
					)
					.unwrap();
				rerun
					.log(
						"odom/raw/encoders/delta",
						&TimeSeriesScalar::new(dist_r.value - dist_l.value).with_color([0, 0, 127]),
					)
					.unwrap();

				// position
				rerun
					.log(
						"odom/pos/x",
						&TimeSeriesScalar::new(self.pos.0.value).with_color([127, 0, 127]),
					)
					.unwrap();
				rerun
					.log(
						"odom/pos/y",
						&TimeSeriesScalar::new(self.pos.1.value).with_color([0, 127, 127]),
					)
					.unwrap();
				rerun
					.log(
						"odom/angle",
						&TimeSeriesScalar::new(self.angle.value * 180.0 / PI)
							.with_color([127, 127, 127]),
					)
					.unwrap();
			});
		}
		Some(())
	}

	pub fn pos(&self) -> (Length, Length) {
		self.pos
	}

	pub fn angle(&self) -> Angle {
		self.angle
	}

	pub fn vel(&self) -> (Velocity, Velocity) {
		self.vel
	}

	pub fn side_vel(&self) -> (Velocity, Velocity) {
		self.side_vel
	}

	pub fn accel(&self) -> (Acceleration, Acceleration) {
		self.accel
	}
}
