use crate::Drive;
use protocol::StatusPkt;
use std::time::{Duration, Instant};

use crate::units::*;
use rerun::{
	archetypes::TimeSeriesScalar, components::Position2D, Arrows3D, Points2D, Points3D, Position3D,
	Vector3D,
};

use uom::ConstZero;

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
}

impl DriveOdom {
	pub fn new(dw: Length) -> Self {
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
		}
	}
	pub fn update(&mut self, pkt: &StatusPkt, drive: &mut Drive) -> Option<()> {
		// get displacement and change in displacement
		// relative to the sides on the robot
		let (dist_l, dist_r) = drive.side_encoders(pkt)?;
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
		if self.last_log.elapsed() > Duration::from_millis(1) {
			self.last_log = Instant::now();
			crate::RERUN_REC.set_time_seconds("odom", crate::PROGRAM_START.elapsed().as_secs_f64());
			crate::RERUN_REC
				.log("odom/vel/x", &TimeSeriesScalar::new(self.vel.0.value))
				.unwrap();
			crate::RERUN_REC
				.log("odom/vel/y", &TimeSeriesScalar::new(self.vel.1.value))
				.unwrap();
			crate::RERUN_REC
				.log(
					"odom/encoders/left",
					&TimeSeriesScalar::new(dist_l.value).with_color([255, 0, 0]),
				)
				.unwrap();
			crate::RERUN_REC
				.log(
					"odom/encoders/right",
					&TimeSeriesScalar::new(dist_r.value).with_color([0, 255, 0]),
				)
				.unwrap();
			let (dist_l, dist_r) = drive.raw_side_encoders(&pkt)?;
			crate::RERUN_REC
				.log(
					"odom/raw/encoders/left",
					&TimeSeriesScalar::new(dist_l.value).with_color([127, 0, 0]),
				)
				.unwrap();
			crate::RERUN_REC
				.log(
					"odom/raw/encoders/right",
					&TimeSeriesScalar::new(dist_r.value).with_color([0, 127, 0]),
				)
				.unwrap();
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
