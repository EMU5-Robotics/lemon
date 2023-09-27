use crate::Drive;
use protocol::StatusPkt;
use std::f64::consts::PI;
use std::time::Instant;

use crate::units::*;

pub struct DriveOdom<'a> {
	last_dist: (Length, Length),
	angle: Angle,
	dw: Length,
	drive: &'a Drive,
	pos: (Length, Length),
	last_update: Instant,
}

impl<'a> DriveOdom<'a> {
	pub fn new(dw: Length, drive: &'a Drive) -> Self {
		Self {
			last_dist: (meter!(0.0), meter!(0.0)),
			dw,
			drive,
			angle: radian!(0.0),
			pos: (meter!(0.0), meter!(0.0)),
			last_update: Instant::now(),
		}
	}
	pub fn update(&mut self, pkt: &StatusPkt) -> Option<(Velocity, Velocity)> {
		// get displacement and change in displacement
		// relative to the sides on the robot
		let (dist_l, dist_r) = self.drive.side_encoders(pkt)?;
		let (diff_l, diff_r) = (dist_l - self.last_dist.0, dist_r - self.last_dist.1);

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
		let time = second!(self.last_update.elapsed().as_secs_f64());
		self.last_dist = (dist_l, dist_r);
		self.last_update = Instant::now();

		let vel = (dx / time, dy / time);

		log::info!(
			"pos: ({:.2}, {:.2}) | angle: ({:.2}) | vel: ({:.2}, {:.2}) | {:.2}ms",
			self.pos.0.value,
			self.pos.1.value,
			self.angle.value * 180.0 / PI,
			vel.0.value,
			vel.1.value,
			time.value * 1000.0
		);

		Some(vel)
	}
}
