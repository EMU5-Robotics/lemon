use crate::Drive;
use protocol::StatusPkt;
use std::time::Instant;

use crate::units::*;

use uom::ConstZero;

pub struct DriveOdom {
	dist: (Length, Length),
	angle: Angle,
	dw: Length,
	pos: (Length, Length),
	update: Instant,
	vel: (Velocity, Velocity),
	accel: (Acceleration, Acceleration),
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
			accel: (ConstZero::ZERO, ConstZero::ZERO),
		}
	}
	pub fn update(&mut self, pkt: &StatusPkt, drive: &mut Drive) -> Option<()> {
		// get displacement and change in displacement
		// relative to the sides on the robot
		let (dist_l, dist_r) = drive.side_encoders(pkt)?;
		let (diff_l, diff_r) = (dist_l - self.dist.0, dist_r - self.dist.1);

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
		let time = second!(self.update.elapsed().as_secs_f64());
		self.dist = (dist_l, dist_r);
		self.update = Instant::now();

		let vel = (dx / time, dy / time);
		let diff_vel = (vel.0 - self.vel.0, vel.1 - self.vel.1);

		let accel = (diff_vel.0 / time, diff_vel.1 / time);
		self.accel = accel;

		self.vel = vel;
		use uom::{
			fmt::DisplayStyle::Abbreviation,
			si::{
				acceleration::meter_per_second_squared, angle::degree, length::meter,
				time::millisecond, velocity::meter_per_second,
			},
		};

		log::info!(
			"pos: ({:+.2}, {:+.2}) | angle: ({:+.2}) | vel: ({:+.2}, {:+.2}) | diff_vel: ({:+.2}, {:+.2}) | accel: ({:+.2}, {:+.2}) | {:.2}",
			self.pos.0.into_format_args(meter, Abbreviation),
			self.pos.1.into_format_args(meter, Abbreviation),
			self.angle.into_format_args(degree, Abbreviation),
			vel.0.into_format_args(meter_per_second, Abbreviation),
			vel.1.into_format_args(meter_per_second, Abbreviation),
			diff_vel.0.into_format_args(meter_per_second, Abbreviation),
			diff_vel.1.into_format_args(meter_per_second, Abbreviation),
                        accel.0.into_format_args(meter_per_second_squared, Abbreviation),
                        accel.1.into_format_args(meter_per_second_squared, Abbreviation),
			time.into_format_args(millisecond, Abbreviation),
		);
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
	pub fn accel(&self) -> (Acceleration, Acceleration) {
		self.accel
	}
}
