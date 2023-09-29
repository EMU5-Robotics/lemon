use crate::Drive;
use protocol::StatusPkt;
use std::time::Instant;

use crate::units::*;

use uom::ConstZero;

pub struct DriveOdom {
	last_dist: (Length, Length),
	angle: Angle,
	dw: Length,
	pos: (Length, Length),
	last_update: Instant,
	last_vel: (Velocity, Velocity),
}

impl DriveOdom {
	pub fn new(dw: Length) -> Self {
		Self {
			last_dist: (ConstZero::ZERO, ConstZero::ZERO),
			dw,
			angle: ConstZero::ZERO,
			pos: (ConstZero::ZERO, ConstZero::ZERO),
			last_update: Instant::now(),
			last_vel: (ConstZero::ZERO, ConstZero::ZERO),
		}
	}
	pub fn update(&mut self, pkt: &StatusPkt, drive: &mut Drive) -> Option<(Velocity, Velocity)> {
		// get displacement and change in displacement
		// relative to the sides on the robot
		let (dist_l, dist_r) = drive.side_encoders(pkt)?;
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
		let diff_vel = (vel.0 - self.last_vel.0, vel.1 - self.last_vel.1);

		let accel = (diff_vel.0 / time, diff_vel.1 / time);

		self.last_vel = vel;
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

		Some(vel)
	}
}
