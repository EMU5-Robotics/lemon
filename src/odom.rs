use crate::Drive;
use protocol::StatusPkt;
use std::f64::consts::PI;
use std::time::Instant;
use uom::{
	si::f64::*,
	si::{angle::radian, length::meter, time::second},
};

pub struct DriveOdom<'a> {
	last_encoder: (Length, Length),
	theta: Angle,
	dw: Length,
	drive: &'a Drive,
	pos: (Length, Length),
	last_update: Instant,
}

impl<'a> DriveOdom<'a> {
	pub fn new(dw: Length, drive: &'a Drive) -> Self {
		let zero = Length::new::<meter>(0.0);
		Self {
			last_encoder: (Length::new::<meter>(0.0), Length::new::<meter>(0.0)),
			dw,
			drive,
			theta: Angle::new::<radian>(0.0),
			pos: (zero, zero),
			last_update: Instant::now(),
		}
	}
	pub fn update(&mut self, pkt: &mut StatusPkt) -> Option<(Velocity, Velocity)> {
		let (adl, adr) = self.drive.side_encoders(pkt)?;
		let (dl, dr) = (adl - self.last_encoder.0, adr - self.last_encoder.1);
		let d = 0.5 * (dl + dr);
		let dtheta: Angle = (0.5 * (dr - dl) / self.dw).into();
		let tmp: f64 = (self.theta.value + 0.5 * dtheta.value).into();
		let (dx, dy) = (
			Length::new::<meter>(d.value * tmp.cos()),
			Length::new::<meter>(d.value * tmp.sin()),
		);
		self.pos.0 += dx;
		self.pos.1 += dy;
		self.theta += dtheta;

		self.last_encoder = (adl, adr);

		let time = Time::new::<second>(self.last_update.elapsed().as_secs_f64());
		self.last_update = Instant::now();
		let vel = (dx / time, dy / time);

		log::info!(
			"pos: ({:.2}, {:.2}) | angle: ({:.2}) | vel: ({:.2}, {:.2})",
			self.pos.0.value,
			self.pos.1.value,
			self.theta.value * 180.0 / PI,
			vel.0.value,
			vel.1.value,
		);
		Some(vel)
	}
}
