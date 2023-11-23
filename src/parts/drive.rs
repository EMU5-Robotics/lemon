use std::{iter, time::Instant};

use crate::{
	state::{Motor, RerunLogger},
	units::*,
};

pub struct Drive {
	pub left: [Motor; 3],
	right: [Motor; 3],
	turn_rate: f32,
	last_encoder: (Length, Length),
	// smooth between last 5 values
	// encoders: [(Length, Length, Instant); 5],
	last_update: Instant,
	logger: RerunLogger,
}

const MAX_MILLIVOLT: f32 = 12_000.0;

impl Drive {
	pub fn new(logger: RerunLogger, left: [Motor; 3], right: [Motor; 3], turn_rate: f32) -> Self {
		if !(0.0..1.0).contains(&turn_rate) {
			panic!("Invalid turn rate");
		}

		Self {
			left,
			right,
			turn_rate,
			last_encoder: (meter!(0.0), meter!(0.0)),
			last_update: Instant::now(),
			logger,
		}
	}

	pub fn drive(&self, fpower: f32, tpower: f32) {
		let lpower = ((fpower + self.turn_rate * tpower).clamp(-1.0, 1.0) * MAX_MILLIVOLT) as i16;
		let rpower = ((fpower - self.turn_rate * tpower).clamp(-1.0, 1.0) * MAX_MILLIVOLT) as i16;

		for (lmotor, rmotor) in iter::zip(&self.left, &self.right) {
			lmotor.voltage(lpower);
			rmotor.voltage(rpower);
		}
	}

	fn to_rpm(vel: Velocity) -> f64 {
		let circumference = std::f64::consts::TAU * meter!(0.04195);
		let time_scale = second!(60.0) / second!(1.0);
		let angular_vel: AngularVelocity = (time_scale * vel / circumference).into();
		angular_vel.value
	}

	pub fn set_velocity(&self, left: Velocity, right: Velocity) {
		let lvel = Self::to_rpm(left);
		let rvel = Self::to_rpm(right);

		for (lmotor, rmotor) in iter::zip(&self.left, &self.right) {
			lmotor.velocity((lvel as i16).clamp(-200, 200));
			rmotor.velocity((rvel as i16).clamp(-200, 200));
		}

		self.logger.with(|rerun, start| {
			use crate::logging::*;
			rerun.set_time_seconds("", start.elapsed().as_secs_f64());
			timeseries(rerun, "target", left.value);
			timeseries(rerun, "target_rpm", lvel);
		});
	}

	pub fn get_encoders(&mut self) -> Option<(Length, Length)> {
		let smooth_window: Time = millisecond!(40.0);

		let (mut l, mut r) = self.get_encoders_raw()?;

		// smooth encoder values to avoid aliasing issues
		let elapsed = second!(self.last_update.elapsed().as_secs_f64());
		let t = (elapsed / smooth_window).value;

		if t < 1.0 {
			l = (1.0 - t) * self.last_encoder.0 + t * l;
			r = (1.0 - t) * self.last_encoder.1 + t * r;
		}

		self.last_update = Instant::now();
		self.last_encoder = (l, r);

		Some((l, r))
	}

	pub fn get_encoders_raw(&self) -> Option<(Length, Length)> {
		// Check the motors are connected for us to read the encoder values
		if !self.left[0].is_connected() || !self.right[0].is_connected() {
			return None;
		}

		let l_rev = match self.left[0].is_reversed() {
			true => -1.0,
			false => 1.0,
		};
		let r_rev = match self.right[0].is_reversed() {
			true => -1.0,
			false => 1.0,
		};

		const MULTIPLIER: f64 = 1.0 / 340000.0;
		Some((
			meter!(self.left[0].position() as f64 * l_rev * MULTIPLIER),
			meter!(self.right[0].position() as f64 * r_rev * MULTIPLIER),
		))
	}
}
