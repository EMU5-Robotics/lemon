use std::{iter, time::Instant};

use crate::{state::Motor, units::*};

pub struct Drive {
	left: [Motor; 3],
	right: [Motor; 3],
	turn_rate: f32,
	last_encoder: (Length, Length),
	// smooth between last 5 values
	// encoders: [(Length, Length, Instant); 5],
	last_update: Instant,
}

const MAX_MILLIVOLT: f32 = 4000.0;

impl Drive {
	pub fn new(left: [Motor; 3], right: [Motor; 3], turn_rate: f32) -> Self {
		if !(0.0..1.0).contains(&turn_rate) {
			panic!("Invalid turn rate");
		}

		Self {
			left,
			right,
			turn_rate,
			last_encoder: (meter!(0.0), meter!(0.0)),
			last_update: Instant::now(),
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

		const MULTIPLIER: f64 = 1.0 / 150000.0;
		Some((
			meter!(self.left[0].position() as f64 * MULTIPLIER),
			meter!(self.right[0].position() as f64 * MULTIPLIER),
		))
	}
}
