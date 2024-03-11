use std::time::Instant;

pub struct PID {
	kp: f64,
	ki: f64,
	kd: f64,
	target: f64,
	ki_integral: f64,
	last_error: f64,
	last_update: Instant,
	first_update: bool,
}

impl PID {
	pub fn new(kp: f64, ki: f64, kd: f64) -> Self {
		Self {
			kp,
			ki,
			kd,
			target: 0.0,
			ki_integral: 0.0,
			last_error: 0.0,
			last_update: Instant::now(),
			first_update: true,
		}
	}
	pub fn set_target(&mut self, target: f64) {
		self.target = target;
	}
	pub fn poll(&mut self, pv: f64) -> f64 {
		let now = Instant::now();
		let diff_t = now.duration_since(self.last_update).as_secs_f64();

		let error = self.target - pv;
		// clegg integration (avoid integral windup)
		// see (wikipedia.org/wiki/Integral_windup)
		if self.last_error.signum() != error.signum() {
			self.ki_integral = 0.0;
		}

		// bumpless operation see (wikipedia.org/wiki/Proportional-integral-derivative_controller#Bumpless_operation)
		self.ki_integral += self.ki * error * diff_t;

		let output = self.kp * error + self.ki_integral + self.kd * (error - self.last_error);

		self.last_error = error;
		self.last_update = now;

		output
	}
}
