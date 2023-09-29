use std::{marker::PhantomData, time::Instant};
use uom::{si::*, ConstZero, ConversionFactor};

pub struct Pid<T: Dimension + ?Sized, O: Dimension + ?Sized> {
	kp: f64,
	ki: f64,
	kd: f64,
	target: Quantity<T, SI<f64>, f64>,
	integral: f64,
	prev_err: f64,
	time: Instant,
	output: PhantomData<O>,
}

impl<T: Dimension + ?Sized, O: Dimension + ?Sized> Pid<T, O> {
	pub fn new(kp: f64, ki: f64, kd: f64, target: Quantity<T, SI<f64>, f64>) -> Self {
		Self {
			kp,
			ki,
			kd,
			target,
			integral: ConstZero::ZERO,
			prev_err: ConstZero::ZERO,
			time: Instant::now(),
			output: PhantomData,
		}
	}
	pub fn step(&mut self, pv: Quantity<T, SI<f64>, f64>) -> Quantity<O, SI<f64>, f64> {
		let pv = pv.value;

		let dt = self.time.duration_since(self.time).as_secs_f64();
		self.time = Instant::now();

		let err = self.target.value - pv;

		let prop_term = self.kp * err;

		// modifications to integration are done to prevent integral windup
		// see https://en.wikipedia.org/wiki/Integral_windup

		// conditional integration
		if err.abs().value() < self.ki.value() {
			self.integral += self.ki * err * dt;
		}
		// clegg integrator
		if (self.prev_err * err).value() < 0.0 {
			self.integral = ConstZero::ZERO;
		}

		let deriv_term = self.kd * (err - self.prev_err) / dt;
		Quantity {
			dimension: PhantomData,
			units: PhantomData,
			value: prop_term + self.integral.value() + deriv_term,
		}
	}
}

pub type AnglePid = Pid<angle::Dimension, ratio::Dimension>;
pub type AngularVelocityPid = Pid<angular_velocity::Dimension, ratio::Dimension>;
pub type VelocityPid = Pid<velocity::Dimension, power::Dimension>;
