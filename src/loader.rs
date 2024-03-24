use crate::brain::Brain;
use crate::motor;
use crate::motor::Motor;

pub struct Loader {
	motors: [(Motor, bool); 2],
}

impl Loader {
	pub fn new(motors: [(u8, bool); 2], brain: &Brain) -> Self {
		Self {
			motors: motors.map(|e| (brain.get_motor(e.0), e.1)),
		}
	}
	pub fn set_side_percent_voltage(&mut self, percent: f64) {
		if percent.abs() > 1.0 {
			log::warn!("Loader::set_side_percent_voltage recieved values outside of [-1, 1]: {percent}. Values will be clamped");
		}

		for (motor, rev) in &mut self.motors {
			let v = if *rev { -percent } else { percent };
			motor.set_target(motor::Target::PercentVoltage(v));
		}
	}
}
