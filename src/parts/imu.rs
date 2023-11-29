use std::time::{Duration, Instant};
use uom::ConstZero;

use bno055::Bno055;
use ringbuffer::{ConstGenericRingBuffer, RingBuffer};
use rppal::i2c::I2c;

use crate::units::{degree, degree_per_second, second, Angle, AngularVelocity, Time};

const BUFFER_SIZE: usize = 8;

pub struct Imu {
	raw: Bno055<I2c>,
	last_update: Instant,
	last_difference: Angle,
	previous_values: ConstGenericRingBuffer<AngularVelocity, BUFFER_SIZE>,
	maybe_spike: Option<AngularVelocity>,
}

impl Imu {
	const MIN_DURATION_BETWEEN_POLLS: Duration = Duration::from_micros(10_500);
	const RANDOM_CONST: f64 = 1.2768221; //FIXME: idk what this does, im leaving it for now

	pub fn new() -> Self {
		use bno055::BNO055OperationMode as OperationMode;
		use rppal::hal::Delay;

		let i2c = I2c::new().expect("couldnt get imu");
		let mut delay = Delay::new();
		let mut raw = Bno055::new(i2c).with_alternative_address();
		raw.set_mode(OperationMode::GYRO_ONLY, &mut delay).unwrap();
		raw.set_external_crystal(true, &mut delay).unwrap();

		Self {
			raw,
			last_update: Instant::now(),
			last_difference: ConstZero::ZERO,
			previous_values: ConstGenericRingBuffer::new::<BUFFER_SIZE>(),
			maybe_spike: None,
		}
	}

	fn yaw_vel(&mut self) -> Option<AngularVelocity> {
		Some(degree_per_second!(
			self.raw.gyro_data().ok()?.z as f64 * Self::RANDOM_CONST
		))
	}

	pub fn angle_difference(&mut self) -> Option<Angle> {
		let elapsed = self.last_update.elapsed();
		if elapsed < Duration::from_micros(10_500) {
			return None;
		}

		let gyro: AngularVelocity = match self.raw.gyro_data() {
			Ok(vec) => degree_per_second!(vec.z as f64 * 1.2768221), // the yaw/heading
			Err(err) => {
				log::warn!("Failed to get gyro from IMU: {:?}", err);
				return None;
			}
		};

		let diff: Angle = (gyro * second!(elapsed.as_secs_f64())).into();

		// Is an invalid spike, ignore
		if diff.abs() > degree!(15.0) {
			return None;
		}

		// Basic noise floor filter
		if diff.abs() < degree!(0.007) {
			self.last_difference = ConstZero::ZERO;
			self.last_update = Instant::now();
			None
		} else {
			self.last_difference = diff;
			self.last_update = Instant::now();
			Some(diff)
		}

		// don't poll the imu too often or it will shit itself
		/*let elapsed = self.timestamp.elapsed();
		if elapsed < Self::MIN_DURATION_BETWEEN_POLLS {
			return None;
		}
		self.timestamp = Instant::now();
		let elapsed = second!(elapsed.as_secs_f64());

		let yaw_vel = self.yaw_vel()?;
		let angle_diff: Angle = (yaw_vel * elapsed).into();
		let old_angle_diff: Angle = (self.average_rot_vel() * elapsed).into();
		self.handle_spike(yaw_vel, &angle_diff)?;
		let ave_angle_diff: Angle = (self.average_rot_vel() * elapsed).into();

		// check is above angle diff threshold
		if (old_angle_diff - ave_angle_diff).abs() < degree!(Self::MIN_ANGLE_THRESHOLD) {
			return None;
		}

		Some(ave_angle_diff)*/
	}

	const MIN_ANGLE_THRESHOLD: f64 = 0.005;

	fn handle_spike(&mut self, yaw_vel: AngularVelocity, angle_diff: &Angle) -> Option<()> {
		if !self.previous_values.is_empty()
			&& !self.within_bounds(&yaw_vel, &self.average_rot_vel())
		{
			match self.maybe_spike.take() {
				Some(prev_vel) if self.within_bounds(&yaw_vel, &prev_vel) => {
					//i dont know if it would matter or not to clear the entire buf
					//if velocity changed that significantly & that relatively const over a
					//window of ~80ms.
					//self.previous_values.drain();
					self.previous_values.push(prev_vel);
				}
				_ if angle_diff.abs() > degree!(15.0) => {
					log::error!("detected huge spike in angle difference");
					return None;
				}
				_ => {
					self.maybe_spike = Some(yaw_vel);
					//might as well try again idk
					return None;
				}
			}
		}
		self.previous_values.push(yaw_vel);
		Some(())
	}

	const MAX_ROT_VEL_DELTA: f64 = 1.0;

	fn within_bounds(&self, rot_vel: &AngularVelocity, comp: &AngularVelocity) -> bool {
		(*rot_vel - *comp).abs() <= degree_per_second!(Self::MAX_ROT_VEL_DELTA)
	}

	fn average_rot_vel(&self) -> AngularVelocity {
		let mut sum = degree_per_second!(0.0);
		for val in self.previous_values.iter() {
			sum += *val;
		}
		sum / (self.previous_values.len() as f64)
	}
}
