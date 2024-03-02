use std::time::Instant;

use rppal::i2c::I2c;

const BIAS: f64 = -0.02714403555;
const MAX_ANGULAR_VEL: f64 = 1000.0;
const MAX_ANGULAR_VEL_CODE: u8 = match MAX_ANGULAR_VEL {
	v if v == 2000.0 => 0x00,
	v if v == 1000.0 => 0x01,
	v if v == 500.0 => 0x02,
	v if v == 250.0 => 0x03,
	v if v == 125.0 => 0x04,
	_ => panic!("Invalid max angular velocity."),
};
const ANGULAR_SCALE: f64 = MAX_ANGULAR_VEL / i16::MAX as f64;

pub struct Bmi088 {
	pub i2c: I2c,
	last_read: Instant,
	last_angular_vel_z: f64,
	heading: f64,
}

impl Bmi088 {
	pub fn new() -> Self {
		let mut i2c = I2c::new().unwrap();
		log::info!("IMU clock speed: {:?}", i2c.clock_speed());

		// gyroscope address
		i2c.set_slave_address(0x68u16).unwrap();
		i2c.write(&[0x0F, MAX_ANGULAR_VEL_CODE]).unwrap();
		// set filtering
		i2c.write(&[0x10, 0x02]).unwrap();
		// read vel_z
		let mut buf = [0u8; 2];
		i2c.write_read(&[0x6u8], &mut buf).unwrap();
		let last_angular_vel_z = i16::from_le_bytes(buf) as f64 * ANGULAR_SCALE;

		Self {
			i2c,
			last_read: Instant::now(),
			last_angular_vel_z,
			heading: 0.0,
		}
	}
	fn read_vel_z(&mut self) -> f64 {
		let mut buf = [0u8; 2];
		match self.i2c.write_read(&[0x6u8], &mut buf) {
			Ok(()) => i16::from_le_bytes(buf) as f64 / (1.0 * 32.768) + BIAS,
			Err(e) => {
				log::warn!("imu read failed: {e}");
				self.last_angular_vel_z
			}
		}
	}
	pub fn heading(&self) -> f64 {
		self.heading
	}
	pub fn calc_heading(&mut self) -> f64 {
		let new_angular_vel_z = self.read_vel_z();
		let now = Instant::now();
		let dt = now.duration_since(self.last_read).as_secs_f64();
		self.heading += new_angular_vel_z * dt;
		self.last_angular_vel_z = new_angular_vel_z;
		self.last_read = now;
		self.heading
	}
	pub fn reset(&mut self) {
		self.last_read = Instant::now();
		self.heading = 0.0;
	}
}
