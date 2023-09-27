use bno055::{BNO055OperationMode, Bno055};
use embedded_hal::blocking::delay::{DelayMs, DelayUs};
use rppal::{
	hal::Delay,
	i2c::{Error as I2cError, I2c},
	system::DeviceInfo,
};

use std::time::{Duration, Instant};

use nalgebra::{Quaternion, UnitQuaternion, Vector3};

fn main() -> Result<(), Box<dyn std::error::Error>> {
	// Print raspi info
	let dev_info = DeviceInfo::new()?;
	println!("Model: {:?}, SOC: {:?}", dev_info.model(), dev_info.soc());

	// Open up the default I2C bus
	let i2c = I2c::new()?;
	println!("Opened I2C bus with ID: {}", i2c.bus());
	println!(
		"Bus has the following capabilites:\n{:#?}",
		i2c.capabilities()
	);

	// Delay object for sleeping
	let mut delay = Delay::new();

	// Create a new BNO055 IMU
	let mut imu = Imu::init(Bno055::new(i2c).with_alternative_address(), &mut delay)?;

	loop {
		let start = Instant::now();

		imu.process()?;
		let pos = imu.get_position();
		println!("{:>9.6},{:>9.6},{:>9.6}", pos.x, pos.y, pos.z);

		let end = Instant::now();
		let delta = Duration::from_millis(10).saturating_sub(end - start);
		delay.delay_us(delta.as_micros() as u32);
	}
}

struct Imu {
	pub imu: Bno055<I2c>,
	vel: Vector3<f64>,
	pos: Vector3<f64>,
	last: Instant,

	pub accel_constant_bias: Vector3<f64>,
}

type ImuResult<T> = Result<T, bno055::Error<I2cError>>;

impl Imu {
	pub fn init(mut imu: Bno055<I2c>, delay: &mut Delay) -> ImuResult<Self> {
		imu.init(delay)?;
		Self::set_remaps(&mut imu, delay)?;
		Self::set_mode(&mut imu, delay)?;

		// Wait for everything to be good
		// while !imu.is_fully_calibrated()? {
		//	delay.delay_ms(5u32);
		// }

		// Now sample bias from sensor (needed?)
		let accel_constant_bias =
			Self::sample_constant_bias(&mut imu, delay, Duration::from_secs(1))?;

		Ok(Self {
			imu,
			vel: Vector3::zeros(),
			pos: Vector3::zeros(),
			last: Instant::now(),

			accel_constant_bias,
		})
	}

	fn set_remaps(_imu: &mut Bno055<I2c>, _delay: &mut Delay) -> ImuResult<()> {
		Ok(())
	}

	fn set_mode(imu: &mut Bno055<I2c>, delay: &mut Delay) -> ImuResult<()> {
		imu.set_mode(BNO055OperationMode::NDOF, delay)?;
		Ok(())
	}

	// 1 of 3 main types of bias
	// also consider
	//     2. temp induced variation (fit quadratic line or smth)
	//     3. bias instablity (harder)
	fn sample_constant_bias(
		imu: &mut Bno055<I2c>,
		delay: &mut Delay,
		sample_time: Duration,
	) -> ImuResult<Vector3<f64>> {
		let sample_count = (sample_time.as_millis() / 10) as usize;
		let sample_inv = 1.0 / (sample_count as f64);

		let mut acceleration_avg = Vector3::zeros();

		for _ in 0..sample_count {
			let sample: Vector3<f64> =
				<_ as Into<Vector3<f32>>>::into(imu.linear_acceleration()?).cast();
			acceleration_avg += sample * sample_inv;
			delay.delay_ms(10u16);
		}

		Ok(acceleration_avg)
	}

	fn process(&mut self) -> ImuResult<()> {
		let orientation: Quaternion<f32> = self.imu.quaternion()?.into();
		let accel = {
			let a: Vector3<f64> =
				<_ as Into<Vector3<f32>>>::into(self.imu.linear_acceleration()?).cast();
			a - self.accel_constant_bias
		};

		let dt = {
			let now = Instant::now();
			let dt = (now - self.last).as_secs_f64();
			self.last = now;
			dt
		};

		let rotation = UnitQuaternion::from_quaternion(orientation)
			.cast()
			.to_rotation_matrix();

		// Estimate position based on changes in acceleration
		let a = rotation * accel;
		self.vel += a * dt;
		self.pos += self.vel + a * (0.5 * dt * dt);

		Ok(())
	}

	pub fn get_position(&self) -> Vector3<f64> {
		self.pos
	}
}
