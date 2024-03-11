use crate::bmi088::Bmi088;
use amt22::Amt22;
use rppal::spi::Spi;

const LEFT_DIST: f64 = 0.05;
const RIGHT_DIST: f64 = 0.05;
const BACK_DIST: f64 = 0.1;

pub struct TrackingWheels {
	back: Amt22<Spi>,
	left: Amt22<Spi>,
	right: Amt22<Spi>,
	// zero offset in rotations
	zeros: [f64; 3],
	distances: [f64; 3],
}

impl TrackingWheels {
	// distance travelled per full rotation in meters
	const TRACKING_CIRCUMFERENCE: f64 = 0.219440246853;
	const ENCODER_TICK_SCALE: f64 = 1.0 / 4096.0;
	pub fn new() -> Self {
		let mut delay = rppal::hal::Delay::new();
		let get_enc = |ss| {
			let spi =
				rppal::spi::Spi::new(rppal::spi::Bus::Spi0, ss, 100_000, rppal::spi::Mode::Mode0)
					.unwrap();
			amt22::Amt22::new(spi, amt22::Resolution::Res12Bit)
		};
		let mut left = get_enc(rppal::spi::SlaveSelect::Ss0);
		let mut right = get_enc(rppal::spi::SlaveSelect::Ss1);
		let mut back = get_enc(rppal::spi::SlaveSelect::Ss2);
		left.reset(Some(&mut delay)).unwrap();
		right.reset(Some(&mut delay)).unwrap();
		back.reset(Some(&mut delay)).unwrap();

		Self {
			// get zero offset measured in rotations
			zeros: [&mut left, &mut right, &mut back].map(Self::enc_to_rotations),
			distances: [0.0; 3],
			left,
			right,
			back,
		}
	}
	// returns signed rotations done
	fn enc_to_rotations(enc: &mut Amt22<Spi>) -> f64 {
		// TODO: remove unwrap
		let (turns, subturns) = enc.read_absolute_position_raw().unwrap();
		turns as f64 + Self::ENCODER_TICK_SCALE * subturns as f64
	}
	pub fn distances(&self) -> [f64; 3] {
		let [l, r, b] = self.distances;
		// account for tracking wheel orientation
		[l, r, b]
	}
	// returns distance in meters
	pub fn calc_distances(&mut self) {
		// get uncorrected rotation count
		let rotations =
			[&mut self.left, &mut self.right, &mut self.back].map(Self::enc_to_rotations);
		// correct for zero offset
		let rotations = [
			rotations[0] - self.zeros[0],
			rotations[1] - self.zeros[1],
			rotations[2] - self.zeros[2],
		];

		/*plot!("encoders (degrees)", rotations.map(|v| v * 360.0));
		plot!(
			"encoders (radians)",
			rotations.map(|v| v * std::f64::consts::TAU)
		);*/

		// multiply by tracking wheel circumference to figure out distance travelled
		self.distances = rotations.map(|v| v * Self::TRACKING_CIRCUMFERENCE);

		//plot!("encoders (distance)", self.distances);
	}
}

pub struct Odometry {
	imu: Bmi088,
	tracking_wheels: TrackingWheels,
	position: [f64; 2],
}

impl Odometry {
	pub fn new(imu_bias: f64) -> Self {
		let mut imu = Bmi088::new(imu_bias);
		imu.reset();
		Self {
			imu,
			tracking_wheels: TrackingWheels::new(),
			position: [0.0; 2],
		}
	}
	pub fn calc_position(&mut self) {
		// gets the heading in radians
		let last_heading = self.imu.heading();
		// gets the distances travelled by each tracking wheel in meters
		let [last_left, last_right, last_back] = self.tracking_wheels.distances();

		// update both the heading and wheel distances
		self.imu.calc_heading();
		self.tracking_wheels.calc_distances();

		// get the new heading and wheel positions
		let heading = self.imu.heading();
		let [left, right, back] = self.tracking_wheels.distances();

		// get the differences
		let diff_heading = heading - last_heading;
		let [diff_left, diff_right, diff_back] =
			[left - last_left, right - last_right, back - last_back];

		let (diff_x, diff_y);
		let (sin, cos) = heading.sin_cos();

		// handle the special case if the heading doesn't change
		if diff_heading.abs() < 0.0005 || diff_back.abs() > 0.1 {
			// local difference
			let diff_x_local = 0.5 * (diff_left + diff_right);
			let diff_y_local = diff_back;
			// global difference
			diff_x = cos * diff_x_local - sin * diff_y_local;
			diff_y = sin * diff_x_local + cos * diff_y_local;
		} else {
			let rt = 0.5
				* (diff_left / diff_heading + LEFT_DIST + diff_right / diff_heading - RIGHT_DIST);
			let rs = diff_back / diff_heading + BACK_DIST;

			let (last_sin, last_cos) = last_heading.sin_cos();
			let (diff_sin, diff_cos) = diff_heading.sin_cos();
			let local_x = rt * diff_sin - rs * (1.0 - diff_cos);
			let local_y = rt * (1.0 - diff_cos) + rs * diff_sin;
			diff_x = last_cos * local_x - last_sin * local_y;
			diff_y = last_sin * local_x + last_cos * local_y;
		}

		self.position[0] += diff_x;
		self.position[1] += diff_y;
	}
	pub fn position(&self) -> [f64; 2] {
		self.position
	}
	pub fn heading(&self) -> f64 {
		self.imu.heading()
	}
	pub fn reset(&mut self) {
		self.imu.reset()
	}
}
