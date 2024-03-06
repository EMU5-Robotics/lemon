use amt22::Amt22;
use bmi088::Bmi088;
use communication::{
	packet::{FromMediator, ToMediator},
	plot,
};
use rppal::spi::Spi;

mod bmi088;

// left: 49mm
// right: 51.5mm
// back: 103mm
/*const LEFT_DIST: f64 = 49e-3;
const RIGHT_DIST: f64 = 51.5e-3;
const BACK_DIST: f64 = 103e-3;*/
const IMU_BIAS: f64 = 0.0004146448; //0.0002138361;

struct TrackingWheels {
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
		[l, r, -b]
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

struct Odometry {
	imu: Bmi088,
	tracking_wheels: TrackingWheels,
	position: [f64; 2],
}

impl Odometry {
	pub fn new() -> Self {
		let mut imu = Bmi088::new(IMU_BIAS);
		imu.reset();
		Self {
			imu,
			tracking_wheels: TrackingWheels::new(),
			position: [0.0; 2],
		}
	}
	pub fn calc_position(&mut self) {
		let last_heading = self.imu.heading();
		let [last_ld, last_rd, last_bd] = self.tracking_wheels.distances();

		self.imu.calc_heading();
		self.tracking_wheels.calc_distances();

		let heading = self.imu.heading();
		let [ld, rd, bd] = self.tracking_wheels.distances();

		let delta_distances = [ld - last_ld, rd - last_rd, bd - last_bd];

		let avg_angle = 0.5 * (heading + last_heading);
		// note we ignore back tracking wheel
		let local_offset = [0.5 * (delta_distances[0] + delta_distances[1]), 0.0];

		let (s, c) = avg_angle.sin_cos();
		let offset = [
			c * local_offset[0] + s * local_offset[1],
			-s * local_offset[0] + c * local_offset[1],
		];

		self.position[0] += offset[0];
		self.position[1] += offset[1];
	}
	pub fn position(&self) -> [f64; 2] {
		self.position
	}
	pub fn heading(&self) -> f64 {
		self.imu.heading()
	}
}

fn main() {
	let mut mediator = communication::Logger::init(false).expect("This only panics when another logger is set. This should never be the case and indicates a problem with the code.");

	let mut odom = Odometry::new();
	loop {
		odom.calc_position();
		if let Ok(events) = mediator.poll_events() {
			for event in events {
				match event {
					ToMediator::Ping => {
						if let Err(e) = mediator.send_event(FromMediator::Pong) {
							eprintln!("Failed to send Pong event: {e}");
						}
					}
					_ => {}
				}
			}
		}
		plot!("heading", odom.heading().to_degrees());
		//plot!("position", odom.position());
		communication::odom(odom.position(), odom.heading());
		std::thread::sleep(std::time::Duration::from_millis(1));
	}
}
