mod bmi088;
mod brain;
mod controller;
mod drivebase;
mod motor;
mod odom;

use bmi088::Bmi088;
use brain::Brain;
use communication::{
	packet::{FromMediator, ToMediator},
	plot, Mediator,
};
use controller::Controller;
use drivebase::Tankdrive;

use std::time::Duration;

const IS_SKILLS: bool = true;
pub const BRAIN_TIMEOUT: Duration = Duration::from_millis(500);

const IMU_BIAS: f64 = 0.0;

fn main() -> ! {
	Robot::run();
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RobotState {
	Off,
	Disabled,
	DriverSkills,
	AutonSkills,
	DriverDriver,
	DriverAuton,
}

impl Default for RobotState {
	fn default() -> Self {
		Self::Off
	}
}

impl RobotState {
	pub fn progress(&mut self, brain_state: brain::State, imu: &mut Bmi088) {
		*self = match (*self, brain_state, IS_SKILLS) {
			(Self::Off, brain::State::Disabled, _) => {
				log::info!("Connection established with the brain.");
				log::info!("Entering Disabled state.");
				Self::Disabled
			}
			(Self::Off, brain::State::Driver, true) => {
				log::warn!("Entered driver skills without first entering the disabled state");
				log::info!("Entering DriverSkills state.");
				Self::DriverSkills
			}
			(Self::Off, brain::State::Auton, true) => {
				log::warn!("Entered auton skills without first entering the disabled state");
				log::info!("Entering DriverSkills state.");
				Self::DriverSkills
			}
			(Self::Disabled, brain::State::Driver, true) => {
				log::info!("Entering DriverSkills state.");
				Self::DriverSkills
			}
			(_, brain::State::Driver, false) => {
				if *self != Self::DriverDriver {
					log::info!("Entering DriverDriver state.");
				}
				Self::DriverDriver
			}
			(_, brain::State::Driver, true) => {
				if *self != Self::DriverSkills {
					log::info!("Entering DriverSkills state.");
				}
				Self::DriverSkills
			}
			(_, brain::State::Auton, false) => {
				if *self != Self::DriverAuton {
					log::info!("Entering DriverAuton state.");
				}
				Self::DriverAuton
			}
			(_, brain::State::Auton, true) => {
				if *self != Self::AutonSkills {
					imu.reset();
					log::info!("Entering AutonSkills state.");
				}
				Self::AutonSkills
			}
			(Self::Disabled, brain::State::Disabled, _) => Self::Disabled,
			(a, b, c) => {
				log::info!("tried: {a:?} | {b:?} | {c:?}");
				todo!()
			}
		};
	}
}

struct Robot {
	state: RobotState,
	brain: Brain,
	controller: Controller,
	drivebase: Tankdrive<3>,
	mediator: Mediator,
	imu: Bmi088,
}

// merge or move these functions?
impl Robot {
	pub fn run() -> ! {
		let mut robot = Self::new();
		robot.main_loop();
	}
	pub fn new() -> Self {
		let mediator = communication::Logger::init(true).expect("This only panics when another logger is set. This should never be the case and indicates a problem with the code.");

		// block until connection is establish with brain
		log::info!("Connecting to the brain.");
		let (brain, controller) = Brain::init();
		log::info!("Connected to the brain.");

		// this is the drivetrain configuration for the nationals hang robot
		let drivebase = Tankdrive::new(
			[(11, false), (12, false), (17, false)],
			[(14, true), (15, true), (16, true)],
			&brain,
		);

		let imu = Bmi088::new(IMU_BIAS);

		Self {
			state: RobotState::default(),
			brain,
			controller,
			drivebase,
			mediator,
			imu,
		}
	}
	pub fn handle_events(&mut self) {
		if let Ok(events) = self.mediator.poll_events() {
			for event in events {
				match event {
					ToMediator::Ping => {
						if let Err(e) = self.mediator.send_event(FromMediator::Pong) {
							eprintln!("Failed to send Pong event: {e}");
						}
					}
					_ => {}
				}
			}
		}
	}
	pub fn main_loop(&mut self) -> ! {
		loop {
			self.handle_events();

			// updates controller, robot state & motors
			self.brain
				.update_state(&mut self.controller, &mut self.state, &mut self.imu);

			match self.state {
				RobotState::Off | RobotState::Disabled => {}
				RobotState::AutonSkills => {
					auton_skills(&mut self.brain, &mut self.imu, &mut self.drivebase)
				}
				RobotState::DriverAuton => {}
				RobotState::DriverSkills => {
					driver(
						&mut self.brain,
						&self.controller,
						&mut self.drivebase,
						&mut self.imu,
					);
				}
				RobotState::DriverDriver => {
					driver(
						&mut self.brain,
						&self.controller,
						&mut self.drivebase,
						&mut self.imu,
					);
				}
			}
		}
	}
}

const TURN_MULTIPLIER: f64 = 0.5;

fn driver(
	brain: &mut Brain,
	controller: &Controller,
	drivebase: &mut Tankdrive<3>,
	imu: &mut Bmi088,
) {
	std::thread::sleep(std::time::Duration::from_millis(1));
	let mut thread_rng = rand::thread_rng();
	use rand::Rng;
	if thread_rng.gen::<f64>() < 1.0 {
		plot!("heading", imu.heading());
	}
	let forward_rate = controller.ly();
	let turning_rate = controller.rx();
	let (l, r) = (
		(forward_rate + turning_rate * TURN_MULTIPLIER).clamp(-1.0, 1.0),
		(forward_rate - turning_rate * TURN_MULTIPLIER).clamp(-1.0, 1.0),
	);

	drivebase.set_side_percent_voltage(l, r);

	brain.write_changes();
}

fn auton_skills(brain: &mut Brain, imu: &mut Bmi088, drivebase: &mut Tankdrive<3>) {
	std::thread::sleep(std::time::Duration::from_millis(1));
}
