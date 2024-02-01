mod brain;
mod controller;
mod drivebase;
mod motor;

use brain::Brain;
use communication::{
	packet::{FromMediator, ToMediator},
	Mediator,
};
use controller::Controller;
use drivebase::Tankdrive;

use std::time::Duration;

use crate::motor::Motor;

const IS_SKILLS: bool = false;
pub const BRAIN_TIMEOUT: Duration = Duration::from_millis(500);

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
	pub fn progress(&mut self, brain_state: brain::State) {
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
				log::info!("Entering DriverSkills state.");
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
					log::info!("Entering AutonSkills state.");
				}
				log::info!("Entering AutonSkills state.");
				Self::AutonSkills
			}
			(_, _, _) => todo!(),
		};
	}
}

struct Robot {
	state: RobotState,
	brain: Brain,
	controller: Controller,
	drivebase: Tankdrive<3>,
	mediator: Mediator,
}

// merge or move these functions?
impl Robot {
	pub fn run() -> ! {
		let mut robot = Self::new();
		robot.main_loop();
	}
	pub fn new() -> Self {
		let mediator = communication::Logger::init().expect("This only panics when another logger is set. This should never be the case and indicates a problem with the code.");

		// block until connection is establish with brain
		let (brain, controller) = Brain::init();
		log::info!("Connected to the brain.");

		// TODO: change to actual robot config
		let drivebase = Tankdrive::new(
			[
				(brain.get_motor(11), false),
				(brain.get_motor(12), false),
				(brain.get_motor(17), false),
			],
			[
				(brain.get_motor(14), true),
				(brain.get_motor(15), true),
				(brain.get_motor(16), true),
			],
		);

		Self {
			state: RobotState::default(),
			brain,
			controller,
			drivebase,
			mediator,
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
				}
			}
		}
	}
	pub fn main_loop(&mut self) -> ! {
		loop {
			self.handle_events();

			// updates controller, robot state & motors
			self.brain
				.update_state(&mut self.controller, &mut self.state);

			match self.state {
				RobotState::Off | RobotState::Disabled => {}
				RobotState::AutonSkills => {}
				RobotState::DriverAuton => {}
				RobotState::DriverSkills => {
					driver(&mut self.brain, &self.controller, &mut self.drivebase);
				}
				RobotState::DriverDriver => {
					driver(&mut self.brain, &self.controller, &mut self.drivebase);
				}
			}
		}
	}
}

const TURN_MULTIPLIER: f64 = 0.5;

fn driver<const N: usize>(
	brain: &mut Brain,
	controller: &Controller,
	drivebase: &mut Tankdrive<N>,
) {
	let forward_rate = controller.ly();
	let turning_rate = controller.rx();
	let (l, r) = (
		(forward_rate + turning_rate * TURN_MULTIPLIER).clamp(-1.0, 1.0),
		(forward_rate - turning_rate * TURN_MULTIPLIER).clamp(-1.0, 1.0),
	);

	drivebase.set_side_percent_voltage(l, r);

	brain.write_changes();
}
