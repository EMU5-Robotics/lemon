mod bmi088;
mod brain;
mod controller;
mod drivebase;
mod motor;
mod odom;
mod pid;

use brain::Brain;
use communication::{
	packet::{FromMediator, ToMediator},
	plot, Mediator,
};
use controller::Controller;
use drivebase::Tankdrive;
use odom::Odometry;
use pid::Pid;
use protocol::device::ControllerButtons;

use std::time::Duration;

use crate::bmi088::ROBOT_A_IMU_BIAS;

const IS_SKILLS: bool = true;
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
	pub fn progress(&mut self, brain_state: brain::State, odom: &mut Odometry) {
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
					odom.reset();
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
	odom: Odometry,
	pid_l: Pid,
	pid_r: Pid,
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
			[(11, false), (12, true), (17, true)],
			[(14, false), (15, true), (16, false)],
			&brain,
		);

		let odom = Odometry::new(ROBOT_A_IMU_BIAS);

		Self {
			state: RobotState::default(),
			brain,
			controller,
			drivebase,
			mediator,
			odom,
			pid_l: Pid::new(2.0, 0.5, 0.0),
			pid_r: Pid::new(2.0, 0.5, 0.0),
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
					ToMediator::Pid((kp, ki, kd)) => {
						self.pid_l.kp = kp;
						self.pid_l.kp = kp;
						self.pid_r.ki = ki;
						self.pid_r.ki = ki;
						self.pid_l.kd = kd;
						self.pid_r.kd = kd;
						self.pid_l.reset();
						self.pid_r.reset();
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
				.update_state(&mut self.controller, &mut self.state, &mut self.odom);

			match self.state {
				RobotState::Off | RobotState::Disabled => {}
				RobotState::AutonSkills => self.auton_skills(),
				RobotState::DriverAuton => {}
				RobotState::DriverSkills => {
					self.driver();
				}
				RobotState::DriverDriver => {
					self.driver();
				}
			}
		}
	}
	fn driver(&mut self) {
		let velocities = self.odom.side_velocities();
		plot!("velocities", velocities);

		communication::odom(self.odom.position(), self.odom.heading());
		let forward_rate = self.controller.ly();
		let turning_rate = self.controller.rx();
		let (mut l, mut r) = (
			(forward_rate + turning_rate * TURN_MULTIPLIER).clamp(-1.0, 1.0),
			(forward_rate - turning_rate * TURN_MULTIPLIER).clamp(-1.0, 1.0),
		);
		if self.controller.pressed(ControllerButtons::A) {
			self.pid_l.set_target(0.2);
			self.pid_r.set_target(0.2);
			self.pid_l.reset();
			self.pid_r.reset();
		}
		if self.controller.held(ControllerButtons::A) {
			l = self.pid_l.poll(velocities[0]).clamp(-1.0, 1.0);
			r = self.pid_r.poll(velocities[1]).clamp(-1.0, 1.0);
			plot!("pid values", [l, r]);
			plot!("errors", [0.2 - velocities[0], 0.2 - velocities[1]]);
		}

		self.drivebase.set_side_percent_voltage(l, r);

		self.brain.write_changes();
		std::thread::sleep(std::time::Duration::from_millis(1));
	}

	fn auton_skills(&mut self) {
		communication::odom(self.odom.position(), self.odom.heading());

		self.brain.write_changes();
		std::thread::sleep(std::time::Duration::from_millis(1));
	}
}

const TURN_MULTIPLIER: f64 = 0.5;
