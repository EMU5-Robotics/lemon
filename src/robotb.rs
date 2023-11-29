use std::time::{Duration, Instant};

use lemon::{
	parts::catapult::*,
	parts::drive::*,
	state::{ControllerButtons, Gearbox, GlobalState},
};

type Robot = lemon::Robot<Parts>;
struct Parts {
	catapult: Catapult,
	loader: Loader,
	first_run: bool,
	auton: Option<CatapultAutonState>,
}

enum CatapultAutonState {
	Wait(Instant),
	FirstRun,
	Calibrating,
	Running,
	Stopped,
}

impl CatapultAutonState {
	pub fn transition(self) -> Self {
		unimplemented!()
	}
}

pub fn main() -> anyhow::Result<()> {
	let robot = Robot::setup(
		create_parts,
		create_drive,
		None,
		Some(user_control),
		Some(auton),
	)?;

	robot.run()
}

fn user_control(robot: &mut Robot) {
	let controller = robot.input.controller;

	let axes = controller.axes_as_f32();
	let l_power = axes[1];
	let r_power = axes[3];
	// let scale = ((l_power.abs() + r_power.abs()) / 2.0).powf(2.0);
	let scale = 0.5;
	robot.base.drive_skid(scale * l_power, scale * r_power);

	if controller.button_pressed(ControllerButtons::A) {
		if robot.catapult.is_primed() {
			robot.catapult.fire();
		} else {
			robot.catapult.prime();
		}
	}
	robot.catapult.transition();

	// if robot.parts.first_run == false {
	// 	robot.parts.loader.start_primed();
	// 	robot.parts.first_run = true;
	// }
	// robot.parts.loader.transition();
}

fn auton(robot: &mut Robot) {
	// Create auton state if it doesn't exist
	if robot.auton.is_none() {
		robot.auton = Some(auton_state(
			&mut robot.parts.loader,
			&mut robot.parts.catapult,
		));
	}
	let mut auton = robot.parts.auton.take().unwrap();

	let loader = &mut robot.parts.loader;
	let catapult = &mut robot.parts.catapult;

	match auton {
		CatapultAutonState::Wait(at) => {
			if Instant::now() > at + Duration::from_secs(10) {
				auton = CatapultAutonState::FirstRun;
			}
		}
		CatapultAutonState::FirstRun => {
			catapult.reset();
			loader.transition();
			catapult.transition();

			auton = CatapultAutonState::Calibrating;
		}
		CatapultAutonState::Calibrating => {
			if catapult.is_calibrated() {
				catapult.prime();
				loader.start_primed();
				auton = CatapultAutonState::Running;
			}
			loader.transition();
			catapult.transition();
		}
		CatapultAutonState::Running => {
			// Hold load if catapult is not primed
			if catapult.is_primed() {
				loader.hold_load = false;
			} else {
				loader.hold_load = true;
			}
			// Transition state
			loader.transition();

			// If we are now resetting, then fire kicker
			if loader.is_ready_to_fire() {
				catapult.fire();
			}

			// Transition state
			catapult.transition();

			// Check if reached count
			if catapult.count() == 11 {
				auton = CatapultAutonState::Stopped;
			}
		}
		CatapultAutonState::Stopped => {
			loader.fold_up = true;
			loader.transition();
			catapult.transition();
		}
	}

	robot.parts.auton = Some(auton);
}

fn auton_state(_loader: &mut Loader, catapult: &mut Catapult) -> CatapultAutonState {
	catapult.reset();

	CatapultAutonState::Wait(Instant::now())
}

fn create_parts(state: &mut GlobalState) -> anyhow::Result<Parts> {
	let loader = Loader::new(
		state.network.rerun_logger(),
		[state.take_motor(19, false), state.take_motor(14, true)],
	);
	state.serial.set_gearboxes(loader.get_gearboxes());

	let catapult = Catapult::new(
		state.network.rerun_logger(),
		[state.take_motor(13, false), state.take_motor(20, true)],
	);
	state.serial.set_gearboxes(catapult.get_gearboxes());

	Ok(Parts {
		loader,
		catapult,
		first_run: false,
		auton: None,
	})
}

fn create_drive(state: &mut GlobalState) -> anyhow::Result<Drive> {
	let drive = Drive::new(
		state.network.rerun_logger(),
		[
			state.take_motor(5, true),
			state.take_motor(7, true),
			state.take_motor(18, true),
		],
		[
			state.take_motor(15, false),
			state.take_motor(16, false),
			state.take_motor(17, false),
		],
		Gearbox::Blue,
		0.8,
	);
	state.serial.set_gearboxes(drive.get_gearboxes());

	Ok(drive)
}
