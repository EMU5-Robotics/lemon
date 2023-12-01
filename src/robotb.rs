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

	if controller.button_pressed(ControllerButtons::UP) {
		robot.base.reversed = false;
	}
	if controller.button_pressed(ControllerButtons::DOWN) {
		robot.base.reversed = true;
	}

	let axes = controller.axes_as_f32();
	let d_power = axes[1];
	let t_power = axes[2];
	robot.base.drive(d_power, t_power);

	if controller.button_pressed(ControllerButtons::A) {
		if robot.catapult.is_primed() {
			robot.catapult.fire();
		} else {
			robot.catapult.prime();
		}
	}
	robot.catapult.transition();

	if controller.button_pressed(ControllerButtons::Y) {
		match robot.parts.loader.state_pos() {
			LoaderPosState::Primed => {
				robot.parts.loader.start_primed();
				robot.parts.loader.fold_out = true;
			}
			LoaderPosState::Folded => {
				robot.parts.loader.start_folded();
				robot.parts.loader.fold_out = true;
			}
			LoaderPosState::Other => {
				robot.parts.loader.reset();
				robot.parts.loader.fold_out = true;
			}
		}
	}
	if controller.button_pressed(ControllerButtons::X) {
		match robot.parts.loader.state_pos() {
			LoaderPosState::Primed | LoaderPosState::Other => {
				robot.parts.loader.start_primed();
				robot.parts.loader.fold_up = true;
			}
			_ => {}
		}
	}
	robot.parts.loader.transition();
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
			loader.hold_load = false;
			loader.transition();
			loader.fold_out = true;
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
			state.take_motor(5, false),
			state.take_motor(7, false),
			state.take_motor(18, false),
		],
		[
			state.take_motor(15, true),
			state.take_motor(16, true),
			state.take_motor(17, true),
		],
		Gearbox::Blue,
		0.6,
	);
	state.serial.set_gearboxes(drive.get_gearboxes());

	Ok(drive)
}
