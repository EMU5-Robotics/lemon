use std::time::Duration;

use crate::{
	odom::*,
	parts::{catapult::*, drive::*},
	path::*,
	pid::*,
	state::{self, GlobalState, InputState, Motor},
	units::*,
};

use protocol::device::{ControllerButtons, Gearbox};
use robot_algorithms::prelude::Vec2;
use uom::ConstZero;

pub fn robotb() -> anyhow::Result<()> {
	let (mut state, _logger, odom) = crate::setup()?;

	let input = state.create_input_state();

	let flipper = state.take_motor(20, true);

	let drive = Drive::new(
		state.network.rerun_logger(),
		[
			state.take_motor(7, false),
			state.take_motor(14, false),
			state.take_motor(15, false),
		],
		[
			state.take_motor(3, true),
			state.take_motor(4, true),
			state.take_motor(8, true),
		],
		Gearbox::Blue,
		0.8,
	);
	let gearboxes = drive.get_gearboxes().into_iter();

	let loader = Catapult::new(
		state.network.rerun_logger(),
		[state.take_motor(19, false), state.take_motor(18, true)],
	);
	let gearboxes = gearboxes.chain(loader.get_gearboxes());
	let catapult = Catapult::new(
		state.network.rerun_logger(),
		[state.take_motor(13, false), state.take_motor(20, true)],
	);
	let gearboxes = gearboxes.chain(catapult.get_gearboxes());

	state
		.serial
		.set_gearboxes(state::generate_gearboxes(gearboxes));

	// auton(state, input, odom, drive, flipper);
	user(state, input, odom, drive, flipper, catapult);
}

fn user(
	mut state: GlobalState,
	mut input: InputState,
	mut odom: DriveImuOdom,
	mut drive: Drive,
	flipper: Motor,
	mut catapult: Catapult,
) -> ! {
	loop {
		/*** Gather all input from serial, sensors, etc. ***/
		if let Some(status_pkt) = state.serial.take_status_pkt() {
			input.update_v5_status(status_pkt);
		} else {
			input.update_inputs();
		}
		input.overwrite_replay_input(&mut state.player);

		/*** Process inputs to parts ***/
		odom.update(&mut drive);

		let axes = input.controller.axes_as_f32();
		let d_power = axes[1];
		let t_power = axes[2];
		drive.drive(d_power, t_power);

		catapult.transition();
		if input.controller.button_pressed(ControllerButtons::B) {
			catapult.prime();
		}
		if input.controller.button_pressed(ControllerButtons::A) {
			catapult.fire();
		}
		if input.controller.button_pressed(ControllerButtons::Y) {
			catapult.reset();
		}

		/*** Write motor outputs to V5 ***/
		state.write_serial_output();
		state.loop_delay();
	}
}

fn auton(
	mut state: GlobalState,
	mut input: InputState,
	mut odom: DriveImuOdom,
	mut drive: Drive,
	flipper: Motor,
) -> ! {
	let mut tpid = AnglePid::new(3.5, 1.0, 0.0, degree!(0.0)); // this will do for now

	let mut path = auton_path(&drive, flipper);

	loop {
		/*** Gather all input from serial, sensors, etc. ***/
		if let Some(status_pkt) = state.serial.take_status_pkt() {
			input.update_v5_status(status_pkt);
		} else {
			input.update_inputs();
		}
		input.overwrite_replay_input(&mut state.player);

		/*** Process inputs to parts ***/
		odom.update(&mut drive);

		let (lv, rv) = match path.follow(&odom, &mut tpid) {
			Some(v) => v,
			None => (ConstZero::ZERO, ConstZero::ZERO),
		};

		drive.set_velocity(lv, rv);

		/*** Write motor outputs to V5 ***/
		state.write_serial_output();
		state.loop_delay();
	}
}

fn auton_path(drive: &Drive, flipper: Motor) -> Path {
	Path::new(vec![
		PathSeg::line(Vec2::new(0.0, 0.0), Vec2::new(0.0, 1.18), false).default_timer(),
		TurnTo::new(degree!(47.0)).default_timer(),
		PathSeg::line(Vec2::new(0.0, 1.18), Vec2::new(0.70, 1.88), false)
			.with_timer(Duration::from_secs(3)),
		PowerMotors::new(
			vec![-12000, 12000, 12000, 12000, -12000, -12000, -12000],
			vec![
				flipper.clone(),
				drive.left[0].clone(),
				drive.left[1].clone(),
				drive.left[2].clone(),
				drive.right[0].clone(),
				drive.right[1].clone(),
				drive.right[2].clone(),
			],
		)
		.with_timer(Duration::from_secs(2)),
		PowerMotors::new(vec![12000], vec![flipper.clone()]).with_timer(Duration::from_secs(2)),
		PathSeg::line(Vec2::new(0.0, 0.0), Vec2::new(0.0, -0.10), true)
			.into_relative()
			.with_timer(Duration::from_secs(1)),
	])
}
