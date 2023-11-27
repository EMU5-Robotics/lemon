use crate::{
	odom::*,
	parts::drive::*,
	path::*,
	pid::*,
	state::{GlobalState, InputState, Motor},
	units::*,
};

use robot_algorithms::prelude::Vec2;
use std::time::Duration;
use uom::ConstZero;

pub fn robota() -> anyhow::Result<()> {
	let (mut state, _logger, odom, drive) = crate::setup()?;

	let input = state.create_input_state();

	let flipper = state.take_motor(15, true);

	auton(state, input, odom, drive, flipper);
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
