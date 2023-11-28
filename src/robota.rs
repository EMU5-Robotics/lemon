use std::time::Duration;

use lemon::{
	parts::drive::Drive,
	path::*,
	state::{Gearbox, GlobalState, Motor},
	units::*,
};
use robot_algorithms::prelude::Vec2;

type Robot = lemon::Robot<Parts>;
struct Parts {
	flipper: Motor,
	path: Option<Path>,
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
	let d_power = axes[1];
	let t_power = axes[2];
	robot.base.drive(d_power, t_power);
}

fn auton(robot: &mut Robot) {
	// Create auton path if it doesn't exist
	if robot.path.is_none() {
		robot.path = Some(auton_path(&robot.base, robot.flipper.clone()));
	}
	let mut path = robot.parts.path.take().unwrap();

	// Follow the path
	let (lv, rv) = match path.follow(&robot.odom, &mut robot.tpid) {
		Some(v) => v,
		None => (ConstZero::ZERO, ConstZero::ZERO),
	};

	// Set the drive speed
	robot.base.set_velocity(lv, rv);

	robot.path = Some(path);
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

fn create_parts(state: &mut GlobalState) -> anyhow::Result<Parts> {
	let flipper = state.take_motor(15, false);

	Ok(Parts {
		flipper,
		path: None,
	})
}

fn create_drive(state: &mut GlobalState) -> anyhow::Result<Drive> {
	let drive = Drive::new(
		state.network.rerun_logger(),
		[
			state.take_motor(4, false),
			state.take_motor(5, false),
			state.take_motor(11, false),
		],
		[
			state.take_motor(1, true),
			state.take_motor(2, true),
			state.take_motor(3, true),
		],
		Gearbox::Blue,
		0.8,
	);
	state.serial.set_gearboxes(drive.get_gearboxes());

	Ok(drive)
}
