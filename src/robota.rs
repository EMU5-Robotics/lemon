use std::time::Duration;

use lemon::{
	parts::drive::Drive,
	path::*,
	state::{ControllerButtons, Gearbox, GlobalState, Motor},
	units::*,
};
use robot_algorithms::prelude::Vec2;

type Robot = lemon::Robot<Parts>;
struct Parts {
	flipper: Motor,
	hang: [Motor; 2],
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

	lemon::move_voltage(
		12_000,
		ControllerButtons::R2,
		ControllerButtons::R1,
		&robot.parts.hang[..],
        controller
	);
	lemon::move_voltage(
		12_000,
		ControllerButtons::L1,
		ControllerButtons::L2,
		&[robot.parts.flipper.clone()],
        controller
	)
}

fn auton(robot: &mut Robot) {
	// Create auton path if it doesn't exist
	if robot.path.is_none() {
		match robot.input.v5_status.1.auton {
			0 => robot.path = Some(auton_path(&robot.base, robot.flipper.clone())),
			_ => robot.path = Some(other_auton_path()),
		}
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
	let open_net = PowerMotors::new(vec![12000], vec![flipper.clone()]).default_timer();
	let drive_forward =
		SetVel::new((meter_per_second!(1.0), meter_per_second!(1.0))).default_timer();

	let drive_forward_and_open_net =
		Aux::new(drive_forward, open_net).with_timer(Duration::from_secs(2));
	Path::new(vec![
		//PowerMotors::new(vec![12000, 12000], hang.to_vec()).with_timer(Duration::from_secs(10)),
		PathSeg::line(Vec2::new(0.0, 0.0), Vec2::new(0.0, 1.49), false).default_timer(),
		TurnTo::new(degree!(60.0)).default_timer(),
		Nop::new().with_timer(Duration::from_millis(500)),
		SetVel::new((meter_per_second!(5.0), meter_per_second!(5.0)))
			.with_timer(Duration::from_secs(3)),
		drive_forward_and_open_net,
		PowerMotors::new(vec![-12000], vec![flipper.clone()]).with_timer(Duration::from_secs(2)),
		SetVel::new((meter_per_second!(-0.5), meter_per_second!(-0.5)))
			.with_timer(Duration::from_secs(1)),
	])
}

fn other_auton_path() -> Path {
	Path::new(vec![
		SetVel::new((meter_per_second!(1.0), meter_per_second!(1.0)))
			.with_timer(Duration::from_millis(500)),
		SetVel::new((meter_per_second!(-1.0), meter_per_second!(-1.0)))
			.with_timer(Duration::from_millis(500)),
	])
}

fn create_parts(state: &mut GlobalState) -> anyhow::Result<Parts> {
	let flipper = state.take_motor(10, false);
	let hang = [state.take_motor(18, true), state.take_motor(19, false)];

	Ok(Parts {
		flipper,
		hang,
		path: None,
	})
}

fn create_drive(state: &mut GlobalState) -> anyhow::Result<Drive> {
	let drive = Drive::new(
		state.network.rerun_logger(),
		[
			state.take_motor(17, false),
			state.take_motor(12, false),
			state.take_motor(11, false),
		],
		[
			state.take_motor(16, true),
			state.take_motor(15, true),
			state.take_motor(14, true),
		],
		Gearbox::Blue,
		0.55,
	);
	state.serial.set_gearboxes(drive.get_gearboxes());

	Ok(drive)
}
