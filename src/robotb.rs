use lemon::{
	parts::catapult::*,
	parts::drive::*,
	state::{ControllerButtons, Gearbox, GlobalState},
};

type Robot = lemon::Robot<Parts>;
struct Parts {
	catapult: Catapult,
	loader: Loader,
}

pub fn main() -> anyhow::Result<()> {
	let robot = Robot::setup(create_parts, create_drive, None, Some(user_control), None)?;

	robot.run()
}

fn user_control(robot: &mut Robot) {
	let controller = robot.input.controller;

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
}

fn create_parts(state: &mut GlobalState) -> anyhow::Result<Parts> {
	let loader = Loader::new(
		state.network.rerun_logger(),
		[state.take_motor(19, false), state.take_motor(18, true)],
	);
	state.serial.set_gearboxes(loader.get_gearboxes());

	let catapult = Catapult::new(
		state.network.rerun_logger(),
		[state.take_motor(13, false), state.take_motor(20, true)],
	);
	state.serial.set_gearboxes(catapult.get_gearboxes());

	Ok(Parts { loader, catapult })
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
