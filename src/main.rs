use crate::{
	odom::*,
	parts::drive::*,
	robota::robota,
	state::{GlobalState, InputState, RerunLogger},
	units::*,
};

mod logging;
mod motion_profile;
mod odom;
mod parts;
mod path;
mod pid;
mod replay;
mod robota;
mod robotb;
mod state;
mod units;

fn main() -> anyhow::Result<()> {
	robota()
}

pub fn setup() -> anyhow::Result<(GlobalState, RerunLogger, DriveImuOdom, Drive)> {
	dotenvy::dotenv().ok();
	let state = GlobalState::new()?;

	logging::setup_field_rerun(state.network.rerun_logger());

	let logger = state.network.rerun_logger();

	let a = logger.clone();
	let odom = std::thread::spawn(move || DriveImuOdom::new(a))
		.join()
		.unwrap()?;

	let drive = parts::drive::Drive::new(
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
		0.8,
	);
	Ok((state, logger, odom, drive))
}
