use crate::{
	odom::*,
	// robota::robota,
	robotb::robotb,
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
// mod robota;
mod robotb;
mod state;
mod units;

fn main() -> anyhow::Result<()> {
	robotb()
}

pub fn setup() -> anyhow::Result<(GlobalState, RerunLogger, DriveImuOdom)> {
	dotenvy::dotenv().ok();
	let state = GlobalState::new()?;

	logging::setup_field_rerun(state.network.rerun_logger());

	let logger = state.network.rerun_logger();

	let a = logger.clone();
	let odom = std::thread::spawn(move || DriveImuOdom::new(a))
		.join()
		.unwrap()?;

	Ok((state, logger, odom))
}
