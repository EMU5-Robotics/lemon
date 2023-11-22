use protocol::device::ControllerButtons;

use crate::{
	replay::Player,
	state::{GlobalState, InputChanges, InputState},
	units::*,
};

mod logging;
mod motion_profile;
mod odom;
mod parts;
mod path;
mod pid;
mod replay;
mod state;
mod units;

/* Test Drive
[
	state.take_motor(10, true),
	state.take_motor(2, true),
	state.take_motor(1, false),
],
[
	state.take_motor(20, false),
	state.take_motor(12, false),
	state.take_motor(11, true),
],
*/

fn main() -> anyhow::Result<()> {
	dotenvy::dotenv().ok();

	let mut state = GlobalState::new()?;
	let mut input = state.create_input_state();

	let mut drive = parts::drive::Drive::new(
		[
			state.take_motor(4, false),
			state.take_motor(5, false),
			state.take_motor(13, false),
		],
		[
			state.take_motor(1, true),
			state.take_motor(2, true),
			state.take_motor(3, true),
		],
		0.8,
	);
	let mut odom = odom::DriveOdom::new(state.network.rerun_logger(), meter!(0.1285));

	let flipper = state.take_motor(11, true);

	loop {
		/*** Gather all input from serial, sensors, etc. ***/
		if let Some(status_pkt) = state.serial.take_status_pkt() {
			input.update_v5_status(status_pkt);
		} else {
			input.update_inputs();
		}
		input.overwrite_replay_input(&mut state.player);

		/*** Process inputs to parts ***/
		// V5 motors are implicitly updated by `update_v5_status()`

		// comp_fsm.transition(&input);

		// Simple testing code for now
		{
			handle_replay(&mut input, &mut state);

			flipper.voltage(flipper_mv(input.controller));

			let axes = input.controller.axes_as_f32();
			let d_power = axes[1];
			let t_power = axes[2];
			drive.drive(d_power, t_power);

			odom.update(&mut drive);
		}

		/*** Write motor outputs to V5 ***/
		state.write_serial_output();
		state.loop_delay(); // Make sure we sleep at least little bit each iteration
	}
}

fn handle_replay(input: &mut InputState, state: &mut GlobalState) {
	// Toggle recording
	if state.player.is_none() && input.controller.button_pressed(ControllerButtons::A) {
		log::info!("Toggle recording");
		if let Err(e) = state.recorder.toggle() {
			log::error!("recorder toggle failed with: {e}");
		}
	}

	// Load and start recording
	if state.player.is_none() && input.controller.button_pressed(ControllerButtons::B) {
		// Load the player and start it
		state.player = Player::from_file("test.replay").map(Player::play).ok();
		// log::info!("Player is {}", if state.player.is_some() {"Some(_)"} else {"None"});
	}

	// Update the recorder
	if let Err(e) = state.recorder.take_event(&input.controller) {
		log::error!("recorder failed to take event with: {e}");
	}
}

fn flipper_mv(input: InputChanges) -> i16 {
	if input.button_held(ControllerButtons::R1) {
		12_000
	} else if input.button_held(ControllerButtons::R2) {
		-12_000
	} else {
		0
	}
}
