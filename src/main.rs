use protocol::device::ControllerButtons;

use crate::{state::GlobalState, units::*};

mod motion_profile;
mod odom;
mod parts;
mod path;
mod pid;
mod replay;
mod state;
mod units;

fn main() -> Result<(), Box<dyn std::error::Error>> {
	let mut state = GlobalState::new()?;
	let mut input = state.create_input_state();

	let mut drive = parts::drive::Drive::new(
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
		0.8,
	);
	let mut odom = odom::DriveOdom::new(state.network.rerun_logger(), meter!(0.1285));

	loop {
		/*** Gather all input from serial, sensors, etc. ***/
		if let Some(status_pkt) = state.serial.take_status_pkt() {
			input.update_v5_status(status_pkt);
		}
		input.update_replay_input(state.player.as_mut());

		/*** Process inputs to parts ***/
		// V5 motors are implicitly updated by `update_v5_status()`

		// comp_fsm.transition(&input);

		// Simple testing code for now
		{
			if input.controller.button_released(ControllerButtons::A)
				|| (input.controller.button_pressed(ControllerButtons::A) && state.recorder.is_on())
			{
				if let Err(e) = state.recorder.toggle() {
					log::error!("recorder toggle failed with: {e}");
				}
			}

			if let Some(player) = state.player.take() {
				if input.controller.button_released(ControllerButtons::B) {
					state.player = Some(player.play());
				}
			}

			if let Err(e) = state.recorder.take_event(&input.controller) {
				log::error!("recorder failed to take event with: {e}");
			}

			let axes = input.controller.axes_as_f32();
			let d_power = axes[0];
			let t_power = axes[1];
			drive.drive(d_power, t_power);

			odom.update(&mut drive);
		}

		/*** Write motor outputs to V5 ***/
		state.write_serial_output();
		state.loop_delay(); // Make sure we sleep at least little bit each iteration
	}
}
