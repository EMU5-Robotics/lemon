pub mod motion_profile;
pub mod odom;
pub mod parts;
pub mod path;
pub mod pid;
pub mod replay;
pub mod state;
mod util;

pub use util::*;

use crate::{
	odom::DriveImuOdom,
	parts::drive::Drive,
	pid::AnglePid,
	state::{InputChanges, FieldControlState, Motor, ControllerButtons, GlobalState, InputState},
	units::*,
};

pub enum CompetitionState {
	Disabled,
	Autonomous,
	UserControl,
}

pub struct Robot<P> {
	pub competition: CompetitionState,
	pub state: GlobalState,
	pub input: InputState,
	pub odom: DriveImuOdom,
	pub base: Drive,
	pub parts: P,
	pub tpid: AnglePid,
	disabled: fn(&mut Robot<P>),
	user_control: fn(&mut Robot<P>),
	autonomous: fn(&mut Robot<P>),
}

impl<P> Robot<P> {
	pub fn setup<G, H>(
		create_parts: G,
		create_drive: H,
		disabled: Option<fn(&mut Robot<P>)>,
		user_control: Option<fn(&mut Robot<P>)>,
		autonomous: Option<fn(&mut Robot<P>)>,
	) -> anyhow::Result<Self>
	where
		G: Fn(&mut GlobalState) -> anyhow::Result<P>,
		H: Fn(&mut GlobalState) -> anyhow::Result<Drive>,
	{
		dotenvy::dotenv().ok();

		let mut state = GlobalState::new()?;
		let input = state.create_input_state();

		util::logging::setup_field_rerun(state.network.rerun_logger());

		let logger = state.network.rerun_logger();
		let odom = std::thread::spawn(move || DriveImuOdom::new(logger))
			.join()
			.unwrap()?;

		let parts = create_parts(&mut state)?;
		let base = create_drive(&mut state)?;

		let tpid = AnglePid::new(2.0, 0.0, 0.0, degree!(0.0));

		state.serial.update_gearboxes();

		let robot = Robot {
			competition: CompetitionState::Disabled,
			state,
			input,
			odom,
			base,
			parts,
			tpid,
			disabled: disabled.unwrap_or(Self::nop),
			user_control: user_control.unwrap_or(Self::nop),
			autonomous: autonomous.unwrap_or(Self::nop),
		};

		Ok(robot)
	}

	pub fn run(mut self) -> ! {
		loop {
			/*** Gather all input from serial, sensors, etc. ***/
			if let Some(status_pkt) = self.state.serial.take_status_pkt() {
				self.input.update_v5_status(status_pkt);
			} else {
				self.input.update_inputs();
			}
			self.input.overwrite_replay_input(&mut self.state.player);
			self.competition = self.input.compute_comp_state();

			/*** Process inputs to parts ***/
			self.odom.update(&mut self.base);

			match self.input.fcs_state() {
				FieldControlState::Joined => {}
				FieldControlState::Left => {}
				FieldControlState::Connected => {}
				FieldControlState::Disconnected => {}
			}

			match self.competition {
				CompetitionState::Disabled => (self.disabled)(&mut self),
				CompetitionState::Autonomous => (self.autonomous)(&mut self),
				CompetitionState::UserControl => (self.user_control)(&mut self),
			}

			/*** Write motor outputs to V5 ***/
			self.state.write_serial_output();
			self.state.loop_delay();
		}
	}

	fn nop(_: &mut Robot<P>) {}
}

impl<P> core::ops::Deref for Robot<P> {
	type Target = P;

	fn deref(&self) -> &Self::Target {
		&self.parts
	}
}

impl<P> core::ops::DerefMut for Robot<P> {
	fn deref_mut(&mut self) -> &mut Self::Target {
		&mut self.parts
	}
}

pub fn move_voltage(
	voltage: i16,
	down: ControllerButtons,
	up: ControllerButtons,
	motors: &[Motor],
    controller: InputChanges
) {
	let voltage = if controller.button_held(down) {
		-voltage
	} else if controller.button_held(up) {
		voltage
	} else {
		0
	};

	for motor in motors {
		motor.voltage(voltage);
	}
}
