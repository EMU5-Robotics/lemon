use std::time::Instant;

use client::coprocessor::serial::{find_v5_port, Serial, SerialSpawner};
use protocol::{
	device::{CompetitionState, ControllerButtons, Gearbox},
	ControlPkt, StatusPkt,
};

use crate::{
	controller::Controller,
	motor::{self, Motor},
	odom::Odometry,
	RobotState,
};

// this is not designed to ever be mutated
#[derive(Debug, Clone)]
pub struct Packet {
	timestamp: Instant,
	brain_state: State,
	pub buttons: ControllerButtons,
	pub axes: [i8; 4],
}

// The functions here are mainly for constructing
// robot state later they should not be directly
// used by the end user.
impl Packet {
	pub fn timestamp(&self) -> Instant {
		self.timestamp
	}
	pub fn brain_state(&self) -> State {
		self.brain_state
	}
}

impl From<(Instant, StatusPkt)> for Packet {
	fn from((timestamp, pkt): (Instant, StatusPkt)) -> Self {
		Self {
			timestamp,
			brain_state: pkt.state.into(),
			buttons: pkt.controller_buttons,
			axes: pkt.controller_axes,
		}
	}
}

pub struct Brain {
	serial: Serial,
	pkt_buffer: [Packet; 2],
	last_update: Instant,
	motors: [Motor; 20],
}

impl Brain {
	pub fn init() -> (Self, Controller) {
		// try establish connection
		let serial_port = loop {
			std::thread::yield_now();
			let Ok(port) = find_v5_port() else {
				continue;
			};
			let Ok(sp) = SerialSpawner::open(&port.0.port_name) else {
				continue;
			};
			break sp;
		};
		let serial = serial_port.spawn_threaded(None);

		let first = loop {
			std::thread::yield_now();
			let Some(pkt) = serial.take_status_pkt() else {
				continue;
			};
			break pkt;
		};
		let second = loop {
			std::thread::yield_now();
			let Some(pkt) = serial.take_status_pkt() else {
				continue;
			};
			break pkt;
		};

		let pkt_buffer = [first.into(), second.into()];

		(
			Self {
				serial,
				pkt_buffer: pkt_buffer.clone(),
				last_update: Instant::now(),
				motors: (1..=20)
					.map(|port| unsafe { Motor::from_port(port) })
					.collect::<Vec<_>>()
					.try_into()
					.unwrap(),
			},
			pkt_buffer.into(),
		)
	}
	// this function is intended to update the robot state
	// false indicates that there is no update
	pub fn update_state(
		&mut self,
		controller: &mut Controller,
		robot_state: &mut RobotState,
		odom: &mut Odometry,
	) -> bool {
		if let Some(data_pkt) = self.serial.take_status_pkt() {
			self.read_motors(&data_pkt.1);
			self.pkt_buffer[1] = data_pkt.into();
			self.pkt_buffer.swap(0, 1);
			self.last_update = Instant::now();

			robot_state.progress(self.pkt_buffer[0].brain_state(), odom);
			*controller = self.pkt_buffer.clone().into();

			true
		} else {
			if self.last_update.elapsed() > crate::BRAIN_TIMEOUT && *robot_state != RobotState::Off
			{
				log::warn!("Connection to the brain has been lost.");
				*robot_state = RobotState::Off;
			}

			false
		}
	}
	pub fn write_changes(&mut self) {
		let mut ctrl_pkt = ControlPkt::default();

		for motor in &self.motors {
			let port = motor.port() as usize;
			if !motor.is_connected() {
				continue;
			}
			match motor.target() {
				motor::Target::Voltage(v) => ctrl_pkt.set_power(port, v, false),
				motor::Target::PercentVoltage(v) => {
					ctrl_pkt.set_power(port, (v * motor::MAX_MILLIVOLT as f64) as i16, false);
				}
				motor::Target::RotationalVelocity(v) => ctrl_pkt.set_power(port, v, true),
				motor::Target::None => ctrl_pkt.set_power(port, 0, false),
			}
		}

		self.serial.set_control_pkt(ctrl_pkt);
	}
	pub fn set_gearboxes(&mut self, gearbox: Gearbox, ports: impl IntoIterator<Item = u8>) {
		self.serial
			.set_gearboxes(ports.into_iter().map(|p| (p, gearbox)));
		self.serial.update_gearboxes();
	}
	fn read_motors(&mut self, status_pkt: &StatusPkt) {
		for motor in &mut self.motors {
			unsafe {
				motor.set_inner(status_pkt.get_motor_state(motor.port() as usize));
			}
		}
	}
	// This should be the only fatal failure point of the robot
	// use get_motor with care (should we make this unsafe?)
	pub fn get_motor(&self, port: u8) -> Motor {
		assert!((1..=20).contains(&port));
		self.motors[port as usize - 1].clone()
	}
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
	Disabled,
	Driver,
	Auton,
}

impl From<CompetitionState> for State {
	fn from(cs: CompetitionState) -> Self {
		if CompetitionState::DISABLED & cs != CompetitionState::empty() {
			Self::Disabled
		} else if CompetitionState::AUTONOMOUS & cs != CompetitionState::empty() {
			Self::Auton
		} else {
			Self::Driver
		}
	}
}
