use std::sync::{Arc, RwLock};

use protocol::device::MotorState;

pub const MAX_MILLIVOLT: i32 = 12000;

// placeholder
#[derive(Debug, Clone)]
pub struct Motor {
	inner: Arc<RwLock<Option<MotorState>>>,
	target: Target,
	port: u8,
}

#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub enum Target {
	#[default]
	None,
	RotationalVelocity(i16),
	Voltage(i16),
	PercentVoltage(f64),
}

impl Motor {
	pub fn from_port(port: u8) -> Self {
		assert!((1..=20).contains(&port));
		Self {
			inner: Default::default(),
			target: Target::default(),
			port,
		}
	}
	pub fn set_target(&mut self, target: Target) {
		match target {
			Target::Voltage(v) => assert!(v.abs() <= 12000),
			Target::PercentVoltage(v) => assert!(v.abs() <= 1.0),
			_ => {}
		}
		self.target = target;
	}
	// this function is marked as unsafe as it should only
	// be called from the brain struct with care
	pub unsafe fn set_inner(&mut self, new_inner: Option<MotorState>) {
		let mut inner = self.inner.write().unwrap();
		*inner = new_inner;
	}
	pub fn port(&self) -> u8 {
		self.port
	}
	pub fn target(&self) -> Target {
		self.target
	}
}
