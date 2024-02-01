use std::sync::{Arc, RwLock};

use protocol::device::MotorState;

pub const MAX_MILLIVOLT: i32 = 12000;

// placeholder
#[derive(Debug, Clone)]
pub struct Motor {
	inner: Arc<RwLock<MotorInner>>,
	port: u8,
}

#[derive(Default, Debug, Clone)]
pub struct MotorInner {
	state: Option<MotorState>,
	target: Target,
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
			inner: Arc::default(),
			port,
		}
	}
	pub fn is_connected(&self) -> bool {
		self.inner.read().unwrap().state.is_some()
	}
	pub fn set_target(&mut self, target: Target) {
		match target {
			Target::Voltage(v) => assert!(v.abs() <= 12000),
			Target::PercentVoltage(v) => assert!(v.abs() <= 1.0),
			_ => {}
		}
		self.inner.write().unwrap().target = target;
	}
	// this function is marked as unsafe as it should only
	// be called from the brain struct with care
	pub unsafe fn set_inner(&mut self, new_inner: Option<MotorState>) {
		let inner_state = &mut self.inner.write().unwrap().state;
		*inner_state = new_inner;
	}
	pub fn port(&self) -> u8 {
		self.port
	}
	pub fn target(&self) -> Target {
		self.inner.read().unwrap().target
	}
}
