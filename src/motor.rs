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
	// this function is only considered safe when called from the brain
	// to create singular set of 20 unique motors
	pub unsafe fn from_port(port: u8) -> Self {
		assert!((1..=20).contains(&port));
		Self {
			inner: Arc::default(),
			port,
		}
	}
	pub fn is_connected(&self) -> bool {
		let Ok(reader) = self.inner.read() else {
			log::error!(
				"Motor on port {} has poisoned lock! Failed to read state.",
				self.port
			);
			return false;
		};
		reader.state.is_some()
	}
	pub fn set_target(&mut self, mut target: Target) {
		match target {
			Target::Voltage(ref mut v) => {
				if v.abs() > 12000 {
					log::warn!("A voltage of {v}mV was passed to set_target. Clamping.");
					*v = (*v).clamp(-12000, 12000);
				}
			}
			Target::PercentVoltage(ref mut v) => {
				if !v.is_normal() && *v != 0.0 {
					log::warn!(
						"An invalid signed percent voltage of {v}mV was passed to set_target. Ignoring."
					);
					return;
				} else if v.abs() > 1.0 {
					log::warn!(
						"A percent voltage of {}% was passed to set_target. Clamping",
						*v * 100.0
					);
					*v = (*v).clamp(-1.0, 1.0);
				}
			}
			_ => {}
		}

		let Ok(ref mut writer) = self.inner.write() else {
			log::error!(
				"Motor on port {} has poisoned lock! Failed to set target for motor.",
				self.port
			);
			return;
		};

		writer.target = target;
	}
	// this function is marked as unsafe as it should only
	// be called from the brain struct with care
	pub unsafe fn set_inner(&mut self, new_inner: Option<MotorState>) {
		let Ok(ref mut writer) = self.inner.write() else {
			log::error!(
				"Motor on port {} has poisoned lock! Failed to set inner for motor.",
				self.port
			);
			return;
		};
		writer.state = new_inner;
	}
	pub fn port(&self) -> u8 {
		self.port
	}
	pub fn target(&self) -> Target {
		let Ok(reader) = self.inner.read() else {
			log::error!(
				"Motor on port {} has poisoned lock! Failed to read target.",
				self.port
			);
			return Target::None;
		};
		reader.target
	}
}
