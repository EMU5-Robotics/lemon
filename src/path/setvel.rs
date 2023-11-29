use super::{IntoSeg, PathSegment};
use crate::{odom::DriveImuOdom, pid::*, units::*};

pub struct SetVel {
	vel: (Velocity, Velocity),
}

impl IntoSeg for SetVel {}

impl SetVel {
	pub fn new(vel: (Velocity, Velocity)) -> Self {
		Self { vel }
	}
}

impl PathSegment for SetVel {
	fn follow(&mut self, _: &DriveImuOdom, _: &mut AnglePid) -> Option<(Velocity, Velocity)> {
		Some(self.vel)
	}
}
