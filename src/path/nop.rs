use crate::{odom::DriveImuOdom, pid::*, units::*};

use uom::ConstZero;

use super::{IntoSeg, PathSegment};

pub struct Nop {}

impl IntoSeg for Nop {}

impl Nop {
	#[allow(dead_code)]
	pub fn new() -> Self {
		Self {}
	}
}

impl PathSegment for Nop {
	fn follow(&mut self, _: &DriveImuOdom, _: &mut AnglePid) -> Option<(Velocity, Velocity)> {
		Some((ConstZero::ZERO, ConstZero::ZERO))
	}
}
