use crate::{odom::DriveImuOdom, pid::*, units::*};

use super::{IntoSeg, PathSegment, Timer};

pub struct Aux {
	main: (Box<dyn PathSegment>, Timer),
	aux: (Box<dyn PathSegment>, Timer),
}

impl IntoSeg for Aux {}

impl Aux {
	#[allow(dead_code)]
	pub fn new(main: (Box<dyn PathSegment>, Timer), aux: (Box<dyn PathSegment>, Timer)) -> Self {
		Self { main, aux }
	}
}

impl PathSegment for Aux {
	fn start_follow(&mut self, odom: &DriveImuOdom, tpid: &mut AnglePid) {
		self.aux.1.start();
		self.main.1.start();
		self.aux.0.start_follow(odom, tpid);
		self.main.0.start_follow(odom, tpid);
	}
	fn follow(&mut self, odom: &DriveImuOdom, tpid: &mut AnglePid) -> Option<(Velocity, Velocity)> {
		let _ = self.aux.0.follow(odom, tpid);
		self.main.0.follow(odom, tpid)
	}
	fn end_follow(&mut self) {
		self.aux.0.end_follow();
		self.main.0.end_follow();
	}
}
