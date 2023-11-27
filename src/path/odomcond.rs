use crate::{odom::DriveImuOdom, pid::*, state::Motor, units::*};

use uom::ConstZero;

use super::{IntoSeg, PathSegment, Timer};

pub struct OdomCond<F>
where
	F: Fn(&DriveImuOdom) -> bool,
{
	cond: F,
	a: (Box<dyn PathSegment>, Timer),
	b: (Box<dyn PathSegment>, Timer),
}

impl<F> IntoSeg for OdomCond<F> where F: for<'a> Fn(&'a DriveImuOdom) -> bool {}

impl<F> OdomCond<F>
where
	F: Fn(&DriveImuOdom) -> bool,
{
	pub fn new(
		cond: F,
		a: (Box<dyn PathSegment>, Timer),
		b: (Box<dyn PathSegment>, Timer),
	) -> Self {
		Self { cond, a, b }
	}
}

impl<F> PathSegment for OdomCond<F>
where
	F: Fn(&DriveImuOdom) -> bool,
{
	fn transform<'a>(a: Box<Self>, odom: &DriveImuOdom) -> (Box<dyn PathSegment + 'a>, Timer)
	where
		Self: 'a,
	{
		if (a.cond)(odom) {
			a.a
		} else {
			a.b
		}
	}
	fn start_follow(&mut self, _: &DriveImuOdom, _: &mut AnglePid) {
		unreachable!()
	}
	fn follow(&mut self, _: &DriveImuOdom, _: &mut AnglePid) -> Option<(Velocity, Velocity)> {
		unreachable!()
	}
	fn end_follow(&mut self) {
		unreachable!()
	}
}
