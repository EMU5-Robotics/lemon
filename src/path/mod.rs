use std::collections::VecDeque;

use crate::{odom::DriveImuOdom, pid::*, units::*};
use std::time::{Duration, Instant};

mod auxiliary;
pub mod nop;
pub mod odomcond;
pub mod pathseg;
pub mod powermotors;
pub mod setvel;
pub mod turnto;

pub use auxiliary::*;
pub use nop::*;
pub use odomcond::*;
pub use pathseg::*;
pub use powermotors::*;
pub use setvel::*;
pub use turnto::*;

pub struct Path {
	segments: VecDeque<(Box<dyn PathSegment>, Timer)>,
	current_segment: Option<(Box<dyn PathSegment>, Timer)>,
}

impl Path {
	pub fn new(segments: Vec<(Box<dyn PathSegment>, Timer)>) -> Self {
		Self {
			segments: segments.into(),
			current_segment: None,
		}
	}
	pub fn follow(
		&mut self,
		odom: &DriveImuOdom,
		turn_pid: &mut AnglePid,
	) -> Option<(Velocity, Velocity)> {
		// check if there is an active segment
		if let Some((ref mut seg, timer)) = &mut self.current_segment {
			// exit if timer is up
			if timer.done() {
				seg.end_follow();
			} else if let Some(vels) = seg.follow(odom, turn_pid) {
				return Some(vels);
			}
		}

		// try get new active segment
		self.current_segment = self.segments.pop_front();

		// new segment exists, start Following
		// and then return follow from recursive call
		if let Some((ref mut seg, ref mut timer)) = &mut self.current_segment {
			seg.start_follow(odom, turn_pid);
			timer.start();
			return self.follow(odom, turn_pid);
		}

		// new segment does not exist return None
		None
	}
}

pub trait PathSegment {
	fn transform<'a>(a: Box<Self>, _: &DriveImuOdom) -> (Box<dyn PathSegment + 'a>, Timer)
	where
		Self: Sized + 'a,
	{
		(a, Default::default())
	}
	fn start_follow(&mut self, _: &DriveImuOdom, _: &mut AnglePid) {}
	fn follow(
		&mut self,
		odom: &DriveImuOdom,
		turn_pid: &mut AnglePid,
	) -> Option<(Velocity, Velocity)>;
	fn end_follow(&mut self) {}
}

pub trait IntoSeg
where
	Self: Sized + PathSegment,
{
	fn default_timer<'a>(self) -> (Box<dyn PathSegment + 'a>, Timer)
	where
		Self: 'a,
	{
		self.with_timer(Duration::from_secs(100))
	}
	fn with_timer<'a>(self, dur: Duration) -> (Box<dyn PathSegment + 'a>, Timer)
	where
		Self: 'a,
	{
		(Box::new(self), Timer::new(dur))
	}
}

impl Default for Timer {
	fn default() -> Self {
		Self::new(Duration::from_secs(100))
	}
}

#[derive(Debug)]
pub struct Timer {
	start: Instant,
	duration: Duration,
}

impl Timer {
	pub fn new(dur: Duration) -> Self {
		Self {
			start: Instant::now(),
			duration: dur,
		}
	}
	pub fn start(&mut self) {
		self.start = Instant::now();
	}
	pub fn done(&self) -> bool {
		self.start.elapsed() > self.duration
	}
}
