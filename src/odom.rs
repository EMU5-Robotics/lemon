use std::f64::consts::PI;
use std::time::{Duration, Instant};

use uom::ConstZero;

use crate::{logging::*, parts::drive::Drive, state::RerunLogger, units::*};

use ringbuffer::{ConstGenericRingBuffer, RingBuffer};
type RingBuf<T, const N: usize> = ConstGenericRingBuffer<T, N>;

fn get_vel<const N: usize>(rb: &RingBuf<Velocity, N>) -> Velocity {
	rb.iter().cloned().reduce(|a, b| a + b).unwrap() / N as f64
}

pub struct DriveOdom {
	dist: (Length, Length),
	angle: Angle,
	dw: Length,
	pos: (Length, Length),
	update: Instant,
	vel: (Velocity, Velocity),
	side_vels: (RingBuf<Velocity, 5>, RingBuf<Velocity, 5>),
	last_log: Instant,
	logger: RerunLogger,
}

impl DriveOdom {
	pub fn new(logger: RerunLogger, dw: Length) -> Self {
		Self {
			dist: (ConstZero::ZERO, ConstZero::ZERO),
			dw,
			angle: ConstZero::ZERO,
			pos: (ConstZero::ZERO, ConstZero::ZERO),
			update: Instant::now(),
			vel: (ConstZero::ZERO, ConstZero::ZERO),
			side_vels: (RingBuf::new(), RingBuf::new()),
			last_log: Instant::now(),
			logger,
		}
	}

	pub fn update(&mut self, drive: &mut Drive) -> Option<()> {
		if self.update.elapsed() < Duration::from_micros(5200) {
			return Some(());
		}
		// get displacement and change in displacement
		// relative to the sides on the robot
		let (dist_l, dist_r) = drive.get_encoders()?;
		let (diff_l, diff_r) = (dist_l - self.dist.0, dist_r - self.dist.1);

		let time = second!(self.update.elapsed().as_secs_f64());
		self.side_vels.0.push(diff_l / time);
		self.side_vels.1.push(diff_r / time);

		// calculate displacement and change in angle for center of the robot
		let diff_dist = 0.5 * (diff_l + diff_r);
		let diff_angle: Angle = (0.5 * (diff_r - diff_l) / self.dw).into();

		// calculate displacement in global space
		let tmp = self.angle + 0.5 * diff_angle;
		let (dx, dy) = tmp.sin_cos();
		let (dx, dy) = (diff_dist * dx, diff_dist * dy);

		// apply global displacement and rotation
		self.pos.0 -= dx;
		self.pos.1 += dy;
		self.angle += diff_angle;

		// update state and calculate running averages
		self.dist = (dist_l, dist_r);
		self.update = Instant::now();

		let vel = (dx / time, dy / time);

		self.vel = vel;
		if self.last_log.elapsed() > Duration::from_millis(0) {
			self.last_log = Instant::now();
			self.logger.with(|rerun, start| {
				rerun.set_time_seconds("odom", start.elapsed().as_secs_f64());

				timeseries(rerun, "odom/side_vel/x", get_vel(&self.side_vels.0).value);
				timeseries(rerun, "odom/side_vel/y", get_vel(&self.side_vels.1).value);

				// encoders
				timeseries_colour(rerun, "odom/encoders/left", dist_l.value, [255, 0, 0]);
				timeseries_colour(rerun, "odom/encoders/right", dist_r.value, [0, 255, 0]);

				// encoders (raw)
				let (dist_l, dist_r) = drive
					.get_encoders_raw()
					.unwrap_or((meter!(0.0), meter!(0.0)));
				timeseries_colour(rerun, "odom/raw/encoders/left", dist_l.value, [127, 0, 0]);
				timeseries_colour(rerun, "odom/raw/encoders/right", dist_r.value, [0, 127, 0]);

				// pos
				timeseries_colour(rerun, "odom/pos/x", self.pos.0.value, [127, 0, 127]);
				timeseries_colour(rerun, "odom/pos/y", self.pos.1.value, [0, 127, 127]);
				timeseries_colour(
					rerun,
					"odom/angle",
					self.angle.value * 180.0 / PI,
					[127, 127, 127],
				);
			});
		}
		Some(())
	}

	pub fn pos(&self) -> (Length, Length) {
		self.pos
	}

	pub fn angle(&self) -> Angle {
		self.angle
	}

	pub fn vel(&self) -> (Velocity, Velocity) {
		self.vel
	}

	pub fn side_vel(&self) -> (Velocity, Velocity) {
		(get_vel(&self.side_vels.0), get_vel(&self.side_vels.1))
	}
}
