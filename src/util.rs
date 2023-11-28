pub mod logging {
	use crate::{state::RerunLogger, units::Length};
	use rerun::{archetypes::TimeSeriesScalar, RecordingStream, Transform3D};

	pub fn timeseries(rerun: &RecordingStream, name: &str, value: f64) {
		rerun.log(name, &TimeSeriesScalar::new(value)).unwrap();
	}

	pub fn timeseries_colour(rerun: &RecordingStream, name: &str, value: f64, col: [u8; 3]) {
		rerun
			.log(name, &TimeSeriesScalar::new(value).with_color(col))
			.unwrap();
	}

	pub fn setup_field_rerun(logger: RerunLogger) {
		logger.with(|rec, _| {
			rec.log_timeless("/", &rerun::ViewCoordinates::RIGHT_HAND_Z_UP)
				.unwrap();
			rec.log_timeless(
				"xyz",
				&rerun::Arrows3D::from_vectors([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
					.with_colors([[255, 0, 0], [0, 255, 0], [0, 0, 255]]),
			)
			.unwrap();

			let robot_center = [-1.0, -1.5, 0.0016 + 0.095];

			let robot = rerun::Boxes3D::from_centers_and_half_sizes(
				[robot_center],
				[(0.175, 0.195, 0.095)],
			)
			.with_rotations([rerun::RotationAxisAngle::new(
				(0.0, 0.0, 1.0),
				rerun::Angle::Degrees(0.0),
			)])
			.with_radii([0.005])
			.with_colors([rerun::Color::from_rgb(255, 0, 0)])
			.with_labels(["robot"]);

			rec.log_timeless("robot", &robot).unwrap();
			rec.flush_blocking();
		});
	}

	pub fn _set_robot_offset(rerun: &RecordingStream, offset: (Length, Length)) {
		rerun
			.log_timeless(
				"robot",
				&Transform3D::from_translation([offset.0.value as f32, offset.1.value as f32, 0.0]),
			)
			.unwrap();
	}
}

pub mod units {
	pub use uom::si::f64::*;
	pub use uom::ConstZero;

	macro_rules! unit {
		($unit:ident, $class_lower:ident, $class_upper:ident) => {
			pub mod $unit {
				#[macro_export]
				macro_rules! $unit {
					($v:expr) => {
						$class_upper::new::<uom::si::$class_lower::$unit>($v)
					};
				}
				pub use $unit;
			}
		};
	}

	unit!(degree, angle, Angle);
	unit!(degree_per_second, angular_velocity, AngularVelocity);
	unit!(kilogram_meter_per_second, momentum, Momentum);
	unit!(meter, length, Length);
	unit!(meter_per_second, velocity, Velocity);
	unit!(millisecond, time, Time);
	unit!(newton, force, Force);
	unit!(radian, angle, Angle);
	unit!(radian_per_second, angular_velocity, AngularVelocity);
	unit!(revolution_per_minute, angular_velocity, AngularVelocity);
	unit!(second, time, Time);
	unit!(watt, power, Power);
	unit!(watt_per_meter, linear_power_density, LinearPowerDensity);

	pub use degree::degree;
	pub use degree_per_second::degree_per_second;
	pub use meter::meter;
	pub use meter_per_second::meter_per_second;
	pub use millisecond::millisecond;
	pub use radian::radian;
	pub use revolution_per_minute::revolution_per_minute;
	pub use second::second;
	pub use watt::watt;
}
