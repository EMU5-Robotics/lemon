use crate::{Angle, Length};
use rerun::archetypes::TimeSeriesScalar;
use rerun::{RecordingStream, Rotation3D, RotationAxisAngle, Transform3D};

pub fn timeseries(rerun: &RecordingStream, name: &str, value: f64) {
	rerun.log(name, &TimeSeriesScalar::new(value)).unwrap();
}
pub fn timeseries_colour(rerun: &RecordingStream, name: &str, value: f64, col: [u8; 3]) {
	rerun
		.log(name, &TimeSeriesScalar::new(value).with_color(col))
		.unwrap();
}

pub fn set_robot_offset(rerun: &RecordingStream, offset: (Length, Length), rotation: Angle) {
	rerun
		.log_timeless(
			"robot",
			&Transform3D::from_translation(
				[offset.0.value as f32, offset.1.value as f32, 0.0],
				/*Rotation3D::AxisAngle(
					RotationAxisAngle::new(
						[0.0, 0.0, 1.0],
						rerun::Angle::Radians(rotation.value as _),
					)
					.into(),
				),*/
			),
		)
		.unwrap();
}
