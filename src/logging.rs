use crate::{state::RerunLogger, Length};
use rerun::archetypes::TimeSeriesScalar;
use rerun::{RecordingStream, Transform3D};

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

		let robot =
			rerun::Boxes3D::from_centers_and_half_sizes([robot_center], [(0.175, 0.195, 0.095)])
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
