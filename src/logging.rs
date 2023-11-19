use rerun::archetypes::TimeSeriesScalar;
use rerun::RecordingStream;

pub fn timeseries(rerun: &RecordingStream, name: &str, value: f64) {
	rerun.log(name, &TimeSeriesScalar::new(value)).unwrap();
}
pub fn timeseries_colour(rerun: &RecordingStream, name: &str, value: f64, col: [u8; 3]) {
	rerun
		.log(name, &TimeSeriesScalar::new(value).with_color(col))
		.unwrap();
}
