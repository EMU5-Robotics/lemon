use robot_algorithms::prelude::*;
use uom::si::f64::Velocity;

pub fn get_profile_velocity(profile: (&[Velocity], &[Vec2]), pos: Pos2) -> Velocity {
	let (vel, points) = profile;
	assert_eq!(vel.len(), points.len());
	assert!(!vel.is_empty());

	let dist = |point: &Vec2| (pos - point).coords.magnitude_squared();

	points
		.iter()
		.zip(vel.iter())
		.reduce(|a, b| if dist(a.0) < dist(b.0) { a } else { b })
		.map(|(_, &v)| v)
		.unwrap()
}
