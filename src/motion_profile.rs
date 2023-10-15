use robot_algorithms::prelude::*;
use uom::si::f64::Velocity;

pub fn get_profile_velocity(profile: (&[Velocity], &[Vec2]), pos: Pos2) -> Velocity {
	let (vel, points) = profile;
	let len = points.len();
	assert_eq!(vel.len(), len);
	assert!(!vel.is_empty());

	let dist = |point: &Vec2| (pos - point).coords.magnitude_squared();

	// find index of closest point
	let min_i = points
		.iter()
		.enumerate()
		.reduce(|a, b| if dist(a.1) < dist(b.1) { a } else { b })
		.map(|(i, _)| i)
		.unwrap();

	// interpolate between adjacent points
	let indicies = if min_i == 0 {
		[0, 1, 3]
	} else if min_i == len - 1 {
		[len - 3, len - 2, len - 1]
	} else {
		[min_i - 1, min_i, min_i + 1]
	};

	// interpolate velocities based on inverse distance
	let distances: Vec<_> = indicies
		.iter()
		// prevent inf
		.map(|i| 1.0 / dist(&points[*i]).max(1e-6))
		.collect();
	let dist_sum: f64 = distances.iter().sum();
	distances
		.iter()
		.zip(vel.iter())
		.map(|(d, v)| (d / dist_sum) * *v)
		.sum()
}
