pub struct Loader {
	motor_a: Motor,
	motor_b: Motor,
	armed_position: i32,
	state: State,
}

#[derive(Debug, Clone, Copy)]
pub enum State {
	InvalidPosition,
	Armed,
	Arming,
	Loading,
	Loaded,
}

fn calibrate(&mut self) {
	// Move loader down
	motor_b.voltage(-0.7);
	motor_a.voltage(-0.7);

	// Check if we have hit the bar
	let current = (motor_a.current() + motor_b.current()) / 2.0;
	if current > STALL_THRESHOLD_AMPS {
		self.armed_position = (motor_a.position() + motor_b.position()) / 2.0;
		self.state = State::Armed;
	} else {
		self.state = State::InvalidPosition;
	}
}

fn arm(&mut self) {
	// Move loader down
	motor_b.voltage(-0.7);
	motor_a.voltage(-0.7);

	// Check if we are near position
}

fn load(&mut self) {

}
