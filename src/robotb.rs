use communication::{
	packet::{FromMediator, ToMediator},
	plot,
};
const IMU_BIAS: f64 = 0.0004146448; //0.0002138361;

mod bmi088;
mod odom;

fn main() {
	let mut mediator = communication::Logger::init(false).expect("This only panics when another logger is set. This should never be the case and indicates a problem with the code.");

	let mut odom = odom::Odometry::new(IMU_BIAS);
	loop {
		odom.calc_position();
		if let Ok(events) = mediator.poll_events() {
			for event in events {
				match event {
					ToMediator::Ping => {
						if let Err(e) = mediator.send_event(FromMediator::Pong) {
							eprintln!("Failed to send Pong event: {e}");
						}
					}
					_ => {}
				}
			}
		}
		//plot!("heading", odom.heading().to_degrees());
		//plot!("position", odom.position());
		communication::odom(odom.position(), odom.heading());
		std::thread::sleep(std::time::Duration::from_millis(1));
	}
}
