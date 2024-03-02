use communication::{
	packet::{FromMediator, ToMediator},
	plot,
};

fn main() {
	let mut mediator = communication::Logger::init().expect("This only panics when another logger is set. This should never be the case and indicates a problem with the code.");

	let spi = rppal::spi::Spi::new(
		rppal::spi::Bus::Spi0,
		rppal::spi::SlaveSelect::Ss0,
		100_000,
		rppal::spi::Mode::Mode0,
	)
	.unwrap();
	let spi_one = rppal::spi::Spi::new(
		rppal::spi::Bus::Spi0,
		rppal::spi::SlaveSelect::Ss1,
		100_000,
		rppal::spi::Mode::Mode0,
	)
	.unwrap();
	let spi_two = rppal::spi::Spi::new(
		rppal::spi::Bus::Spi0,
		rppal::spi::SlaveSelect::Ss2,
		100_000,
		rppal::spi::Mode::Mode0,
	)
	.unwrap();

	let mut delay = rppal::hal::Delay::new();
	let mut encoder = amt22::Amt22::new(spi, amt22::Resolution::Res12Bit);
	let mut encoder_one = amt22::Amt22::new(spi_one, amt22::Resolution::Res12Bit);
	let mut encoder_two = amt22::Amt22::new(spi_two, amt22::Resolution::Res12Bit);
	encoder.reset(Some(&mut delay)).unwrap();
	encoder_one.reset(Some(&mut delay)).unwrap();
	encoder_two.reset(Some(&mut delay)).unwrap();
	loop {
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
		let position = encoder.read_absolute_position_raw().unwrap();
		let position_one = encoder_one.read_absolute_position_raw().unwrap();
		let position_two = encoder_two.read_absolute_position_raw().unwrap();
		plot!(
			"enc0",
			position.0 as f64 * 360.0 + position.1 as f64 * 360.0 / 4095.0
		);
		plot!(
			"enc1",
			position_one.0 as f64 * 360.0 + position_one.1 as f64 * 360.0 / 4095.0
		);
		plot!(
			"enc2",
			position_two.0 as f64 * 360.0 + position_two.1 as f64 * 360.0 / 4095.0
		);
		log::info!("{position:?} | {position_one:?} | {position_two:?}");
		std::thread::sleep(std::time::Duration::from_millis(5));
	}
}
