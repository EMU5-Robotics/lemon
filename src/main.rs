use std::time::Duration;

use fern::colors::{Color, ColoredLevelConfig};

fn main() {
	create_logger();

	let port = client::coprocessor::Port::open("/dev/ttyACM1").unwrap();
	let data = port.spawn_threaded();

	let mut i = 0;
	loop {
		let input = { data.recv_pkt_lock().unwrap().clone() };
		let mut output = { data.send_pkt_lock().unwrap().clone() };

		let voltage = (input.controller_axes[1] as f64 / 127.0) * 12.0;
		output.set_motor(0, (voltage * 1000.0) as i16);
		output.set_motor(1, (voltage * -1000.0) as i16);

		if i % 50 == 0 {
			log::info!("recv: {:?}", input);
			log::info!("send: {:?}\n", output);
		}
		i += 1;

		{
			*data.send_pkt_lock().unwrap() = output;
		}

		std::thread::sleep(Duration::from_millis(10));
	}
}

pub fn create_logger() {
	let colors = ColoredLevelConfig::new()
		.error(Color::Red)
		.warn(Color::Yellow)
		.info(Color::Cyan)
		.debug(Color::Magenta);

	fern::Dispatch::new()
		.format(move |out, message, record| {
			out.finish(format_args!(
				"{} {} [{}] {}",
				chrono::Local::now().format("%H:%M:%S"),
				colors.color(record.level()),
				record.target(),
				message
			))
		})
		.level(log::LevelFilter::Debug)
		.chain(std::io::stderr())
		.apply()
		.unwrap();
}
