use std::time::Duration;

use fern::colors::{Color, ColoredLevelConfig};

fn main() {
	create_logger();

	let port = client::coprocessor::Port::open("/dev/ttyACM1").unwrap();
	let _data = port.spawn_threaded();
	// let res = client::coprocessor::rtt_test(&mut port, 1024, 1, 0xAA).unwrap();
	// println!("{}", client::coprocessor::rtt_score(&res));

	loop {
		// log::info!("recv: {:?}", data.recv_pkt_lock().unwrap());
		// log::info!("send: {:?}\n", data.send_pkt_lock().unwrap());

		std::thread::sleep(Duration::from_millis(100));
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
