use std::{
	sync::{
		atomic::{AtomicBool, AtomicI16, AtomicI32, AtomicU32, AtomicU8, Ordering},
		Arc, Mutex,
	},
	time::{Duration, Instant},
};

use client::{
	coprocessor::serial::{find_v5_port, Serial, SerialSpawner},
	network::{listen_for_server, Client, ClientConfiguration},
};
use protocol::{device::ControllerButtons, ControlPkt, StatusPkt};
use rerun::{RecordingStream, RecordingStreamBuilder};

use crate::replay::{Player, Recorder};

pub struct GlobalState {
	/// The serial link to the V5
	pub serial: Serial,
	/// An either connected or disconnected network link
	pub network: Network,
	/// Replay system with a recorder and an optional current player
	pub recorder: Recorder,
	pub player: Option<Player>,
	/// Timing information
	loop_last: Instant,
	/// Hidden reference to motors used by components, used for output state extraction
	motors: Arc<[Motor]>,
	taken_motors: [bool; 20],
}

impl GlobalState {
	pub fn new() -> anyhow::Result<Self> {
		let network = match std::env::var("NETWORK_DEBUG") {
			Ok(_) => {
				let mut network = Network::connect();
				network.wait_for_rerun_server();
				network
			}
			Err(_) => Network::disconnected(),
		};

		let serial = {
			let ports = find_v5_port()?;
			let serial_port = SerialSpawner::open(&ports.0.port_name)?;
			serial_port.spawn_threaded(None)
		};

		let motors = (1..=20)
			.map(Motor::new)
			.collect::<Vec<_>>()
			.into_boxed_slice()
			.into();

		Ok(Self {
			serial,
			network,
			recorder: Recorder::new(),
			player: None,
			loop_last: Instant::now(),
			motors,
			taken_motors: [false; 20],
		})
	}

	pub fn take_motor(&self, port: usize, reversed: bool) -> Motor {
		assert!((1..=20).contains(&port), "Invalid motor port");
		match self.taken_motors[port - 1] {
			true => panic!("Motor has already been taken"),
			false => {
				let motor = self.motors[port - 1].clone();
				motor.set_reversed(reversed);
				motor
			}
		}
	}

	/// Only call this function once and reuse the input state
	pub fn create_input_state(&mut self) -> InputState {
		let v5_status = loop {
			// Wait until we get the first status packet back
			match self.serial.take_status_pkt() {
				Some(pkt) => break pkt,
				None => std::thread::sleep(Duration::from_millis(1)),
			}
		};

		InputState::new(v5_status, self.motors.clone())
	}

	pub fn write_serial_output(&mut self) {
		// Create blank control packet
		let devices = self.serial.copy_devices();
		let mut control = ControlPkt::from_devices(&devices);

		// Update the motors
		for motor in self.motors.iter() {
			// Check that the motor is first connected
			if devices.get_port(motor.port() as _).is_motor() {
				// Get the state of the output
				let power = motor.0.voltage.load(Ordering::Acquire);
				let reversed = motor.0.reversed.load(Ordering::Acquire);
				// Set the power
				control.set_power(motor.port() as _, if reversed { -power } else { power })
			}
		}

		// Set the control packet in the serial stream
		self.serial.set_control_pkt(control);
	}

	pub fn loop_delay(&mut self) {
		let current_time = Instant::now();
		let duration = Duration::from_millis(2).saturating_sub(current_time - self.loop_last);
		std::thread::sleep(duration);
		self.loop_last = current_time;
	}
}

pub struct InputState {
	pub v5_status: (Instant, StatusPkt),
	v5_status_last: (Instant, StatusPkt),
	pub controller: InputChanges,
	// gyro_rotation: f32
	motors: Arc<[Motor]>,
}

impl InputState {
	fn new(v5_status: (Instant, StatusPkt), motors: Arc<[Motor]>) -> Self {
		Self {
			v5_status_last: v5_status.clone(),
			v5_status,
			controller: InputChanges::NO_CHANGE,
			motors,
		}
	}

	pub fn update_v5_status(&mut self, v5_status: (Instant, StatusPkt)) {
		std::mem::swap(&mut self.v5_status_last, &mut self.v5_status);
		self.v5_status = v5_status;
		self.controller = InputChanges::from_difference(&self.v5_status_last.1, &self.v5_status.1);

		for (port, state) in self.v5_status.1.motors() {
			let motor = &self.motors[port as usize];
			motor.0.connected.store(true, Ordering::Release);
			motor.0.current.store(state.current as _, Ordering::Release);
		}

		for (port, position) in self.v5_status.1.encoders() {
			let motor = &self.motors[port as usize];
			motor.0.position.store(position, Ordering::Release);
		}
	}

	pub fn update_replay_input(&mut self, player: Option<&mut Player>) {
		let player = match player {
			Some(player) if player.is_playing() => player,
			_ => return,
		};

		// Override the input with the player input
		if let Some(event) = player.get_events().iter().next() {
			self.controller = event.1;
		}
	}
}

struct NetworkInner {
	network_client: Client,
}

#[derive(Clone)]
pub struct Network(Arc<Mutex<(Option<NetworkInner>, RerunLogger)>>);

impl Network {
	pub fn disconnected() -> Self {
		let rerun = RecordingStreamBuilder::new("lemon")
			.connect_opts(
				"192.168.222.179:9876".parse().unwrap(),
				Some(Duration::from_millis(2_000)),
			)
			.unwrap(); /*RecordingStreamBuilder::new("lemon")
		   .save("recording.rrd")
		   .unwrap();*/
		rerun::Logger::new(rerun.clone())
			.with_path_prefix("logs")
			.with_filter(rerun::default_log_filter())
			.init()
			.unwrap();
		let logger = RerunLogger(Instant::now(), Some(rerun));

		Network(Arc::new(Mutex::new((None, logger))))
	}

	pub fn connect() -> Self {
		let network = Network(Arc::new(Mutex::new((None, RerunLogger::default()))));

		let net = network.clone();

		std::thread::spawn(move || {
			// Try and find the server and connect to it
			let addr = listen_for_server().unwrap();
			let client = Client::connect_threaded(addr, ClientConfiguration::default());
			// Update resources
			net.0.lock().unwrap().0 = Some(NetworkInner {
				network_client: client,
			});
		});

		network
	}

	pub fn wait_for_rerun_server(&mut self) {
		let program_start = Instant::now();
		loop {
			let mut mutex = self.0.lock().unwrap();
			if let Some(net) = mutex.0.as_mut() {
				let pkt = net.network_client.rx.recv().unwrap();
				match pkt.msg {
					common::protocol::ControlMessage::ManageMessage(msg) => match msg {
						common::protocol::ManageMessage::AnnounceRerunServer(addr) => {
							let rerun =
								RecordingStreamBuilder::new(net.network_client.name.clone())
									.connect_opts(addr, Some(Duration::from_millis(2_000)))
									.unwrap_or(
										RecordingStreamBuilder::new("lemon (file fallback)")
											.save("recording.rrd")
											.unwrap(),
									);
							rerun::Logger::new(rerun.clone())
								.with_path_prefix("logs")
								.with_filter(rerun::default_log_filter())
								.init()
								.unwrap();
							mutex.1 = RerunLogger(program_start, Some(rerun));
							return;
						}
						_ => {}
					},
					_ => {}
				}
			}
			drop(mutex);
			std::thread::sleep(Duration::from_millis(1));
		}
	}

	pub fn rerun_logger(&self) -> RerunLogger {
		self.0.lock().unwrap().1.clone()
	}
}

#[derive(Clone)]
pub struct RerunLogger(Instant, Option<RecordingStream>);

impl RerunLogger {
	#[inline]
	pub fn with<F: FnOnce(&RecordingStream, Instant)>(&self, f: F) {
		match self.1 {
			Some(ref stream) => {
				f(stream, self.0);
			}
			None => (),
		}
	}
}

impl Default for RerunLogger {
	fn default() -> Self {
		RerunLogger(Instant::now(), None)
	}
}

#[derive(Debug, Copy, Clone)]
pub struct InputChanges {
	pub pressed: ControllerButtons,
	pub released: ControllerButtons,
	#[doc(hidden)] // use the function in stead
	pub held: ControllerButtons,
	#[doc(hidden)] // use the function instead
	pub axes: [i8; 4],
	#[doc(hidden)] // use the function instead
	pub axes_changed: bool,
}

impl InputChanges {
	pub const NO_CHANGE: Self = InputChanges {
		pressed: ControllerButtons::empty(),
		released: ControllerButtons::empty(),
		held: ControllerButtons::empty(),
		axes: [0; 4],
		axes_changed: false,
	};

	pub fn from_difference(last: &StatusPkt, current: &StatusPkt) -> Self {
		let pressed = current.controller_buttons & !last.controller_buttons;
		let released = !current.controller_buttons & last.controller_buttons;
		let held = current.controller_buttons;

		Self {
			pressed,
			released,
			held,
			axes: current.controller_axes,
			axes_changed: current.controller_axes == last.controller_axes,
		}
	}

	/// Returns `true` if there has been a change between the last and current pair
	/// of inputs from the user
	#[inline]
	pub fn change_occured(&self) -> bool {
		self.pressed != ControllerButtons::empty()
			|| self.released != ControllerButtons::empty()
			|| self.axes_changed
	}

	/// Return `true` if any of the provided buttons have been released at all.
	#[inline]
	pub fn button_released(&self, buttons: ControllerButtons) -> bool {
		self.released & buttons != ControllerButtons::empty()
	}

	/// Return `true` if any of the provided buttons have been pressed at all.
	#[inline]
	pub fn button_pressed(&self, buttons: ControllerButtons) -> bool {
		self.pressed & buttons != ControllerButtons::empty()
	}

	/// Return `true` if any of the provided buttons are being held at all.
	#[inline]
	pub fn button_held(&self, buttons: ControllerButtons) -> bool {
		self.held & buttons != ControllerButtons::empty()
	}

	#[inline]
	pub fn axes_changed(&self) -> bool {
		self.axes_changed
	}

	#[inline]
	pub fn axes(&self) -> [i8; 4] {
		self.axes.clone()
	}

	#[inline]
	pub fn axes_as_f32(&self) -> [f32; 4] {
		[
			self.axes[0] as f32 / 127.0,
			self.axes[1] as f32 / 127.0,
			self.axes[2] as f32 / 127.0,
			self.axes[3] as f32 / 127.0,
		]
	}
}

struct MotorInner {
	// Metadata
	port: AtomicU8,
	connected: AtomicBool,
	reversed: AtomicBool,
	// Sent
	voltage: AtomicI16,
	// Received
	position: AtomicI32,
	current: AtomicU32,
}

#[derive(Clone)]
pub struct Motor(Arc<MotorInner>);

impl Motor {
	fn new(port: u8) -> Self {
		let motor = MotorInner {
			port: AtomicU8::new(port),
			connected: AtomicBool::new(false),
			reversed: AtomicBool::new(false),
			voltage: AtomicI16::new(0),
			position: AtomicI32::new(0),
			current: AtomicU32::new(0),
		};
		Motor(Arc::new(motor))
	}

	#[inline]
	fn disconnect(&self) {
		self.0.connected.store(false, Ordering::Release);
		self.0.voltage.store(0, Ordering::Release);
		self.0.position.store(0, Ordering::Release);
		self.0.current.store(0, Ordering::Release);
	}

	#[inline]
	pub fn port(&self) -> u8 {
		self.0.port.load(Ordering::Acquire)
	}

	#[inline]
	pub fn is_connected(&self) -> bool {
		self.0.connected.load(Ordering::Acquire)
	}

	#[inline]
	pub fn set_reversed(&self, reverse: bool) {
		self.0.reversed.store(reverse, Ordering::Release);
	}

	#[inline]
	pub fn voltage(&self, voltage: i16) {
		self.0.voltage.store(voltage, Ordering::Release);
	}

	#[inline]
	pub fn position(&self) -> i32 {
		self.0.position.load(Ordering::Acquire)
	}

	#[inline]
	pub fn current(&self) -> u32 {
		self.0.current.load(Ordering::Acquire)
	}
}
