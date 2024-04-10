use communication::{
    packet::{FromMediator, ToMediator},
    plot,
};

mod brain;
mod controller;
mod loader;
mod motor;
mod robot;

use brain::Brain;

use loader::Loader;
use motor::Motor;
use protocol::device::ControllerButtons;
use robot::RobotState;

use controller::Controller;
use std::time::Duration;

use communication::Mediator;

const IS_SKILLS: bool = true;
pub const BRAIN_TIMEOUT: Duration = Duration::from_millis(500);

fn main() -> ! {
    Robot::run();
}

struct Robot {
    state: RobotState,
    brain: Brain,
    controller: Controller,
    mediator: Mediator,
    loader: Loader,
}

// merge or move these functions?
impl Robot {
    pub fn run() -> ! {
        let mut robot = Self::new();
        robot.main_loop();
    }
    pub fn new() -> Self {
        let mediator = communication::Logger::init(true).expect("This only panics when another logger is set. This should never be the case and indicates a problem with the code.");

        // block until connection is establish with brain
        log::info!("Connecting to the brain.");
        let (brain, controller) = Brain::init();
        log::info!("Connected to the brain.");

        let loader = Loader::new([(4, false), (2, true)], &brain);

        Self {
            state: RobotState::default(),
            brain,
            controller,
            mediator,
            loader,
        }
    }
    pub fn handle_events(&mut self) {
        if let Ok(events) = self.mediator.poll_events() {
            for event in events {
                match event {
                    ToMediator::Ping => {
                        if let Err(e) = self.mediator.send_event(FromMediator::Pong) {
                            eprintln!("Failed to send Pong event: {e}");
                        }
                    }
                    _ => {}
                }
            }
        }
    }
    pub fn main_loop(&mut self) -> ! {
        //let mut puncher_motor = self.brain.get_motor(3);
        loop {
            self.handle_events();

            // updates controller, robot state & motors
            let new_state = self.brain.update_state(&mut self.controller, &self.state);
            if new_state != self.state {
                log::info!("State changed from {:?} to {new_state:?}", self.state);
            }
            self.state = new_state;

            match self.state {
                RobotState::DriverAuton => {}
                RobotState::DriverSkills => {
                    self.driver();
                }
                RobotState::DriverDriver => {
                    self.driver();
                }
                _ => {}
            }
        }
    }
    fn driver(&mut self) {
        let mut forward_rate = self.controller.ly();
        //let forward_rate_two = self.controller.ry();
        /*if forward_rate < 0.0 {
            forward_rate *= 0.5;
        }*/

        if self.controller.held(ControllerButtons::Y) {
            forward_rate = 1.0;
        }
        self.loader.set_side_percent_voltage(forward_rate);

        /*if forward_rate_two == 0.0 {
            puncher.set_target(motor::Target::RotationalVelocity(0));
        } else {
            puncher.set_target(motor::Target::PercentVoltage(forward_rate_two));
        }*/

        self.brain.write_changes();
        std::thread::sleep(std::time::Duration::from_millis(1));
    }
}
