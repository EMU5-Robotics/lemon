mod bmi088;
mod brain;
mod controller;
mod drivebase;
mod motor;
mod odom;
mod path;
mod pid;
mod robot;

use brain::Brain;
use communication::{
    packet::{FromMediator, ToMediator},
    Mediator,
};
use controller::Controller;
use drivebase::Tankdrive;
use odom::Odometry;
use pid::Pid;
use protocol::device::ControllerButtons;
use robot::RobotState;

use std::time::Duration;

use crate::bmi088::ROBOT_A_IMU_BIAS;

const IS_SKILLS: bool = true;
pub const BRAIN_TIMEOUT: Duration = Duration::from_millis(500);

fn main() -> ! {
    Robot::run();
}

struct Robot {
    state: RobotState,
    brain: Brain,
    controller: Controller,
    drivebase: Tankdrive<3>,
    mediator: Mediator,
    odom: Odometry,
    pid_angle: Pid,
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
        let (mut brain, controller) = Brain::init();
        log::info!("Connected to the brain.");

        // this is the drivetrain configuration for the nationals hang robot
        let drivebase = Tankdrive::new(
            [(11, false), (12, true), (17, true)],
            [(14, false), (15, true), (16, false)],
            protocol::device::Gearbox::Blue,
            &mut brain,
        );

        let odom = Odometry::new(ROBOT_A_IMU_BIAS);

        Self {
            state: RobotState::default(),
            brain,
            controller,
            drivebase,
            mediator,
            odom,
            pid_angle: Pid::new(0.35, 0.035, 2.2),
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
                    ToMediator::Pid((kp, ki, kd)) => {
                        self.pid_angle.kp = kp;
                        self.pid_angle.ki = ki;
                        self.pid_angle.kd = kd;
                        self.pid_angle.reset();
                        log::info!("PID values (angle) changed to {kp}|{ki}|{kd}");
                    }
                    _ => {}
                }
            }
        }
    }
    pub fn main_loop(&mut self) -> ! {
        let mut is_pid = false;
        loop {
            self.handle_events();

            // updates controller, robot state & motors
            let new_state = self
                .brain
                .update_state(&mut self.controller, &mut self.state);
            if new_state != self.state {
                log::info!("State changed from {:?} to {new_state:?}", self.state);

                // reset odom at start of auton
                if new_state == RobotState::AutonSkills || new_state == RobotState::DriverAuton {
                    self.odom.reset();
                }
            }
            self.state = new_state;

            match self.state {
                RobotState::Off | RobotState::Disabled => {}
                RobotState::AutonSkills => self.auton_skills(),
                RobotState::DriverAuton => {}
                RobotState::DriverSkills => {
                    self.driver(&mut is_pid);
                }
                RobotState::DriverDriver => {
                    self.driver(&mut is_pid);
                }
            }
        }
    }
    fn driver(&mut self, is_pid: &mut bool) {
        self.odom.calc_position();

        communication::odom(self.odom.position(), self.odom.heading());
        let forward_rate = self.controller.ly();
        let turning_rate = self.controller.rx();
        let (mut l, mut r) = (
            (forward_rate + turning_rate * TURN_MULTIPLIER).clamp(-1.0, 1.0),
            (forward_rate - turning_rate * TURN_MULTIPLIER).clamp(-1.0, 1.0),
        );
        use communication::plot;
        plot!("heading (degrees)", self.odom.heading().to_degrees());
        if self.controller.pressed(ControllerButtons::A) {
            self.pid_angle
                .set_target(self.odom.heading() + std::f64::consts::FRAC_PI_2);
            self.pid_angle.reset();
        }

        if self.controller.held(ControllerButtons::A) {
            let pw = self.pid_angle.poll(self.odom.heading()).clamp(-1.0, 1.0);
            l = -pw;
            r = pw;
        }

        // for some reason the gearbox doesn't set properly
        self.drivebase.set_side_percent_max_rpm(l, r, 200.0);

        self.brain.write_changes();
        std::thread::sleep(Duration::from_millis(1));
    }

    fn auton_skills(&mut self) {
        communication::odom(self.odom.position(), self.odom.heading());

        self.brain.write_changes();
        std::thread::sleep(std::time::Duration::from_millis(1));
    }
}

const TURN_MULTIPLIER: f64 = 0.5;
