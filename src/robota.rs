mod bmi088;
mod brain;
mod controller;
mod drivebase;
mod motor;
mod odom;
mod path;
mod pid;
mod robot;
mod vec;

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
        use communication::path::Action;
        use std::f64::consts::*;
        let mut tuning_start = std::time::Instant::now();
        let mut start_heading = 0.0;
        let (startx, starty) = (-0.3, -1.55);
        let actions = [
            /*Action::TurnRelAbs { angle: FRAC_PI_4 },
            Action::TurnRelAbs { angle: -FRAC_PI_4 },
            Action::MoveRelAbs { rel: 1.0 },*/
            Action::MoveTo {
                pos: [0.9 - startx, -1.55 - starty],
            },
            Action::MoveTo {
                pos: [1.5 - startx, -1.0 - starty],
            },
            Action::MoveTo {
                pos: [1.6 - startx, -0.84 - starty],
            },
            Action::MoveTo {
                pos: [1.6 - startx, -1.1 - starty],
            },
            Action::MoveTo { pos: [0.94, 0.67] },
        ];
        let ram = crate::path::Ram::new(0.5, std::time::Duration::from_millis(300));
        //let mut auton_path = crate::path::Route::new(&actions);
        let mut auton_path_two = crate::path::Path::new_from_actions(&actions);
        //auton_path_two.extend(Box::new(ram));
        let mut angle_pid = Pid::new(0.35, 0.035, 2.2);
        loop {
            self.handle_events();

            // updates controller, robot state & motors
            let new_state = self.brain.update_state(&mut self.controller, &self.state);
            if new_state != self.state {
                log::info!("State changed from {:?} to {new_state:?}", self.state);

                // reset odom at start of auton
                if new_state == RobotState::AutonSkills || new_state == RobotState::DriverAuton {
                    self.odom.reset();
                }
            }
            self.state = new_state;

            self.odom.calc_position();

            match self.state {
                RobotState::Off | RobotState::Disabled => {}
                RobotState::AutonSkills => self.auton_skills(&mut auton_path_two, &mut angle_pid),
                RobotState::DriverAuton => {}
                RobotState::DriverSkills => {
                    self.driver(&mut tuning_start, &mut start_heading);
                }
                RobotState::DriverDriver => {
                    self.driver(&mut tuning_start, &mut start_heading);
                }
            }
            self.brain.write_changes();
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
    }
    fn driver(&mut self, tuning_start: &mut std::time::Instant, start_heading: &mut f64) {
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

        if self.controller.pressed(ControllerButtons::B) {
            *start_heading = self.odom.heading();
            *tuning_start = std::time::Instant::now();
            log::info!(
                "PID tuning started with heading: {} ({}deg)",
                start_heading,
                start_heading.to_degrees()
            );
        } else if self.controller.released(ControllerButtons::B) {
            let grad =
                (self.odom.heading() - *start_heading) / tuning_start.elapsed().as_secs_f64();
            log::info!(
                "PID tuning finished with drift of {grad} ({}deg).",
                grad.to_degrees()
            );
        }

        // prevent the robot from moving when "tuning" the IMU
        if !self.controller.held(ControllerButtons::B) {
            // for some reason the gearbox doesn't set properly
            self.drivebase.set_side_percent_max_rpm(l, r, 200.0);
        }
    }

    fn auton_skills(&mut self, route: &mut crate::path::Path, angle_pid: &mut Pid) {
        use communication::plot;
        plot!("pos", self.odom.position());
        plot!("heading", self.odom.heading().to_degrees());
        communication::odom(self.odom.position(), self.odom.heading());

        let [l, r] = route.follow(&self.odom, angle_pid);
        plot!("lr", [l, r]);
        self.drivebase.set_side_percent_max_rpm(l, r, 200.0);
    }
}

const TURN_MULTIPLIER: f64 = 0.5;
