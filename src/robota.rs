mod bmi088;
mod brain;
mod controller;
mod drivebase;
mod motor;
mod odom;
mod path;
mod pid;
mod robot;
mod triports;
mod vec;

use crate::path::*;
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

        let odom = Odometry::new(0.004167368000717639 - 0.007987093436054596, 0x69u16); //ROBOT_A_IMU_BIAS, 0x69u16);

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
        let mut tuning_start = std::time::Instant::now();
        let mut start_heading = 0.0;
        use crate::triports::*;
        let mut angle_pid = Pid::new(0.35, 0.035, 2.2);
        //let mut auton_path = auton_path_a(&mut self.brain);
        //let left_triport = self.brain.get_triport(1);
        //let right_triport = self.brain.get_triport(2);
        let triports: Vec<_> = (1..=8).map(|i| self.brain.get_triport(i)).collect();
        /*let mut auton_path = Path::new(vec![
            Box::new(ChangeTriports::new(
                triports.clone(), //vec![left_triport.clone()],
                TriportChange::Active,
            )),
            Box::new(TimedSegment::new(Box::new(Nop {}), Duration::from_secs(1))),
            Box::new(ChangeTriports::new(
                triports.clone(),
                TriportChange::Inactive,
            )),
            Box::new(TimedSegment::new(Box::new(Nop {}), Duration::from_secs(1))),
            /*Box::new(RepeatSegment::new(
                Box::new(Path::new(vec![
                    Box::new(ChangeTriports::new(
                        triports.clone(), //vec![left_triport, right_triport],
                        TriportChange::Toggle,
                    )),
                    Box::new(TimedSegment::new(Box::new(Nop {}), Duration::from_secs(1))),
                ])),
                5,
            )),*/
        ]);*/
        let mut auton_path = auton_path_a(&mut self.brain, true);
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
                RobotState::AutonSkills => self.auton_skills(&mut auton_path, &mut angle_pid),
                RobotState::DriverAuton => self.auton(&mut auton_path, &mut angle_pid),
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
        log::info!("{:?} @ {:?}", self.odom.position(), self.odom.heading());

        if self.controller.pressed(ControllerButtons::Y) {
            log::info!("TOGGLED");
            let triport = self.brain.get_triport(1);
            let triport_two = self.brain.get_triport(2);
            triport.toggle();
            triport_two.toggle();
        }
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
            self.drivebase.set_side_percent_voltage(l, r); //set_side_percent_max_rpm(l, r, 200.0);
        }
    }
    fn auton(&mut self, route: &mut crate::path::Path, angle_pid: &mut Pid) {
        let [l, r] = route.follow(&self.odom, angle_pid);
        //plot!("lr", [l, r]);
        self.drivebase.set_side_percent_max_rpm(l, r, 200.0);
        log::info!("auton program: {}", self.brain.auton_program());
    }

    fn auton_skills(&mut self, route: &mut crate::path::Path, angle_pid: &mut Pid) {
        use communication::plot;
        plot!("pos", self.odom.position());
        plot!("heading", self.odom.heading().to_degrees());
        communication::odom(self.odom.position(), self.odom.heading());

        let [l, r] = route.follow(&self.odom, angle_pid);
        //plot!("lr", [l, r]);
        self.drivebase.set_side_percent_max_rpm(l, r, 200.0);
    }
}

const TURN_MULTIPLIER: f64 = 0.5;
fn load_balls(brain: &mut Brain, n: usize) -> Path {
    let kicker = [(brain.get_motor(13), false), (brain.get_motor(1), true)];
    let kick_ball = Path::new(vec![
        Box::new(TimedSegment::new(
            Box::new(PowerMotors::new(kicker.clone(), -1.0)),
            Duration::from_millis(220),
        )),
        /*Box::new(TimedSegment::new(
            Box::new(Nop {}),
            Duration::from_millis(400),
        )),*/
        Box::new(TimedSegment::new(
            Box::new(PowerMotors::new(kicker.clone(), 0.6)),
            Duration::from_millis(300),
        )),
        Box::new(TimedSegment::new(
            Box::new(Nop {}),
            Duration::from_millis(800),
        )),
    ]);
    Path::new(vec![
        Box::new(TimedSegment::new(
            Box::new(PowerMotors::new(kicker.clone(), -0.8)),
            Duration::from_millis(80),
        )),
        Box::new(TimedSegment::new(
            Box::new(PowerMotors::new(kicker.clone(), 0.2)),
            Duration::from_millis(100),
        )),
        Box::new(TimedSegment::new(
            Box::new(Nop {}),
            Duration::from_millis(500),
        )),
        Box::new(RepeatSegment::new(Box::new(kick_ball), n)),
    ])
}

fn auton_path_a(brain: &mut Brain, mirror: bool) -> Path {
    use crate::triports::*;
    let (out_wing, in_wing) = if mirror {
        (brain.get_triport(2), brain.get_triport(1))
    } else {
        (brain.get_triport(1), brain.get_triport(2))
    };
    Path::new(vec![
        //Box::new(load_balls(brain, 11)),
        Box::new(MinSegment::TurnTo(if mirror {
            -85f64.to_radians()
        } else {
            85f64.to_radians()
        })),
        Box::new(ChangeTriports::new(vec![out_wing], TriportChange::Active)),
        Box::new(Ram::new(0.3, Duration::from_millis(800))),
        Box::new(MinSegment::TurnTo(if mirror {
            -55f64.to_radians()
        } else {
            55f64.to_radians()
        })),
        Box::new(Ram::new(0.4, Duration::from_millis(700))),
        Box::new(MinSegment::TurnTo(if mirror {
            -50f64.to_radians()
        } else {
            50f64.to_radians()
        })),
        Box::new(Ram::new(0.4, Duration::from_millis(2000))),
        Box::new(SpeedLimiter::new(
            Path::new(vec![Box::new(MinSegment::MoveTo([
                /*1.4878628699954215,
                -1.6524648667892217,*/
                1.4864, -1.72915,
            ]))]),
            0.45,
        )),
        //Box::new(MinSegment::MoveTo([1.70, -1.80])),
        Box::new(MinSegment::TurnTo(0f64.to_radians())),
        //Box::new(MinSegment::MoveTo([2.350580889611332, -1.6244943507316716])),
        Box::new(MinSegment::MoveTo([2.1804, -1.81912])),
        Box::new(Ram::new(-0.2, Duration::from_millis(500))),
        Box::new(MinSegment::TurnTo(0.55)),
        Box::new(Ram::new(0.80, Duration::from_millis(500))),
        Box::new(Ram::new(-0.4, Duration::from_millis(1300))),
        Box::new(MinSegment::MoveTo([2.1804, -1.81912])),
        Box::new(MinSegment::TurnTo(0.55)),
        Box::new(Ram::new(0.80, Duration::from_millis(500))),
    ])
}
