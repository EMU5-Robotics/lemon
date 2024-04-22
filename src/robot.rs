use crate::brain;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RobotState {
    Off,
    Disabled,
    DriverSkills,
    AutonSkills,
    DriverDriver,
    DriverAuton,
}

impl RobotState {
    pub fn from_brain_state(brain_state: brain::State, is_skills: bool) -> Self {
        match (brain_state, is_skills) {
            (brain::State::Disabled, _) => Self::Disabled,
            (brain::State::Auton, true) => Self::AutonSkills,
            (brain::State::Auton, false) => Self::DriverAuton,
            (brain::State::Driver, true) => Self::DriverSkills,
            (brain::State::Driver, false) => Self::DriverDriver,
        }
    }
}

impl Default for RobotState {
    fn default() -> Self {
        Self::Off
    }
}

/*impl RobotState {
    pub fn progress(&mut self, brain_state: brain::State, odom: &mut Odometry) {
        *self = match (*self, brain_state, IS_SKILLS) {
            (Self::Off, brain::State::Disabled, _) => {
                log::info!("Connection established with the brain.");
                log::info!("Entering Disabled state.");
                Self::Disabled
            }
            (Self::Off, brain::State::Driver, true) => {
                log::warn!("Entered driver skills without first entering the disabled state");
                log::info!("Entering DriverSkills state.");
                Self::DriverSkills
            }
            (Self::Off, brain::State::Auton, true) => {
                log::warn!("Entered auton skills without first entering the disabled state");
                log::info!("Entering DriverSkills state.");
                Self::DriverSkills
            }
            (Self::Disabled, brain::State::Driver, true) => {
                log::info!("Entering DriverSkills state.");
                Self::DriverSkills
            }
            (_, brain::State::Driver, false) => {
                if *self != Self::DriverDriver {
                    log::info!("Entering DriverDriver state.");
                }
                Self::DriverDriver
            }
            (_, brain::State::Driver, true) => {
                if *self != Self::DriverSkills {
                    log::info!("Entering DriverSkills state.");
                }
                Self::DriverSkills
            }
            (_, brain::State::Auton, false) => {
                if *self != Self::DriverAuton {
                    log::info!("Entering DriverAuton state.");
                }
                Self::DriverAuton
            }
            (_, brain::State::Auton, true) => {
                if *self != Self::AutonSkills {
                    odom.reset();
                    log::info!("Entering AutonSkills state.");
                }
                Self::AutonSkills
            }
            (Self::Disabled, brain::State::Disabled, _) => Self::Disabled,
            (a, b, c) => {
                log::info!("tried: {a:?} | {b:?} | {c:?}");
                todo!()
            }
        };
    }
}*/
