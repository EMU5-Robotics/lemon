use communication::path::Action;

use crate::odom::Odometry;
use crate::pid::Pid;

use std::collections::VecDeque;
use std::f64::consts::{PI, TAU};

/// Each auton "path" is a Route which is created
/// from a vector of Actions (communication::path::Action)
/// which then gets turned into a more minimal set of Actions
/// represented by the MinSegment struct. When it is time to
/// a MinSegment it gets turned into a ProcessedSegment which
/// contains state needed to follow the "segment"

#[derive(Debug, Clone, Copy)]
enum MinSegment {
    MoveTo([f64; 2]),
    MoveRel((f64, [f64; 2], [f64; 2])),
    TurnTo(f64),
    TurnRel(f64),
}

enum ProcessedSegment {
    MoveRel {
        start: [f64; 2],
        end: [f64; 2],
        dist: f64,
    },
    TurnTo {
        start_heading: f64,
        target_heading: f64,
    },
}

struct Route {
    current_segment: usize,
    prev_pos: [f64; 2],
    segments: Vec<MinSegment>,
    seg_buf: VecDeque<ProcessedSegment>,
    cur_seg: Option<ProcessedSegment>,
    angle_pid: Pid,
}

impl Route {
    pub fn new(actions: Vec<Action>) -> Self {
        let mut pos = [0.0, 0.0];
        let mut heading = 0.0;
        let mut minpaths = Vec::new();
        use communication::path::Action::*;
        for action in actions.into_iter() {
            match action {
                StartAt {
                    pos: npos,
                    heading: nheading,
                } => {
                    pos = npos;
                    heading = nheading;
                }
                MoveRel { rel } => {
                    let (s, c) = heading.sin_cos();
                    pos = [pos[0] + rel * c, pos[1] + rel * s];
                    minpaths.push(MinSegment::MoveRel((rel, [0.0; 2], [0.0; 2])));
                }
                MoveRelAbs { rel } => {
                    let (s, c) = heading.sin_cos();
                    pos = [pos[0] + rel * c, pos[1] + rel * s];
                    minpaths.push(MinSegment::MoveTo(pos));
                }
                MoveTo { pos: npos } => {
                    pos = npos;
                    minpaths.push(MinSegment::MoveTo(npos));
                }
                TurnRel { angle } => {
                    heading += angle;
                    minpaths.push(MinSegment::TurnRel(angle));
                }
                TurnRelAbs { angle } => {
                    heading += angle;
                    minpaths.push(MinSegment::TurnTo(heading));
                }
                TurnTo { heading: nheading } => {
                    heading = nheading;
                    minpaths.push(MinSegment::TurnTo(heading));
                }
            }
        }
        Self {
            current_segment: 0,
            segments: minpaths,
            prev_pos: [0.0; 2],
            seg_buf: VecDeque::new(),
            cur_seg: None,
            angle_pid: Pid::new(0.35, 0.035, 2.2),
        }
    }
    fn optimise_target_heading(heading: f64, target: f64) -> f64 {
        let mut delta = target - heading;
        // map delta into [-TAU, TAU]
        delta %= TAU;
        // map delta into [0, TAU]
        if delta < 0.0 {
            delta += TAU;
        }
        // map delta into [-PI, PI]
        if delta > PI {
            delta -= TAU;
        }
        heading + delta
    }
    /// transform_segments will transform paths
    /// into other using paths i.e. MoveTo -> TurnTo + MoveRel
    /// or TurnRel to TurnTo as the robot can currently
    /// only deal with relative (straight) movement and
    /// absolute turns (though this might change soon with
    /// regards to the absolute headings)
    fn transform_segments(&mut self, odom: &Odometry) {
        // use MoveRel until lateral control exists use MoveRel
        let heading = odom.heading();
        match self.segments[self.current_segment] {
            // note that this allows a suboptimal turn but
            // such a turn is likely to be intentional
            // unlike with TurnTo
            MinSegment::TurnRel(rel) => {
                self.seg_buf.push_back(ProcessedSegment::TurnTo {
                    start_heading: heading,
                    target_heading: heading + rel,
                });
            }
            // ensure TurnTo takes most optimal turn
            // (don't turn more then half a turn)
            MinSegment::TurnTo(target) => {
                self.seg_buf.push_back(ProcessedSegment::TurnTo {
                    start_heading: heading,
                    target_heading: Self::optimise_target_heading(heading, target),
                });
            }
            MinSegment::MoveTo(pos) => {
                let opos = odom.position();
                let diff = [pos[0] - opos[0], pos[1] - opos[1]];
                let target_heading = diff[1].atan2(diff[0]);
                let len = (diff[0].powi(2) + diff[1].powi(2)).sqrt();
                self.seg_buf.push_back(ProcessedSegment::TurnTo {
                    start_heading: heading,
                    target_heading: Self::optimise_target_heading(heading, target_heading),
                });
                self.seg_buf.push_back(ProcessedSegment::MoveRel {
                    start: opos,
                    end: pos,
                    dist: len,
                });
            }
            _ => {}
        }
    }
    /// start_follow will set any state required for
    /// pathing to work properly, for instance this
    /// will set the angle PID for TurnTo
    /// only TurnTo and MoveRel branches should be reachable
    /// as this method should be called after transform_segment
    fn start_follow(&mut self) {
        self.cur_seg = self.seg_buf.pop_back();
        match self.cur_seg {
            Some(ProcessedSegment::TurnTo { target_heading, .. }) => {
                self.angle_pid.set_target(target_heading);
                self.angle_pid.reset();
            }
            Some(ProcessedSegment::MoveRel { .. }) => {}
            None => unreachable!(),
        }
    }

    pub fn follow(&mut self, odom: &Odometry) -> [f64; 2] {
        // if there isn't a started segment start one or
        // exist if the route has ended
        if self.cur_seg.is_none() {
            if self.seg_buf.is_empty() {
                if self.current_segment < self.segments.len() {
                    self.transform_segments(odom);
                    self.current_segment += 1;
                } else {
                    return [0.0, 0.0];
                }
            }

            self.start_follow();
        }

        let current_segment = self.cur_seg.as_ref().unwrap();

        // check if the current segment is finished
        // and start another one if that is the case
        // the end condition for turning is an absolute error of < 2 deg
        // and angular speed of < 1 deg/s
        // the end conditon for a relative (forward movement) is
        // 1: early exit when the distance from the nearest point on the
        // path is more then xcm, from there add a MoveTo(end_point)
        // 2: velocity < 1cm/s and distance from end point < xcm
        // note: the velocity should be low due to a trapezoidal profile
        // 3: facing more then += 3 deg away from target
        match current_segment {
            ProcessedSegment::TurnTo { target_heading, .. }
                if (odom.heading() - target_heading).abs() < 2f64.to_radians()
                    && odom.angular_velocity().abs() < 1f64.to_radians() =>
            {
                log::info!(
                    "Finished segment {} - TurnTo({}).",
                    self.current_segment,
                    target_heading
                );
                self.cur_seg = None;
                return self.follow(odom);
            }
            ProcessedSegment::MoveRel { .. } => todo!(),
            _ => {}
        }

        // actually follow the segement
        // for a turning segment this is a simple as setting
        // the target for the angle pid and taking the output
        // from PID::poll() into the drivetrain
        // for a relative move segment we need to find the
        // closest point on the relative line segement
        // we then use the trapezoidal profile to figure out what velocity
        // we need to set the motors to. Since we don't have lateral control
        // this just sends equal power to both sides but this will change
        // when pure pursuit gets added
        match current_segment {
            ProcessedSegment::TurnTo { .. } => {
                let pow = self.angle_pid.poll(odom.heading());
                [pow, -pow]
            }
            ProcessedSegment::MoveRel { .. } => todo!(),
        }
    }
}

const ACEL_TIME: f64 = 1.5;
const ACEL: f64 = 1.0 / ACEL_TIME;

// velocity profile for straight paths based the scalar projection
// of pos vec2 onto end vec2 relative to start. It is a modified
// trapezoid profile (where it does not start quite at zero to avoid
// stalling due targeting a velocity of zero)
// note since a trapezoid shape with velocity with respect to time
// is not a trapezoid shape with velocity respect to distance
// rather then having linear sides (v = at) we have a square root
// v = sqrt(2da)
// velocity and acceleration are scaled such that v = 1 is the max
// velocity
fn velocity_profile(start: [f64; 2], end: [f64; 2], pos: [f64; 2]) -> f64 {
    use crate::vec::Vec2;
    let start: Vec2 = start.into();
    let end: Vec2 = end.into();
    let pos: Vec2 = pos.into();

    // first we find the project distance along the path
    let path_dist = (end - start).mag();
    let proj_norm = (end - start) / path_dist;
    let path = pos - start;
    let dist = path.dot(proj_norm);

    // we then get the distance from the closest end
    let halfway = 0.5 * path_dist;
    let from_hw = (halfway - dist).abs();
    let from_closest_end = (halfway - from_hw).max(0.0);

    // we then convert that to a velocity and cap it at the max velocity
    let mut velocity = (2.0 * from_closest_end * ACEL).sqrt().min(1.0);
    if dist < halfway {
        // note we don't allow for zero velocity near the start of the path
        velocity = velocity.max(0.1);
    }
    velocity
}
