use communication::path::Action;

use crate::odom::Odometry;
use crate::pid::Pid;
use crate::vec::Vec2;

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
    MoveRel(f64),
    TurnTo(f64),
    TurnRel(f64),
}

#[derive(Debug)]
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

pub struct Route {
    current_segment: usize,
    segments: Vec<MinSegment>,
    seg_buf: Vec<ProcessedSegment>,
    cur_seg: Option<ProcessedSegment>,
    angle_pid: Pid,
}

impl Route {
    pub fn new(actions: &[Action]) -> Self {
        let mut pos = [0.0, 0.0];
        let mut heading = 0.0;
        let mut minpaths = Vec::new();
        use communication::path::Action::*;
        for action in actions {
            match action {
                StartAt {
                    pos: npos,
                    heading: nheading,
                } => {
                    pos = *npos;
                    heading = *nheading;
                }
                MoveRel { rel } => {
                    let (s, c) = heading.sin_cos();
                    pos = [pos[0] + rel * c, pos[1] + rel * s];
                    minpaths.push(MinSegment::MoveRel(*rel));
                }
                MoveRelAbs { rel } => {
                    let (s, c) = heading.sin_cos();
                    pos = [pos[0] + rel * c, pos[1] + rel * s];
                    minpaths.push(MinSegment::MoveTo(pos));
                }
                MoveTo { pos: npos } => {
                    pos = *npos;
                    minpaths.push(MinSegment::MoveTo(*npos));
                }
                TurnRel { angle } => {
                    heading += angle;
                    minpaths.push(MinSegment::TurnRel(*angle));
                }
                TurnRelAbs { angle } => {
                    heading += angle;
                    minpaths.push(MinSegment::TurnTo(heading));
                }
                TurnTo { heading: nheading } => {
                    heading = *nheading;
                    minpaths.push(MinSegment::TurnTo(heading));
                }
            }
        }
        Self {
            current_segment: 0,
            segments: minpaths,
            seg_buf: Vec::new(),
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
    /// transform_segment will transform paths
    /// into other using paths i.e. MoveTo -> TurnTo + MoveRel
    /// or TurnRel to TurnTo as the robot can currently
    /// only deal with relative (straight) movement and
    /// absolute turns (though this might change soon with
    /// regards to the absolute headings)
    /// NOTE - segments are returned in reverse order
    /// so that when you extend the seg_buf with the
    /// returned vector it is in the correct order
    /// for when the next segment is popped from
    /// the BACK of the seg_buf
    fn transform_segment(&self, segment: &MinSegment, odom: &Odometry) -> Vec<ProcessedSegment> {
        // use MoveRel until lateral control exists use MoveRel
        let heading = odom.heading();
        match segment {
            // note that this allows a suboptimal turn but
            // such a turn is likely to be intentional
            // unlike with TurnTo
            MinSegment::TurnRel(rel) => {
                vec![ProcessedSegment::TurnTo {
                    start_heading: heading,
                    target_heading: heading + rel,
                }]
            }
            // ensure TurnTo takes most optimal turn
            // (don't turn more then half a turn)
            MinSegment::TurnTo(target) => {
                vec![ProcessedSegment::TurnTo {
                    start_heading: heading,
                    target_heading: Self::optimise_target_heading(heading, *target),
                }]
            }
            MinSegment::MoveTo(pos) => {
                let opos = odom.position();
                let diff = [pos[0] - opos[0], pos[1] - opos[1]];
                let target_heading = diff[1].atan2(diff[0]);
                let len = (diff[0].powi(2) + diff[1].powi(2)).sqrt();
                // note order is reversed
                vec![
                    ProcessedSegment::MoveRel {
                        start: opos,
                        end: *pos,
                        dist: len,
                    },
                    ProcessedSegment::TurnTo {
                        start_heading: heading,
                        target_heading: Self::optimise_target_heading(heading, target_heading),
                    },
                ]
            }
            MinSegment::MoveRel(rel) => {
                let opos = odom.position();
                vec![ProcessedSegment::MoveRel {
                    start: opos,
                    end: [opos[0] + heading.cos() * rel, opos[1] + heading.sin() * rel],
                    dist: *rel,
                }]
            }
        }
    }
    /// start_follow will set any state required for
    /// pathing to work properly, for instance this
    /// will set the angle PID for TurnTo
    /// only TurnTo and MoveRel branches should be reachable
    /// as this method should be called after transform_segment
    fn start_follow(&mut self) {
        self.cur_seg = self.seg_buf.pop();
        log::info!("New seg started: {:?}", self.cur_seg);
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
                    let new_segs =
                        self.transform_segment(&self.segments[self.current_segment], odom);
                    self.seg_buf.extend(new_segs);
                    self.current_segment += 1;
                } else {
                    return [0.0, 0.0];
                }
            }

            self.start_follow();
        }

        let current_segment = &self.cur_seg.as_ref().unwrap();

        // check if the current segment is finished
        // and start another one if that is the case
        // the end condition for turning is an absolute error of < 2 deg
        // and angular speed of < 1 deg/s
        // the end conditon for a relative (forward movement) is
        // 1: early exit when the distance from the nearest point on the
        // path is more then 5cm, from there add a MoveTo segment
        // 2: velocity < 1cm/s and distance from end point < 5cm
        // note: the velocity should be low due to a trapezoidal profile
        // 3: facing more then += 3 deg away from target, from there add
        // a MoveTo segment
        match current_segment {
            ProcessedSegment::TurnTo { target_heading, .. }
                if (odom.heading() - target_heading).abs() < 2f64.to_radians()
                    && odom.angular_velocity().abs() < 1f64.to_radians() =>
            {
                log::info!(
                    "Finished segment {} - TurnTo({}) with heading ({}).",
                    self.current_segment,
                    target_heading,
                    odom.heading()
                );
                self.cur_seg = None;
                return self.follow(odom);
            }
            ProcessedSegment::MoveRel { start, end, .. } => {
                let ideal_heading = (end[1] - start[1]).atan2(end[0] - start[0]);
                let ideal_heading = Self::optimise_target_heading(odom.heading(), ideal_heading);
                // check heading is within +-3 deg
                if (odom.heading() - ideal_heading).abs() > 8f64.to_radians() {
                    let new_segs = self.transform_segment(&MinSegment::MoveTo(*end), odom);
                    self.seg_buf.extend(new_segs);
                    log::warn!("MoveRel failed due to exceeding a +- 8deg heading ({} vs {}). Creating MoveTo segment.", odom.heading(), ideal_heading);
                    self.cur_seg = None;
                    return self.follow(odom);
                }

                // check if distance from closest point is greater then 5cm
                // We can get this distance from finding the height of the triangle
                // with the base defined by [start, end] and the third point at pos.
                // From there we can find the area with herons formula and then
                // solve for the height from the base length and area.
                let end: Vec2 = end.into();
                let start: Vec2 = start.into();
                let pos: Vec2 = odom.position().into();
                let base = (end - start).mag();
                let end_dist = (end - pos).mag();
                let start_dist = (start - pos).mag();
                let s = (end_dist + start_dist + base) * 0.5;
                let area = (s * (s - end_dist) * (s - start_dist) * (s - base)).sqrt();
                let near_dist = 2.0 * area / base;
                if near_dist > 0.10 {
                    let new_segs =
                        self.transform_segment(&MinSegment::MoveTo([end.x(), end.y()]), odom);
                    self.seg_buf.extend(new_segs);
                    log::warn!("Distance from closest point exceeds 10cm ({near_dist}). Creating MoveTo segment. pos: ({}, {})", pos.x(), pos.y());
                    self.cur_seg = None;
                    return self.follow(odom);
                }

                // finish the segment if distance to end point is less then
                // 5cm and (average side) velocity is < 1cm/s
                /*use rand::{thread_rng, Rng};
                if thread_rng().gen::<f32>() < 0.01 {
                    log::info!("{end_dist} | {}", 2.0 * area / base);
                }*/
                use communication::plot;
                plot!("dists", [end_dist, 2.0 * area / base]);
                plot!("end", [end.x(), end.y()]);
                if 0.5 * (odom.side_velocities()[0] + odom.side_velocities()[1]) < 0.01
                    && end_dist < 0.05
                    || (end_dist < start_dist && start_dist > base)
                {
                    log::info!(
                        "Finished segment {} - MoveRel(start: {:?}, end: {:?}).",
                        self.current_segment,
                        start,
                        end
                    );
                    self.cur_seg = None;
                    return self.follow(odom);
                }
            }
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
                [-pow, pow]
            }
            ProcessedSegment::MoveRel { start, end, dist } => {
                let pow = velocity_profile(start.into(), end.into(), *dist, odom.position().into());
                [pow; 2]
            }
        }
    }
}

// time to get to accelerate to max velocity from zero
// decelerating should be quicker then acceleration
// due to braking but since we have a symmetrical profile
// (partially for stability in deccelerate) we use acceleration
// time rather then adopt an asymmetrical model
const ACCEL_TIME: f64 = 1.5;
const ACCEL: f64 = 1.0 / ACCEL_TIME;

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
fn velocity_profile(start: Vec2, end: Vec2, path_dist: f64, pos: Vec2) -> f64 {
    // first we find the projected distance along the path
    let proj_norm = (end - start) / path_dist;
    let path = pos - start;
    let dist = path.dot(proj_norm);

    // we then get the distance from the closest end
    let halfway = 0.5 * path_dist;
    let from_hw = (halfway - dist).abs();
    // if the dist is negative or longer then the path
    // clamp it to the ends
    let from_closest_end = (halfway - from_hw).max(0.0);

    // we then convert that to a velocity and cap it at the max velocity
    let mut velocity = (2.0 * from_closest_end * ACCEL).sqrt().min(1.0);
    if dist < halfway {
        // we don't allow for zero velocity near the start of the path
        // as that would stall the robot instead we opt for 10% of max speed
        velocity = velocity.max(0.1);
    }
    velocity
}

pub struct Path {
    // this is a stack so the last element in
    // the vector is the first that will be run
    pub segments: VecDeque<Box<dyn PathSegment>>,
    pub current_segment: Option<Box<dyn PathSegment>>,
}

impl Path {
    pub fn new(reversed_segments: Vec<Box<dyn PathSegment>>) -> Self {
        Self {
            segments: reversed_segments.into_iter().rev().collect(),
            current_segment: None,
        }
    }
    pub fn extend(&mut self, v: Box<dyn PathSegment>) {
        self.segments.push_front(v);
    }
    pub fn new_from_actions(actions: &[Action]) -> Self {
        let mut pos = [0.0, 0.0];
        let mut heading = 0.0;
        let mut minpaths = Vec::new();
        use communication::path::Action::*;
        for action in actions {
            match action {
                StartAt {
                    pos: npos,
                    heading: nheading,
                } => {
                    pos = *npos;
                    heading = *nheading;
                }
                MoveRel { rel } => {
                    let (s, c) = heading.sin_cos();
                    pos = [pos[0] + rel * c, pos[1] + rel * s];
                    minpaths.push(MinSegment::MoveRel(*rel));
                }
                MoveRelAbs { rel } => {
                    let (s, c) = heading.sin_cos();
                    pos = [pos[0] + rel * c, pos[1] + rel * s];
                    minpaths.push(MinSegment::MoveTo(pos));
                }
                MoveTo { pos: npos } => {
                    pos = *npos;
                    minpaths.push(MinSegment::MoveTo(*npos));
                }
                TurnRel { angle } => {
                    heading += angle;
                    minpaths.push(MinSegment::TurnRel(*angle));
                }
                TurnRelAbs { angle } => {
                    heading += angle;
                    minpaths.push(MinSegment::TurnTo(heading));
                }
                TurnTo { heading: nheading } => {
                    heading = *nheading;
                    minpaths.push(MinSegment::TurnTo(heading));
                }
            }
        }
        Self::new(
            minpaths
                .into_iter()
                .map(|v| -> Box<dyn PathSegment> { Box::new(v) })
                .collect(),
        )
    }
}

impl Path {
    fn transform_segments(&mut self, odom: &Odometry, angle_pid: &mut Pid) {
        if self.current_segment.is_some() {
            return;
        }

        while let Some(mut new_seg) = self.segments.pop_back() {
            if new_seg.finished_transform() {
                log::info!("started new segment: {new_seg:?}");
                new_seg.start(odom, angle_pid);
                self.current_segment = Some(new_seg);
                return;
            }
            self.segments.extend(new_seg.transform(odom));
        }
    }
    pub fn follow(&mut self, odom: &Odometry, angle_pid: &mut Pid) -> [f64; 2] {
        // get new segments if needed
        self.transform_segments(odom, angle_pid);

        // exit when no segments could be transformed
        let Some(seg) = self.current_segment.as_mut() else {
            return [0.0; 2];
        };

        // end segment and start next
        if let Some(new_segments) = seg.end_follow(odom) {
            if new_segments.is_empty() {
                log::info!("segment_ended: {seg:?} and added new segments: {new_segments:?}");
            } else {
                log::info!("segment_ended: {seg:?}");
            }
            self.segments.extend(new_segments);
            self.current_segment = None;
            return self.follow(odom, angle_pid);
        }

        seg.follow(odom, angle_pid)
    }
    pub fn ended(&self) -> bool {
        self.current_segment.is_none() && self.segments.is_empty()
    }
}

pub trait PathSegment: std::fmt::Debug {
    fn transform<'a>(self: Box<Self>, odom: &Odometry) -> Vec<Box<dyn PathSegment + 'a>>;
    fn finished_transform(&self) -> bool;
    fn start(&mut self, odom: &Odometry, angle_pid: &mut Pid);
    fn follow(&self, odom: &Odometry, angle_pid: &mut Pid) -> [f64; 2];
    fn end_follow<'a>(&mut self, odom: &Odometry) -> Option<Vec<Box<dyn PathSegment + 'a>>>;
}

#[derive(Debug)]
struct TurnTo {
    start_heading: f64,
    target_heading: f64,
}

impl PathSegment for TurnTo {
    fn transform<'a>(self: Box<Self>, _: &Odometry) -> Vec<Box<dyn PathSegment + 'a>> {
        unreachable!("transform should never get called since finished_transform is true")
    }
    fn finished_transform(&self) -> bool {
        true
    }
    fn start(&mut self, odom: &Odometry, angle_pid: &mut Pid) {
        self.target_heading = optimise_target_heading(odom.heading(), self.target_heading);
        angle_pid.set_target(self.target_heading);
        angle_pid.reset();
    }
    fn follow(&self, odom: &Odometry, angle_pid: &mut Pid) -> [f64; 2] {
        let pow = angle_pid.poll(odom.heading());
        [-pow, pow]
    }
    fn end_follow<'a>(&mut self, odom: &Odometry) -> Option<Vec<Box<dyn PathSegment + 'a>>> {
        if (odom.heading() - self.target_heading).abs() < 2f64.to_radians()
            && odom.angular_velocity().abs() < 1f64.to_radians()
        {
            log::info!(
                "Finished segment - TurnTo({}) with heading ({}).",
                self.target_heading,
                odom.heading()
            );
            return Some(vec![]);
        }
        None
    }
}

impl PathSegment for MinSegment {
    fn transform<'a>(self: Box<Self>, odom: &Odometry) -> Vec<Box<dyn PathSegment + 'a>> {
        let heading = odom.heading();
        match *self {
            // note that this allows a suboptimal turn but
            // such a turn is likely to be intentional
            // unlike with TurnTo
            MinSegment::TurnRel(rel) => {
                vec![Box::new(TurnTo {
                    start_heading: heading,
                    target_heading: heading + rel,
                })]
            }
            // ensure TurnTo takes most optimal turn
            // (don't turn more then half a turn)
            MinSegment::TurnTo(target) => {
                vec![Box::new(TurnTo {
                    start_heading: heading,
                    target_heading: optimise_target_heading(heading, target),
                })]
            }
            MinSegment::MoveTo(pos) => {
                let opos = odom.position();
                let diff = [pos[0] - opos[0], pos[1] - opos[1]];
                let target_heading = diff[1].atan2(diff[0]);
                let len = (diff[0].powi(2) + diff[1].powi(2)).sqrt();
                // note order is reversed because of stack
                vec![
                    Box::new(MoveRel {
                        start: opos,
                        end: pos,
                        dist: len,
                    }),
                    Box::new(TurnTo {
                        start_heading: heading,
                        target_heading: optimise_target_heading(heading, target_heading),
                    }),
                ]
            }
            MinSegment::MoveRel(rel) => {
                let opos = odom.position();
                vec![Box::new(MoveRel {
                    start: opos,
                    end: [opos[0] + heading.cos() * rel, opos[1] + heading.sin() * rel],
                    dist: rel,
                })]
            }
        }
    }
    fn finished_transform(&self) -> bool {
        false
    }
    fn start(&mut self, _: &Odometry, _: &mut Pid) {
        unreachable!("segment should be always be transformed")
    }
    fn follow(&self, _: &Odometry, _: &mut Pid) -> [f64; 2] {
        unreachable!("segment should be always be transformed")
    }
    fn end_follow<'a>(&mut self, _: &Odometry) -> Option<Vec<Box<dyn PathSegment + 'a>>> {
        unreachable!("segment should be always be transformed")
    }
}

#[derive(Debug)]
struct MoveRel {
    start: [f64; 2],
    end: [f64; 2],
    dist: f64,
}

impl PathSegment for MoveRel {
    fn transform<'a>(self: Box<Self>, _: &Odometry) -> Vec<Box<dyn PathSegment + 'a>> {
        unreachable!("transform should never get called since finished_transform is true")
    }
    fn finished_transform(&self) -> bool {
        true
    }
    fn start(&mut self, _: &Odometry, _: &mut Pid) {}
    fn follow(&self, odom: &Odometry, _: &mut Pid) -> [f64; 2] {
        let pow = velocity_profile(
            self.start.into(),
            self.end.into(),
            self.dist,
            odom.position().into(),
        );
        [pow; 2]
    }
    fn end_follow<'a>(&mut self, odom: &Odometry) -> Option<Vec<Box<dyn PathSegment + 'a>>> {
        let ideal_heading = (self.end[1] - self.start[1]).atan2(self.end[0] - self.start[0]);
        let ideal_heading = optimise_target_heading(odom.heading(), ideal_heading);
        // check heading is within +-3 deg
        if (odom.heading() - ideal_heading).abs() > 8f64.to_radians() {
            let new_segs = Box::new(MinSegment::MoveTo(self.end));
            log::warn!("MoveRel failed due to exceeding a +- 8deg heading ({} vs {}). Creating MoveTo segment.", odom.heading(), ideal_heading);
            return Some(vec![new_segs]);
        }

        // check if distance from closest point is greater then 5cm
        // We can get this distance from finding the height of the triangle
        // with the base defined by [start, end] and the third point at pos.
        // From there we can find the area with herons formula and then
        // solve for the height from the base length and area.
        let end: Vec2 = self.end.into();
        let start: Vec2 = self.start.into();
        let pos: Vec2 = odom.position().into();
        let base = (end - start).mag();
        let end_dist = (end - pos).mag();
        let start_dist = (start - pos).mag();
        let s = (end_dist + start_dist + base) * 0.5;
        let area = (s * (s - end_dist) * (s - start_dist) * (s - base)).sqrt();
        let near_dist = 2.0 * area / base;
        if near_dist > 0.10 {
            let new_segs = Box::new(MinSegment::MoveTo([end.x(), end.y()]));
            log::warn!("Distance from closest point exceeds 10cm ({near_dist}). Creating MoveTo segment. pos: ({}, {})", pos.x(), pos.y());
            return Some(vec![new_segs]);
        }

        // finish the segment if distance to end point is less then
        // 5cm and (average side) velocity is < 1cm/s
        use communication::plot;
        plot!("dists", [end_dist, 2.0 * area / base]);
        plot!("end", [end.x(), end.y()]);
        if 0.5 * (odom.side_velocities()[0] + odom.side_velocities()[1]) < 0.01 && end_dist < 0.05
            || (end_dist < start_dist && start_dist > base)
        {
            log::info!(
                "Finished segment - MoveRel(start: {:?}, end: {:?}).",
                start,
                end
            );
            return Some(Vec::new());
        }
        None
    }
}

#[derive(Debug)]
pub struct Ram {
    pow: f64,
    dur: std::time::Duration,
    start: std::time::Instant,
}

impl Ram {
    pub fn new(pow: f64, dur: std::time::Duration) -> Self {
        Self {
            pow,
            dur,
            start: std::time::Instant::now(),
        }
    }
}

impl PathSegment for Ram {
    fn transform<'a>(self: Box<Self>, _: &Odometry) -> Vec<Box<dyn PathSegment + 'a>> {
        unreachable!("transform should never get called since finished_transform is true")
    }
    fn finished_transform(&self) -> bool {
        true
    }
    fn start(&mut self, _: &Odometry, _: &mut Pid) {
        self.start = std::time::Instant::now();
    }
    fn follow(&self, _: &Odometry, _: &mut Pid) -> [f64; 2] {
        [self.pow; 2]
    }
    fn end_follow<'a>(&mut self, _: &Odometry) -> Option<Vec<Box<dyn PathSegment + 'a>>> {
        if self.start.elapsed() > self.dur {
            return Some(Vec::new());
        }
        None
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
