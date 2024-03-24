use communication::path::Action;

use crate::odom::Odometry;

#[derive(Debug, Clone, Copy)]
enum MinSegment {
    MoveTo([f64; 2]),
    MoveRel(f64),
    TurnTo(f64),
    TurnRel(f64),
}

struct Route {
    current_segment: usize,
    prev_pos: [f64; 2],
    segments: Vec<MinSegment>,
    ended: bool,
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
                    minpaths.push(MinSegment::MoveRel(rel));
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
            ended: false,
        }
    }
    pub fn start_follow(&mut self, odom: &Odometry) {
        if self.ended {
            return;
        }
        self.current_segment += 1;
        if self.current_segment >= self.segments.len() {
            self.ended = true;
            return;
        }

        // use MoveRel until lateral control exists use MoveRel
        let heading = odom.heading();
        match self.segments[self.current_segment] {
            /*MinSegment::MoveRel(rel) => {
                let pos = odom.position();
                let (s, c) = heading.sin_cos();
                self.segments[self.current_segment] =
                    MinSegment::MoveTo([pos[0] + rel * c, pos[1] + rel * s]);
            }*/
            MinSegment::TurnRel(rel) => {
                self.segments[self.current_segment] = MinSegment::TurnTo(heading + rel);
            }
            MinSegment::MoveTo(pos) => {
                let opos = odom.position();
                let diff = [pos[0] - opos[0], pos[1] - opos[1]];
                let target_heading = diff[1].atan2(diff[0]);
                let len = (diff[0].powi(2) + diff[1].powi(2)).sqrt();
                self.segments[self.current_segment] = MinSegment::MoveRel(len);
                self.segments
                    .insert(self.current_segment, MinSegment::TurnTo(target_heading));
            }
            _ => {}
        }
        log::info!(
            "Starting segment: {:?}",
            self.segments[self.current_segment]
        );
    }
    pub fn follow(&self, odom: &Odometry) -> [f64; 2] {
        if self.ended {
            return [0.0, 0.0];
        }
        let pos = odom.position();
        let heading = odom.heading();

        let current_segment = &self.segments[self.current_segment];

        // check end condition
        match current_segment {
            MinSegment::MoveTo(npos) => {
                todo!()
            }
            MinSegment::TurnTo(nheading) => {}
            _ => {}
        }

        // follow segement

        todo!()
    }
}
