use protocol::device::ControllerButtons;

use crate::brain::Packet;

pub struct Controller {
    last: ControllerButtons,
    current: ControllerButtons,
    axes: [f64; 4],
}

impl From<[Packet; 2]> for Controller {
    fn from([first, second]: [Packet; 2]) -> Self {
        // -128 should never be reported by first.axes[_]
        let axes = [
            first.axes[0] as f64 / 127.0,
            first.axes[1] as f64 / 127.0,
            first.axes[2] as f64 / 127.0,
            first.axes[3] as f64 / 127.0,
        ];
        Self {
            last: second.buttons,
            current: first.buttons,
            axes,
        }
    }
}

impl Controller {
    pub fn lx(&self) -> f64 {
        self.axes[0]
    }
    pub fn ly(&self) -> f64 {
        self.axes[1]
    }
    pub fn rx(&self) -> f64 {
        self.axes[2]
    }
    pub fn ry(&self) -> f64 {
        self.axes[3]
    }
    // helper function to check if a button matching with a bit is activated
    // in ControllerButtons. This also checks if only a single bit is being
    // matched against as we should not (at least not yet) be matching against
    // multiple buttons unless it is a bug.
    fn bit_matches(button: ControllerButtons, matcher: ControllerButtons) -> bool {
        let bits = button.bits();
        if bits.count_ones() != 1 || bits > ControllerButtons::R2.bits() {
            log::warn!("Invalid button passed to Packet::held(): {:#b}", bits);
        }
        matcher.bits() & bits == bits
    }
    pub fn held(&self, button: ControllerButtons) -> bool {
        Self::bit_matches(button, self.current)
    }
    pub fn released(&self, button: ControllerButtons) -> bool {
        // matches last pkt but not current
        Self::bit_matches(button, self.last) && !Self::bit_matches(button, self.current)
    }
    pub fn pressed(&self, button: ControllerButtons) -> bool {
        // matches current pkt but not last
        !Self::bit_matches(button, self.last) && Self::bit_matches(button, self.current)
    }
    // we update last to current to avoid problems where since
    // the brain updates slower we handle release/pressed code
    // multiple times
    pub fn update_no_change(&mut self) {
        self.last = self.current;
    }
}
