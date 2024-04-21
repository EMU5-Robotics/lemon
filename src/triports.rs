use std::sync::{
    atomic::{AtomicU8, Ordering},
    Arc,
};

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TriportChange {
    Active,
    Inactive,
    Toggle,
}

#[derive(Debug, Clone)]
pub struct Triport {
    pub triports: Arc<AtomicU8>,
    pub index: u8,
}

impl Triport {
    pub unsafe fn new(triports: Arc<AtomicU8>, index: u8) -> Self {
        Self { triports, index }
    }
    pub fn change(&self, change: TriportChange) {
        match change {
            TriportChange::Active => self.set_active(),
            TriportChange::Inactive => self.set_inactive(),
            TriportChange::Toggle => self.toggle(),
        };
    }
    pub fn set_active(&self) {
        self.triports.fetch_or(1 << self.index, Ordering::Relaxed);
    }
    pub fn set_inactive(&self) {
        self.triports
            .fetch_and(!(1 << self.index), Ordering::Relaxed);
    }
    pub fn toggle(&self) {
        let old_val = self.triports.fetch_xor(1 << self.index, Ordering::SeqCst);
        log::info!("old val: {old_val}");
    }
}
