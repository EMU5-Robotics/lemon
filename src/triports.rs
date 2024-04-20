use std::sync::atomic::{AtomicU8, Ordering};

pub struct Triport<'a> {
    pub triports: &'a AtomicU8,
    pub index: u8,
}

impl<'a> Triport<'a> {
    pub unsafe fn new(triports: &'a AtomicU8, index: u8) -> Self {
        Self { triports, index }
    }
    pub fn set_active(&self) {
        self.triports.fetch_or(1 << self.index, Ordering::Relaxed);
    }
    pub fn set_inactive(&self) {
        self.triports
            .fetch_and(!(1 << self.index), Ordering::Relaxed);
    }
}
