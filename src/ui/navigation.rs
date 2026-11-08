use crate::touch::TouchEvent;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Page {
    Clock,
    Launcher,
    Notifications,
    Settings,
    About,
    Firmware,
}

/// At most two levels are needed for clock -> launcher -> page. This keeps
/// navigation bounded and requires no allocation.
pub struct Navigation {
    page: Page,
    stack: [Page; 2],
    depth: usize,
}

impl Navigation {
    pub const fn new() -> Self {
        Self {
            page: Page::Clock,
            stack: [Page::Clock; 2],
            depth: 0,
        }
    }

    pub const fn page(&self) -> Page {
        self.page
    }

    pub fn clock(&mut self) -> bool {
        let changed = self.page != Page::Clock;
        self.page = Page::Clock;
        self.depth = 0;
        changed
    }

    /// Wake on the clock, return home from other pages, or sleep from home.
    pub fn button(&mut self, awake: bool) -> bool {
        let stay_awake = !awake || self.page != Page::Clock;
        self.clock();
        stay_awake
    }

    pub fn notifications(&mut self) {
        self.page = Page::Notifications;
        self.stack[0] = Page::Clock;
        self.depth = 1;
    }

    fn open(&mut self, page: Page) -> bool {
        if self.page == page || self.depth == self.stack.len() {
            return false;
        }
        self.stack[self.depth] = self.page;
        self.depth += 1;
        self.page = page;
        true
    }

    fn back(&mut self) -> bool {
        if self.depth == 0 {
            return false;
        }
        self.depth -= 1;
        self.page = self.stack[self.depth];
        true
    }

    pub fn handle(&mut self, event: TouchEvent) -> bool {
        match (self.page, event) {
            (Page::Clock, TouchEvent::SwipeUp) => self.open(Page::Launcher),
            (Page::Clock, TouchEvent::SwipeDown) => self.open(Page::Notifications),
            (Page::Clock, TouchEvent::SwipeRight) => self.open(Page::Settings),
            (Page::Clock, TouchEvent::SwipeLeft) => self.open(Page::About),
            (Page::Launcher, TouchEvent::Tap { y: 70..=124, .. }) => self.open(Page::Settings),
            (Page::Launcher, TouchEvent::Tap { y: 125..=180, .. }) => self.open(Page::About),
            (Page::Launcher, TouchEvent::Tap { y: 181..=239, .. }) => self.open(Page::Firmware),
            (Page::Launcher, TouchEvent::SwipeDown)
            | (Page::Notifications, TouchEvent::SwipeUp)
            | (Page::Settings, TouchEvent::SwipeLeft)
            | (Page::Settings, TouchEvent::SwipeDown)
            | (Page::About, TouchEvent::SwipeDown)
            | (Page::About, TouchEvent::SwipeRight) => self.back(),
            (Page::Firmware, TouchEvent::SwipeDown) => self.back(),
            _ => false,
        }
    }
}
