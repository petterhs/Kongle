use crate::touch::TouchEvent;
use crate::ui_layout as layout;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Page {
    Clock,
    System,
    Notifications,
    Settings,
    About,
    Firmware,
    Home,
    Activity,
    Media,
    Preferences,
    HeartRate,
}

/// Clock -> settings -> system -> details. This keeps
/// navigation bounded and requires no allocation.
pub struct Navigation {
    page: Page,
    stack: [Page; 3],
    depth: usize,
}

impl Navigation {
    pub const fn new() -> Self {
        Self {
            page: Page::Clock,
            stack: [Page::Clock; 3],
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

    pub fn media(&mut self) {
        self.clock();
        self.open(Page::Media);
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
            (Page::Clock, TouchEvent::SwipeUp) => self.open(Page::Settings),
            (Page::Clock, TouchEvent::SwipeDown) => self.open(Page::Notifications),
            (Page::Clock, TouchEvent::SwipeRight) => self.open(Page::Home),
            (Page::Clock, TouchEvent::SwipeLeft) => self.open(Page::Media),
            (
                Page::Clock,
                TouchEvent::Tap {
                    x: 104..=180,
                    y: 0..=29,
                },
            ) => self.open(Page::Notifications),
            (
                Page::Clock,
                TouchEvent::Tap {
                    x: 0..=119,
                    y: 156..=201,
                },
            ) => self.open(Page::Activity),
            (
                Page::Clock,
                TouchEvent::Tap {
                    x: 120..=239,
                    y: 156..=201,
                },
            ) => self.open(Page::HeartRate),
            (Page::Settings, event) if layout::tapped(event, layout::DISPLAY) => {
                self.open(Page::Preferences)
            }
            (Page::Settings, event) if layout::tapped(event, layout::SYSTEM) => {
                self.open(Page::System)
            }
            (Page::System, event) if layout::tapped(event, layout::ABOUT) => self.open(Page::About),
            (Page::System, event) if layout::tapped(event, layout::FIRMWARE) => {
                self.open(Page::Firmware)
            }
            (Page::System, TouchEvent::SwipeDown)
            | (Page::Notifications, TouchEvent::SwipeUp)
            | (Page::Notifications, TouchEvent::SwipeRight)
            | (Page::Settings, TouchEvent::SwipeLeft)
            | (Page::Settings, TouchEvent::SwipeDown)
            | (Page::About, TouchEvent::SwipeDown)
            | (Page::About, TouchEvent::SwipeRight) => self.back(),
            (Page::Home, TouchEvent::SwipeLeft)
            | (Page::Home, TouchEvent::SwipeDown)
            | (Page::Activity, TouchEvent::SwipeRight)
            | (Page::Activity, TouchEvent::SwipeDown)
            | (Page::HeartRate, TouchEvent::SwipeRight)
            | (Page::HeartRate, TouchEvent::SwipeDown)
            | (Page::Media, TouchEvent::SwipeRight)
            | (Page::Media, TouchEvent::SwipeDown)
            | (Page::Preferences, TouchEvent::SwipeRight)
            | (Page::Preferences, TouchEvent::SwipeDown) => self.back(),
            (Page::Firmware, TouchEvent::SwipeDown | TouchEvent::SwipeRight) => self.back(),
            _ => false,
        }
    }
}
