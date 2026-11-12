use crate::{
    button_gestures::{Click, Gestures},
    device::battery::BatteryState,
    language::Language,
    notifications::parse_new_alert,
    ota::UpdateStatus,
    touch::TouchEvent,
    ui::WatchUi,
    ui_text, TimeState,
};
use embedded_graphics::{pixelcolor::Rgb565, prelude::*};

#[test]
fn pulse_requires_explicit_start_and_all_page_exits_request_stop() {
    use crate::heart_rate::{Phase, Reading, State};
    for exit in 0..5 {
        let mut ui = WatchUi::new();
        assert!(ui.handle_touch(TouchEvent::Tap { x: 170, y: 180 }));
        assert!(ui.heart_visible());
        assert_eq!(ui.take_heart_request(), None);
        assert!(!ui.handle_touch(TouchEvent::Tap { x: 50, y: 170 }));
        assert_eq!(ui.take_heart_request(), Some(true));
        ui.set_heart(
            State {
                phase: Phase::Measuring,
                last: Some(Reading {
                    bpm: 72,
                    at_ms: 5000,
                }),
                started_ms: 0,
            },
            5000,
        );
        match exit {
            0 => {
                ui.handle_touch(TouchEvent::SwipeRight);
            }
            1 => {
                ui.handle_button(true);
            }
            2 => {
                ui.show_clock();
            }
            3 => {
                ui.open_media();
            }
            _ => {
                ui.receive_notification(parse_new_alert(b"\0\x01\0Mail\0Hi").unwrap(), true);
            }
        }
        assert!(!ui.heart_visible());
        assert_eq!(ui.take_heart_request(), Some(false));
    }
    let mut ui = WatchUi::new();
    ui.handle_touch(TouchEvent::Tap { x: 170, y: 180 });
    ui.handle_touch(TouchEvent::Tap { x: 50, y: 170 });
    ui.handle_button(true); // Exit before the owner even consumes Start.
    assert_eq!(ui.take_heart_request(), Some(false));
}

#[test]
fn pulse_updates_preserve_panel_and_button_borders_and_fit_both_languages() {
    use crate::heart_rate::{Phase, Reading, State};
    let mut ui = WatchUi::new();
    ui.handle_touch(TouchEvent::Tap { x: 170, y: 180 });
    let mut frame = Frame::new();
    render(&mut ui, &mut frame);
    let before = frame.pixels.clone();
    ui.set_heart(
        State {
            phase: Phase::Measuring,
            last: Some(Reading {
                bpm: 72,
                at_ms: 5000,
            }),
            started_ms: 0,
        },
        5000,
    );
    ui.draw_heart(&mut frame);
    for (index, (old, new)) in before.iter().zip(&frame.pixels).enumerate() {
        let x = index % 240;
        let y = index / 240;
        let field = (20..220).contains(&x)
            && ([40..120, 122..146, 205..229].iter().any(|r| r.contains(&y)));
        let button_label = (28..208).contains(&x) && (158..186).contains(&y);
        if !field && !button_label {
            assert_eq!(old, new, "pulse overwrote {},{}", x, y);
        }
    }
    ui.show_clock();
    ui.handle_touch(TouchEvent::SwipeUp);
    ui.handle_touch(TouchEvent::Tap { x: 50, y: 160 }); // Display.
    ui.handle_touch(TouchEvent::Tap { x: 50, y: 120 }); // Bokmål.
    ui.show_clock();
    ui.handle_touch(TouchEvent::Tap { x: 170, y: 180 });
    for phase in [
        Phase::Idle,
        Phase::Measuring,
        Phase::NoSignal,
        Phase::Missing,
        Phase::Fault,
    ] {
        ui.set_heart(
            State {
                phase,
                last: Some(Reading {
                    bpm: 220,
                    at_ms: 1000,
                }),
                started_ms: 0,
            },
            99_999_999,
        );
        render(&mut ui, &mut frame);
    }
    ui.open_media();
    render(&mut ui, &mut frame);
    let before = frame.pixels.clone();
    ui.set_heart(
        State {
            phase: Phase::Measuring,
            last: Some(Reading {
                bpm: 72,
                at_ms: 1000,
            }),
            started_ms: 0,
        },
        1000,
    );
    ui.draw_heart(&mut frame);
    assert_eq!(before, frame.pixels);
    assert_eq!(frame.clipped, 0);
}

#[test]
fn pulse_loader_keeps_waiting_and_does_not_repaint_digits_or_borders() {
    use crate::heart_rate::{Phase, Reading, State};
    let mut ui = WatchUi::new();
    ui.handle_touch(TouchEvent::Tap { x: 170, y: 180 });
    let measuring = State {
        phase: Phase::Measuring,
        started_ms: 100,
        ..State::EMPTY
    };
    ui.set_heart(measuring, 100);
    assert!(ui.heart_loading());
    let mut frame = Frame::new();
    render(&mut ui, &mut frame);
    frame.preview("pulse-start");
    let initial = frame.pixels.clone();
    // Continue past the estimated startup time. Elapsed time must never
    // fabricate a reading or finish the loader; only a fresh result does.
    for now in [300, 5700, 11100, 11300, 14500, 40300] {
        assert!(ui.set_heart(measuring, now));
        assert!(ui.heart_loading());
        frame.written.clear();
        ui.draw_heart(&mut frame);
        for p in &frame.written {
            assert!(
                !(40..120).contains(&p.y),
                "loader repainted large digits at {p:?}"
            );
            assert!(
                !(148..198).contains(&p.y),
                "loader repainted button at {p:?}"
            );
            assert!((20..220).contains(&p.x) && (140..145).contains(&p.y));
        }
        assert_eq!(
            initial[40 * 240..120 * 240],
            frame.pixels[40 * 240..120 * 240]
        );
    }
    frame.preview("pulse-waiting");
    let with_reading = State {
        last: Some(Reading {
            bpm: 72,
            at_ms: 40400,
        }),
        ..measuring
    };
    ui.set_heart(with_reading, 40400);
    assert!(!ui.heart_loading());
    ui.draw_heart(&mut frame);
    frame.preview("pulse-reading");
    assert_ne!(
        initial[40 * 240..120 * 240],
        frame.pixels[40 * 240..120 * 240]
    );
    // Expiry returns to dashes and the loader rather than showing stale BPM.
    ui.set_heart(with_reading, 43000);
    assert!(ui.heart_loading());
    ui.draw_heart(&mut frame);
    assert_eq!(
        initial[40 * 240..120 * 240],
        frame.pixels[40 * 240..120 * 240]
    );
    assert_eq!(frame.clipped, 0);
}

#[test]
fn live_steps_only_redraw_their_region_and_never_overwrite_media() {
    let mut ui = WatchUi::new();
    let mut frame = Frame::new();
    render(&mut ui, &mut frame);
    let before = frame.pixels.clone();
    let steps = crate::steps::State {
        session: Some(12345),
        ..crate::steps::State::EMPTY
    };
    assert!(ui.set_steps(steps));
    ui.draw_steps(&mut frame);
    assert_eq!(before[..169 * 240], frame.pixels[..169 * 240]);
    assert_eq!(before[199 * 240..], frame.pixels[199 * 240..]);
    assert_ne!(before, frame.pixels);
    assert!(!ui.set_steps(steps));

    ui.handle_touch(TouchEvent::Tap { x: 50, y: 180 });
    render(&mut ui, &mut frame);
    let before = frame.pixels.clone();
    ui.set_steps(crate::steps::State::EMPTY);
    ui.draw_steps(&mut frame);
    assert_eq!(before[..32 * 240], frame.pixels[..32 * 240]);
    assert_eq!(before[212 * 240..], frame.pixels[212 * 240..]);
    assert_ne!(before, frame.pixels);

    ui.open_media();
    render(&mut ui, &mut frame);
    let before = frame.pixels.clone();
    ui.set_steps(crate::steps::State {
        session: Some(u32::MAX),
        ..crate::steps::State::EMPTY
    });
    ui.draw_steps(&mut frame);
    assert_eq!(before, frame.pixels);
    assert_eq!(frame.clipped, 0);
}

#[test]
fn double_press_has_a_bounded_window_and_hold_does_not_become_double() {
    let mut g = Gestures::new();
    g.press(0);
    assert_eq!(g.release(70), Click::First);
    assert!(!g.expire(419));
    g.press(400);
    assert_eq!(g.release(480), Click::Double);
    assert_eq!(g.deadline(), None);
    assert!(!g.expire(999));
    g.press(1000);
    assert_eq!(g.release(1100), Click::First);
    assert!(g.expire(1450));
    assert!(!g.expire(1451));
    g.press(1500);
    assert_eq!(g.release(1600), Click::First);
    g.press(1700);
    assert_eq!(g.release(2400), Click::First); // Second press became a hold.
    g.cancel(); // Recovery hold cancels all click state.
    assert_eq!(g.deadline(), None);
    g.press(3000);
    assert_eq!(g.release(4000), Click::First); // First press was a hold too.
    g.press(4100);
    assert_eq!(g.release(4200), Click::First);
}

#[test]
fn latin1_notifications_survive_parsing_and_wrapping_without_split_bytes() {
    let notification =
        parse_new_alert("\0\u{1}\0Blåbær\0På vei hjem – bientôt".as_bytes()).unwrap();
    assert_eq!(notification.title, "Blåbær");
    assert_eq!(notification.body, "På vei hjem ? bientôt");
    let original = "æøåæøåæøå ærlig tekst";
    let mut remaining = original;
    let mut rebuilt = std::string::String::new();
    while !remaining.trim().is_empty() {
        let (line, rest) = ui_text::next_line(remaining, 4);
        assert!(!line.is_empty());
        assert!(line.chars().count() <= 4);
        rebuilt.push_str(line);
        remaining = rest;
    }
    assert_eq!(rebuilt.replace(' ', ""), original.replace(' ', ""));
    for year in [2024, 2025, 2026] {
        for month in 1..=12 {
            for day in 1..=31 {
                for language in [Language::English, Language::Bokmal] {
                    assert!(language.date(year, month, day).chars().count() <= 21);
                }
            }
        }
    }
    assert_eq!(Language::English.date(2026, 11, 9), "Monday 9 Nov");
    assert_eq!(Language::Bokmal.date(2026, 11, 9), "mandag 9. nov");
}

struct Frame {
    pixels: std::vec::Vec<Rgb565>,
    clipped: usize,
    written: std::vec::Vec<Point>,
}
impl Frame {
    fn new() -> Self {
        Self {
            pixels: vec![crate::theme::BACKGROUND; 240 * 240],
            clipped: 0,
            written: vec![],
        }
    }
    fn preview(&self, name: &str) {
        if let Ok(directory) = std::env::var("KONGLE_UI_PREVIEW_DIR") {
            let mut bytes = b"P6\n240 240\n255\n".to_vec();
            // Calculate in u16 to avoid overflowing RGB565 channel values.
            for p in &self.pixels {
                bytes.extend_from_slice(&[
                    (u16::from(p.r()) * 255 / 31) as u8,
                    (u16::from(p.g()) * 255 / 63) as u8,
                    (u16::from(p.b()) * 255 / 31) as u8,
                ]);
            }
            std::fs::create_dir_all(&directory).unwrap();
            std::fs::write(
                std::path::Path::new(&directory).join(format!("{name}.ppm")),
                bytes,
            )
            .unwrap();
        }
    }
}
impl OriginDimensions for Frame {
    fn size(&self) -> Size {
        Size::new(240, 240)
    }
}
impl DrawTarget for Frame {
    type Color = Rgb565;
    type Error = core::convert::Infallible;
    fn draw_iter<I: IntoIterator<Item = Pixel<Rgb565>>>(
        &mut self,
        pixels: I,
    ) -> Result<(), Self::Error> {
        for Pixel(p, c) in pixels {
            self.written.push(p);
            if (0..240).contains(&p.x) && (0..240).contains(&p.y) {
                self.pixels[p.y as usize * 240 + p.x as usize] = c;
            } else {
                self.clipped += 1;
            }
        }
        Ok(())
    }
}
fn time() -> TimeState {
    TimeState {
        year: 2026,
        month: 11,
        day: 9,
        hours: 12,
        minutes: 48,
        seconds: 0,
    }
}
fn render(ui: &mut WatchUi, frame: &mut Frame) {
    frame.clear(crate::theme::BACKGROUND).unwrap();
    ui.draw_full(
        frame,
        Some(time()),
        Some(BatteryState {
            mv: 3910,
            percent: 76,
            charging: false,
        }),
        UpdateStatus::Idle,
        true,
    );
}

#[test]
fn notification_bell_only_indicates_unread_content_and_has_no_count() {
    let mut ui = WatchUi::new();
    let mut frame = Frame::new();
    let indicator = |frame: &Frame, x0: usize, x1: usize| {
        (0..28).any(|y| (x0..x1).any(|x| frame.pixels[y * 240 + x] != crate::theme::BACKGROUND))
    };
    ui.draw_notification_indicator(&mut frame);
    assert!(!indicator(&frame, 110, 184));
    ui.receive_notification(parse_new_alert(b"\0\x01\0Mail\0Hello").unwrap(), false);
    ui.draw_notification_indicator(&mut frame);
    assert!(indicator(&frame, 114, 138));
    assert!(
        !indicator(&frame, 140, 184),
        "notification count must not be rendered"
    );
    ui.handle_touch(TouchEvent::SwipeDown); // Opening the inbox acknowledges it.
    ui.handle_button(true); // Return home; the stored message remains.
    ui.draw_notification_indicator(&mut frame);
    assert!(!indicator(&frame, 110, 184));
    ui.receive_notification(parse_new_alert(b"\0\x01\0Mail\0New").unwrap(), false);
    ui.draw_notification_indicator(&mut frame);
    assert!(indicator(&frame, 114, 138));
    ui.handle_touch(TouchEvent::SwipeDown); // Explicitly opening the inbox marks it seen.
    ui.handle_button(true);
    ui.draw_notification_indicator(&mut frame);
    assert!(!indicator(&frame, 110, 184));
}

#[test]
fn connection_and_notification_updates_cannot_overwrite_nonclock_headers() {
    let mut ui = WatchUi::new();
    let mut frame = Frame::new();
    ui.receive_notification(parse_new_alert(b"\0\x01\0Mail\0Hello").unwrap(), false);
    ui.handle_touch(TouchEvent::SwipeUp);
    render(&mut ui, &mut frame);
    let before = frame.pixels.clone();
    ui.draw_connection(&mut frame, false);
    ui.draw_notification_indicator(&mut frame);
    assert_eq!(frame.pixels, before);
    assert_eq!(crate::theme::CLOCK, crate::theme::HEADING);
}

#[test]
fn bluetooth_is_blue_when_connected_and_clears_completely_on_disconnect() {
    let mut ui = WatchUi::new();
    let mut frame = Frame::new();
    render(&mut ui, &mut frame);
    let pixels = |frame: &Frame| {
        (0..24)
            .flat_map(|y| (206..230).map(move |x| frame.pixels[y * 240 + x]))
            .collect::<std::vec::Vec<_>>()
    };
    let icon = pixels(&frame);
    assert!(icon.contains(&crate::theme::BLUETOOTH));
    assert!(icon
        .iter()
        .all(|&p| p == crate::theme::BACKGROUND || p == crate::theme::BLUETOOTH));
    ui.draw_connection(&mut frame, false);
    assert!(pixels(&frame)
        .iter()
        .all(|&p| p == crate::theme::BACKGROUND));
    assert_eq!(
        crate::ble_identity::display_address([0x13, 0x42, 0xcf, 0x27, 0xd2, 0xc2]),
        "C2:D2:27:CF:42:13"
    );
    assert_eq!(
        crate::ble_identity::display_address([0xcc, 0x42, 0xcf, 0x27, 0xd2, 0xc2]),
        "C2:D2:27:CF:42:CC"
    );
}

#[test]
fn media_protocol_validates_lengths_and_does_not_replay_across_connections() {
    use crate::media::{Command, State};
    let mut state = State::new();
    state.session = 7;
    assert!(!state.available());
    state.apply(4, "Blåbær".as_bytes()).unwrap();
    state.apply(2, &[1]).unwrap();
    state.apply(6, &1234u32.to_be_bytes()).unwrap();
    assert_eq!(state.position, 1234);
    let before = state.clone();
    for (field, data) in [
        (2, &[2][..]),
        (6, &[1, 2][..]),
        (4, &[0xff][..]),
        (15, &[0][..]),
    ] {
        assert!(state.apply(field, data).is_err());
        assert_eq!(state, before);
    }
    let mut ui = WatchUi::new();
    ui.set_media(state.clone());
    ui.open_media();
    assert_eq!(ui.take_media_request().unwrap().command, Command::Open);
    ui.handle_touch(TouchEvent::Tap { x: 30, y: 170 });
    let request = ui.take_media_request().unwrap();
    assert_eq!(request.session, 7);
    assert_eq!(request.command, Command::Previous);
    ui.handle_touch(TouchEvent::Tap { x: 120, y: 170 });
    assert_eq!(ui.take_media_request().unwrap().command, Command::Pause);
    let mut frame = Frame::new();
    render(&mut ui, &mut frame);
    frame.preview("media-playing");
    let header = frame.pixels[..24 * 240].to_vec();
    state.apply(2, &[0]).unwrap();
    ui.set_media(state);
    ui.draw_media(&mut frame);
    assert_eq!(header, frame.pixels[..24 * 240]);
    ui.handle_touch(TouchEvent::Tap { x: 120, y: 170 });
    assert_eq!(ui.take_media_request().unwrap().command, Command::Play);
    ui.set_media(State::new());
    ui.handle_touch(TouchEvent::Tap { x: 190, y: 170 });
    assert!(ui.take_media_request().is_none());
}

#[test]
fn renderer_fits_pages_and_preserves_screen_during_settings_and_clock_ticks() {
    use TouchEvent::*;
    let mut ui = WatchUi::new();
    ui.set_ble_address([0x13, 0x42, 0xcf, 0x27, 0xd2, 0xc2]);
    let mut frame = Frame::new();
    render(&mut ui, &mut frame);
    frame.preview("clock-en");
    // Inactive segments must disappear into black, leaving only the colored
    // active segments inside the digit area, including after a minute change.
    for y in 66..136 {
        for x in 20..230 {
            let pixel = frame.pixels[y * 240 + x];
            assert!(pixel == crate::theme::CLOCK || pixel == crate::theme::BACKGROUND);
        }
    }
    let before = frame.pixels.clone();
    let mut next = time();
    next.minutes += 1;
    ui.draw_time(&mut frame, next);
    for (index, (old, new)) in before.iter().zip(&frame.pixels).enumerate() {
        if old != new {
            assert!(
                (66..=140).contains(&(index / 240)),
                "clock tick changed another region"
            );
        }
    }
    ui.handle_touch(SwipeUp);
    render(&mut ui, &mut frame);
    frame.preview("settings-en");
    let before = frame.pixels.clone();
    assert!(!ui.handle_touch(Tap { x: 100, y: 65 }));
    ui.draw_changed_settings(&mut frame);
    assert_ne!(before, frame.pixels);
    for (index, (old, new)) in before.iter().zip(&frame.pixels).enumerate() {
        if old != new {
            assert!(
                (50..=77).contains(&(index / 240)),
                "brightness cleared another region"
            );
        }
    }
    ui.handle_touch(Tap { x: 40, y: 170 });
    render(&mut ui, &mut frame);
    ui.handle_touch(Tap { x: 40, y: 130 }); // Bokmål.
    render(&mut ui, &mut frame);
    frame.preview("preferences-nb");
    ui.show_clock();
    render(&mut ui, &mut frame);
    frame.preview("clock-nb");
    for gesture in [SwipeRight, SwipeUp, SwipeLeft] {
        ui.show_clock();
        ui.handle_touch(gesture);
        render(&mut ui, &mut frame);
    }
    ui.open_media();
    render(&mut ui, &mut frame);
    frame.preview("media-nb");
    ui.show_clock();
    ui.handle_touch(SwipeRight);
    render(&mut ui, &mut frame);
    frame.preview("home-nb");
    ui.receive_notification(
        parse_new_alert("\0\u{1}\0Blåbær\0På vei hjem. Æ Ø Å é ü ñ".as_bytes()).unwrap(),
        true,
    );
    render(&mut ui, &mut frame);
    frame.preview("notification-nb");
    ui.show_clock();
    ui.handle_touch(SwipeUp);
    ui.handle_touch(Tap { x: 50, y: 210 });
    for y in [60, 110] {
        ui.handle_touch(Tap { x: 50, y });
        render(&mut ui, &mut frame);
        frame.preview(if y == 60 { "about-nb" } else { "firmware-nb" });
        ui.handle_touch(SwipeDown);
    }
    assert_eq!(frame.clipped, 0, "renderer drew outside 240x240");
}
