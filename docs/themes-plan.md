# Watch themes: proposed design

Use one firmware with a small runtime-selectable palette. Start with Catppuccin
Mocha as the default; add other flavors as named compiled palettes when useful.
Use the [official palette](https://github.com/catppuccin/palette) as the source,
pin the version and retain its required attribution when adding palette data.
This document proposes the design; it does not change the current watch colors.

Define a `Theme` with RGB565 roles: background, surface, text, muted text, accent,
warning, error and success. Screens/icons use these roles instead of their own
color constants. Eight two-byte colors take sixteen bytes per palette; palette
storage is small compared with fonts, images and a framebuffer. Reuse the same
layout, icons and seven-segment font across palettes. Start by converting the
existing renderer's colors to these roles before adding selection controls.

The UI owns the selected theme ID. A selection invalidates the current page;
ordinary settings still invalidate only their row. Apply pending changes before
backlight-on during wake, and use one controlled redraw for an explicit theme
change while awake. BLE updates while asleep should not wake the screen merely
to change colors.

Later, expose supported theme IDs and a selected-ID read/write through a versioned
Kongle capability. Watch Settings and Furu call the same validated setter. Reject
unsupported IDs and changes during firmware operations; serialize redraws in the
UI task. Add the capability to Furu's Rust/TypeScript/catalog together. Avoid
arbitrary palette uploads or separate firmware artifacts per theme initially.

Keep selection in RAM initially, as other watch preferences are today. Persist
preferences only after the shared LittleFS design has been validated through
resets and InfiniTime rollback. Theme configuration should not force experimental
flash persistence into ordinary release builds.

Check physical legibility at all brightness settings, disconnected/low-battery
states, inactive segments and notification text. RGB565 conversion loses color
precision, so inspect the display rather than assuming desktop previews match.

Font follow-up: use an embedded-graphics Latin-1 font for Western European text,
retain printable UTF-8 characters in notification parsing, and wrap by characters
instead of byte offsets. This includes æ/ø/å and several other accented letters.
Broader European coverage needs a chosen glyph set/fallback strategy, not just
three extra glyphs. Measure the linked font size before adding more scripts.
