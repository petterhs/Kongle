# Mocha / TUI exploration

Local proposal, not an agreed navigation redesign or firmware implementation.
Open [the interactive sketch](mocha-tui.html) in a browser. It works offline;
all values are illustrative. No firmware or PR is changed by opening it.

Hardware testing now takes precedence over the browser sketch: the current local
firmware uses a black background, 20 px text and two-pixel outlines after Mocha's
base looked gray and 18 px text looked too small on the LCD. The header now has
only an unread bell, hidden when acknowledged, without a count. Earlier sketch
descriptions below record the design exploration; see
[local testing notes](local-ui-testing.md) for the running firmware behavior.

## Visual direction

Keep the seven-segment clock; use monospaced text for labels and content.
Borrow the visual grammar of a terminal UI: quiet rounded outlines, a title
interrupting the upper border, aligned values and restrained accent colors.
Do not borrow desktop density, keyboard selection, tiny controls or persistent
navigation instructions. One or two panels per page are enough at 240 × 240.

After the first review, use 18 px UI text as the starting point. Keep notification
count and icon in the top status row; remove the dedicated inbox panel from the
clock. Put weekday and abbreviated date together in the clock panel title.
Use two columns of compact statuses below it, each with an icon on the left and
value on the right: ungrouped step count (2342), indoor/outdoor temperature and an
illustrative humidity value. No "steps" label on the clock. These are example
slots, not a decision to always show four populated values. Missing/stale remote
data still needs a distinct treatment. English is the default; the sketch also
has an external Bokmål selector. Do not turn the clock into a dashboard with
several competing panels.

Use Catppuccin Mocha base/text, surface2 outlines and blue accents initially.
The sketch uses the official palette v1.8.0, with optional RGB565 quantization.
Its browser font and antialiasing are approximate: it is not a pixel-exact
embedded-graphics preview or a brightness simulator. Keep inactive segments
subtle; test the outline contrast on the real LCD at low brightness.

Implement panel outlines and title gaps as geometric primitives, not Unicode
box-drawing glyphs. Keep embedded-graphics and the current display driver;
Ratatui is a visual reference, not a proposed firmware dependency.
Retain partial redraws, display sleep and rendering before backlight-on. A
full framebuffer needs 115,200 bytes before any BLE/application memory, so this
direction must work without one. Theme colors are inexpensive; large font
atlases, repeated drawing and assets are the costs to measure.

## Navigation to explore

Prototype a small set of glanceable main views: Home controls ← Clock → Notifications.
No wraparound, so the clock has a predictable position. Swipe up from the clock
opens quick settings; About and firmware confirmation live under System rather
than consuming primary swipe directions. A short button press always wakes to
the clock, returns home from other pages, or sleeps when already on the clock.
Keep the long-hold recovery/reset behavior.

Activity opens by tapping the step count. Temperature slots open home controls
in the sketch. Explore double press for media instead of consuming clock space
with a media button. The sketch's separate double-press button only previews
the destination, not physical button timing. A firmware implementation must not
delay wake or let a medium-hold shortcut trigger before a continuing reset hold;
see the capability plan before implementing it.

This is a hypothesis, not a decision. Compare it with the current launcher on
the dev watch. Start with these tasks: checking the time, reading/dismissing a
notification, changing brightness, returning home and finding firmware status.
Record mistaken gestures and whether tasks can be done with one hand. Avoid
adding settings merely to accommodate an unclear default.

The sketch's reverse swipes are provisional. Lists will need vertical scrolling;
decide a consistent back action before adding long lists. A reliable physical
home button provides an escape but should not be the only way out of a detail
page. Recovery confirmation must retain its explicit two-step interaction;
the sketch only shows its destination and cannot confirm anything.

## Fonts

First compare embedded-graphics `mono_font::iso_8859_1::FONT_9X18` and
`FONT_10X20` using real notifications. Both are available in the installed
embedded-graphics 0.8.1 source; no new font dependency is required. Latin-1
includes æ/ø/å and many Western European letters, but does not cover all European
languages. Keep procedural seven-segment digits independent of the text font.

Changing the font alone is insufficient: the current notification parser
replaces non-ASCII text, and wrapping uses byte offsets. Preserve valid UTF-8,
wrap at character boundaries and define a missing-glyph fallback before enabling
the expanded font. Measure flash size with the chosen glyph table linked.
Consider a custom licensed bitmap typeface only after the built-in comparison.

## Language and future integrations

See [language, glance data and remote controls](watch-capabilities.md) for the
proposed English/Bokmål language selection, future resource packs, Home Assistant
bridge and media controls. The sketch includes illustrative 2 × 2 home controls
and audiobook controls; they send no commands to real devices. Their navigation
placement is deliberately undecided, so use the external preview buttons.

## Relationship to Furu

Share semantic colors, typography intent and status language; use layouts suited
to the phone. The watch is for glanceable information and immediate actions.
Furu owns detailed history, firmware discovery/changelogs and profile management.
Device cards should lead with the chosen nickname, then detected firmware/version
and connection state. Keep profile details and BLE diagnostics in device details.

## Implementation sequence

1. Agree the information hierarchy using the sketch and actual tasks.
2. Add semantic theme colors and a small titled-panel helper in the existing
   renderer. Port one clock and one settings view to the dev unit first.
3. Validate contrast, font size, touch targets, wake behavior and partial redraws.
4. Expand navigation only after those two screens feel right. No PR until the
   maintainer is happy with local hardware testing.

See [the existing theme architecture plan](../themes-plan.md) for compiled
palettes selected at runtime, optional BLE selection and persistence constraints.

## Palette attribution

Palette source: [Catppuccin palette](https://github.com/catppuccin/palette),
version 1.8.0. MIT License, Copyright (c) 2021 Catppuccin.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
