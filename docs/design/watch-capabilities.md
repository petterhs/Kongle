# Language, glance data and remote controls

Design proposals following the Mocha sketch review. No new firmware protocol,
Android integration or resource partition is implemented by this document.

## Language: start with compiled English and Norwegian Bokmål

Use stable message IDs, such as `SettingsBrightness`, with complete `en` and `nb`
catalogs compiled into firmware. Store UTF-8 strings in read-only flash, not a
heap-allocated copy of each catalog. Keep font encoding/glyph lookup separate
from string storage. Select strings when rendering; do not cache translated
labels in UI state. Derive weekdays and dates from the selected locale too.
English is the firmware default. Do not silently change it to the phone language;
Furu can offer Bokmål as an explicit setup choice.

Use a small typed catalog initially. A build/host check should require both
catalogs to cover every ID. Introduce plural rules when counts appear in prose;
avoid assembling sentences from fragments. Missing future translations fall
back to English; missing glyphs use a visible fallback rather than dropping text.
Do not translate notification content, media titles or user-assigned names.

Expose the same validated language setter in watch Settings and a future
versioned BLE configuration capability. Advertise supported locale IDs and
current selection. Furu's own UI language and the watch's language are separate
preferences; start Furu at the phone locale and offer an explicit watch-language
choice. A one-time setup can offer the phone language, but reconnect must not
overwrite a deliberate watch choice. The watch owns its current preference;
Furu reads it rather than continuously enforcing its last cached value.

Language selection should work without a phone. Apply it before backlight-on if
asleep, otherwise redraw the active page once. Persist using the established
settings mechanism after checking what is actually merged; do not add ad-hoc
flash writes or assume a RAM-only setting survives reset. If persistence is not
available, make the temporary limitation explicit during development.

Text for two small menus is likely inexpensive, but record actual flash deltas
before deciding how many locales to bundle. Expanded font atlases and shaping
may cost much more than strings. Latin-1 covers the initial pair; it does not
provide all European scripts, right-to-left layout or CJK rendering.

## Optional downloadable resources later

BLE transfer of translation data is feasible: it is a small, infrequent bulk
transfer rather than continuous streaming. It is unnecessary complexity for two
languages. Keep firmware-defined IDs and a schema version so external catalogs
can be added later without redesigning the UI.

If bundled fonts/catalogs outgrow the application slot, evaluate one selected
external-flash language/font pack, rather than assuming all languages must be
stored on every watch. Transfer through Furu with schema/glyph requirements,
length limits, checksum and atomic activation. A firmware update must explicitly
accept the pack version or fall back to built-in English. Fonts/layout for a
new script can require firmware changes even when translation strings are external.

Keep essential boot/recovery/update text built in. Do not assume a free resource
partition: audit the current external-flash map first, including bootloader
recovery assets, OTA secondary slot and any InfiniTime/LittleFS ownership.
Validate interrupted writes, power loss, firmware rollback, old packs with new
firmware and switching to/from InfiniTime before making resource packs a default.

## A few bounded display slots, not downloadable watch programs

See [the shared LittleFS investigation](shared-littlefs.md) before planning step
persistence or cross-firmware totals. Sharing storage does not automatically
share a step-counter format with InfiniTime.

The first configurable clock status can select steps or an available remote
value, with a label, unit, timestamp and availability. Keep the number of slots
small and the rendering shapes compiled into Kongle. Unknown remote values must
not appear as zero, and stale readings must be distinguishable from live data.

Integrations belong in Furu. Define a narrow internal capability interface when
the first integration is built; avoid designing a general dynamic plugin runtime
in advance. Configure connections, entities and labels on the phone. Kongle gets
bounded data and action IDs, never credentials, arbitrary scripts or HA JSON.
Use native/background ownership for ongoing sessions, independent of an open
Furu route. Existing Android lifecycle limitations still need testing.

## Home Assistant

Flow: Kongle ←BLE→ Furu ←network→ Home Assistant. No phone or reachable HA means
remote controls are unavailable; do not queue a light toggle for later execution.

Start with four phone-configured actions on a dedicated 2 × 2 watch page. Furu
maps action IDs to HA entities/services/scripts. Show pending, success or failure
from actual acknowledgements; a BLE delivery acknowledgement is not evidence
that a lamp changed. Avoid duplicate action execution after reconnect/retry;
toggle in particular is not idempotent. A later entity-state subscription can
show actual on/off status, distinct from the result of sending a command.

HA's [WebSocket API](https://contributors.home-assistant.io/core/internals/websocket-api/)
supports authenticated subscriptions and service calls. Subscribe only to the
selected values and send compact changes over BLE. Do not wake the display for
every remote update. On reconnection request a fresh snapshot instead of assuming
cached temperature/light state is current.

The proposed primary navigation is Home controls ← Clock → Notifications.
Activity opens from tapping the clock's step count. Clock slots display an icon
plus value without a "steps" label or thousands separator. Indoor and outdoor
temperature should have distinct icons; show units and retain data freshness.
The sketch's fourth humidity slot is illustrative, not a required default.

### Default and optional clock fields

After physical UI review, show only steps in the current firmware prototype.
Do not fill the space with unavailable temperature/humidity placeholders. When
heart-rate support is implemented, consider steps plus an optional pulse field;
displaying a pulse field must not implicitly enable continuous measurement.
Begin with an explicit on-demand measurement, sensor off afterward, and retain
the last reading with its age. Interval/background measurement is a later
separate preference, after checking power and measurement quality on hardware.

Upstream's [HeartRateTask](https://github.com/InfiniTimeOrg/InfiniTime/blob/main/src/heartratetask/HeartRateTask.cpp)
explicitly starts/stops measurements and supports optional background intervals.
The [HRS3300 driver](https://github.com/InfiniTimeOrg/InfiniTime/blob/main/src/drivers/Hrs3300.cpp)
turns off both sensing and LED drive when disabled. Its LED-current setting is
not a measurement of the entire watch's average current; do not derive battery
life or numerical accuracy from this setting alone. Compare actual battery
trends and measurements before choosing automatic sampling defaults.

Useful future fields include an active timer/countdown or next alarm (local),
phone battery/next calendar item (phone bridge), outside weather temperature,
indoor temperature from a selected HA sensor, or a selected stock/crypto price.
None is required to occupy all four positions. Weather-service data is distinct
from an indoor thermometer and from an outdoor sensor at the user's home.

Use the same bounded clock-field contract regardless of provider: stable field
ID, icon, compact value, unit/currency, source timestamp and availability. Furu
selects the source, ordering and enabled fields. Keep enabled positions stable
during temporary disconnects, with stale/unavailable indication; hide disabled
fields rather than constantly rearranging the clock as data arrives.

HA can supply an initial market-price sensor without a dedicated Furu market
integration. Later a native Furu provider can use the same display contract.
Select the update cadence on the phone; do not make the watch poll providers
or redraw a price every second. Show currency and freshness; keep detailed
quotes, charts and provider configuration on the phone or a detail page.

Publishing battery/steps/connection as durable HA sensors is a separate part of
the integration: evaluate HA's integration/registration model or
[MQTT sensors and discovery](https://www.home-assistant.io/integrations/sensor.mqtt/).
The latter adds a broker dependency. Avoid choosing an entity-publication scheme
implicitly just because WebSocket service calls are convenient. Include source
timestamps and unavailable state; publish new measurements, not a high-rate poll.

## Music and audiobooks

Furu can bridge Android's active media session to Kongle. Android
[MediaController](https://developer.android.com/reference/android/media/session/MediaController)
allows control via an enabled notification listener (or another authorized
session token). Request any missing user access explicitly during feature setup.
Select the active session in Furu and handle multiple players; never guess a
different target when an old watch command arrives after the session changes.

Use InfiniTime's existing BLE music protocol, not a new Kongle service. The
[protocol documentation](https://github.com/InfiniTimeOrg/InfiniTime/blob/main/doc/ble.md#music-control)
and [MusicService implementation](https://github.com/InfiniTimeOrg/InfiniTime/blob/main/src/components/ble/MusicService.cpp)
define service `00000000-78fc-48fe-8e23-433b3a1942d0`, event characteristic
`00000001-78fc-48fe-8e23-433b3a1942d0` (watch notifies, Furu subscribes), plus
phone-written status/title/artist/album and other metadata characteristics.
Match each characteristic's properties and encoding; do not copy incidental
upstream bugs such as duplicate characteristic definitions or unsafe decoding.

[Event values](https://github.com/InfiniTimeOrg/InfiniTime/blob/main/src/components/ble/MusicService.h):
play `0x00`, pause `0x01`, next `0x03`, previous `0x04`, volume up/down `0x05`/`0x06`,
and screen-open `0xe0`. Start with three large icon targets: previous, play/pause,
next. Furu maps previous/next to Android session skipToPrevious/skipToNext (or
equivalent media-key dispatch for the selected player). Let the player interpret
those commands; do not encode ten-second seeking in Kongle. The user's earbuds
already demonstrate this desirable context-dependent behavior with their apps.

Android's [TransportControls](https://developer.android.com/reference/android/media/session/MediaController.TransportControls)
distinguishes previous/next, rewind/fast-forward and absolute seeking. Not every
app maps previous/next to audiobook skips; test Spotify and the user's audiobook
app first. If a player only exposes custom skip actions, add a phone-side mapping
later without changing the BLE event protocol or claiming all players behave
identically. Never silently fall back to controlling a different active player.

Explore a double button press as the media shortcut. Preserve immediate first
press wake; do not wait for a double-press timeout before showing the clock.
Define how to prevent transient sleep on the second press while already awake.
A medium hold is an alternative, but only execute its shortcut on release so a
continuing recovery/reset hold cannot also trigger a media action. Prototype
timing on hardware before settling the default. Furu-configurable shortcut
mapping comes later through a bounded supported-action list; recovery remains
reserved. The browser preview only has an explicit destination button and does
not simulate physical double-press timing.

Refresh state on changes and on entering the media page; avoid streaming a
seconds counter while the display is asleep. Mark an ended/disconnected session
unavailable and do not replay queued play/pause/skip requests later. Media titles
and remote labels use the same UTF-8/glyph handling as notifications.

## Suggested order

The 0.4.0 stable gates are [media, HR and measured step support](../release-notes/v0.4.0.md).
See the [sensor investigation and test plan](step-counter-plan.md) before choosing
a step-counting configuration. Current primary page positions are Home ← Clock →
Media, with Notifications above and Settings below; double button press opens Media.

Later, screen long press on the clock can open a day/week agenda synced from
Furu. The current touch decoder has contact/tap/swipe events but no complete
coordinate-preserving long-press recognizer. Plan press/release timing, movement
cancellation, conflict with taps/swipes and exclusion from sleep/OTA first.
Furu owns Android calendar permissions and chosen calendars; send a bounded
agenda with event IDs, start/end, all-day status and timezone/freshness metadata.
Do not imply the watch can query a calendar directly, and distinguish an empty
agenda from unsynced/disconnected data. This is future scope, not enabled now.

1. Finish the local Mocha clock/settings experiment with 18 px text and an
   unobtrusive notification counter. Keep current sleep and home-button behavior.
2. Implement the shared UTF-8 rendering prerequisite and built-in `en`/`nb`.
3. Add media controls as the first phone-backed action page, testing the frequent
   rewind use case and Android background/reconnection behavior.
4. Add the optional HA bridge and four-action page; then remote clock statuses
   and HA sensor publication. Reuse proven transport patterns, not a speculative
   universal plugin protocol.
