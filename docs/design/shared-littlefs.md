# Sharing InfiniTime's LittleFS: steps and other Kongle data

Research and proposed compatibility rules, not an implementation of step
persistence. Reviewed upstream source on 2026-11-09; recheck before enabling writes.

## Files, not addresses for each value

[InfiniTime FS.h](https://github.com/InfiniTimeOrg/InfiniTime/blob/main/src/components/fs/FS.h)
places LittleFS at external-flash `0x0b4000`, size `0x34c000`, with 4096-byte
blocks. The block-device adapter must know that region. Mounting LittleFS then
reads its metadata and resolves paths; adding files does not require a Kongle
address table or discovering a memory address for each counter. Do not scan
arbitrary flash and guess a writable filesystem based on incidental contents.

The existing `battery-persist-experimental` feature already mounts this region
and uses Kongle-prefixed battery files. It never formats on mount failure.
That is a useful foundation, not evidence that all cross-firmware write cases
have been validated. Keep filesystem version/configuration compatible; library
RAM cache sizes need not all be identical to describe the same disk format.

Ordinary new InfiniTime files can coexist with Kongle files. Use a dedicated
`/kongle/` namespace for future data and leave unrelated files/attributes alone.
Do not migrate the existing battery paths without an explicit migration plan.
Bound our file sizes and retain filesystem headroom so logs cannot fill the
volume and prevent InfiniTime from writing settings/resources. Do not delete
foreign files to make space.

## Step data is not already shared

InfiniTime 1.16.1's
[MotionController](https://github.com/InfiniTimeOrg/InfiniTime/blob/1.16.1/src/components/motion/MotionController.h)
keeps today/yesterday counters in a RAM circular buffer. Its
[implementation](https://github.com/InfiniTimeOrg/InfiniTime/blob/1.16.1/src/components/motion/MotionController.cpp)
updates those counters and publishes BLE values, rather than writing a step file.
The [system startup](https://github.com/InfiniTimeOrg/InfiniTime/blob/1.16.1/src/systemtask/SystemTask.cpp)
also soft-resets the motion sensor. Do not claim that mounting LittleFS lets
Kongle import or update a common existing InfiniTime step record.

Kongle can persist its own dated counters, but unmodified InfiniTime will not
consume that file. Seamless combined daily totals across firmware would need
an agreed file format adopted upstream, or a companion-side reconciliation
design with a way to restore a counter. Investigate restore/write capabilities
before promising this. Copying a RAM structure layout into a file does not
create an interoperable persistence contract.

For Kongle-owned history, define a versioned record containing date/day identity,
counter semantics and validation data. Handle invalid/unset time, midnight,
timezone/time correction, sensor resets and reboot restore without double
counting. Persist bounded checkpoints/deltas rather than one flash write per
step. Decide and document how much recent progress can be lost on power loss.

## Compatibility boundary

Adding an unrelated file is not the major risk. Changes to the partition map,
disk format, mount assumptions, formatting behavior or another firmware's
resource cleanup can break interoperability. No independent firmware can
guarantee compatibility with every future upstream change without any validation.

[InfiniTime FS.cpp](https://github.com/InfiniTimeOrg/InfiniTime/blob/main/src/components/fs/FS.cpp)
currently formats when mounting fails. Kongle must not upgrade disk features in
a way that makes the older InfiniTime unable to mount, even if Kongle can read
the upgraded disk. A new namespace prevents name collisions, not this failure.

Before enabling routine shared writes:

- Work on a backup/copy of a real InfiniTime filesystem image first. Mount and
  verify old files without formatting or on-disk migration.
- Add, append, replace and bound only Kongle files. Verify foreign contents
  remain intact and the original InfiniTime can still mount/read/write them.
- Test with extra unknown files, near-full storage and interrupted commits.
- Test both directions through OTA, trial rollback and recovery/resource updates
  on the dev unit. Confirm which update mechanisms intentionally replace data.
- Treat unsupported or unreadable filesystems as persistence unavailable and
  continue with RAM data. Never erase/reformat automatically in Kongle.

This avoids needing a list of every InfiniTime file. Keep a small compatibility
test against supported upstream versions and review only filesystem-affecting
changes when adding support for new versions.
