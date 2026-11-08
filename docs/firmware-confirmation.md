# Manual firmware confirmation

Launcher → Firmware shows the firmware version and boot state. A valid trial
offers **Confirm firmware**. The next screen explains that reset will no longer
restore the old image; **Confirm now** is in a different screen region, while
the original button's region becomes **Keep trial**. Repeated touch reports
cannot confirm. Button navigation, leaving the page or a notification cancels
the pending confirmation. There is no automatic confirmation.

In MCUBoot builds the operation rereads the primary trailer, requires valid
magic, a completed swap (`copy_done`) and a completely erased `image_ok` word,
then uses Embassy NVMC to program `[01 FF FF FF]` at `0x7BFE8`. It never erases a
page or writes the secondary image, bootloader, recovery image or filesystem.
Readback must classify as confirmed before the screen reports success. Unknown
or partially written flag words are refused. Standalone and debugger-flashed
non-trial images cannot request the write. Confirmation is refused while an
update is preparing, receiving, validating or awaiting activation.

This follows the pinned PineTime layout and MCUBoot's `boot_set_confirmed` /
`boot_write_image_ok` logic in [bootutil_misc.c](https://github.com/mcu-tools/mcuboot/blob/v1.5.0/boot/bootutil/src/bootutil_misc.c).
Publishing a release or downloading an update does not confirm a watch.
Confirmation stops automatic rollback on reset; recovery remains a separate
bootloader operation.

Host tests cover the trailer policy, refused corrupt flags, write word,
two-step touch behavior, cancellation and navigation. They cannot prove NVMC
behavior on the actual device. Before daily-driver confirmation, test on the
development board:

1. OTA a distinct image as a trial; verify reset still reverts before confirming.
2. Open Firmware, cancel with Keep trial/button/swipe, and verify rollback is
   still available. Check duplicate taps in the first button's area do not confirm.
3. OTA again, explicitly confirm, check the flag through SWD/readback and reset;
   the same firmware should remain installed and show Firmware confirmed.
4. Verify another OTA can start after confirmation and its new trial can revert.
5. Check the bootloader's recovery path independently. Interrupt the single-word
   write only on a development board with a working debug/recovery path.

Settings changes (battery percentage, brightness and vibration) now redraw only
their affected row/labels while leaving the backlight on. Navigation and wake
still use the full-page rendering path.
