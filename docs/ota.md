# Kongle OTA development sequence

Builds with `--features mcuboot,ota-activation` accept application-only
Nordic legacy DFU from Furu, erase and program the external secondary slot,
read it back, mark it for an MCUBoot **test swap**, and show transfer progress
on the watch. The `ota-staging` feature alone still rejects final validation.
Standalone builds do not erase external flash. Neither feature confirms a trial
image: a subsequent reset should revert to the prior firmware.

Before updating *from InfiniTime*, open Settings → Firmware on the watch and
confirm that the running image is validated. [InfiniTime 1.16.1's DFU
receiver](https://github.com/InfiniTimeOrg/InfiniTime/blob/1.16.1/src/components/ble/DfuService.cpp)
erases the secondary slot at the start of an update even if the current image
is still a trial. That slot may contain the only rollback copy. Its `Image OK`
message confirms the transfer CRC; it does not confirm that MCUBoot selected
the new image.

## Boot and recovery

The pinned PineTime bootloader 1.0.1 owns the image swap and its button menu.
It samples the button for about five seconds after startup: hold through blue
to revert to the previous image, or through red to load recovery firmware.
Kongle now restarts after a three-second hold when built with `mcuboot`, so
continue holding through the bootloader's menu. A short press toggles the
display. The standalone build does not use the long hold to reset.

The bootloader's watchdog reverts an unconfirmed trial image after a reset.
Kongle feeds that watchdog but does not automatically confirm the image.
The explicit Firmware page action is documented in
[firmware confirmation](firmware-confirmation.md).

## Receiver and activation work

The SPI driver shares the display bus on P0.02/03/04, with flash CS on P0.05.
It accepts the two 4 MiB JEDEC flash families supported by the bootloader and
restricts image writes to `0x40000..0xB3000`, reserving the final 4 KiB of
the OTA slot for MCUBoot swap metadata. Erasure starts
with the trailer sector and verifies that the old pending-image marker is gone
before erasing the rest. It never touches bootloader assets below `0x40000` or
the filesystem above `0xB4000`.

Inside `devenv shell`, build and flash the development board with
`bash scripts/flash_app.sh`. Generate a matching test package with
`bash scripts/build-dfu-package.sh`.
This produces `kongle-mcuboot-app-dfu-<version>.zip` under the MCUBoot build
<<<<<<< HEAD
directory. These scripts default to `mcuboot,ota-activation`; set
`KONGLE_FEATURES` explicitly to test a different feature set.
The package builder appends four erased bytes when the MCUBoot image length is
an exact multiple of 200. [InfiniTime 1.16.1's DFU receiver](https://github.com/InfiniTimeOrg/InfiniTime/blob/1.16.1/src/components/ble/DfuService.cpp)
only writes MCUBoot's pending-image marker from its partially filled final
200-byte buffer. Without this padding it can show `Image OK` but then reboot
straight into the old firmware. The padding is outside the image's hashed
payload and TLV; the init-packet CRC covers the padded DFU bytes.
The activation validator also accepts InfiniTime's application-only MCUBoot
DFU ZIPs. InfiniTime 1.16.1 uses a 32-byte image header, whereas Kongle's
image has a 512-byte header. It reads the header length from each image and
checks its bounds, rather than requiring either specific length. A full flash
readback and a matching init-packet CRC are required before the trial marker
is written; MCUBoot performs its own image validation when it boots.
In Furu, enable the `infinitime.dfu` feature for the Kongle device profile
before selecting the ZIP. Furu should finish the transfer, and Kongle should
reset into the new image. The watch shows `Trial: reset reverts` while the
image remains unconfirmed. A package built with `ota-staging` alone will still
end in rejected validation (`opcode 0x04`, status `0x05`).
If using `bash scripts/debug.sh` for RTT logs, start it **before** the OTA:
the script issues `reset run` on startup, which would immediately revert a
trial image if run afterward.

Before trying a closed device or adding manual confirmation:

1. Verify JEDEC ID, full-slot erase, page programming, readback, percentage,
   interruption behavior, bad CRC rejection, and recovery from BLE disconnect
   on the development board. Its stuck button means the button-hold path needs
   a separate hardware test later; ST-Link reset can test the bootloader path.
2. Verify the primary-trailer guard on hardware. It blocks staging when
   `copy_done` indicates a swapped but unconfirmed trial image, because the
   secondary slot may hold its rollback copy. Test a debugger-flashed primary,
   confirmed image, and unconfirmed trial before allowing this on closed
   devices. Keep staging opt-in until this is proven.
3. Test with different old and new builds so the swap and subsequent revert are
   visible. The activation feature writes only the secondary trailer magic,
   checks it, then resets after acknowledging DFU opcode 5. It leaves
   `image_ok` erased. After the trial boots, use an ST-Link reset to test that
   MCUBoot returns to the prior image. Keep the debugger connected and a known
   good image available for recovery.
4. Test interrupted transfers, corrupted images, flash write failures, trial
   boot, automatic rollback, manual blue revert, and red recovery. The stuck
   button prevents the last two tests on this board for now.
5. Add an explicit, deliberate trial-image confirmation step only after those
   scenarios pass. Do not auto-confirm at startup: that would remove the
   bootloader's rollback safety net.

The watch percentage means **bytes written and read back from external flash**,
not that the image is installed. Progress display updates only when the integer
percentage changes; validation and trial states are shown separately.

References: [PineTime bootloader](https://github.com/InfiniTimeOrg/pinetime-mcuboot-bootloader/tree/1.0.1),
[InfiniTime DFU implementation](https://github.com/InfiniTimeOrg/InfiniTime/blob/main/src/components/ble/DfuService.cpp),
[InfiniTime flash map](https://github.com/InfiniTimeOrg/InfiniTime/blob/main/src/components/fs/FS.h).
