# Kongle OTA development sequence

This branch provides an **opt-in staging receiver** for the ST-Link development
board. Builds with `--features mcuboot,ota-staging` accept application-only
Nordic legacy DFU from Furu, erase and program the external secondary slot,
read it back, and show transfer progress on the watch. Ordinary builds do not
erase external flash. The receiver deliberately rejects the final validation
request after a valid readback, so Furu reports that the update has **not**
been installed. It does not mark an image pending or confirm a trial image.
Keep the daily-driver watch on InfiniTime until the complete flow has passed
development-board testing.

## Boot and recovery

The pinned PineTime bootloader 1.0.1 owns the image swap and its button menu.
It samples the button for about five seconds after startup: hold through blue
to revert to the previous image, or through red to load recovery firmware.
Kongle now restarts after a three-second hold when built with `mcuboot`, so
continue holding through the bootloader's menu. A short press still changes
brightness. The standalone build does not use the long hold to reset.

The bootloader's watchdog reverts an unconfirmed trial image after a reset.
Kongle currently feeds that watchdog but does **not** confirm the image. A
future explicit confirmation action should be separate from successful boot
and should be tested only after rollback and recovery work on the dev board.

## Receiver and activation work

The SPI driver shares the display bus on P0.02/03/04, with flash CS on P0.05.
It accepts the two 4 MiB JEDEC flash families supported by the bootloader and
restricts image writes to the OTA slot (`0x40000..0xB4000`). Erasure starts
with the trailer sector and verifies that the old pending-image marker is gone
before erasing the rest. It never touches bootloader assets below `0x40000` or
the filesystem above `0xB4000`.

Inside `devenv shell`, build and flash the development board with
`KONGLE_FEATURES=mcuboot,ota-staging bash scripts/flash_app.sh`. Generate a
matching test package with
`KONGLE_FEATURES=mcuboot,ota-staging bash scripts/build-dfu-package.sh`.
This produces `kongle-mcuboot-app-dfu-<version>.zip` under the MCUBoot build
directory. The default `scripts/flash_app.sh` build uses `mcuboot` alone and
does not enable staging.
In Furu, enable the `infinitime.dfu` feature for the Kongle device profile
before selecting the ZIP. Furu will transfer the image, then report a rejected
validation. This is the expected result while activation is disconnected.

Before enabling installation:

1. Verify JEDEC ID, full-slot erase, page programming, readback, percentage,
   interruption behavior, bad CRC rejection, and recovery from BLE disconnect
   on the development board. Its stuck button means the button-hold path needs
   a separate hardware test later; ST-Link reset can test the bootloader path.
2. Verify the primary-trailer guard on hardware. It blocks staging when
   `copy_done` indicates a swapped but unconfirmed trial image, because the
   secondary slot may hold its rollback copy. Test a debugger-flashed primary,
   confirmed image, and unconfirmed trial before allowing this on closed
   devices. Keep staging opt-in until this is proven.
3. Review the full readback CRC, MCUBoot header and size checks. The optional
   `ota-activation` feature compiles a trailer writer, but it has no production
   caller or SPI implementation. Keep it disconnected until the write and
   readback sequence has passed development-board testing.
4. When enabling activation in a later PR, write the MCUBoot magic to the last
   16 bytes of the secondary slot only after verification, read it back, and
   reset only after the final DFU response has reached Furu. Test interrupted
   transfers, corrupted images, flash write failures, trial boot, automatic
   rollback, manual blue revert, and red recovery with ST-Link available.
5. Add an explicit, deliberate trial-image confirmation step only after those
   scenarios pass. Do not auto-confirm at startup: that would remove the
   bootloader's rollback safety net.

The watch percentage means **bytes written and read back from external flash**,
not that the image is installed. Progress display updates only when the integer
percentage changes; validation and staged states are shown separately.

References: [PineTime bootloader](https://github.com/InfiniTimeOrg/pinetime-mcuboot-bootloader/tree/1.0.1),
[InfiniTime DFU implementation](https://github.com/InfiniTimeOrg/InfiniTime/blob/main/src/components/ble/DfuService.cpp),
[InfiniTime flash map](https://github.com/InfiniTimeOrg/InfiniTime/blob/main/src/components/fs/FS.h).
