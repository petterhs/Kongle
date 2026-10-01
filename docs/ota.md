# Kongle OTA development sequence

This branch prepares the watch UI and the MCUBoot activation boundary. It does
**not** accept firmware over BLE, write external flash, mark an image pending,
or confirm a booted trial image. Keep the daily-driver watch on InfiniTime until
the complete flow has passed development-board testing.

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

1. Add an external SPI flash driver for both PineTime flash variants. Verify
   JEDEC ID, reads, sector erase, page boundaries, and readback using only the
   secondary OTA slot (`0x40000..0xB4000`) on the development board. Never
   erase the bootloader assets below `0x40000` or the filesystem above
   `0xB4000`.
2. Add InfiniTime-compatible Nordic **legacy** DFU GATT characteristics. Furu
   already sends an application-only package over this protocol. Reject
   invalid lengths, unsupported image types, out-of-order packets, and writes
   beyond the secondary slot. Persist progress only after SPI writes succeed;
   publish `UpdateStatus::Receiving` with the accepted and total byte counts.
   Show verification, ready, and failure states separately from the percentage.
3. Read the entire staged image back from SPI flash and compare its CRC16 with
   the Nordic init packet. Check the MCUBoot header, its declared image size,
   and the slot boundary. Only then create `ActivationPlan`. The optional
   `ota-activation` feature compiles a trailer writer, but it has no production
   call site or flash implementation. Keep it disconnected until the write and
   readback sequence has been reviewed and tested on the development board.
4. When enabling activation in a later PR, write the MCUBoot magic to the last
   16 bytes of the secondary slot only after verification, read it back, and
   reset only after the final DFU response has reached Furu. Test interrupted
   transfers, corrupted images, flash write failures, trial boot, automatic
   rollback, manual blue revert, and red recovery with ST-Link available.
5. Add an explicit, deliberate trial-image confirmation step only after those
   scenarios pass. Do not auto-confirm at startup: that would remove the
   bootloader's rollback safety net.

The watch percentage means **bytes safely accepted**, not that the image is
valid or installed. At 100%, show a separate verification state until the
image is ready. Furu should likewise distinguish transfer completion from
installation and reconnection after reboot.

References: [PineTime bootloader](https://github.com/InfiniTimeOrg/pinetime-mcuboot-bootloader/tree/1.0.1),
[InfiniTime DFU implementation](https://github.com/InfiniTimeOrg/InfiniTime/blob/main/src/components/ble/DfuService.cpp),
[InfiniTime flash map](https://github.com/InfiniTimeOrg/InfiniTime/blob/main/src/components/fs/FS.h).
