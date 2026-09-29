# Bootloader

The bootloader binary used by Kongle comes from
`https://github.com/InfiniTimeOrg/pinetime-mcuboot-bootloader/releases/tag/1.0.1`.

The checked-in binary was compared byte-for-byte with that release's
`bootloader-1.0.1.bin`. Run `sha256sum --check bootloader/SHA256SUMS` from the
repository root to verify the pinned copy. This bootloader uses unsigned images;
the image hash checks integrity, not publisher identity.

Flash the `bootloader-1.0.1.bin` file in this directory to the PineTime using:

```bash
sudo bash scripts/flash_bootloader.sh
```

This programs the MCUBoot bootloader at address `0x00000000`. The Kongle application
is then built as an MCUBoot image and flashed to the primary slot at `0x00008000`
via `scripts/flash_app.sh`.
