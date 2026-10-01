MEMORY
{
  /* NOTE K = KiBi = 1024 bytes */
  /* Standalone: no bootloader; firmware runs from 0x00000000.
   * Use this for cargo run --release (rapid dev without MCUBoot).
   */
  FLASH : ORIGIN = 0x00000000, LENGTH = 512K
  RAM : ORIGIN = 0x20000000, LENGTH = 64K
}
