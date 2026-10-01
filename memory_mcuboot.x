MEMORY
{
  /* NOTE K = KiBi = 1024 bytes */
  /* Internal flash layout with MCUBoot (PineTime):
   *
   * 0x00000000 - 0x00006FFF : Bootloader          (28 KiB)
   * 0x00007000 - 0x00007FFF : Log                (4 KiB, reserved)
   * 0x00008000 - 0x0007BFFF : Primary app slot   (464 KiB, IMAGE_0)
   * 0x0007C000 - 0x0007CFFF : Scratch            (4 KiB)
   *
   * MCUBoot prepends a 0x200-byte header in the app slot, so the
   * actual firmware (vector table + .text) is linked from 0x00008200.
   * The usable FLASH length is the slot size (0x74000) minus header
   * size (0x200) => 0x73E00 bytes.
   */
  FLASH : ORIGIN = 0x00008200, LENGTH = 0x73E00
  RAM : ORIGIN = 0x20000000, LENGTH = 64K
}
