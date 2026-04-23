/* STM32G071 memory layout for AM32-compatible bootloader */
MEMORY
{
  FLASH : ORIGIN = 0x08001000, LENGTH = 60K
  RAM   : ORIGIN = 0x20000000, LENGTH = 36K
}
