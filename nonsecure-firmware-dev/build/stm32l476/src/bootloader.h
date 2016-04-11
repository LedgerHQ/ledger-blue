
#ifndef BOOTLOADER_H
#define BOOTLOADER_H

typedef void (*appmain_t) (unsigned int button_press_duration);

typedef void (*boot_t) (void);

typedef struct bootloader_configuration_s {
  unsigned short crc;
  // address of the main address, must be set according to BLX spec ( ORed with 1 when jumping into Thumb code)
  appmain_t appmain;
  // VTOR value for interrupt vector delegation to the user code, must be aligned on 32 bits
  // to be done in called code // unsigned int vtor;

#define BOOTSECTOR_MAGIC 0xDEADB007
  unsigned int bootsector_magic;
  boot_t bootsector_main;

} bootloader_configuration_t;

extern bootloader_configuration_t N_bootloader_configuration;

// ensure placement for bootsector delegation to be always stable
__attribute__ ((section(".blmain")))
void main(volatile unsigned int button_press_duration);

#endif // BOOTLOADER_H