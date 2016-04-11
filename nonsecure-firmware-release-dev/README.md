# Official Non Secure firmware images for Ledger Blue 

This directory contains the deterministic hex files and signed image of the non secure (STM32) firmware images for Ledger Blue 

You can reflash the images using JTAG or in non secure bootloader mode : 

  - Turn on Ledger Blue, keeping the button pressed until "secure bootloader", then "bootloader" is displayed
  - Use python -m ledgerblue.runScript --fileName stm32l476_seproxyhal.patch_apdu      

