# Non Secure Firmware for Ledger Blue 

This project contains the proxy firmware running on the STM32L4 Non Secure MCU on Ledger Blue, executing commands on behalf of the ST31 Secure Element 

This part is getting heavily redesigned for Ledger Blue commercial release in June - it is mostly provided to let developers play with lower layers, or write their own totally different code reusing the device hardware.   

To build the firmware, run the makeall script in build/stm32l476 

You can reflash the firmware using JTAG or following the procedure described along with the [official release images](https://github.com/LedgerHQ/ledger-blue/tree/master/nonsecure-firmware-release-dev)

