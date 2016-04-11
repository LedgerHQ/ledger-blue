# Ledger Blue Developer Edition

The Ledger Blue is a personal lightweight mobile device architectured around a ST31 secure element, featuring a touch screen and USB/NFC/BLE connectivity.
It is based on [BOLOS](https://medium.com/@Ledger/introducing-bolos-blockchain-open-ledger-operating-system-b9893d09f333) (Blockchain Open Ledger Operating System), where apps can run securely in full isolation and leverage the main secrets (BIP39 seed) through allocated derivations.

The Blue is available now as an [developer edition](https://www.ledgerwallet.com/products/9-ledger-blue), for enthousiasts and early adopters.

# Starting guide

The Ledger Blue Developer Edition is shipped without any application and without the onboarding process (to initialize the PIN and create/restore the master seed). The factory PIN is 1234.

In the coming days we are going to publish a tutorial on how you can compile and upload your first apps, as well as procedure to update BOLOS firmware to the latest version (which will include the onboarding process).

More developer documentation (SDK, tutorials, examples) will be added in [the coming weeks](https://github.com/LedgerHQ/ledger-blue/blob/master/TODO.md).

# Content of this repository

## Software
* [Ledger Blue development environment](https://github.com/LedgerHQ/ledger-blue/tree/master/env-dev)
* [Python tools for the Ledger Blue](https://github.com/LedgerHQ/ledger-blue/tree/master/loader-python)
* [Ledger Blue Secure SDK](https://github.com/LedgerHQ/ledger-blue/tree/master/secure-sdk-dev)
* [Sample applications for Ledger Blue](https://github.com/LedgerHQ/ledger-blue/tree/master/sample-apps)

## Firmware
* [Non Secure Firmware for Ledger Blue](https://github.com/LedgerHQ/ledger-blue/tree/master/nonsecure-firmware-dev)
* [Official Non Secure firmware images for Ledger Blue](https://github.com/LedgerHQ/ledger-blue/tree/master/nonsecure-firmware-release-dev)

## Hardware
* [Schematics & Assembly Ledger Blue Developer Edition](https://github.com/LedgerHQ/ledger-blue/tree/master/schematics-dev)
* [Casing Ledger Blue Developer Edition](https://github.com/LedgerHQ/ledger-blue/tree/master/casing-dev)
