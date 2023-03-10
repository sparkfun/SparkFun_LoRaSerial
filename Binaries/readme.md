Programming SAMD21 in Circuit
===========================================================

LoRaSerial uses a SAMD21. This folder contains the various production and release binaries for the device. Please see [Updating LoRaSerial Firmware](https://docs.sparkfun.com/SparkFun_LoRaSerial/firmware_update/) for instructions on how to update the firmware on the device.

This folder also contains binaries combined with a UF2 bootloader. These binaries are loaded during production to allow both the bootloader and firmware to be loaded in one step. These files, along with atprogram.exe are used with a J-Link programmer to load the combined binary onto a target device.