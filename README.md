SparkFun LoRaSerial
========================================

[![SparkFun LoRaSerial](https://cdn.sparkfun.com//assets/parts/1/8/9/4/0/19311-SparkFun_LoRaSerial_Kit_-_915MHz-01.jpg)](https://www.sparkfun.com/products/19311)

[*SparkFun LoRaSerial Kit - 915MHz (WRL-19311)*](https://www.sparkfun.com/products/19311)

Head to the [LoRaSerial Product Manual](http://docs.sparkfun.com/SparkFun_LoRaSerial/) for technical documentation.

LoRaSerial is a pair of serial radio modems that simply passes serial back and forth. Do you have a system that needs to report data every few seconds? Is it located beyond WiFi or Bluetooth range? Do you need 9 mile/15 kilometer range? LoRaSerial transmits data over LoRa giving it incredible range compared to other methods. We’ve regularly transmitted over 9 miles/15km line-of-sight using two LoRaSerial radios, right out of the box.

LoRaSerial utilizes a 1 watt 915MHz transceiver and an open source protocol to and transmit AES encrypted data at 4800bps or approximately 480 bytes per second. Video streaming, this is not. LoRaSerial is very good at getting whatever data you need from point A to point B, encrypted, without configuration. The radios automatically frequency hop (FHSS) between channels to avoid collisions. The ‘airspeed’ or data rate is configurable up to approximately 2,000 bytes per second for shorter range, or for *extremely* long range transmissions, as low as 400 bytes per second.

LoRaSerial radios support simple point to point and point as well as multipoint broadcasts. Multipoint broadcasting makes it ideal for GNSS RTK and many other scenarios where one device needs to produce data and many devices need to consume that data.

The LoRaSerial firmware supports an innovative and simple to use ‘training’ method. Pressing the train button on both radios will generate a new random network ID and AES encryption key and share them between radios. This makes pairing radios in the field as simple as a button press. WiFi WPS this is not. Bring the radios near each other and the LoRaSerial training method is simple and secure.

Currently, SparkFun is offering radios utilizing 915MHz modems that are allowed in most parts of the world. Please check your local restrictions. The radios are fully configurable to restrict frequencies, channels, dwell time, power output, and a variety of other settings to make the radios compatible with your local regulations.

Repository Contents
-------------------

* **/Binaries** - Compiled firmware, ready to load
* **/Documents** - Datasheets and additional product information
* **/Firmware** - Main Arduino firmware
* **/Hardware** - Eagle PCB files

Documentation
--------------

* **[LoRaSerial Product Manual](http://docs.sparkfun.com/SparkFun_LoRaSerial/)** - Configuration guide for the SparkFun LoRaSerial.

License Information
-------------------

This product is _**open source**_! 

Please review the LICENSE.md file for license information. 

If you have any questions or concerns on licensing, please contact technical support on our [SparkFun forums](https://forum.sparkfun.com/viewforum.php?f=152).

Distributed as-is; no warranty is given.

- Your friends at SparkFun.
