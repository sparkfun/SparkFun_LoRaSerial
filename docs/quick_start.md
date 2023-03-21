# Quick Start Guide

![Two Linked LoRaSerials](img/SparkFun%20LoRaSerial%20Enclosed%20-%2020029-4.jpg)

LoRaSerial radios are very easy to use and ship ready to connect out of the box. Simply attach the antennas, plug two units into USB ports, open a terminal at each of the USB serial ports, and start passing data back and forth.

![Passing text between two terminals](Original/SparkFun%20LoRaSerial%20-%20P2P%20Serial%20Terminals.gif)

*Passing simple text data between two radios*

The default settings are as follows:

* **Mode:** Point to Point
* **Frequency:** 902 to 928 MHz
* **Channels:** 50
* **Frequency Hopping:** Enabled
* **Broadcast Power:** 30 dbm
* **Airspeed:** 4800 bps
* **Serial:** 57600 bps
* **Encryption:** Enabled
* **Encryption Key:** Default<sup>1</sup>
* **Software CRC:** Enabled

<sup>1</sup> The radios default to a published encryption key. For maximum security, we recommend changing the key and network ID value using command mode or the [training process](http://docs.sparkfun.com/SparkFun_LoRaSerial/training/).

Serial can be passed into the unit either through the USB port, or over the serial connector. If the serial connector is used, the baud rate is 57600 bps by default.

LoRaSerial is designed to get small amounts of data from point A to point B as easily as possible. If you've got a sensor, or Arduino, or any device that can output serial, then it can transmit data over long distances. LoRaSerial has been used to send data over 9 miles (14 km) line-of-sight.

## Additional Documentation

The LoRaSerial products are described in the [introduction](http://docs.sparkfun.com/SparkFun_LoRaSerial/intro/). The [radios](http://docs.sparkfun.com/SparkFun_LoRaSerial/hardware_overview/) support several [operating modes](http://docs.sparkfun.com/SparkFun_LoRaSerial/operating_modes/).

* [Point-to-Point](https://docs.sparkfun.com/SparkFun_LoRaSerial/operating_modes/#point-to-point) with guaranteed delivery or the link breaks
* [Multipoint](https://docs.sparkfun.com/SparkFun_LoRaSerial/operating_modes/#multipoint), two or more LoRaSerial radios, is best for real-time applications but uses broadcast datagrams that may be lost
* [Virtual-Circuit](https://docs.sparkfun.com/SparkFun_LoRaSerial/operating_modes/#virtual-circuits) supports multipoint with guaranteed delivery or the link breaks. This mode uses a special serial interface.

Enter [command mode](http://docs.sparkfun.com/SparkFun_LoRaSerial/at_commands/) to change modes and adjust the parameters for that mode of operation. [Training](http://docs.sparkfun.com/SparkFun_LoRaSerial/training/) is the process to distribute the set of parameters from a server radio (server=1) to the other client radios (server=0). Training can be done one radio at a time or multiple radios at once.

The green LEDs by default display a received signal strength indication. The more LEDs that are on the better the signal. However other [LED patterns](http://docs.sparkfun.com/SparkFun_LoRaSerial/led_states/) are available to provide more data on the radio behavior.

Occasionally, SparkFun may release new firmware for LoRaSerial. The
[firmware update](http://docs.sparkfun.com/SparkFun_LoRaSerial/firmware_update/) procedure enables you to load the firmware into the LoRaSerial radio. Advanced users may also want to [build](http://docs.sparkfun.com/SparkFun_LoRaSerial/firmware_build/) the [open source LoRaSerial firmware](https://github.com/sparkfun/SparkFun_LoRaSerial).

### Hardware Documentation

The following hardware documents are available:

* [Schematic](https://cdn.sparkfun.com/assets/9/3/0/6/e/SparkFun_LoRaSerial_915MHz_-_1W.pdf)
* [Eagle Files](https://cdn.sparkfun.com/assets/d/e/c/e/5/SparkFun_LoRaSerial_915MHz_-_1W.zip)
* [Semtech SX1276 datasheet](https://cdn.sparkfun.com/assets/7/7/3/2/2/SX1276_Datasheet.pdf)
* [Ebyte E19-915M30S Datasheet](https://cdn.sparkfun.com/assets/6/3/e/e/3/E19-915M30S_Usermanual_EN_v1.20.pdf)
* [Atmel SAMD-21 datasheet](https://cdn.sparkfun.com/datasheets/Dev/Arduino/Boards/Atmel-42181-SAM-D21_Datasheet.pdf)
* [Understanding LoRa PHY](https://wirelesspi.com/understanding-lora-phy-long-range-physical-layer/) - A good article on the math behind Semtec's Long Range modulation technique
