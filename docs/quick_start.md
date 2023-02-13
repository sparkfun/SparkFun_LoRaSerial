# Quick Start Guide

LoRaSerial radios are very easy to use and ship ready to connect out of the box. Simply attach the antennas, plug two units into USB ports, open a terminal at each of the USB serial ports, and start passing data back and forth.

![Passing text between two terminals](Original/SparkFun%20LoRaSerial%20-%20P2P%20Serial%20Terminals.gif)

*Passing simple text data between two radios*

The default settings are as follows:

* **Mode:** Point to Point
* **Frequency:** 902 to 928MHz
* **Channels:** 50
* **Frequency Hopping:** Enabled
* **Broadcast Power:** 30dbm
* **Airspeed:** 4800bps
* **Serial:** 57600bps
* **Encryption:** Enabled
* **Encryption Key:** Default<sup>1</sup>
* **Software CRC:** Enabled

Serial can be passed into the unit either through the USB port, or over the serial connector. If the serial connector is used, the baud rate is 57600bps by default.

<sup>1</sup> The radios default to a published encryption key. For maximum security, we recommend changing the key using the training process.

LoRaSerial is designed to get small amounts of data from point A to point B as easily as possible. If you've got a sensor, or Arduino, or any device that can output serial, then it can transmit data over long distances. LoRaSerial has been used to send data over 9 miles (14km) line-of-sight.