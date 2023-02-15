# LED States

A variety of LEDs are visible on the LoRaSerial radio to give the user feedback about the state of the system. 

![LEDs on the LoRaSerial](img/SparkFun_LoRaSerial_LEDs.png)

*LEDs on the LoRaSerial*

## LED Mode 1

In the default mode (selectedLedUse = 1):

* The RSSI LEDs indicate the signal strength of the last received packet. 
* The blue TX LED indicates when serial data is received over the radio and sent to the host, either over USB or hardware serial.
* The yellow RX LED indicates when serial data is received from the host and sent over the radio.

## LED Mode 0

For users who want additional LED feedback about the RF link, setting selectedLedUse to 0 has the following effects:

* The blue LED will blink any time a heartbeat is received.
* The yellow LED will blink when a channel hop occurs.
* The 1st RSSI LED from the top indicates data is being transmitted.
* The 2nd RSSI LED from the top indicates the last received RSSI.
* The 3rd RSSI LED from the top turns on when the link is up (data can pass).
* The 4th RSSI LED from the top indicates data being successfully received.