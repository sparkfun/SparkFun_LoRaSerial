LoRaSerial Documents and Airspeed Calculations
========================================

[![SparkFun LoRaSerial Airspeed Spreadsheet](LoRaSerial%20Airspeed%20Spreadsheet.jpg)](https://docs.google.com/spreadsheets/d/1qyJa3ldE-KDUHwHNSctBMccPTVRwKbmAsodkdvOc3-8/edit?usp=sharing)

The available airspeeds (40 to 38400) used in LoRaSerial were picked to, as closely as possible, mimic modem baud rates. Other spread factors, bandwidths, and coding rates can be used if desired.

[This spreadsheet](https://docs.google.com/spreadsheets/d/1qyJa3ldE-KDUHwHNSctBMccPTVRwKbmAsodkdvOc3-8/edit?usp=sharing) can be used to inspect the airtime formulas.

Additionally, this folder contains the various datasheets for the SX1276 IC, common to all LoRaSerial modules, as well as the individual datasheets for the different module types (915, 868, 433, etc).

[Atmel SAM D21G Datasheet](https://cdn.sparkfun.com/datasheets/Dev/Arduino/Boards/Atmel-42181-SAM-D21_Datasheet.pdf)

## Modes of Operation

The LoRaSerial radio operates in one of three modes:
* Point-to-Point (default)
* Multipoint
* Virtual Circuit

Point-to-Point mode provides guaranteed message delivery or the link breaks.  The radio performs data retransmission if either the data frame was lost or its acknowledgement was lost.  This can continue indefinitely if MaxResends equals zero (default) or for a limited number of retries in the range of (1 - 255).

Multipoint mode provides a datagram service.  The LoRaSerial radios will send the data frame without a guarantee that the frame will be received by the remote radio. Lost frames are lost, the radio does no perform retransmission.  If the application is not able to tolerate the lost frames then another protocol layer needs to be implemented on the host computer between the radio and the application that provides the necessary services to the application.

Virtual circuit mode enables a group of radios to communicate with each other. The radio links provide guaranteed message delivery or the link is broken. One radio in the group is designated as the server and provides the channel timer synchronization for the client radios, think of a star configuration with the server at the center.  Data communications with the virtual circuit mode is all point-to-point.  Communications between the radio and the host CPU use a special virtual circuit header to identify where to send the host to radio data, or where to deliver the radio to host data.  More information is available [here](https://github.com/sparkfun/SparkFun_LoRaSerial/blob/release_candidate/Documents/Virtual_Circuits.md).
