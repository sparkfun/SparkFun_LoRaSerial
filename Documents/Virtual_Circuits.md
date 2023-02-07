Virtual Circuit Mode
====================

Multiple radios can communicate to each other in Virtual Circuit mode.  This mode provides guaranteed message delivery or the link breaks.  Virtual circuit mode supports:
* Up to 32 radios
* Virtual circuit serial interface
* Local command support
* Remote command support
* [Header file](https://github.com/sparkfun/SparkFun_LoRaSerial/blob/release_candidate/Firmware/LoRaSerial_Firmware/Virtual_Circuit_Protocol.h) for C and C++

## Virtual Circuit Network Configuration

A LoRaSerial virtual circuit network consists of a single server radio and up to 31 client radios.  The server transmits HEARTBEAT frames that provide the synchronization signal for the channel hop timer.  Each time the timer fires, the radio switches to a new center frequency (channel).

Clients wait to receive a HEARTBEAT frame from the server on channel 0.  If the selectLedUse is set to LED_VC (2) then the blue LED will flash upon when the server's HEARTBEAT frame is received.  After receiving the server's HEARTBEAT frame the client synchronizes its hop timer and starts hopping channels.  If the selectLedUse is set to LED_VC (2) then each hop is indicated by a flash of the yellow LED.

The virtual circuit server assigns virtual circuit ID numbers to all of the client radios.  This may be done during radio training or during the first time the radio is connected to the network.  After receiving a HEARTBEAT frame from the server, the client sends a HEARTBEAT frame with the source VC address of VC_UNASSIGNED and the radio's 16-byte unique ID.  The server echos this HEARTBEAT replacing the source VC with the assigned VC number.  While the client is using VC_UNASSIGNED as it's VC number, the client radio compares the unique ID in the HEARTBEAT frame to the local radio's unique ID value.  If the values match then the client replaces its VC number with the assigned VC number.

For consistent processing, the server records the client unique ID values in the non-volatile memory.  These values are read and loaded into the virtual circuit table when the system boots again.

### Radio Parameters

Select a LoRaSerial radio for the virtual circuit server.  Set the following parameters on the server radio.

The following parameters should be set to enable virtual circuit mode:
* AT=OperatingMode=2 - Select virtual circuit mode
* AT-NetID=n - Choose a unique value (n) for your network
* AT-VerifyNetID=1 - Eliminate communications from other radios which are not using the NetID value
* AT-EnableCRC16=? - Enable software CRC-16 by setting the value to 1, otherwise set the value to zero to disable software CRC
* AT-SelectLedUse=2 - The use of LEDS_VC (2) flashes the blue LED when server HEARTBEAT frames are received by the clients, providing a visual indication that the radio is synchronizing with the server
* AT-Server=1 - Define this radio as the virtual circuit server
* ATW - Write the parameters to the non-volatile memory (NVM).

Now enter training mode with the ATT command.  The training enables the server to pass these parameters (except for server) to the client radios.  On the client radios, with power applied, press the button at the top of the client radio to enter training mode.  The radio will obtain the parameters from the server, write them to NVM and then reboot using the new parameters.

After all of the client radios are trained, the ATZ command may be entered on the server radio to cause it to reboot.

## Virtual Circuit Communications

Communication is possible between the server and client or between clients. A three-way handshake must be performed prior to normal data communications to synchronize the ACK numbers.  By default, the three-way handshake is not performed.  The user or application initiates the handshake using the AT-CmdVc=n to select the client or server radio (n) for communications.  Next the ATC command initiates the three-way handshake between the local radio and the remote radio (n).

After the three-way handshake communications between the two radios is possible in virtual circuit mode.

## Virtual Circuit Number

The virtual circuit number of the local radio is obtained by issuing the ATI11 command to the local radio.  The response returns a number in the range of zero (0) to MAX_VC.  The value VC_UNASSIGNED is returned between reset until the server assigns a virtual circuit number to the local radio.

## Virtual Circuit Serial Interface

In virtual circuit mode all communications over the serial port must be proceeded by a VC_SERIAL_MESSAGE_HEADER.  Data not proceeded by a VC_SERIAL_MESSAGE_HEADER data structure is discarded!  The VC_SERIAL_MESSAGE_HEADER is defined in the [Virtual_Circuit_Protocol.h](https://github.com/sparkfun/SparkFun_LoRaSerial/blob/release_candidate/Firmware/LoRaSerial_Firmware/Virtual_Circuit_Protocol.h) header file.  This data structure is four (4) bytes long starting with the value START_OF_VC_SERIAL (2).  The next byte specifies the length of the binary data following the VC_SERIAL_MESSAGE_HEADER plus the size of VC_RADIO_MESSAGE_HEADER (3).  The other two bytes are virtual circuit numbers for the destination and source virtual circuits.

The server radio is identified by VC_SERVER (0) and client radios have VC numbers between 1 and MAX_VC.  A data message sent from the server to client 2 would have the source VC set to VC_SERVER (0) and the destination VC set to 2.

### Radio Responses

The LoRaSerial radio will return the following responses:

* Data - Returns VC_DATA_ACK_NACK_MESSAGE.  The response is sent to PC_DATA_ACK when the data is acknowledged by the remote radio.  A response of PC_DATA_NACKis returned when the link is broken.
* Local Command - A response is sent to PC_COMMAND_COMPLETE upon command completion with the status VC_CMD_SUCCESS or VC_CMD_ERROR.
* Remote Command - The command is acknowledged with a VC_DATA_ACK_NACK_MESSAGE sent to the PC_DATA_ACK or PC_DATA_NACK destination port.  The actual command response is sent to the local radio's VC ored with PC_REMOTE_RESPONSE.  After the command response text is delivered, the command status is returned to the destination VC of PC_COMMAND_COMPLETE with the status of VC_CMD_SUCCESS or VC_CMD_ERROR.

## Local Command Support

One pair of virtual circuit numbers allows the local host to communicate with the command interface on the local radio.  The data portion of the message contains the command to be executed.
* VC_COMMAND: Destination VC used by the host computer to send a command to the local radio to be executed immediately
* PC_COMMAND: Source VC for the local command message

## Remote Command Support

The VC number range from 32 to 63 is reserved for remote command execution. The VC number equals the target radio number (0 - 31) or-ed with PC_REMOTE_COMMAND. This VC number is placed in the destination VC field and the local radio VC number is placed in the source VC field.
