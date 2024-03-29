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
* AT-EncryptionKey=? - Choose a unique encryption key for the network
* AT-SelectLedUse=2 - The use of LEDS_VC (2) flashes the blue LED when server HEARTBEAT frames are received by the clients, providing a visual indication that the radio is synchronizing with the server
* AT-Server=1 - Define this radio as the virtual circuit server
* ATW - Write the parameters to the non-volatile memory (NVM).

Now enter training mode with the ATT command.  The training enables the server to pass these parameters (except for server) to the client radios.  On the client radios, with power applied, press the button at the top of the client radio to enter training mode.  The radio will obtain the parameters from the server, write them to NVM and then reboot using the new parameters.

After all of the client radios are trained, the ATZ command may be entered on the server radio to cause it to reboot.

#### Example Virtual Circuit Initialization Script for Training

    +++
    AT=OperatingMode=2
    AT-NetID=3
    AT-VerifyNetID=1
    AT-EnableCRC16=1
    AT-EncryptionKey=54637374546373745463737454637374
    AT-SelectLedUse=2
    AT-Server=1
    ATW
    ATT

## Virtual Circuit Communications

Communication is possible between the server and client or between clients. A three-way handshake must be performed prior to normal data communications to synchronize the ACK numbers.  By default, the three-way handshake is not performed.  The user or application initiates the handshake using the AT-CmdVc=n to select the client or server radio (n) for communications.  Next the ATC command initiates the three-way handshake between the local radio and the remote radio (n).

After the three-way handshake communications between the two radios is possible in virtual circuit mode.

## Virtual Circuit Number

The virtual circuit number of the local radio is obtained by issuing the ATI11 command to the local radio.  The response returns a number in the range of zero (0) to MAX_VC.  The value VC_UNASSIGNED is returned between reset until the server assigns a virtual circuit number to the local radio.

## Virtual Circuit Serial Interface

In virtual circuit mode all communications over the serial port must be proceeded by a VC_SERIAL_MESSAGE_HEADER.  Data not proceeded by a VC_SERIAL_MESSAGE_HEADER data structure is discarded!  The VC_SERIAL_MESSAGE_HEADER is defined in the [Virtual_Circuit_Protocol.h](https://github.com/sparkfun/SparkFun_LoRaSerial/blob/release_candidate/Firmware/LoRaSerial_Firmware/Virtual_Circuit_Protocol.h) header file.  This data structure is four (4) bytes long starting with the value START_OF_VC_SERIAL (2).  The next byte specifies the length of the binary data following the VC_SERIAL_MESSAGE_HEADER plus the size of VC_RADIO_MESSAGE_HEADER (3).  The other two bytes are virtual circuit numbers for the destination and source virtual circuits.

The server radio is identified by VC_SERVER (0) and client radios have VC numbers between 1 and MAX_VC.  A data message sent from the server to client 3 would have the source VC set to VC_SERVER (0) and the destination VC set to 3.  Example:

    START_OF_VC_SERIAL
    +-----+-----+-----+-----+-----+-----+-----+-----+-----+
    |  2  |  8  |  3  |  0  |  H  |  e  |  l  |  l  |  o  |
    +-----+-----+-----+-----+-----+-----+-----+-----+-----+
          Length  Dest  Src   Data

### Radio Responses

The LoRaSerial radio will return the following responses:

* Data response
* Local command response
* Remote command response
* VC state response
* Reconnect serial response

#### Data Response

When sending a data message, the radio returns a VC_DATA_ACK_NACK_MESSAGE.  The response is sent to PC_DATA_ACK (0xe2) when the data is acknowledged by the remote radio.  A response of PC_DATA_NACK (0xe3) is returned when the link is broken.

    START_OF_VC_SERIAL
    +-----+-----+-----+-----+-----+-----+-----+-----+-----+
    |  2  |  8  |  B  |  A  |  H  |  e  |  l  |  l  |  o  |
    +-----+-----+-----+-----+-----+-----+-----+-----+-----+
          Length  Dest  Src   Data

    Host A              Radio A             Radio B             Host B
    Data Message  --->
                        Data Message  --->
                                      <---  ACK
                                            Data Message  --->
                  <---  VC_DATA_ACK_NACK_MESSAGE

    START_OF_VC_SERIAL
    +-----+-----+--------+-----+
    |  2  |  3  |  0xe2  |  B  |
    +-----+-----+--------+-----+
          Length   Dest    Src

#### Local Command Response
When sending a local command, the radio responds with VC_COMMAND_COMPLETE_MESSAGE sent to PC_COMMAND_COMPLETE (0xe5) with the status VC_CMD_SUCCESS (0) or VC_CMD_ERROR (1).

One pair of virtual circuit numbers allows the local host to communicate with the command interface on the local radio.  The data portion of the message contains the command to be executed.

* VC_COMMAND (0xfe): Destination VC used by the host computer to send a command to the local radio to be executed immediately

* PC_COMMAND (0xe0): Source VC for the local command message


        START_OF_VC_SERIAL
        +-----+-----+------+------+-----+-----+-----+-----+-----+------+------+
        |  2  |  8  | 0xe5 | 0xe0 |  A  |  T  |  I  |  1  |  1  | 0x0d | 0x0a |
        +-----+-----+------+------+-----+-----+-----+-----+-----+------+------+
              Length  Dest   Src    Command

        Host A                 Radio A
        Command Message  --->
                         <---  Command response (ASCII text)
        +------+------+---+---+---+---+---+---+...+------+------+---+---+------+------+
        | 0x0d | 0x0a | m | y | V | c | : |   | A | 0x0d | 0x0a | O | K | 0x0d | 0x0a |
        +------+------+---+---+---+---+---+---+...+...+--+------+---+---+------+------+
                         <---  VC_COMMAND_COMPLETE_MESSAGE

        START_OF_VC_SERIAL
        +-----+-----+--------+-----+-----+
        |  2  |  4  |  0xe2  |  A  |  0  |
        +-----+-----+--------+-----+-----+
              Length   Dest    Src  VC_CMD_SUCCESS

#### Remote Command Response

A command sent to a remote radio is acknowledged with a VC_DATA_ACK_NACK_MESSAGE sent to the PC_DATA_ACK (0xe2) or PC_DATA_NACK (0xe3) destination port.  The actual command response is sent to the local radio's VC ored with PC_REMOTE_RESPONSE (0xc0).  After the command response text is delivered, the command status is returned to the destination VC of PC_COMMAND_COMPLETE (0xe5) with the status of VC_CMD_SUCCESS (0) or VC_CMD_ERROR (1).

The VC number range from 32 to 63 is reserved for remote command execution. The VC number equals the target radio number (0 - 31) or-ed with PC_REMOTE_COMMAND (0x20). This VC number is placed in the destination VC field and the local radio VC number is placed in the source VC field.

The following example sends an ATI11 command from radio A to radio B to get radio
B's VC number:

    START_OF_VC_SERIAL
    +-----+------+----------+-----+-----+-----+-----+-----+-----+------+------+
    |  2  |  10  | 0x20 + B |  A  |  A  |  T  |  I  |  1  |  1  | 0x0d | 0x0a |
    +-----+------+----------+-----+-----+-----+-----+-----+-----+------+------+
          Length    Dest     Src   Command

    Host A                 Radio A               Radio B
    Command Message  --->
                           Remote Command  --->
                                           <---  ACK
                     <---  VC_DATA_ACK_NACK_MESSAGE

    START_OF_VC_SERIAL
    +-----+-----+--------+-----+
    |  2  |  3  |  0xe2  |  B  |
    +-----+-----+--------+-----+
          Length   Dest    Src

                                           <---  Remote Command Response
                     <---  Remove Command Response
    +---+----+----------+---+------+------+---+---+---+---+---+---+...+------+------+---+---+------+------+
    | 2 | 18 | 0xc0 + A | B | 0x0d | 0x0a | m | y | V | c | : |   | B | 0x0d | 0x0a | O | K | 0x0d | 0x0a |
    +---+----+----------+---+------+------+---+---+---+---+---+---+...+...+--+------+---+---+------+------+

                                           <---  VC_COMMAND_COMPLETE_MESSAGE
                     <---  VC_COMMAND_COMPLETE_MESSAGE

    START_OF_VC_SERIAL
    +-----+-----+--------+-----+-----+
    |  2  |  4  |  0xe5  |  B  |  0  |
    +-----+-----+--------+-----+-----+
          Length   Dest    Src  VC_CMD_SUCCESS

#### VC State Response

As VC links change states, the host is notified of the state changes with a VC_STATE_MESSAGE sent to PC_LINK_STATUS (0xe1).  An example where radio A detects the initial HEARTBEAT frame from radio B and notifies host A of the VC link change:

    Host A        Radio A        Radio B

                           <---  HEARTBEAT
            <---  VC_STATE_MESSAGE

    START_OF_VC_SERIAL
    +-----+-----+--------+-----+----------+
    |  2  |  4  |  0xe1  |  B  |  Status  |
    +-----+-----+--------+-----+----------+
          Length   Dest    Src  VC_CMD_SUCCESS

The status values are:

* VC_STATE_LINK_DOWN (0): HEARTBEATs not received
* VC_STATE_LINK_ALIVE (1): Receiving HEARTBEATs, waiting for UNKNOWN_ACKS
* VC_STATE_SEND_UNKNOWN_ACKS (2): ATC command received, sending UNKNOWN_ACKS
* VC_STATE_WAIT_SYNC_ACKS (3): UNKNOWN_ACKS sent, waiting for SYNC_ACKS
* VC_STATE_WAIT_ZERO_ACKS (4): SYNC_ACKS sent, waiting for ZERO_ACKS
* VC_STATE_CONNECTED (5): ZERO_ACKS received, ACKs cleared, ready to send data

#### Reconnect Serial Response

The ATZ command causes the system to reboot.  Prior to the reboot, the radio responds with a message sent to PC_SERIAL_RECONNECT (0xe4).  The local radio takes a couple of seconds to reset and causes the USB serial device to go off-line.  The timing is critical here, the application must close the serial connection before the LoRaSerial USB device goes off-line.  If the host is still holding the USB serial connection open when the LoRaSerial USB serial port goes off-line then the host may not be able to release it in time before the LoRaSerial attempts to connect its USB serial port.  In this case, the LoRaSerial serial port may show up as a new device on the host system and
further communications with the radio would no longer be possible using the previous device path.

An example is below:

    START_OF_VC_SERIAL
    +-----+-----+------+------+-----+-----+-----+------+------+
    |  2  |  8  | 0xe5 | 0xe0 |  A  |  T  |  Z  | 0x0d | 0x0a |
    +-----+-----+------+------+-----+-----+-----+------+------+
           Length  Dest   Src  Command

        Host A                 Radio A
        Command Message  --->
                         <---  Command response (ASCII text)

        +---+---+------+------+
        | O | K | 0x0d | 0x0a |
        +---+---+------+------+

                         <---  Reconnect message

        START_OF_VC_SERIAL
        +-----+-----+--------+-----+
        |  2  |  3  |  0xe4  |  A  |
        +-----+-----+--------+-----+
              Length   Dest    Src

                         <---  VC_COMMAND_COMPLETE_MESSAGE

        START_OF_VC_SERIAL
        +-----+-----+--------+-----+-----+
        |  2  |  4  |  0xe2  |  A  |  0  |
        +-----+-----+--------+-----+-----+
              Length   Dest    Src  VC_CMD_SUCCESS

## Example Program

The [VcServerTest.c](https://github.com/sparkfun/SparkFun_LoRaSerial/blob/release_candidate/Firmware/Tools/VcServerTest.c) is an example C program that communicates with the VC server using the virtual circuit serial interface.  The first parameter is the device path to the local LoRaSerial radio.  The second parameter specifies the destination VC number to send commands or data.

The example program has the following features:

* <code>--reset</code> command line option
* <code>--break</code> command line option
* Gets the local radio's virtual circuit number
* Opens a virtual circuit to the target VC
* Passes entered serial data to the target VC
* Displays responses from the target VC

### Debug Defines

There are some useful defines that may be set to one (1) to display the host's interaction with the radio.  These defines are:

* DEBUG_PC_TO_RADIO
* DEBUG_RADIO_TO_PC
* DUMP_RADIO_TO_PC

Setting these defines will display the radio communications in hexadecimal and ASCII.

### <code>--reset</code>

The reset command line option is not used very often, but sends an ATZ command to the local radio.

### <code>--break</code>

The break command line option sends an ATB command to the local radio causing it to delay for 5 times the heartbeatTimeout interval.  This delay is sufficient to break all links with any remote virtual circuits.  Following the delay, the radio returns to the RADIO_RESET state and brings up links to the other radios.
