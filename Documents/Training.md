LoRaSerial Training
===================

# Goals

There are two goals for LoRaSerial training:

1 Establish a common set of communication parameters between a set of LoRaSerial radios.
2 Make it easy to test an initial pair of LoRaSerial radios through the use of the training button.

# Methods of Training

There are two methods of initiating training:

* Pressing the training button on the top of the LoRaSerial module
* Entering the ATT command

Training is performed by a server radio sending parameters to the client radio.  Upon reception the client radio saves the parameters in non-volatile memory and then reboots with the new parameters.

## Training Button Behavior

Pressing and releasing the training button for different lengths of time initiates the following behaviors:

* 2 Seconds:
    * Enter training as a client, cylon pattern appears on the green LEDs
    * Exit training (client or server) without saving parameter changes, cylon pattern stops and previous LED pattern returns
* 5 Seconds - Enter training as a server, yellow LED flashes 3 times when 5 seconds of press time is detected, cylon pattern appears on the green LEDs when the button is released
* 15 Seconds - Reset LoRaSerial to factory settings and erase NVM, blue LED flashes 3 times when 15 seconds of press time is detected

# Temporary Training Server

It is possible to use temporary training servers for multipoint and virtual circuit modes.  The only difference with a temporary server and a regular server is that the parameters are not saved to the non-volatile storage using the ATW command before exiting command mode.

# Point-To-Point Training

To get a known set of parameters on the client radio, use command mode to initialize the parameters on the server.  Using the training buttons on both radios only changes
the network ID and encryption key values, it does not change any of the other parameters such as AirSpeed.

Point-to-Point Training Examples:

Server:
* Enter command mode using +++
* Optional, start from factory defaults by issuing the ATF command
* Set the desired parameters
* Set Server=1
* Enter training mode using ATT command
* After the client is trained
* Saves new parameters for netID and encryptionKey
* Reboots

Server (no clients):
* Enter command mode with +++
* Set any of the desired parameters
* Enter training mode with ATT command
* No client to be trained
* Exit command mode with ATO or ATZ

Assuming the radios are already able to communicate, use the training button to select a random NetID and EncryptionKey value.

Server:
* 2 second training button press
* Does not receive training from a server
* After 3 seconds, generates new netID and encryption key and automatically switches to server mode
* Wait for client to be trained
* Saves new parameters for netID and encryptionKey
* Reboots

Server:
* 5 second training button press
* Generates new netID and encryption key
* Wait for client to be trained
* Saves new parameters for netID and encryptionKey
* Reboots

Client:
* 2 second training button press
* Receives training parameters from server
* Saves new parameters
* Reboots

Client:
* Enter command mode using +++
* Enter training using the ATT command
* Receives training parameters from server
* Saves new parameters
* Reboots

Point-To-Point Timeout Examples:

Client:
* 2 second training button press
* Before three seconds
* 2 second training button press, exits training without receiving any new parameters

Server:
* 2 second training button press
* Does not receive training from a server
* After 3 seconds, generates new netID and encryption key and automatically switches to server mode
* 2 second training button press, exits training and does not save new netID or encryption key

Server:
* 5 second training button press
* Generates new netID and encryption key
* 2 second training button press, exits training and does not save new netID or encryption key

# Multi-Point Training

It is recommend to use a command script to initialize the server radio when performing multipoint traning.  The client radios can either use command mode or the training button to enter training mode.

Temporary Server (Doesn't save settings):
* If not already at factory reset, hold the training button down for 15 seconds
* Connect to the LoRaSerial radio via USB or the serial port
* Enter command mode with +++
* Start from factory defaults by issuing the ATF command
* Issue the following commands:
    * AT-OperatingMode=0
    * AT-Server=1
    * AT-SelectLedUse=1
    * ATG
* Set any of the other parameters
* Enter training mode with ATT command
* Wait for clients to be trained
* Exit command mode with ATO command or reboot with ATZ command

Server (Saves settings):
* If not already at factory reset, hold the training button down for 15 seconds
* Connect to the LoRaSerial radio via USB or the serial port
* Enter command mode with +++
* Start from factory defaults by issuing the ATF command
* Issue the following commands:
    * AT-OperatingMode=0
    * AT-Server=1
    * AT-SelectLedUse=1
    * ATG
* Set any of the other parameters
* Enter training mode with ATT command
* Wait for clients to be trained
* Save parameters with ATW command
* Always reboot with ATZ command

Client:
* 2 second training button press
* Receives training parameters from server
* Saves new parameters
* Reboots

Client:
* Connect to the LoRaSerial radio via USB or the serial port
* Enter command mode using +++
* Enter training using the ATT command
* Receives training parameters from server
* Saves new parameters
* Reboots

# Virtual Circuit Training

It is recommend to use a command script to initialize the server radio when performing virtual-circuit traning.  The client radios can either use command mode or the training button to enter training mode.

Temporary Server (Doesn't save settings):
* If not already at factory reset, hold the training button down for 15 seconds
* Connect to the LoRaSerial radio via USB or the serial port
* Enter command mode with +++
* Start from factory defaults by issuing the ATF command
* Issue the following commands:
    * AT-OperatingMode=2
    * AT-Server=1
    * AT-SelectLedUse=2
    * ATG
* Set any of the other parameters
* Enter training mode with ATT command
* Wait for clients to be trained
* Exit command mode with ATO command or reboot with ATZ command

Server (Saves settings):
* If not already at factory reset, hold the training button down for 15 seconds
* Connect to the LoRaSerial radio via USB or the serial port
* Enter command mode with +++
* Start from factory defaults by issuing the ATF command
* Issue the following commands:
    * AT-OperatingMode=2
    * AT-Server=1
    * AT-SelectLedUse=2
    * ATG
* Set any of the other parameters
* Enter training mode with ATT command
* Wait for clients to be trained
* Save parameters with ATW command
* Always reboot with ATZ command

Client:
* 2 second training button press
* Receives training parameters from server
* Saves new parameters
* Reboots

Client:
* Connect to the LoRaSerial radio via USB or the serial port
* Enter command mode using +++
* Enter training using the ATT command
* Receives training parameters from server
* Saves new parameters
* Reboots

# Training Parameters

The training parameters for radio communication fall into two groups:

* Radio parameters
* Radio protocol parameters

Other optional sets of training parameters may be communicated between the radios when the corresponding copyXxxx parameter is set to true (1).  The optional parameter sets are:

* Serial parameters
* Debug parameters
* Trigger parameters

The following sections describe the communication parameters.

## Radio Parameters

The following parameters describe the radio transmit operation:

* AutoTuneFrequency - Adjust the frequency based upon the last received frame's frequency error.

* FrequencyHop - Enable or disable frequency hopping.  Legal operation in the United States requires that this value be set to true (1) which is the default.  This parameter specifies whether the radio will change frequencies to a new channel every MaxDwellTime milliseconds.

* FrequencyMax - Maximum frequency the radio will use specified in MegaHertz.  The default value is 928.0 MHz.

* FrequencyMin - Minimum frequency the radio will use specified in MegaHertz.  The default value is 902.0 MHz.

* MaxDwellTime - Number of milliseconds that the LoRaSerial radio uses a specific frequency (channel).  After this time, the radio switches (hops) to another frequency.  For operation within the United States this value must not be greater than 400 milliseconds which is the default.

* NumberOfChannels - Specify the number of unique frequencies that will be used by the LoRaSerial radio.  For operation within the United States this value needs to be a minimum of 50 which is the default value.

* RadioBroadcastPower_dbm - Transmit power level for the LoRaSerial radio.  The valid range for this parameter is from 14 (25 milliWatts) to 30 dBm (1 Watt).  The default value is 30 dBm.

### Frame Synchronization Parameters

The following parameters specify data that the radios use for data synchronization and frame reception:

* RadioPreambleLength - Number of symbols to transmit at the beginning of the frame to notify the receiving radios of the incoming frame.  The range of values are 6 to 65535 with the default being 8.  The preamble is used to synchronize the PLLs in the radio to properly detect the symbol boundaries.  NOTE: Different lengths does *not* guarantee a remote radio privacy. 8 to 11 works. 8 to 15 drops some. 8 to 20 is silent.

* RadioSyncWord - The sync word following the preamble indicates the destination set of radios.  Note that different sync words do *not* guarantee a remote radio will not receive a rogue frame.

### AirSpeed

The parameter AirSpeed is a simplification that selects values for the following parameters to approximate the baud rate implied by the AirSpeed.

* HeartbeatTimeout
* RadioBandwidth
* RadioCodingRate
* RadioSpreadFactor
* TxToRxUsec

Valid values for AirSpeed are (40, 150, 400, 1200, 2400, 4800, 9600, 19200, 28800 and 38400) and the default is 4800.  After AirSpeed is set, it is possible to modify any of the parameters above.  Note that AirSpeed is just an easy way to set the parameters above to known value.  AirSpeed is not an actual parameter that is used for training, only the parameters listed above are used during training.  More detail for these parameters is provided below:

* HeartbeatTimeout - Heartbeats are transmitted on a regular basis by the server and in point-to-point and virtual circuit modes by the clients.  This parameter specifies the time in milliseconds during which a HEARTBEAT frame should be transmitted.  If a HEARTBEAT frame is not received within three (3) times this interval then the point-to-point or virtual circuit link is broken.  The default heartbeatTimeout is 5000 milliseconds (5 seconds).

* RadioBandwidth - Bandwidth of the spread spectrum signal specified in kiloHertz.  The default is 500.0 kHz and other supported values are: 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125 and 250 kHz.

* RadioCodingRate - Number of bits to send 4 data bits of information.  This is used for forward error correction, allowing the receiver to correct the incoming bit stream.  Valid values are 5 through 8 where higher coding rates ensure less frames are dropped.  The default value is 8.

* RadioSpreadFactor - Number of bits in a symbol.  Valid values are 6 to 12 with the default value of 9.  Larger values enable longer range while reducing the data rate.

* TxToRxUsec - Transition time from end of transmission to the end of reception on the remote radio specified in microseconds.

## Radio protocol parameters

The following parameters describe the contents of the data frame sent between the LoRaSerial radios:

* DataScrambling - Enable (1) or disable (0) the use of IBM's Data Whitening algorithm to reduce DC bias

* EnableCRC16 - Enable (1) or disable (0) adding a software generated CRC-16 to the end of the data frame.

* EncryptData - Enable (1) or disable (0) the use of AES encryption for each data frame

* EncryptionKey - The 16 byte encryption key specified in hexadecimal.  While SparkFun provides a default value, it is strongly recomended to change this value for your own networks.

* FramesToYield - When requested by the remote radio, suppress transmission for this number of max packet frames.  The valid range is 0 to 255 with a default value of 3.

* MaxResends - Number of retransmission attempt to make for a each frame when an ACK is not received from the destination radio.  The valid range is 0 to 255.  The default of zero represents infinite and retries will continue as long as the two radios are receiving HEARTBEAT frames from each other.  The transmission will fail only when the link breaks due to not receiving HEARTBEAT frames.

* NetID - A unique value denoting the network of LoRaSerial radios.  All radios in the network must share the same ID value.  This value is in addition to the radioSyncWord.

* OperatingMode - The following modes of operation are supported by the LoRaSerial radios:

    * MODE_MULTIPOINT (0) - A single server with multiple clients, all radios are sending datagram frames which are not guarranteed to be received by the other radios.  This mode is great when real-time transmission is necessary and the application is able to tolerate some loss of data.

    * MODE_POINT_TO_POINT (1, default) - Communications between only two LoRaSerial radios with guarranteed delivery of frames or the link breaks.

    * MODE_VIRTUAL_CIRCUIT - A single server with multiple clients that supports multipoint communications with guarranteed delivery or the link breaks.  This mode uses a special protocol over the serial link to be able to specify the destination radio for transmission and the receive radio for reception.  More information is available [here](https://github.com/sparkfun/SparkFun_LoRaSerial/blob/release_candidate/Documents/Virtual_Circuits.md).

* OverheadTime - The number of milliseconds to add to ACK and datagram times before ACK timeout occurs.  The default is 10 milliseconds.

* SelectLedUse - Select how to display information on the LEDs

* Server - Enable (1) or disable (0) the server mode for training and for multipoint or virtual circuit operation.  The default is client mode (0).

* VerifyRxNetID - Enable (1) or disable (0) the verification of the netID value during reception.  The default is enabled (1).
