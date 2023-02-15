# AT Commands

LoRaSerial radios are very flexible. By default, the radio is looking for serial communication at 57600bps. Open the terminal of your choice and enter +++ and wait for an OK. The radio is now ready for an AT command.

Below is a brief list of commands:

| AT Command | Command Description                      |
|------------|------------------------------------------|
| +++        | Enter command mode                       |
| AT         | Reports OK                               |
| ATI        | Show board variant, firmware             |
| ATI0       | Show all user settable parameters        |
| ATI1       | Show board variant                       |
| ATI2       | Show firmware version                    |
| ATI3       | Display latest RSSI                      |
| ATI4       | Generate random byte based on RSSI       |
| ATI5       | Show max possible bytes per second       |
| ATI6       | Show AES Key Values                      |
| ATI7       | Show FHSS Channel                        |
| RTI?       | Show equivalent remote setting           |
|            |                                          |
| ATO        | Exit command mode                        |
| ATZ        | Reboot the radio                         |
| AT&W       | Write current parameters to EEPROM       |
| AT&F       | Reset all parameters to factory defaults |
| AT&T=RSSI  | Show RX packet RSSI, SNR, and FreqError  |
| ATT        | Enter Training Mode                      |
| ATF        | Enter Training Mode with Defaults        |
|            |                                          |
| ATSn?      | Display a parameter                      |
| ATSn=X     | Set a parameter                          |
|            |                                          |
| RTSn?      | Display remote parameter                 |
| RTSn=X     | Set remote parameter                     |

*Table of AT Commands*

For a PDF of the AT commands, click [here](https://cdn.sparkfun.com/assets/learn_tutorials/2/6/1/7/LoRaSerial_Radio_Registers_-_AT_Commands.pdf).

The main user settable parameters are listed below and available [here](https://cdn.sparkfun.com/assets/learn_tutorials/2/6/1/7/LoRaSerial_Radio_Registers_-_Parameters.pdf) in PDF format. A parameter is set using the ATSx command. For example sending ‘ATS21=1’ will enable debug printing. This setting can be stored in NVM (non-volatile memory) by sending the ‘AT&W’ command. 

Remote configuration is supported. If two radios are linked, all AT commands can be sent to the remote radio using the RT equivalent (ie, RTZ will reboot the remote radio). **Be careful** as it is very possible to break the link. For example, setting the remote AES key should be done first before setting the local AES key.

[![Table of common parameters](https://cdn.sparkfun.com/r/600-600/assets/learn_tutorials/2/6/1/7/SparkFun_LoRaSerial_-_Parameters.png)](https://cdn.sparkfun.com/assets/learn_tutorials/2/6/1/7/LoRaSerial_Radio_Registers_-_Parameters.pdf)

*Table of common parameters*

The table of common parameters is available [here](https://cdn.sparkfun.com/assets/learn_tutorials/2/6/1/7/LoRaSerial_Radio_Registers_-_Parameters.pdf) in PDF format.

* **SerialSpeed** - Controls the baud rate in bits-per-second used over the UART connector. Data sent over USB will be sent/received regardless of this setting. Default is 57600bps. Allowed values are 2400, 4800, 9600, 14400, 19200, 38400, 57600, and 115200bps.

* **AirSpeed** - This is the effective rate in bits-per-second at which data is sent over the air. In general, the lower the air speed, the greater the transmission distance. LoRaSerial uses large 4,000 byte buffers to receive and send serial over USB or UART at the *SerialSpeed* and begins sending that data in chunks over the air at the AirSpeed. The *AirSpeed* setting does not have to match the SerialSpeed. It is recommended to limit the total incoming data to match the AirSpeed. For example, regularly sending a group of 300 bytes with an air speed of 4800 bps (480 bytes per second) will allow the radio sufficient bandwidth. Sending 1,000 bytes per second with an air speed of 4800 bps (480 bytes per second) will within a few seconds overwhelm the link leading to buffer overflow and data loss. Default is 4800bps. Allowed values are 0, 40, 150, 400, 1200, 2400, 4800, 9600, 19200, 28800, and 38400 bits per second. 0 is a special value that allows the Bandwidth, SpreadFactor, and CodingRate settings to be used instead.

* **NetID** - In PointToPoint mode each radio is paired with another radio. They will have unique frequencies that they will use. However, crosstalk is always possible. If a packet is received with a non-matching NetID it is discarded. Changing the NetID is a handy way to make sure your radio pair does not interfere with another pair of radios in the same vicinity.

* **PointToPoint** - In point to point mode, two radios pass packets and acknowledge (confirm) the receipt of a given packet. Lost packets are automatically retransmitted up to *MaxRends* number of tries. When PointToPoint is turned off, acknowledgements are turned off. Any radio using the same settings will be able to receive packets from each other. This is most useful when you have one ‘base’ transmitter with multiple ‘rovers’ receiving that data.

* **EncryptData** - By default all packets are encrypted using 128 bit AES GCM. Disabling this will not get greater range or bandwidth. Disabling encryption will allow all packets to be seen in clear text via an SDR or other monitoring device.

* **EncryptionKey** - This is the 16 byte key used for AES encryption. While this can be set via command, we recommend using the train feature as this is much faster and less error prone.

* **DataScrambling** - Enabling data scrambling will send all packets through an *[IBM data whitening](https://www.nxp.com/docs/en/application-note/AN5070.pdf)* process. This removes long sets of 1s or 0s from the packet to reduce DC bias during transmission. This is generally not needed and is not recommended when AES encryption is enabled. By default scrambling is turned off.

* **TxPower** - The LoRaSerial uses a high power 1W transceiver. By default, all transmissions are sent at the highest possible power of 30dBm which is compliant with FCC Part 15.247 when used with an antenna that has a gain of 6dBi or less. If your local regulations require lower transmission power this setting can be lowered. Allowed values are 30 down to 14dBm. Note: The chosen setting is the actual measured transmit power at the SMA connector. An internal lookup table sets the radio settings accordingly.

* **FrequencyMin/FrequencyMax** - These are the lower and upper bounds for the allowed transmission frequencies in megahertz. By default this is 902.0 to 928.0. 

* **NumberOfChannels** - The available spectrum (default is 902MHz to 928MHz) is divided by this number of channels to create the channel spacing and allowed frequency list (aka the ‘hop table’). The default is 50 channels to meet FCC Part 15.247 compliance and may be changed to meet local regulations.

* **FrequencyHop** - The LoRaSerial implements frequency hopping spread spectrum (FHSS) by default to meet FCC Part 15.247 compliance. Turning off frequency hopping is not recommended unless You Know What You’re Doing™.

* **MaxDwellTime** - The number of milliseconds of transmission allowed on a given frequency before hopping intra-packet. The default is 400ms to be compliant with FCC Part 15.247. This means the radio will change its frequency to the next channel in the hop table during the packet transmission. Note this is the maximum dwell time; depending on the air speed setting the radio may have a hopping period that is shorter than the dwell time.

* **Bandwidth** - The bandwidth used around a given frequency during LoRa transmissions. Setting is in kHz. This setting is overwritten if the AirSpeed setting is non-zero. It is recommended to use the air speed setting unless you are very aware of the consequences. Change AirSpeed to 0 to use a custom bandwidth. In general a lower bandwidth number provides longer range, but lower overall data rate. Allowed bandwidths: 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125.0, 250.0, and 500.0kHz.

* **SpreadFactor** - The spread factor used during LoRa transmissions. It is recommended to use the air speed setting unless you are very aware of the consequences. This setting is overwritten if the AirSpeed setting is non-zero. Change AirSpeed to 0 to use a custom spread factor. In general a higher spread factor provides longer range, but lower overall data rate. Allowed spread factors: 6 to 12 (inclusive).

* **CodingRate** - The coding rate used during LoRa transmissions. It is recommended to use the air speed setting unless you are very aware of the consequences. This setting is overwritten if the AirSpeed setting is non-zero. Change AirSpeed to 0 to use a custom coding rate. In general a higher spread factor provides longer range, but lower overall data rate. Allowed spread factors: 6 to 12 (inclusive).

* **SyncWord** - The byte used to synchronize LoRa transmissions. In general this is set to 0x12 for non-LoRaWAN networks. Note that two LoRa radios with the same settings but different sync words have been shown to intermittently receive packets from each other. Therefore, using a unique synch word does *not* guarantee exclusivity. Allowed values: 0 to 255.

* **PreambleLength** - The number of sync words to send at the start of a packet. Note that two LoRa radios with the same settings but different preamble lengths have been shown to intermittently receive packets from each other. Therefore, using a unique preamble length does *not* guarantee exclusivity. Allowed values: 6 to 65535.

* **FrameSize** - The number of bytes to be received before initiating a transmission. The default is 253. Allowed values: 16 to 253.

* **FrameTimeout** - The number of milliseconds of timeout before a partial packet is sent. For example if a partial frame of 12 bytes are received, the radio will wait this amount for more bytes before initiating a transmission. The default is 50ms. Allowed values: 10 to 2000ms.

* **Debug** - Enabling debug messages will print additional information about the radio’s state and packet information. Default is off.

* **Echo** - By default the radio will not echo the incoming serial. This is helpful at times if a user is typing data directly into a terminal. During AT configuration echo is turned on regardless of this setting.

* **HeartBeatTimeout** - When PointToPoint mode is enabled, a radio will send a ping packet every HeartBeatTimeout number of milliseconds if there is no data traffic. The default is 5000ms (5 seconds). Allowed values: 250 to 65535ms.

* **FlowControl** - If flow control is enabled, the radio will not print data if CTS is low (host is telling the radio to hold its horses). If flow control is enabled, the radio will pull RTS low if its serial buffer is full (radio is telling the host to hold its horses). CTS and RTS pins are only exposed on the UART connector but apply to both USB and serial data streams. By default, flow control is turned off. Internal pullups are used so RTS and CTS can be left floating if not used.

* **AutoTune** - Based on the frequency error of the last received packet, tune the receive frequency accordingly. Currently, this feature is not recommended. Default is off.

* **DisplayPacketQuality** - Show RSSI (received signal strength indicator), SNR (signal to noise ratio), and FreqError (frequency error) for all received packets.

* **MaxResends** - If PointToPoint is enabled, a radio will transmit a data packet and then wait for a response. If no response is received within 125% of the response packet’s air time, the data will be resent up to MaxResends. Default is 2. Allowed values: 0 to 255.




## Radio Parameters

The following parameters describe the radio transmission operation:

* AutoTuneFrequency - Adjust the frequency based on the last received frame's frequency error.

* FrequencyHop - Enable or disable frequency hopping. Legal operation in the United States requires that this value be set to true (1) which is the default. This parameter specifies whether the radio will change frequencies to a new channel every MaxDwellTime milliseconds.

* FrequencyMax - Maximum frequency the radio will use specified in MegaHertz. The default value is 928.0 MHz.

* FrequencyMin - Minimum frequency the radio will use specified in MegaHertz. The default value is 902.0 MHz.

* MaxDwellTime - Number of milliseconds that the LoRaSerial radio uses a specific frequency (channel). After this time, the radio switches (hops) to another frequency. For operation within the United States this value must not be greater than 400 milliseconds which is the default.

* NumberOfChannels - Specify the number of unique frequencies that will be used by the LoRaSerial radio. For operation within the United States this value needs to be a minimum of 50 which is the default value.

* RadioBroadcastPower_dbm - Transmit power level for the LoRaSerial radio. The valid range for this parameter is from 14 (25 milliWatts) to 30 dBm (1 Watt). The default value is 30 dBm.

### Frame Synchronization Parameters

The following parameters specify data that the radios use for data synchronization and frame reception:

* RadioPreambleLength - Number of symbols to transmit at the beginning of the frame to notify the receiving radios of the incoming frame. The range of values are 6 to 65535 with the default being 8. The preamble is used to synchronize the PLLs in the radio to properly detect the symbol boundaries. NOTE: Different lengths does *not* guarantee a remote radio privacy. 8 to 11 works. 8 to 15 drops some. 8 to 20 is silent.

* RadioSyncWord - The sync word following the preamble indicates the destination set of radios. Note that different sync words do *not* guarantee a remote radio will not receive a rogue frame.

### AirSpeed

The parameter AirSpeed is a simplification that selects values for the following parameters to approximate the baud rate implied by the AirSpeed.

* HeartbeatTimeout
* RadioBandwidth
* RadioCodingRate
* RadioSpreadFactor
* TxToRxUsec

Valid values for AirSpeed are (40, 150, 400, 1200, 2400, 4800, 9600, 19200, 28800 and 38400) and the default is 4800. After AirSpeed is set, it is possible to modify any of the parameters above. Note that AirSpeed is just an easy way to set the parameters above to known value. AirSpeed is not an actual parameter that is used for training, only the parameters listed above are used during training. More detail for these parameters is provided below:

* HeartbeatTimeout - Heartbeats are transmitted on a regular basis by the server and in point-to-point and virtual circuit modes by the clients. This parameter specifies the time in milliseconds during which a HEARTBEAT frame should be transmitted. If a HEARTBEAT frame is not received within three (3) times this interval then the point-to-point or virtual circuit link is broken. The default heartbeatTimeout is 5000 milliseconds (5 seconds).

* RadioBandwidth - Bandwidth of the spread spectrum signal specified in kiloHertz. The default is 500.0 kHz and other supported values are: 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125 and 250 kHz.

* RadioCodingRate - Number of bits to send 4 data bits of information. This is used for forward error correction, allowing the receiver to correct the incoming bit stream. Valid values are 5 through 8 where higher coding rates ensure less frames are dropped. The default value is 8.

* RadioSpreadFactor - Number of bits in a symbol. Valid values are 6 to 12 with the default value of 9. Larger values enable longer range while reducing the data rate.

* TxToRxUsec - Transition time from end of transmission to the end of reception on the remote radio specified in microseconds.

## Radio protocol parameters

The following parameters describe the contents of the data frame sent between the LoRaSerial radios:

* DataScrambling - Enable (1) or disable (0) the use of IBM's Data Whitening algorithm to reduce DC bias

* EnableCRC16 - Enable (1) or disable (0) adding a software generated CRC-16 to the end of the data frame.

* EncryptData - Enable (1) or disable (0) the use of AES encryption for each data frame

* EncryptionKey - The 16 byte encryption key specified in hexadecimal. While SparkFun provides a default value, it is strongly recomended to change this value for your own networks.

* FramesToYield - When requested by the remote radio, suppress transmission for this number of max packet frames. The valid range is 0 to 255 with a default value of 3.

* MaxResends - Number of retransmission attempt to make for a each frame when an ACK is not received from the destination radio. The valid range is 0 to 255. The default of zero represents infinite and retries will continue as long as the two radios are receiving HEARTBEAT frames from each other. The transmission will fail only when the link breaks due to not receiving HEARTBEAT frames.

* NetID - A unique value denoting the network of LoRaSerial radios. All radios in the network must share the same ID value. This value is in addition to the radioSyncWord.

* OperatingMode - The following modes of operation are supported by the LoRaSerial radios:

  * MODE_MULTIPOINT (0) - A single server with multiple clients, all radios are sending datagram frames which are not guarranteed to be received by the other radios. This mode is great when real-time transmission is necessary and the application is able to tolerate some loss of data.

  * MODE_POINT_TO_POINT (1, default) - Communications between only two LoRaSerial radios with guarranteed delivery of frames or the link breaks.

  * MODE_VIRTUAL_CIRCUIT - A single server with multiple clients that supports multipoint communications with guarranteed delivery or the link breaks. This mode uses a special protocol over the serial link to be able to specify the destination radio for transmission and the receive radio for reception. More information is available [here](https://github.com/sparkfun/SparkFun_LoRaSerial/blob/release_candidate/Documents/Virtual_Circuits.md).

* OverheadTime - The number of milliseconds to add to ACK and datagram times before ACK timeout occurs. The default is 10 milliseconds.

* SelectLedUse - Select how to display information on the LEDs

* Server - Enable (1) or disable (0) the server mode for training and for multipoint or virtual circuit operation. The default is client mode (0).

* VerifyRxNetID - Enable (1) or disable (0) the verification of the netID value during reception. The default is enabled (1).