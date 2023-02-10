# Operating Modes

LoRaSerial radios can operate in one of three modes.

* Point to Point
* Multipoint
* Virtual Circuit

## Point to Point

By default, radios use Point to Point mode for communication. In this mode, data is limited to two paired radios. 

Radio Linkup is performed at Channel 0: Both radios power on and begin pinging each other until another radio is detected. Once a handshake is complete, both radios begin hopping through their channel tables. 

All data packets are acknowledged, guaranteeing that data will get across the link. If data is not able to traverse the link for 3 * Heartbeat Timeout (5 seconds by default) the link is marked as down and any data in the buffer is flushed. Radio Linkup is then initiated.

Benefits of P2P:
* Data is sent until it is successfully received.
* Simple to set up and use.

Disadvantages of P2P:
* Limited to data being shared between two radios.
* Lots of data being sent across a spotty link can lead to the link going down. 
* Retransmits will add delays to the transmission time.

## MultiPoint

In Multipoint mode, data is sent out from one radio to all radios. Data *is not* acknowledged, and therefore is not guaranteed to be delivered. 

One radio must be designated as Server. This radio is responsible for sending out heartbeat packets that synchronize the system.

Radio Linkup is performed during a discovery phase: The Server radio immediately begins hopping the channel table. When a Client radio is powered on, it will begin traversing the channel table, sending out pings. If no response is found, the Client radio will return to channel 0. The Server radio always transmits a heartbeat on channel 0. Once a heartbeat is detected, the Client radio begins hopping through its channel tables in sync with the Server. 

Benefits of Multipoint:
* Data is sent until it is successfully received.
* Simple to set up and use.

Disadvantages of P2P:
* Additional set up: One radio must be configured as Server.
* Lots of data being sent across a spotty link can lead to the link going down. 
* Retransmits will add delays to the transmission time.


## Virtual Circuit