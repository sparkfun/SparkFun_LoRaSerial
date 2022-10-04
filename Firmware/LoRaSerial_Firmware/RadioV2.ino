//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//Point-To-Point: Bring up the link
//
//A three way handshake is used to get both systems to agree that data can flow in both
//directions.  This handshake is also used to synchronize the HOP timer.
/*
                    System A                 System B

                     RESET                     RESET
                       |                         |
             Channel 0 |                         | Channel 0
                       V                         V
           .----> P2P_NO_LINK               P2P_NO_LINK
           |           | Tx PING                 |
           | Timeout   |                         |
           |           V                         |
           | P2P_WAIT_TX_PING_DONE               |
           |           |                         |
           |           | Tx Complete - - - - - > | Rx PING
           |           |   Start Rx              |
           |           |   MAX_PACKET_SIZE       |
           |           V                         V
           `---- P2P_WAIT_ACK_1                  +<----------------------.
                       |                         | Tx PING ACK1          |
                       |                         V                       |
                       |              P2P_WAIT_TX_ACK_1_DONE             |
                       |                         |                       |
          Rx PING ACK1 | < - - - - - - - - - - - | Tx Complete           |
                       |                         |   Start Rx            |
                       |                         |   MAX_PACKET_SIZE     |
                       |                         |                       |
                       V                         V         Timeout       |
           .---------->+                   P2P_WAIT_ACK_2 -------------->+
           |   TX PING |                         |                       ^
           |      ACK2 |                         |                       |
           |           V                         |                       |
           | P2P_WAIT_TX_ACK_2_DONE              |                       |
           |           | Tx Complete - - - - - > | Rx PING ACK2          |
           | Stop      |   Start HOP timer       |   Start HOP Timer     | Stop
           | HOP       |   Start Rx              |   Start Rx            | HOP
           | Timer     |   MAX_PACKET_SIZE       |   MAX_PACKET_SIZE     | Timer
           |           |                         |                       |
           | Rx        |                         |                       |
           | PING ACK  V                         V         Rx PING       |
           `----- P2P_LINK_UP               P2P_LINK_UP -----------------’
                       |                         |
                       | Rx Data                 | Rx Data
                       |                         |
                       V                         V

  Two timers are in use:
    datagramTimer:  Set at end of transmit, measures ACK timeout
    heartbeatTimer: Set upon entry to P2P_NO_LINK, measures time to send next PING
*/
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//First packet in the three way handshake to bring up the link
void xmitDatagramP2PPing()
{
  unsigned long currentMillis;

  currentMillis = millis();
  memcpy(endOfTxData, &currentMillis, sizeof(currentMillis));
  endOfTxData += sizeof(unsigned long);

  /*
                     endOfTxData ---.
                                    |
                                    V
      +--------+---------+----------+----------+
      |        |         |          | Optional |
      | NET ID | Control |  Millis  | Trailer  |
      | 8 bits | 8 bits  | 4 bytes  |  8 bits  |
      +--------+---------+----------+----------+
  */

  txControl.datagramType = DATAGRAM_PING;
  txControl.ackNumber = 0;
  transmitDatagram();
}

//Second packet in the three way handshake to bring up the link
void xmitDatagramP2PAck1()
{
  //  unsigned long currentMillis;
  //
  //  currentMillis = millis();
  //  memcpy(endOfTxData, &currentMillis, sizeof(currentMillis));
  //  endOfTxData += sizeof(unsigned long);

  uint8_t channelTimerElapsed = (millis() - timerStart) / CHANNEL_TIMER_DIVISOR;
  memcpy(endOfTxData, &channelTimerElapsed, sizeof(channelTimerElapsed));
  endOfTxData += sizeof(uint8_t);

  /*
                     endOfTxData ---.
                                    |
                                    V
      +--------+---------+----------+----------+
      |        |         |          | Optional |
      | NET ID | Control |  Millis  | Trailer  |
      | 8 bits | 8 bits  | 4 bytes  |  8 bits  |
      +--------+---------+----------+----------+
  */

  txControl.datagramType = DATAGRAM_ACK_1;
  txControl.ackNumber = 0;
  transmitDatagram();
}

//Last packet in the three way handshake to bring up the link
void xmitDatagramP2PAck2()
{
  //  unsigned long currentMillis;
  //
  //  currentMillis = millis();
  //  memcpy(endOfTxData, &currentMillis, sizeof(currentMillis));
  //  endOfTxData += sizeof(unsigned long);

  //  uint8_t channelTimerElapsed = (millis() - timerStart) / CHANNEL_TIMER_DIVISOR;
  //  memcpy(endOfTxData, &channelTimerElapsed, sizeof(channelTimerElapsed));
  //  endOfTxData += sizeof(uint8_t);

  /*
                     endOfTxData ---.
                                    |
                                    V
      +--------+---------+----------+----------+
      |        |         |          | Optional |
      | NET ID | Control |  Millis  | Trailer  |
      | 8 bits | 8 bits  | 4 bytes  |  8 bits  |
      +--------+---------+----------+----------+
  */

  txControl.datagramType = DATAGRAM_ACK_2;
  txControl.ackNumber = 0;
  transmitDatagram();
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Point-to-Point Data Exchange
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Send a command datagram to the remote system
void xmitDatagramP2PCommand()
{
  /*
                       endOfTxData ---.
                                      |
                                      V
      +--------+---------+---  ...  ---+----------+
      |        |         |             | Optional |
      | NET ID | Control |    Data     | Trailer  |
      | 8 bits | 8 bits  |   n bytes   |  8 bits  |
      +--------+---------+-------------+----------+
  */

  txControl.datagramType = DATAGRAM_REMOTE_COMMAND;
  txControl.ackNumber = txAckNumber;
  txAckNumber = (txAckNumber + ((datagramsExpectingAcks & (1 << txControl.datagramType)) != 0)) & 3;
  transmitDatagram();
}

//Send a command response datagram to the remote system
void xmitDatagramP2PCommandResponse()
{
  /*
                       endOfTxData ---.
                                      |
                                      V
      +--------+---------+---  ...  ---+----------+
      |        |         |             | Optional |
      | NET ID | Control |    Data     | Trailer  |
      | 8 bits | 8 bits  |   n bytes   |  8 bits  |
      +--------+---------+-------------+----------+
  */

  txControl.datagramType = DATAGRAM_REMOTE_COMMAND_RESPONSE;
  txControl.ackNumber = txAckNumber;
  txAckNumber = (txAckNumber + ((datagramsExpectingAcks & (1 << txControl.datagramType)) != 0)) & 3;
  transmitDatagram();
}

//Send a data datagram to the remote system
void xmitDatagramP2PData()
{
  /*
                       endOfTxData ---.
                                      |
                                      V
      +--------+---------+---  ...  ---+----------+
      |        |         |             | Optional |
      | NET ID | Control |    Data     | Trailer  |
      | 8 bits | 8 bits  |   n bytes   |  8 bits  |
      +--------+---------+-------------+----------+
  */

  txControl.datagramType = DATAGRAM_DATA;
  txControl.ackNumber = txAckNumber;
  txAckNumber = (txAckNumber + ((datagramsExpectingAcks & (1 << txControl.datagramType)) != 0)) & 3;
  transmitDatagram();
}

//Heartbeat packet to keep the link up
void xmitDatagramP2PHeartbeat()
{
  /*
          endOfTxData ---.
                         |
                         V
      +--------+---------+----------+
      |        |         | Optional |
      | NET ID | Control | Trailer  |
      | 8 bits | 8 bits  |  8 bits  |
      +--------+---------+----------+
  */

  txControl.datagramType = DATAGRAM_HEARTBEAT;

  //Orig
  //txControl.ackNumber = 0;

  txControl.ackNumber = txAckNumber;
  txAckNumber = (txAckNumber + ((datagramsExpectingAcks & (1 << txControl.datagramType)) != 0)) & 3;

  transmitDatagram();
}

//Create short packet of 2 control bytes - do not expect ack
void xmitDatagramP2PAck()
{
  //  unsigned long currentMillis = millis();
  //  memcpy(endOfTxData, &currentMillis, sizeof(currentMillis));
  //  endOfTxData += sizeof(unsigned long);

  uint8_t channelTimerElapsed = millis() - timerStart;
  systemPrint("channelTimerElapsedPure: ");
  systemPrintln(channelTimerElapsed);
  channelTimerElapsed /= CHANNEL_TIMER_DIVISOR;
  systemPrint("channelTimerElapsedDiv: ");
  systemPrintln(channelTimerElapsed);

  //uint8_t channelTimerElapsed = (millis() - timerStart) / CHANNEL_TIMER_DIVISOR;
  memcpy(endOfTxData, &channelTimerElapsed, sizeof(channelTimerElapsed));
  endOfTxData += sizeof(uint8_t);


  /*
          endOfTxData ---.
                         |
                         V
      +--------+---------+----------+
      |        |         | Optional |
      | NET ID | Control | Trailer  |
      | 8 bits | 8 bits  |  8 bits  |
      +--------+---------+----------+
  */

  txControl.datagramType = DATAGRAM_DATA_ACK;
  txControl.ackNumber = rxAckNumber;
  transmitDatagram();
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//Datagram reception
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Determine the type of datagram received
PacketType rcvDatagram()
{
  CONTROL_U8 * control;
  PacketType datagramType;
  static uint8_t expectedRxAck;
  uint8_t receivedNetID;

  //Save the receive time
  rcvTimeMillis = millis();

  //Get the received datagram
  radio.readData(incomingBuffer, MAX_PACKET_SIZE);
  rxDataBytes = radio.getPacketLength();
  rxData = incomingBuffer;

  /*
      |<---------------------- rxDataBytes ---------------------->|
      |                                                           |
      +----------+----------+------------+---  ...  ---+----------+
      | Optional | Optional |  Optional  |             | Optional |
      |  NET ID  | Control  | SF6 Length |    Data     | Trailer  |
      |  8 bits  |  8 bits  |   8 bits   |   n bytes   |  8 bits  |
      +----------+----------+------------+-------------+----------+
      ^
      |
      '---- rxData
  */

  //Display the received data bytes
  if (settings.printRfData || settings.debugReceive)
  {
    systemPrintln("----------");
    systemPrintTimestamp();
    systemPrint("RX: Data ");
    systemPrint(rxDataBytes);
    systemPrint(" (0x");
    systemPrint(rxDataBytes, HEX);
    systemPrintln(") bytes");
    petWDT();
    if (settings.printRfData && rxDataBytes)
      dumpBuffer(incomingBuffer, rxDataBytes);
  }

  if (settings.dataScrambling == true)
    radioComputeWhitening(incomingBuffer, rxDataBytes);

  if (settings.encryptData == true)
    decryptBuffer(incomingBuffer, rxDataBytes);

  //All packets must include the 2-byte control header
  if (rxDataBytes < minDatagramSize)
  {
    //Display the packet contents
    if (settings.printPktData || settings.debugReceive)
    {
      systemPrintTimestamp();
      systemPrint("RX: Bad packet");
      systemPrint(rxDataBytes);
      systemPrint(" (0x");
      systemPrint(rxDataBytes, HEX);
      systemPrintln(") bytes");
      petWDT();
      if (settings.printRfData && rxDataBytes)
        dumpBuffer(incomingBuffer, rxDataBytes);
    }
    return (PACKET_BAD);
  }

  /*
      |<---------------------- rxDataBytes ---------------------->|
      |                                                           |
      +----------+----------+------------+---  ...  ---+----------+
      | Optional | Optional |  Optional  |             | Optional |
      |  NET ID  | Control  | SF6 Length |    Data     | Trailer  |
      |  8 bits  |  8 bits  |   8 bits   |   n bytes   |  8 bits  |
      +----------+----------+------------+-------------+----------+
      ^
      |
      '---- rxData
  */

  //Process the trailer

  //Verify the netID if necessary
  if (settings.pointToPoint || settings.verifyRxNetID)
  {
    receivedNetID = *rxData++;
    if (settings.debugReceive)
    {
      systemPrintTimestamp();
      systemPrint("RX: NetID ");
      systemPrint(receivedNetID);
      systemPrint(" (0x");
      systemPrint(receivedNetID, HEX);
      systemPrint(")");
      if (receivedNetID != settings.netID)
      {
        systemPrint(" expecting ");
        systemPrintln(settings.netID);
        petWDT();
        if (settings.printPktData && rxDataBytes)
          dumpBuffer(incomingBuffer, rxDataBytes);
        return (PACKET_NETID_MISMATCH);
      }
      systemPrintln();
    }
  }

  /*
      |<---------------------- rxDataBytes ---------------------->|
      |                                                           |
      +----------+----------+------------+---  ...  ---+----------+
      | Optional | Optional |  Optional  |             | Optional |
      |  NET ID  | Control  | SF6 Length |    Data     | Trailer  |
      |  8 bits  |  8 bits  |   8 bits   |   n bytes   |  8 bits  |
      +----------+----------+------------+-------------+----------+
                 ^
                 |
                 '---- rxData
  */

  //Get the control byte
  datagramType = DATAGRAM_DATA;
  if (settings.pointToPoint)
  {
    control = (CONTROL_U8 *)rxData++;
    datagramType = (PacketType)control->datagramType;
    rxAckNumber = control->ackNumber;
    if (settings.debugReceive)
      printControl(*(uint8_t *)control);
    if (datagramType >= MAX_DATAGRAM_TYPE)
      return (PACKET_BAD);
  }

  /*
      |<---------------------- rxDataBytes ---------------------->|
      |                                                           |
      +----------+----------+------------+---  ...  ---+----------+
      | Optional | Optional |  Optional  |             | Optional |
      |  NET ID  | Control  | SF6 Length |    Data     | Trailer  |
      |  8 bits  |  8 bits  |   8 bits   |   n bytes   |  8 bits  |
      +----------+----------+------------+-------------+----------+
                            ^
                            |
                            '---- rxData
  */

  //Get the spread factor 6 length
  if (datagramType == DATAGRAM_SF6_DATA)
  {
    if (rxDataBytes >= (*rxData + minDatagramSize))
      rxDataBytes = *rxData++;
    else
    {
      if (settings.debugReceive)
      {
        systemPrintTimestamp();
        systemPrint("Invalid SF6 length, received SF6 length");
        systemPrint(*rxData);
        systemPrint(" > ");
        systemPrint((int)rxDataBytes - minDatagramSize);
        systemPrintln(" received bytes");
      }
      return (PACKET_BAD);
    }
  }
  rxDataBytes -= minDatagramSize;

  //Verify the ACK number last so that the expected ACK number can be updated
  //                     txAckNumber ----> rxAckNumber == expectedTxAck
  //    expectedTxAck == rxAckNumber <---- txAckNumber
  if (settings.pointToPoint)
  {
    switch (datagramType)
    {
      case DATAGRAM_DATA_ACK:
        if (rxAckNumber != expectedTxAck)
        {
          if (settings.debugReceive)
          {
            systemPrintTimestamp();
            systemPrint("Invalid ACK number, received ");
            systemPrint(rxAckNumber);
            systemPrint(" expecting ");
            systemPrintln(expectedTxAck);
          }
          return (PACKET_BAD);
        }

        //Increment the expected ACK number
        expectedTxAck = (expectedTxAck + 1) & 3;
        break;

      case DATAGRAM_DATA:
      case DATAGRAM_SF6_DATA:
      case DATAGRAM_REMOTE_COMMAND:
      case DATAGRAM_REMOTE_COMMAND_RESPONSE:
      case DATAGRAM_HEARTBEAT:
        if (rxAckNumber != expectedRxAck)
        {
          //Determine if this is a duplicate datagram
          if (rxAckNumber == ((expectedRxAck - 1) & 3))
          {
            linkDownTimer = millis();
            return PACKET_DUPLICATE;
          }

          //Not a duplicate
          return PACKET_BAD;
        }

        //Receive this data packet and set the next expected RX ACK number
        expectedRxAck = (expectedRxAck + 1) & 3;
        break;
    }
  }

  /*
                                         |<-- rxDataBytes -->|
                                         |                   |
      +----------+----------+------------+------  ...  ------+----------+
      | Optional | Optional |  Optional  |                   | Optional |
      |  NET ID  | Control  | SF6 Length |       Data        | Trailer  |
      |  8 bits  |  8 bits  |   8 bits   |      n bytes      |  8 bits  |
      +----------+----------+------------+-------------------+----------+
                                         ^
                                         |
                                         '---- rxData
  */

  //Display the packet contents
  if (settings.printPktData || settings.debugReceive)
  {
    systemPrintTimestamp();
    systemPrint("RX: Packet data ");
    systemPrint(rxDataBytes);
    systemPrint(" (0x");
    systemPrint(rxDataBytes, HEX);
    systemPrintln(") bytes");
    petWDT();
    if (settings.printPktData && rxDataBytes)
      dumpBuffer(rxData, rxDataBytes);
  }

  //Display the datagram type
  if (settings.debugDatagrams)
  {
    systemPrintTimestamp();
    systemPrint("RX: ");
    systemPrint(v2DatagramType[datagramType]);
    switch (datagramType)
    {
      default:
        systemPrintln();
        break;

      case DATAGRAM_DATA:
      case DATAGRAM_DATA_ACK:
      case DATAGRAM_SF6_DATA:
      case DATAGRAM_REMOTE_COMMAND:
      case DATAGRAM_REMOTE_COMMAND_RESPONSE:
      case DATAGRAM_HEARTBEAT:
        if (settings.pointToPoint)
        {
          systemPrint(" (ACK #");
          systemPrint(rxAckNumber);
          systemPrint(")");
        }
        systemPrintln();
        break;
    }
  }

  //Process the packet
  linkDownTimer = millis();
  return datagramType;
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//Datagram transmission
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Push the outgoing packet to the air
void transmitDatagram()
{
  uint8_t control;
  uint8_t * header;
  uint8_t length;

  //Determine the packet size
  txDatagramSize = endOfTxData - outgoingPacket;
  length = txDatagramSize - headerBytes;

  //Process the packet
  if (settings.debugDatagrams)
  {
    systemPrintTimestamp();
    systemPrint("TX: ");
    systemPrint(v2DatagramType[txControl.datagramType]);
    switch (txControl.datagramType)
    {
      default:
        systemPrintln();
        break;

      case DATAGRAM_DATA:
      case DATAGRAM_DATA_ACK:
      case DATAGRAM_SF6_DATA:
      case DATAGRAM_REMOTE_COMMAND:
      case DATAGRAM_REMOTE_COMMAND_RESPONSE:
        if (settings.pointToPoint)
        {
          systemPrint(" (ACK #");
          systemPrint(txControl.ackNumber);
          systemPrint(")");
        }
        systemPrintln();
        break;
    }
  }

  /*
                                        endOfTxData ---.
                                                       |
                                                       V
      +----------+----------+------------+---  ...  ---+----------+
      | Optional | Optional |  Optional  |             | Optional |
      |  NET ID  | Control  | SF6 Length |    Data     | Trailer  |
      |  8 bits  |  8 bits  |   8 bits   |   n bytes   |  8 bits  |
      +----------+----------+------------+-------------+----------+
      |                                  |             |
      |                                  |<- Length -->|
      |<--------- txDatagramSize --------------------->|
  */

  //Display the packet contents
  if (settings.printPktData || settings.debugTransmit)
  {
    systemPrintTimestamp();
    systemPrint("TX: Packet data ");
    systemPrint(length);
    systemPrint(" (0x");
    systemPrint(length, HEX);
    systemPrintln(") bytes");
    petWDT();
    if (settings.printPktData)
      dumpBuffer(&endOfTxData[-length], length);
  }

  //Build the datagram header
  header = outgoingPacket;
  if (headerBytes && settings.debugTransmit)
  {
    systemPrintTimestamp();
    systemPrint("TX: Header");
    systemPrint(headerBytes);
    systemPrint(" (0x");
    systemPrint(headerBytes);
    systemPrintln(") bytes");
  }

  //Add the netID if necessary
  if (settings.pointToPoint || settings.verifyRxNetID)
  {
    *header++ = settings.netID;

    //Display the netID value
    if (settings.debugTransmit)
    {
      systemPrintTimestamp();
      systemPrint("    NetID: ");
      systemPrint(settings.netID);
      systemPrint(" (0x");
      systemPrint(settings.netID, HEX);
      systemPrintln(")");
      petWDT();
    }
  }

  //Add the control byte if needed
  if (settings.pointToPoint)
  {
    control = *(uint8_t *)&txControl;
    *header++ = control;

    //Display the control value
    if (settings.debugTransmit)
      printControl(control);
  }

  //Add the spread factor 6 length is required
  if (txControl.datagramType == DATAGRAM_SF6_DATA)
  {
    *header++ = length;
    txDatagramSize = MAX_PACKET_SIZE - trailerBytes; //We're now going to transmit a full size datagram
    if (settings.debugTransmit)
    {
      systemPrintTimestamp();
      systemPrint("    SF6 Length: ");
      systemPrintln(length);
    }
  }

  /*
      +----------+----------+------------+---  ...  ---+
      | Optional | Optional |  Optional  |             |
      |  NET ID  | Control  | SF6 Length |    Data     |
      |  8 bits  |  8 bits  |   8 bits   |   n bytes   |
      +----------+----------+------------+-------------+
      |                                                |
      |<--------------- txDatagramSize --------------->|
  */

  //Add the datagram trailer
  txDatagramSize += trailerBytes;

  //Display the trailer
  if (trailerBytes && settings.debugTransmit)
  {
    systemPrintTimestamp();
    systemPrint("TX: Trailer ");
    systemPrint(trailerBytes);
    systemPrint(" (0x");
    systemPrint(trailerBytes);
    systemPrintln(") bytes");
  }

  /*
      +----------+----------+------------+---  ...  ---+----------+
      | Optional | Optional |  Optional  |             | Optional |
      |  NET ID  | Control  | SF6 Length |    Data     | Trailer  |
      |  8 bits  |  8 bits  |   8 bits   |   n bytes   |  8 bits  |
      +----------+----------+------------+-------------+----------+
      |                                                           |
      |<-------------------- txDatagramSize --------------------->|
  */

  //Encrypt the datagram
  if (settings.encryptData == true)
    encryptBuffer(outgoingPacket, txDatagramSize);

  //Scramble the datagram
  if (settings.dataScrambling == true)
    radioComputeWhitening(outgoingPacket, txDatagramSize);

  packetSent = 0; //This is the first time this packet is being sent

  //If we are trainsmitting at high data rates the receiver is often not ready for new data. Pause for a few ms (measured with logic analyzer).
  if (settings.airSpeed == 28800 || settings.airSpeed == 38400)
    delay(2);

  //Reset the buffer data pointer for the next transmit operation
  endOfTxData = &outgoingPacket[headerBytes];

  //Compute the delay for the ACK datagram
  if (datagramsExpectingAcks & (1 << txControl.datagramType))
    txDelay = settings.txAckMillis;
  else
    txDelay = random(0, 1000);

  //Transmit this datagram
  retransmitDatagram();
}

//Print the control byte value
void printControl(uint8_t value)
{
  CONTROL_U8 * control = (CONTROL_U8 *)&value;

  systemPrintTimestamp();
  systemPrint("    Control: 0x");
  systemPrintln(value, HEX);
  systemPrintTimestamp();
  systemPrint("        ACK # ");
  systemPrintln(value & 3);
  systemPrintTimestamp();
  systemPrint("        datagramType ");
  systemPrintln(v2DatagramType[control->datagramType]);
  petWDT();
}

//The previous transmission was not received, retransmit the datagram
void retransmitDatagram()
{
  /*
      +----------+----------+------------+---  ...  ---+----------+
      | Optional | Optional |  Optional  |             | Optional |
      |  NET ID  | Control  | SF6 Length |    Data     | Trailer  |
      |  8 bits  |  8 bits  |   8 bits   |   n bytes   |  8 bits  |
      +----------+----------+------------+-------------+----------+
      |                                                           |
      |<-------------------- txDatagramSize --------------------->|
  */

  //Display the transmitted packet bytes
  if (settings.printRfData || settings.debugTransmit)
  {
    systemPrintTimestamp();
    systemPrint("TX: ");
    if (packetSent)
      systemPrint("Retransmit ");
    systemPrint ("Data ");
    systemPrint(txDatagramSize);
    systemPrint(" (0x");
    systemPrint(txDatagramSize, HEX);
    systemPrintln(") bytes");
    petWDT();
    if (settings.printRfData)
      dumpBuffer(outgoingPacket, txDatagramSize);
  }

  int state = radio.startTransmit(outgoingPacket, txDatagramSize);
  if (state == RADIOLIB_ERR_NONE)
  {
    xmitTimeMillis = millis();
    packetAirTime = calcAirTime(txDatagramSize); //Calculate packet air size while we're transmitting in the background
    uint16_t responseDelay = packetAirTime / settings.responseDelayDivisor; //Give the receiver a bit of wiggle time to respond
    if (settings.debugTransmit)
    {
      systemPrintTimestamp();
      systemPrint("TX: PacketAirTime ");
      systemPrint(packetAirTime);
      systemPrintln(" mSec");

      systemPrintTimestamp();
      systemPrint("TX: responseDelay ");
      systemPrint(responseDelay);
      systemPrintln(" mSec");
    }
    packetAirTime += responseDelay;
  }
  else if (settings.debugTransmit)
  {
    systemPrintTimestamp();
    systemPrint("TX: Transmit error, state ");
    systemPrintln(state);
  }

  datagramTimer = millis(); //Move timestamp even if error
}

void startChannelTimer()
{
  channelTimer.enableTimer();
  triggerEvent(TRIGGER_HOP_TIMER_START);

}

void stopChannelTimer()
{
  channelTimer.disableTimer();
  triggerEvent(TRIGGER_HOP_TIMER_STOP);
}

//Given the remote unit's amount of channelTimer that has elapsed, and size of the ack received
//Adjust our own channelTimer interrupt to be synchronized with the remote unit
void syncChannelTimer(uint8_t sizeOfDatagram)
{
  triggerEvent(TRIGGER_SYNC_CHANNEL);

  uint16_t datagramAirTime = calcAirTime(sizeOfDatagram); //Calculate how much time it took for the datagram to be transmitted

  uint8_t channelTimerElapsed;
  memcpy(&channelTimerElapsed, rxData, sizeof(channelTimerElapsed));
  channelTimerElapsed *= CHANNEL_TIMER_DIVISOR;
  systemPrint("channelTimerElapsedPure: ");
  systemPrintln(channelTimerElapsed);
  channelTimerElapsed += datagramAirTime;
  channelTimerElapsed += SYNC_PROCESSING_OVERHEAD;

  partialTimer = true;
  channelTimer.disableTimer();
  channelTimer.setInterval_MS(settings.maxDwellTime - channelTimerElapsed, channelTimerHandler); //Shorten our hardware timer to match our mate's
  channelTimer.enableTimer();

  systemPrint("channelTimerElapsed: ");
  systemPrintln(channelTimerElapsed);
  systemPrint("partialTimerDuration: ");
  systemPrintln(settings.maxDwellTime - channelTimerElapsed);
}
