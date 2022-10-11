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
  /*
          endOfTxData ---.
                         |
                         V
      +--------+---------+----------+
      |        |         |          |
      | NET ID | Control | Trailer  |
      | 8 bits | 8 bits  |  8 bits  |
      +--------+---------+----------+
  */

  txControl.datagramType = DATAGRAM_PING;
  txControl.ackNumber = 0;
  transmitDatagram();
}

//Second packet in the three way handshake to bring up the link
void xmitDatagramP2PAck1()
{
  uint16_t channelTimerElapsed = millis() - timerStart;
  memcpy(endOfTxData, &channelTimerElapsed, sizeof(channelTimerElapsed));
  endOfTxData += sizeof(channelTimerElapsed);

  /*
                     endOfTxData ---.
                                    |
                                    V
      +--------+---------+----------+----------+
      |        |         | Channel  | Optional |
      | NET ID | Control |  Timer   | Trailer  |
      | 8 bits | 8 bits  | 2 bytes  |  8 bits  |
      +--------+---------+----------+----------+
  */

  txControl.datagramType = DATAGRAM_ACK_1;
  txControl.ackNumber = 0;
  transmitDatagram();
}

//Last packet in the three way handshake to bring up the link
void xmitDatagramP2PAck2()
{
  /*
          endOfTxData ---.
                         |
                         V
      +--------+---------+----------+
      |        |         |          |
      | NET ID | Control | Trailer  |
      | 8 bits | 8 bits  |  8 bits  |
      +--------+---------+----------+
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
  txControl.ackNumber = expectedDatagramNumber;
  expectedDatagramNumber = (expectedDatagramNumber + ((datagramsExpectingAcks & (1 << txControl.datagramType)) != 0)) & 3;
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
  txControl.ackNumber = expectedDatagramNumber;
  expectedDatagramNumber = (expectedDatagramNumber + ((datagramsExpectingAcks & (1 << txControl.datagramType)) != 0)) & 3;
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
  txControl.ackNumber = expectedDatagramNumber;
  expectedDatagramNumber = (expectedDatagramNumber + ((datagramsExpectingAcks & (1 << txControl.datagramType)) != 0)) & 3;
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
  txControl.ackNumber = expectedDatagramNumber;
  expectedDatagramNumber = (expectedDatagramNumber + ((datagramsExpectingAcks & (1 << txControl.datagramType)) != 0)) & 3;
  transmitDatagram();
}

//Create short packet of 2 control bytes - do not expect ack
void xmitDatagramP2PAck()
{
  uint16_t channelTimerElapsed = millis() - timerStart;
  memcpy(endOfTxData, &channelTimerElapsed, sizeof(channelTimerElapsed));
  endOfTxData += sizeof(channelTimerElapsed);

  /*
                     endOfTxData ---.
                                    |
                                    V
      +--------+---------+----------+----------+
      |        |         | Channel  | Optional |
      | NET ID | Control |  Timer   | Trailer  |
      | 8 bits | 8 bits  | 2 bytes  |  8 bits  |
      +--------+---------+----------+----------+
  */

  txControl.datagramType = DATAGRAM_DATA_ACK;
  txControl.ackNumber = expectedAckNumber;
  expectedAckNumber = (expectedAckNumber + 1) & 3;
  transmitDatagram();
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Multi-Point Data Exchange
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Send a data datagram to the remote system
void xmitDatagramMpDatagram()
{
  /*
                          endOfTxData ---.
                                         |
                                         V
      +----------+---------+---  ...  ---+----------+
      | Optional |         |             | Optional |
      |  NET ID  | Control |    Data     | Trailer  |
      |  8 bits  | 8 bits  |   n bytes   |  8 bits  |
      +----------+---------+-------------+----------+
  */

  txControl.datagramType = DATAGRAM_DATAGRAM;
  txControl.ackNumber = 0;
  transmitDatagram();
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//Datagram reception
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Determine the type of datagram received
PacketType rcvDatagram()
{
  PacketType datagramType;
  uint8_t receivedNetID;
  CONTROL_U8 rxControl;

  //Get the received datagram
  radio.readData(incomingBuffer, MAX_PACKET_SIZE);
  rxDataBytes = radio.getPacketLength();
  rxData = incomingBuffer;

  /*
      |<---------------------- rxDataBytes ---------------------->|
      |                                                           |
      +----------+----------+------------+---  ...  ---+----------+
      | Optional |          |  Optional  |             | Optional |
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
    systemPrint("RX: ");
    systemPrint((settings.dataScrambling || settings.encryptData) ? "Encrypted " : "Unencrypted ");
    systemPrint("Frame ");
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

  //Display the received data bytes
  if ((settings.dataScrambling || settings.encryptData)
    && (settings.printRfData || settings.debugReceive))
  {
    systemPrintTimestamp();
    systemPrint("RX: Unencrypted Frame ");
    systemPrint(rxDataBytes);
    systemPrint(" (0x");
    systemPrint(rxDataBytes, HEX);
    systemPrintln(") bytes");
    petWDT();
    if (settings.printRfData && rxDataBytes)
      dumpBuffer(incomingBuffer, rxDataBytes);
  }

  //All packets must include the 2-byte control header
  if (rxDataBytes < minDatagramSize)
  {
    //Display the packet contents
    if (settings.printPktData || settings.debugReceive)
    {
      systemPrintTimestamp();
      systemPrint("RX: Bad Frame ");
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
      | Optional |          |  Optional  |             | Optional |
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
      | Optional |          |  Optional  |             | Optional |
      |  NET ID  | Control  | SF6 Length |    Data     | Trailer  |
      |  8 bits  |  8 bits  |   8 bits   |   n bytes   |  8 bits  |
      +----------+----------+------------+-------------+----------+
                 ^
                 |
                 '---- rxData
  */

  //Get the control byte
  rxControl = *((CONTROL_U8 *)rxData++);
  datagramType = rxControl.datagramType;
  uint8_t packetNumber = rxControl.ackNumber;
  if (settings.debugReceive)
    printControl(*((uint8_t *)&rxControl));
  if (datagramType >= MAX_DATAGRAM_TYPE)
    return (PACKET_BAD);

  /*
      |<---------------------- rxDataBytes ---------------------->|
      |                                                           |
      +----------+----------+------------+---  ...  ---+----------+
      | Optional |          |  Optional  |             | Optional |
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

  //Verify the packet number last so that the expected datagram or ACK number can be updated
  if (settings.pointToPoint)
  {
    switch (datagramType)
    {
      default:
        break;

      case DATAGRAM_DATA_ACK:
        if (packetNumber != expectedAckNumber)
        {
          if (settings.debugReceive)
          {
            systemPrintTimestamp();
            systemPrint("Invalid ACK number, received ");
            systemPrint(packetNumber);
            systemPrint(" expecting ");
            systemPrintln(expectedAckNumber);
          }
          return (PACKET_BAD);
        }

        //Increment the expected ACK number
        expectedAckNumber = (expectedAckNumber + 1) & 3;
        break;

      case DATAGRAM_DATA:
      case DATAGRAM_SF6_DATA:
      case DATAGRAM_REMOTE_COMMAND:
      case DATAGRAM_REMOTE_COMMAND_RESPONSE:
      case DATAGRAM_HEARTBEAT:
        if (packetNumber != expectedDatagramNumber)
        {
          //Determine if this is a duplicate datagram
          if (packetNumber == ((expectedDatagramNumber - 1) & 3))
          {
            linkDownTimer = millis();
            return PACKET_DUPLICATE;
          }

          //Not a duplicate
          if (settings.debugReceive)
          {
            systemPrintTimestamp();
            systemPrint("Invalid datagram number, received ");
            systemPrint(packetNumber);
            systemPrint(" expecting ");
            systemPrintln(expectedDatagramNumber);
          }
          return PACKET_BAD;
        }

        //Receive this data packet and set the next expected datagram number
        expectedDatagramNumber = (expectedDatagramNumber + 1) & 3;
        break;
    }
  }

  /*
                                         |<-- rxDataBytes -->|
                                         |                   |
      +----------+----------+------------+------  ...  ------+----------+
      | Optional |          |  Optional  |                   | Optional |
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
    systemPrint("RX: Datagram ");
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
          systemPrint(packetNumber);
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
      case DATAGRAM_HEARTBEAT:
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
      | Optional |          |  Optional  |             | Optional |
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
    systemPrint("TX: Datagram ");
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

  //Add the control byte
  control = *(uint8_t *)&txControl;
  *header++ = control;

  //Display the control value
  if (settings.debugTransmit)
    printControl(control);

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
      | Optional |          |  Optional  |             |
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
      | Optional |          |  Optional  |             | Optional |
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
    systemPrint("TX: Unencrypted Frame ");
    systemPrint(txDatagramSize);
    systemPrint(" (0x");
    systemPrint(txDatagramSize, HEX);
    systemPrintln(") bytes");
    petWDT();
    if (settings.printRfData)
      dumpBuffer(outgoingPacket, txDatagramSize);
  }

  //Encrypt the datagram
  if (settings.encryptData == true)
    encryptBuffer(outgoingPacket, txDatagramSize);

  //Scramble the datagram
  if (settings.dataScrambling == true)
    radioComputeWhitening(outgoingPacket, txDatagramSize);

  //Display the transmitted packet bytes
  if ((settings.printRfData || settings.debugTransmit)
    && (settings.encryptData || settings.dataScrambling))
  {
    systemPrintTimestamp();
    systemPrint("TX: Encrypted Frame ");
    systemPrint(txDatagramSize);
    systemPrint(" (0x");
    systemPrint(txDatagramSize, HEX);
    systemPrintln(") bytes");
    petWDT();
    if (settings.printRfData)
      dumpBuffer(outgoingPacket, txDatagramSize);
  }

  //If we are trainsmitting at high data rates the receiver is often not ready for new data. Pause for a few ms (measured with logic analyzer).
  if (settings.airSpeed == 28800 || settings.airSpeed == 38400)
    delay(2);

  //Reset the buffer data pointer for the next transmit operation
  endOfTxData = &outgoingPacket[headerBytes];

  //Compute the time needed for this packet. Part of ACK timeout.
  datagramAirTime = calcAirTime(txDatagramSize);

  //Transmit this datagram
  packetSent = 0; //This is the first time this packet is being sent
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
  if (control->datagramType < MAX_DATAGRAM_TYPE)
    systemPrintln(v2DatagramType[control->datagramType]);
  else
  {
    systemPrint("Unknown ");
    systemPrintln(control->datagramType);
  }
  petWDT();
}

//The previous transmission was not received, retransmit the datagram
void retransmitDatagram()
{
  /*
      +----------+----------+------------+---  ...  ---+----------+
      | Optional |          |  Optional  |             | Optional |
      |  NET ID  | Control  | SF6 Length |    Data     | Trailer  |
      |  8 bits  |  8 bits  |   8 bits   |   n bytes   |  8 bits  |
      +----------+----------+------------+-------------+----------+
      |                                                           |
      |<-------------------- txDatagramSize --------------------->|
  */

  //Display the transmitted packet bytes
  if (packetSent && (settings.printRfData || settings.debugTransmit))
  {
    systemPrintTimestamp();
    systemPrint("TX: Retransmit ");
    systemPrint((settings.encryptData || settings.dataScrambling) ? "Encrypted " : "Unencrypted ");
    systemPrint("Frame ");
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
    packetAirTime = calcAirTime(txDatagramSize); //Calculate packet air size while we're transmitting in the background
    uint16_t responseDelay = packetAirTime / responseDelayDivisor; //Give the receiver a bit of wiggle time to respond
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
  channelTimer.disableTimer();
  channelTimer.setInterval_MS(settings.maxDwellTime, channelTimerHandler);
  channelTimer.enableTimer();
  timerStart = millis(); //ISR normally takes care of this but allow for correct ACK sync before first ISR
  triggerEvent(TRIGGER_HOP_TIMER_START);
}

void stopChannelTimer()
{
  channelTimer.disableTimer();
  triggerEvent(TRIGGER_HOP_TIMER_STOP);
}

//Given the remote unit's amount of channelTimer that has elapsed,
//adjust our own channelTimer interrupt to be synchronized with the remote unit
void syncChannelTimer()
{
  triggerEvent(TRIGGER_SYNC_CHANNEL);

  uint16_t channelTimerElapsed;
  memcpy(&channelTimerElapsed, rxData, sizeof(channelTimerElapsed));
  channelTimerElapsed += ackAirTime;
  channelTimerElapsed += SYNC_PROCESSING_OVERHEAD;

  if (channelTimerElapsed > settings.maxDwellTime) channelTimerElapsed -= settings.maxDwellTime;

  partialTimer = true;
  channelTimer.disableTimer();
  channelTimer.setInterval_MS(settings.maxDwellTime - channelTimerElapsed, channelTimerHandler); //Shorten our hardware timer to match our mate's
  channelTimer.enableTimer();
}

//This function resets the heartbeat time and re-rolls the random time
//Call when something has happened (ACK received, etc) where clocks have been sync'd
void resetHeartbeat()
{
  heartbeatTimer = millis();
  heartbeatRandomTime = random(settings.heartbeatTimeout * 8 / 10, settings.heartbeatTimeout); //20-100%
}
