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
      |        |         | Optional |
      | NET ID | Control | Trailer  |
      | 8 bits | 8 bits  |  8 bits  |
      +--------+---------+----------+
  */

  if (settings.debugDatagrams)
  {
    systemPrintTimestamp();
    systemPrintln("TX: Ping");
  }
  txControl.datagramType = DATAGRAM_PING;
  transmitDatagram();
}

//Second packet in the three way handshake to bring up the link
void xmitDatagramP2PAck1()
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

  if (settings.debugDatagrams)
  {
    systemPrintTimestamp();
    systemPrintln("TX: Ack-1");
  }
  txControl.datagramType = DATAGRAM_ACK_1;
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
      |        |         | Optional |
      | NET ID | Control | Trailer  |
      | 8 bits | 8 bits  |  8 bits  |
      +--------+---------+----------+
  */

  if (settings.debugDatagrams)
  {
    systemPrintTimestamp();
    systemPrintln("TX: Ack-2");
  }
  txControl.datagramType = DATAGRAM_ACK_2;
  transmitDatagram();
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Point-to-Point Data Exchange
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Send a data packet to the remote system
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

  if (settings.debugTransmit)
  {
    systemPrintTimestamp();
    systemPrintln("TX: Data");
  }
  txControl.datagramType = DATAGRAM_DATA;
  transmitDatagram();
}

//Create short packet of 2 control bytes - do not expect ack
void xmitDatagramP2PAck()
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

  if (settings.debugTransmit)
  {
    systemPrintTimestamp();
    systemPrintln("TX: Ack");
  }
  txControl.datagramType = DATAGRAM_DATA_ACK;
  transmitDatagram();
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//Datagram reception
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Determine the type of datagram received
PacketType rcvDatagram()
{
  uint8_t ackNumber;
  CONTROL_U8 * control;
  uint8_t * data;
  PacketType datagramType;
  uint8_t incomingBuffer[MAX_PACKET_SIZE];
  uint8_t receivedNetID;

  //Get the received datagram
  radio.readData(incomingBuffer, MAX_PACKET_SIZE);
  uint8_t receivedBytes = radio.getPacketLength();
  data = incomingBuffer;

  /*
      |<--------------------- receivedBytes --------------------->|
      |                                                           |
      +----------+----------+------------+---  ...  ---+----------+
      | Optional | Optional |  Optional  |             | Optional |
      |  NET ID  | Control  | SF6 Length |    Data     | Trailer  |
      |  8 bits  |  8 bits  |   8 bits   |   n bytes   |  8 bits  |
      +----------+----------+------------+-------------+----------+
      ^
      |
      '---- data
  */

  //Display the received data bytes
  if (settings.printRfData || settings.debugReceive)
  {
    systemPrintln("----------");
    systemPrintTimestamp();
    systemPrint("RX: Data ");
    systemPrint(receivedBytes);
    systemPrint(" (0x");
    systemPrint(receivedBytes, HEX);
    systemPrintln(") bytes");
    petWDT();
    if (settings.printRfData && receivedBytes)
      dumpBuffer(incomingBuffer, receivedBytes);
  }

  if (settings.dataScrambling == true)
    radioComputeWhitening(incomingBuffer, receivedBytes);

  if (settings.encryptData == true)
    decryptBuffer(incomingBuffer, receivedBytes);

  //All packets must include the 2-byte control header
  if (receivedBytes < minDatagramSize)
  {
    //Display the packet contents
    if (settings.printPktData || settings.debugReceive)
    {
      systemPrintTimestamp();
      systemPrint("RX: Bad packet");
      systemPrint(receivedBytes);
      systemPrint(" (0x");
      systemPrint(receivedBytes, HEX);
      systemPrintln(") bytes");
      petWDT();
      if (settings.printRfData && receivedBytes)
          dumpBuffer(incomingBuffer, receivedBytes);
    }
    return (PACKET_BAD);
  }

  /*
      |<--------------------- receivedBytes --------------------->|
      |                                                           |
      +----------+----------+------------+---  ...  ---+----------+
      | Optional | Optional |  Optional  |             | Optional |
      |  NET ID  | Control  | SF6 Length |    Data     | Trailer  |
      |  8 bits  |  8 bits  |   8 bits   |   n bytes   |  8 bits  |
      +----------+----------+------------+-------------+----------+
      ^
      |
      '---- data
  */

  //Process the trailer

  //Verify the netID if necessary
  if (settings.pointToPoint || settings.verifyRxNetID)
  {
    receivedNetID = *data++;
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
        if (settings.printPktData && receivedBytes)
          dumpBuffer(incomingBuffer, receivedBytes);
        return (PACKET_NETID_MISMATCH);
      }
      systemPrintln();
    }
  }

  /*
      |<--------------------- receivedBytes --------------------->|
      |                                                           |
      +----------+----------+------------+---  ...  ---+----------+
      | Optional | Optional |  Optional  |             | Optional |
      |  NET ID  | Control  | SF6 Length |    Data     | Trailer  |
      |  8 bits  |  8 bits  |   8 bits   |   n bytes   |  8 bits  |
      +----------+----------+------------+-------------+----------+
                 ^
                 |
                 '---- data
  */

  //Get the control byte
  datagramType = DATAGRAM_DATA;
  if (settings.pointToPoint)
  {
    control = (CONTROL_U8 *)data++;
    datagramType = (PacketType)control->datagramType;
    if (settings.debugReceive)
      printControl(*(uint8_t *)control);
    if (datagramType >= MAX_DATAGRAM_TYPE)
      return (PACKET_BAD);

    //Verify the ACK number
    //                     txAckNumber ----> rxAckNumber == expectedTxAck
    //    expectedTxAck == rxAckNumber <---- txAckNumber
    ackNumber = control->ackNumber;
    if (expectedTxAck != ackNumber)
    {
      if (settings.debugReceive)
      {
        systemPrintTimestamp();
        systemPrint("Invalid ACK number, received ");
        systemPrint(ackNumber);
        systemPrint(" expecting ");
        systemPrintln(expectedTxAck);
      }
      return (PACKET_BAD);
    }
  }

  /*
      |<--------------------- receivedBytes --------------------->|
      |                                                           |
      +----------+----------+------------+---  ...  ---+----------+
      | Optional | Optional |  Optional  |             | Optional |
      |  NET ID  | Control  | SF6 Length |    Data     | Trailer  |
      |  8 bits  |  8 bits  |   8 bits   |   n bytes   |  8 bits  |
      +----------+----------+------------+-------------+----------+
                            ^
                            |
                            '---- data
  */

  //Get the spread factor 6 length
  if (datagramType == DATAGRAM_SF6_DATA)
  {
    if (receivedBytes >= (*data + minDatagramSize))
      receivedBytes = *data++;
    else
    {
      if (settings.debugReceive)
      {
        systemPrintTimestamp();
        systemPrint("Invalid SF6 length, received SF6 length");
        systemPrint(*data);
        systemPrint(" > ");
        systemPrint((int)receivedBytes - minDatagramSize);
        systemPrintln(" received bytes");
      }
      return (PACKET_BAD);
    }
  }
  receivedBytes -= minDatagramSize;

  //Verify the ACK number last so that the expected ACK number can be updated
  //                     txAckNumber ----> rxAckNumber == expectedTxAck
  //    expectedTxAck == rxAckNumber <---- txAckNumber
  if (settings.pointToPoint)
  {
    if (expectedTxAck != ackNumber)
    {
      if (settings.debugReceive)
      {
        systemPrintTimestamp();
        systemPrint("Invalid ACK number, received ");
        systemPrint(ackNumber);
        systemPrint(" expecting ");
        systemPrintln(expectedTxAck);
      }
      return (PACKET_BAD);
    }

    //Increment the expected ACK number
    expectedTxAck = (expectedTxAck + ((datagramsExpectingAcks & (1 << datagramType)) != 0)) & 3;
  }

  /*
                                         |<- receivedBytes ->|
                                         |                   |
      +----------+----------+------------+------  ...  ------+----------+
      | Optional | Optional |  Optional  |                   | Optional |
      |  NET ID  | Control  | SF6 Length |       Data        | Trailer  |
      |  8 bits  |  8 bits  |   8 bits   |      n bytes      |  8 bits  |
      +----------+----------+------------+-------------------+----------+
                                         ^
                                         |
                                         '---- data
  */

  //Display the packet contents
  if (settings.printPktData || settings.debugReceive)
  {
    systemPrintTimestamp();
    systemPrint("RX: Packet data ");
    systemPrint(receivedBytes);
    systemPrint(" (0x");
    systemPrint(receivedBytes, HEX);
    systemPrintln(") bytes");
    petWDT();
    if (settings.printPktData && receivedBytes)
      dumpBuffer(incomingBuffer, receivedBytes);
  }

  //Process the packet
  if (settings.debugDatagrams)
  {
    systemPrintTimestamp();
    systemPrint("RX: ");
    systemPrintln(v2DatagramType[datagramType]);
  }
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
    txControl.ackNumber = txAckNumber;
    txAckNumber = (txAckNumber + ((datagramsExpectingAcks & (1 << txControl.datagramType)) != 0)) & 3;
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

  setRadioFrequency(false); //Return home before every transmission

  int state = radio.startTransmit(outgoingPacket, txDatagramSize);
  if (state == RADIOLIB_ERR_NONE)
  {
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
    txDelay = settings.maxDwellTime;
  }
  else if (settings.debugTransmit)
  {
    systemPrintTimestamp();
    systemPrint("TX: Transmit error, state ");
    systemPrintln(state);
  }

  datagramTimer = millis(); //Move timestamp even if error
}

void startHopTimer()
{
  triggerEvent(TRIGGER_HOP_TIMER_START);
}

void stopHopTimer()
{
  triggerEvent(TRIGGER_HOP_TIMER_STOP);
}
