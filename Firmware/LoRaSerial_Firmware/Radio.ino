//Determine the type of packet received and print data as necessary
//The LoRa radio handles CRC and FHSS for us so we presume the packet was meant for us
//We check the packet netID for validity
//If the packet is a short/ping packet, ack
//If the packet is marked as resend, check if it's a duplicate
//Move any data to incomingPacket buffer
PacketType identifyPacketType()
{
  //Receive the data packet
  radio.readData(incomingBuffer, MAX_PACKET_SIZE);
  uint8_t receivedBytes = radio.getPacketLength();

  /*
        +-------------------------+------------+--------+---------+
        |                         |  Optional  |        |         |
        |          data           | SF6 length | NET ID | Control |
        |       0 - n bytes       |   8 bits   | 8 bits | 8 bits  |
        +-------------------------+------------+--------+---------+
        |                                                         |
        |<-------------------- receivedBytes -------------------->|
  */

  //Display the received data bytes
  if (settings.printRfData || settings.debugReceive)
  {
    petWDT();
    systemPrintln("----------");
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
  if (receivedBytes < 2)
  {
    //Display the packet contents
    if (settings.printPktData || settings.debugReceive)
    {
      petWDT();
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
        +-------------------------+------------+--------+---------+
        |                         |  Optional  |        |         |
        |          data           | SF6 length | NET ID | Control |
        |         n bytes         |   8 bits   | 8 bits | 8 bits  |
        +-------------------------+------------+--------+---------+
        |                                      |
        |<---------- receivedBytes ----------->|
  */

  //Pull out control header
  uint8_t receivedNetID = incomingBuffer[receivedBytes - 2];
  *(uint8_t *)&receiveTrailer = incomingBuffer[receivedBytes - 1];
  receivedBytes -= 2; //Remove control bytes

  //Display the control header
  if (settings.debugReceive)
  {
    petWDT();
    systemPrint("RX: NetID ");
    systemPrint(receivedNetID);
    systemPrint(" (0x");
    systemPrint(receivedNetID, HEX);
    systemPrintln(")");

    petWDT();
    systemPrint("RX: Control");
    systemPrint(" 0x");
    systemPrintln(*(uint8_t *)&receiveTrailer, HEX);
    if (*(uint8_t *)&receiveTrailer)
    {
      if (receiveTrailer.resend)                systemPrintln("    0x01: resend");
      if (receiveTrailer.ack)                   systemPrintln("    0x02: ack");
      if (receiveTrailer.remoteCommand)         systemPrintln("    0x04: remoteCommand");
      if (receiveTrailer.remoteCommandResponse) systemPrintln("    0x08: remoteCommandResponse");
      if (receiveTrailer.train)                 systemPrintln("    0x10: train");
    }
    petWDT();
  }

  /*
        +-------------------------+------------+
        |                         |  Optional  |
        |          data           | SF6 length |
        |         n bytes         |   8 bits   |
        +-------------------------+------------+
        |                                      |
        |<---------- receivedBytes ----------->|
  */

  //SF6 requires an implicit header which means there is no dataLength in the header
  //Instead, we manually store it after the data and before NetID
  if (settings.radioSpreadFactor == 6)
  {
    //We've either received a control packet (2 bytes) or a data packet
    if (receivedBytes)
    {
      receivedBytes -= 1; //Remove the manual packetSize byte from consideration
      receivedBytes = incomingBuffer[receivedBytes]; //Obtain actual packet data length
    }
  }

  /*
        +-------------------------+
        |          data           |
        |         n bytes         |
        +-------------------------+
        |                         |
        |<---- receivedBytes ---->|
  */

  //Display the packet contents
  if (settings.printPktData || settings.debugReceive)
  {
    petWDT();
    systemPrint("RX: Packet data ");
    systemPrint(receivedBytes);
    systemPrint(" (0x");
    systemPrint(receivedBytes, HEX);
    systemPrintln(") bytes");
    petWDT();
    if (settings.printPktData && receivedBytes)
      dumpBuffer(incomingBuffer, receivedBytes);
  }

  if ((receivedNetID != settings.netID)
      && ((settings.pointToPoint == true) || (settings.verifyRxNetID == true)))
  {
    if (settings.debugReceive)
    {
      petWDT();
      systemPrint("RX: netID ");
      systemPrint(receivedNetID);
      systemPrint(" expecting ");
      systemPrintln(settings.netID);
    }
    return (PACKET_NETID_MISMATCH);
  }

  //----------
  //Handle ACKs and duplicate packets
  //----------

  if ((receiveTrailer.ack == 1)
      && (receiveTrailer.remoteCommand == 0)
      && (receiveTrailer.remoteCommandResponse == 0))
  {
    if (settings.debugReceive)
    {
      petWDT();
      systemPrintln("RX: Ack packet");
    }
    return (PACKET_ACK);
  }

  if (receiveTrailer.resend == 1)
  {
    //We've received a packet that is marked as a retransmit
    //Determine if this is a duplicate of a previous packet
    if (receivedBytes == lastPacketSize)
    {
      //Check packet contents
      if (memcmp(lastPacket, incomingBuffer, lastPacketSize) == 0)
      {
        if (settings.debugReceive)
        {
          petWDT();
          systemPrintln("RX: Duplicate received. Acking again.");
        }
        return (PACKET_DUPLICATE); //It's a duplicate. Ack then ignore
      }
    }
    else
    {
      //We've received a resend but it's different from the lastPacket
      //Go ahead and print it
    }
  }

  //----------
  //Handle control packets
  //----------

  //We have empty data packet, this is a control packet used for pinging/scanning
  if (receivedBytes == 0)
  {
    //If this packet is marked as training data, someone is sending training ping
    if (receiveTrailer.train == 1)
    {
      if (settings.debugReceive)
      {
        petWDT();
        systemPrintln("RX: Training Control Packet");
      }
      return (PACKET_TRAINING_PING);
    }

    //If this packet is marked as a remote command, it's either an ack or a zero length packet (not known)
    if (receiveTrailer.remoteCommand == 1)
    {
      if (receiveTrailer.ack == 1)
      {
        if (settings.debugReceive)
        {
          petWDT();
          systemPrintln("RX: Command Ack");
        }
        return (PACKET_COMMAND_ACK);
      }

      if (settings.debugReceive)
      {
        petWDT();
        systemPrintln("RX: Unknown Command");
      }
      return (PACKET_BAD);
    }

    //If this packet is marked as a remote command response, it's either an ack or a zero length packet (not known)
    if (receiveTrailer.remoteCommandResponse == 1)
    {
      if (receiveTrailer.ack == 1)
      {
        if (settings.debugReceive)
        {
          petWDT();
          systemPrintln("RX: Command Response Ack");
        }
        return (PACKET_COMMAND_RESPONSE_ACK);
      }

      if (settings.debugReceive)
      {
        petWDT();
        systemPrintln("RX: Unknown Response Command");
      }
      return (PACKET_BAD);
    }

    //Not training, not command packet, just a ping
    if (settings.debugReceive)
    {
      petWDT();
      systemPrintln("RX: Control Packet");
    }
    return (PACKET_PING);
  }

  //----------
  //Handle data packets
  //----------

  //Update lastPacket details with current packet
  memcpy(lastPacket, incomingBuffer, receivedBytes);
  lastPacketSize = receivedBytes;

  //If this packet is marked as training data,
  //payload contains new AES key and netID which will be processed externally
  if (receiveTrailer.train == 1)
  {
    if (settings.debugReceive)
    {
      petWDT();
      systemPrintln("RX: Training Data");
    }
    return (PACKET_TRAINING_DATA);
  }

  if (receiveTrailer.remoteCommand == 1)
  {
    //New data from remote
    if (settings.debugReceive)
    {
      petWDT();
      systemPrintln("RX: Command Data");
    }
    return (PACKET_COMMAND_DATA);
  }

  if (receiveTrailer.remoteCommandResponse == 1)
  {
    //New response data from remote
    if (settings.debugReceive)
    {
      petWDT();
      systemPrintln("RX: Command Response Data");
    }
    return (PACKET_COMMAND_RESPONSE_DATA);
  }

  //Return the data to the user
  if (settings.debugReceive)
  {
    petWDT();
    systemPrintln("RX: Return user data");
  }
  return (PACKET_DATA);
}

//Apply settings to radio
//Called after begin() and once user exits from command interface
void configureRadio()
{
  bool success = true;

  //Determine if we are using AirSpeed or custom settings
  if (settings.airSpeed != 0)
  {
    switch (settings.airSpeed)
    {
      case (0):
        //Custom settings - use settings without modification
        break;
      case (40):
        settings.radioSpreadFactor = 11;
        settings.radioBandwidth = 62.5;
        settings.radioCodingRate = 8;
        break;
      case (150):
        settings.radioSpreadFactor = 10;
        settings.radioBandwidth = 62.5;
        settings.radioCodingRate = 8;
        break;
      case (400):
        settings.radioSpreadFactor = 10;
        settings.radioBandwidth = 125;
        settings.radioCodingRate = 8;
        break;
      case (1200):
        settings.radioSpreadFactor = 9;
        settings.radioBandwidth = 125;
        settings.radioCodingRate = 8;
        break;
      case (2400):
        settings.radioSpreadFactor = 10;
        settings.radioBandwidth = 500;
        settings.radioCodingRate = 8;
        break;
      case (4800):
        settings.radioSpreadFactor = 9;
        settings.radioBandwidth = 500;
        settings.radioCodingRate = 8;
        break;
      case (9600):
        settings.radioSpreadFactor = 8;
        settings.radioBandwidth = 500;
        settings.radioCodingRate = 7;
        break;
      case (19200):
        settings.radioSpreadFactor = 7;
        settings.radioBandwidth = 500;
        settings.radioCodingRate = 7;
        break;
      case (28800):
        settings.radioSpreadFactor = 6;
        settings.radioBandwidth = 500;
        settings.radioCodingRate = 6;
        break;
      case (38400):
        settings.radioSpreadFactor = 6;
        settings.radioBandwidth = 500;
        settings.radioCodingRate = 5;
        break;
      default:
        if ((settings.debug == true) || (settings.debugRadio == true))
        {
          systemPrint("Unknown airSpeed: ");
          systemPrintln(settings.airSpeed);
        }
        break;
    }
  }

  if (radio.setFrequency(channels[0]) == RADIOLIB_ERR_INVALID_FREQUENCY)
    success = false;

  channelNumber = 0;

  //The SX1276 and RadioLib accepts a value of 2 to 17, with 20 enabling the power amplifier
  //Measuring actual power output the radio will output 14dBm (25mW) to 27.9dBm (617mW) in constant transmission
  //So we use a lookup to convert between the user's chosen power and the radio setting
  int radioPowerSetting = covertdBmToSetting(settings.radioBroadcastPower_dbm);
  if (radio.setOutputPower(radioPowerSetting) == RADIOLIB_ERR_INVALID_OUTPUT_POWER)
    success = false;

  if (radio.setBandwidth(settings.radioBandwidth) == RADIOLIB_ERR_INVALID_BANDWIDTH)
    success = false;

  if (radio.setSpreadingFactor(settings.radioSpreadFactor) == RADIOLIB_ERR_INVALID_SPREADING_FACTOR)
    success = false;

  if (radio.setCodingRate(settings.radioCodingRate) == RADIOLIB_ERR_INVALID_CODING_RATE)
    success = false;

  if (radio.setSyncWord(settings.radioSyncWord) != RADIOLIB_ERR_NONE)
    success = false;

  if (radio.setPreambleLength(settings.radioPreambleLength) == RADIOLIB_ERR_INVALID_PREAMBLE_LENGTH)
    success = false;

  //SF6 requires an implicit header. We will transmit 255 bytes for most packets and 2 bytes for ACK packets.
  if (settings.radioSpreadFactor == 6)
  {
    if (radio.implicitHeader(MAX_PACKET_SIZE) != RADIOLIB_ERR_NONE)
      success = false;
  }
  else
  {
    if (radio.explicitHeader() != RADIOLIB_ERR_NONE)
      success = false;
  }

  radio.setDio0Action(dio0ISR); //Called when transmission is finished
  radio.setDio1Action(dio1ISR); //Called after a transmission has started, so we can move to next freq

  if (pin_rxen != PIN_UNDEFINED)
    radio.setRfSwitchPins(pin_rxen, pin_txen);

  // HoppingPeriod = Tsym * FreqHoppingPeriod
  // Given defaults of spreadfactor = 9, bandwidth = 125, it follows Tsym = 4.10ms
  // HoppingPeriod = 4.10 * x = Yms. Can be as high as 400ms to be within regulatory limits
  uint16_t hoppingPeriod = settings.maxDwellTime / calcSymbolTime(); //Limit FHSS dwell time to 400ms max. / automatically floors number
  if (hoppingPeriod > 255) hoppingPeriod = 255; //Limit to 8 bits.
  if (settings.frequencyHop == false) hoppingPeriod = 0; //Disable
  if (radio.setFHSSHoppingPeriod(hoppingPeriod) != RADIOLIB_ERR_NONE)
    success = false;

  controlPacketAirTime = calcAirTime(2); //Used for response timeout during RADIO_LINKED_ACK_WAIT
  uint16_t responseDelay = controlPacketAirTime / responseDelayDivisor; //Give the receiver a bit of wiggle time to respond
  controlPacketAirTime += responseDelay;

  //Precalculate the ACK packet time
  ackAirTime = calcAirTime(4); //We assume all ACKs have max of 4 bytes

  if ((settings.debug == true) || (settings.debugRadio == true))
  {
    systemPrint("Freq: ");
    systemPrintln(channels[0], 3);
    systemPrint("radioBandwidth: ");
    systemPrintln(settings.radioBandwidth);
    systemPrint("radioSpreadFactor: ");
    systemPrintln(settings.radioSpreadFactor);
    systemPrint("radioCodingRate: ");
    systemPrintln(settings.radioCodingRate);
    systemPrint("radioSyncWord: ");
    systemPrintln(settings.radioSyncWord);
    systemPrint("radioPreambleLength: ");
    systemPrintln(settings.radioPreambleLength);
    systemPrint("calcSymbolTime: ");
    systemPrintln(calcSymbolTime());
    systemPrint("HoppingPeriod: ");
    systemPrintln(hoppingPeriod);
    systemPrint("controlPacketAirTime: ");
    systemPrintln(controlPacketAirTime);
    systemPrint("ackAirTime: ");
    systemPrintln(ackAirTime);
  }

  if (success == false)
  {
    reportERROR();
    systemPrintln("Radio init failed. Check settings.");
  }
  if ((settings.debug == true) || (settings.debugRadio == true))
    systemPrintln("Radio configured");
}

//Set radio frequency
void setRadioFrequency(bool rxAdjust)
{
  float frequency;

  frequency = channels[channelNumber];
  if (rxAdjust)
    frequency -= frequencyCorrection;
  if (settings.printFrequency)
  {
    systemPrintTimestamp();
    systemPrint(frequency);
    systemPrintln(" MHz");
  }
  radio.setFrequency(frequency);
}

void returnToReceiving()
{
  int state;
  if (settings.radioSpreadFactor > 6)
  {
    state = radio.startReceive();
  }
  else
  {
    if (expectingAck && settings.pointToPoint == true)
    {
      radio.implicitHeader(2);
      state = radio.startReceive(2); //Expect a control packet
      triggerEvent(TRIGGER_RTR_2BYTE);
      expectingAck = false; //Do not return to this receiving configuration if something goes wrong
    }
    else
    {
      radio.implicitHeader(MAX_PACKET_SIZE);
      state = radio.startReceive(MAX_PACKET_SIZE); //Expect a full data packet
      triggerEvent(TRIGGER_RTR_255BYTE);
    }
  }

  if (state != RADIOLIB_ERR_NONE) {
  if ((settings.debug == true) || (settings.debugRadio == true))
    {
      systemPrint("Receive failed: ");
      systemPrintln(state);
    }
  }
}

//Create short packet of 2 control bytes - query remote radio for proof of life (ack)
void sendPingPacket()
{
  /*
      +--------+---------+
      | NET ID | Trailer |
      | 8 bits | 8 bits  |
      +--------+---------+
      |                  |
      |<-- packetSize -->|
  */

  if (settings.debugDatagrams)
    systemPrintln("TX: Ping ");
  responseTrailer.ack = 0; //This is not an ACK to a previous transmission
  responseTrailer.resend = 0; //This is not a resend
  responseTrailer.train = 0; //This is not a training packet
  responseTrailer.remoteCommand = 0; //This is not a remote command packet
  responseTrailer.remoteCommandResponse = 0; //This is not a response to a previous remote command

  packetSize = 2;
  packetSent = 0; //Reset the number of times we've sent this packet

  //SF6 requires an implicit header which means there is no dataLength in the header
  //Because we cannot predict when a ping packet will be received, the receiver will always
  //expecting 255 bytes. Pings must be increased to 255 bytes. ACKs are still 2 bytes.
  if (settings.radioSpreadFactor == 6)
  {
    //Manually store actual data length 3 bytes from the end (before NetID)
    //Manual packet size is whatever has been processed + 1 for the manual packetSize byte
    outgoingPacket[maxDatagramSize] = packetSize + 1;
    packetSize = MAX_PACKET_SIZE; //We're now going to transmit 255 bytes
  }

  expectingAck = true; //We expect destination to ack
  sendPacket();
}

//Create packet of current data + control bytes - expect ACK from recipient
void sendDataPacket()
{
  /*
      +---  ...  ---+--------+---------+
      |    Data     | NET ID | Trailer |
      |   n bytes   | 8 bits | 8 bits  |
      +-------------+--------+---------+
      |                                |
      |<--------- packetSize --------->|
  */

  if (settings.debugDatagrams)
    systemPrintln("TX: Data ");
  responseTrailer.ack = 0; //This is not an ACK to a previous transmission
  responseTrailer.resend = 0; //This is not a resend
  responseTrailer.train = 0; //This is not a training packet
  responseTrailer.remoteCommand = 0; //This is not a remote command packet
  responseTrailer.remoteCommandResponse = 0; //This is not a response to a previous remote command

  packetSize += 2; //Make room for control bytes
  packetSent = 0; //Reset the number of times we've sent this packet

  //SF6 requires an implicit header which means there is no dataLength in the header
  if (settings.radioSpreadFactor == 6)
  {
    //Manually store actual data length 3 bytes from the end (before NetID)
    //Manual packet size is whatever has been processed + 1 for the manual packetSize byte
    outgoingPacket[maxDatagramSize] = packetSize + 1;
    packetSize = MAX_PACKET_SIZE; //We're now going to transmit 255 bytes
  }

  expectingAck = true; //We expect destination to ack
  sendPacket();
}

//Create packet of current data + control bytes - expect ACK from recipient
void sendResendPacket()
{
  /*
      +---  ...  ---+--------+---------+
      |    Data     | NET ID | Trailer |
      |   n bytes   | 8 bits | 8 bits  |
      +-------------+--------+---------+
      |                                |
      |<--------- packetSize --------->|
  */

  if (settings.debugDatagrams)
    systemPrintln("TX: Resend ");
  responseTrailer.ack = 0; //This is not an ACK to a previous transmission
  responseTrailer.resend = 1; //This is a resend
  responseTrailer.train = 0; //This is not a training packet
  //responseTrailer.remoteCommand = ; //Don't modify the remoteCommand bit; we may be resending a command packet
  //responseTrailer.remoteCommandResponse = ; //Don't modify the remoteCommand bit; we may be resending a command packet

  //packetSize += 2; //Don't adjust the packet size
  //packetSent = 0; //Don't reset
  expectingAck = true; //We expect destination to ack
  sendPacket();
}

//Create short packet of 2 control bytes - do not expect ack
void sendAckPacket()
{
  /*
      +--------+---------+
      | NET ID | Trailer |
      | 8 bits | 8 bits  |
      +--------+---------+
      |                  |
      |<-- packetSize -->|
  */

  if (settings.debugDatagrams)
    systemPrintln("TX: Ack ");
  responseTrailer.ack = 1; //This is an ACK to a previous reception
  responseTrailer.resend = 0; //This is not a resend
  responseTrailer.train = 0; //This is not a training packet
  responseTrailer.remoteCommand = 0; //This is not a remote command packet
  responseTrailer.remoteCommandResponse = 0; //This is not a response to a previous remote command

  packetSize = 2;
  packetSent = 0; //Reset the number of times we've sent this packet
  expectingAck = false; //We do not expect destination to ack
  sendPacket();
}

//Create short packet of 2 control bytes with train = 1
void sendTrainingPingPacket()
{
  /*
      +--------+---------+
      | NET ID | Trailer |
      | 8 bits | 8 bits  |
      +--------+---------+
      |                  |
      |<-- packetSize -->|
  */

  if (settings.debugDatagrams)
    systemPrintln("TX: Training Ping ");
  responseTrailer.ack = 0; //This is not an ACK to a previous transmission
  responseTrailer.resend = 0; //This is not a resend
  responseTrailer.train = 1; //This is a training packet
  responseTrailer.remoteCommand = 0; //This is not a remote command packet
  responseTrailer.remoteCommandResponse = 0; //This is not a response to a previous remote command

  packetSize = 2;
  packetSent = 0; //Reset the number of times we've sent this packet

  //SF6 is not used during training

  expectingAck = true; //We expect destination to ack
  sendPacket();
}

//Create packet of AES + netID with training = 1
void sendTrainingDataPacket()
{
  /*
      +----------------+------------+--------+---------+
      | Encryption Key | New NET ID | NET ID | Trailer |
      |    16 bytes    |   8 bits   | 8 bits | 8 bits  |
      +----------------+------------+--------+---------+
      |                                                |
      |<----------------- packetSize ----------------->|
  */

  if (settings.debugDatagrams)
    systemPrintln("TX: Training Data ");
  responseTrailer.ack = 0; //This is not an ACK to a previous transmission
  responseTrailer.resend = 0; //This is not a resend
  responseTrailer.train = 1; //This is training packet
  responseTrailer.remoteCommand = 0; //This is not a remote command packet
  responseTrailer.remoteCommandResponse = 0; //This is not a response to a previous remote command

  packetSize = sizeof(trainEncryptionKey) + sizeof(trainNetID);

  for (uint8_t x = 0 ; x < sizeof(trainEncryptionKey) ; x++)
    outgoingPacket[x] = trainEncryptionKey[x];
  outgoingPacket[packetSize - 1] = trainNetID;

  packetSize += 2;
  packetSent = 0; //Reset the number of times we've sent this packet

  //During training we do not use spread factor 6

  expectingAck = false; //We do not expect destination to ack
  sendPacket();
}

//Create short packet of 2 control bytes - do not expect ack
void sendCommandAckPacket()
{
  /*
      +--------+---------+
      | NET ID | Trailer |
      | 8 bits | 8 bits  |
      +--------+---------+
      |                  |
      |<-- packetSize -->|
  */

  if (settings.debugDatagrams)
    systemPrintln("TX: Command Ack ");
  responseTrailer.ack = 1; //This is an ACK to a previous reception
  responseTrailer.resend = 0; //This is not a resend
  responseTrailer.train = 0; //This is not a training packet
  responseTrailer.remoteCommand = 1; //This is a remote command packet
  responseTrailer.remoteCommandResponse = 0; //This is not a response to a previous command

  packetSize = 2;
  packetSent = 0; //Reset the number of times we've sent this packet
  expectingAck = false; //We do not expect destination to ack
  sendPacket();
}

//Create packet of serial command with remote command = 1, ack = 0
void sendCommandDataPacket()
{
  /*
      +---------+--------+---------+
      | Command | NET ID | Trailer |
      | n bytes | 8 bits | 8 bits  |
      +---------+--------+---------+
      |                            |
      |<------- packetSize ------->|
  */

  if (settings.debugDatagrams)
    systemPrintln("TX: Command Data ");
  responseTrailer.ack = 0; //This is not an ACK to a previous transmission.
  responseTrailer.resend = 0; //This is not a resend
  responseTrailer.train = 0; //This is not training packet
  responseTrailer.remoteCommand = 1; //This is a remote control packet
  responseTrailer.remoteCommandResponse = 0; //This is not a response to a previous command

  packetSize += 2; //Make room for control bytes
  packetSent = 0; //Reset the number of times we've sent this packet

  //SF6 requires an implicit header which means there is no dataLength in the header
  if (settings.radioSpreadFactor == 6)
  {
    //Manually store actual data length 3 bytes from the end (before NetID)
    //Manual packet size is whatever has been processed + 1 for the manual packetSize byte
    outgoingPacket[maxDatagramSize] = packetSize + 1;
    packetSize = MAX_PACKET_SIZE; //We're now going to transmit 255 bytes
  }

  expectingAck = true; //We expect destination to ack
  sendPacket();
}

//Create short packet of 2 control bytes - do not expect ack
void sendCommandResponseAckPacket()
{
  /*
      +--------+---------+
      | NET ID | Trailer |
      | 8 bits | 8 bits  |
      +--------+---------+
      |                  |
      |<-- packetSize -->|
  */

  if (settings.debugDatagrams)
    systemPrintln("TX: Command Response Ack ");
  responseTrailer.ack = 1; //This is an ACK to a previous reception
  responseTrailer.resend = 0; //This is not a resend
  responseTrailer.train = 0; //This is not a training packet
  responseTrailer.remoteCommand = 0; //This is not a remote command packet
  responseTrailer.remoteCommandResponse = 1; //This is a response to a previous command

  packetSize = 2;
  packetSent = 0; //Reset the number of times we've sent this packet
  expectingAck = false; //We do not expect destination to ack
  sendPacket();
}

//Create packet of serial command with remote command = 1, ack = 0
void sendCommandResponseDataPacket()
{
  /*
      +----------+--------+---------+
      | Response | NET ID | Trailer |
      | n bytes  | 8 bits | 8 bits  |
      +----------+--------+---------+
      |                             |
      |<-------- packetSize ------->|
  */

  if (settings.debugDatagrams)
    systemPrintln("TX: Command Response Data ");
  responseTrailer.ack = 0; //This is not an ACK to a previous transmission.
  responseTrailer.resend = 0; //This is not a resend
  responseTrailer.train = 0; //This is not training packet
  responseTrailer.remoteCommand = 0; //This is a remote control packet
  responseTrailer.remoteCommandResponse = 1; //This is a response to a previous command

  packetSize += 2; //Make room for control bytes
  packetSent = 0; //Reset the number of times we've sent this packet

  //SF6 requires an implicit header which means there is no dataLength in the header
  if (settings.radioSpreadFactor == 6)
  {
    //Manually store actual data length 3 bytes from the end (before NetID)
    //Manual packet size is whatever has been processed + 1 for the manual packetSize byte
    outgoingPacket[maxDatagramSize] = packetSize + 1;
    packetSize = MAX_PACKET_SIZE; //We're now going to transmit 255 bytes
  }

  expectingAck = true; //We expect destination to ack
  sendPacket();
}

//Push the outgoing packet to the air
void sendPacket()
{
  /*
      +---  ...  ---+------------+--------+---------+
      |             |  Optional  |        |         |
      |    Data     | SF6 Length | NET ID | Trailer |
      |   n bytes   |   8 bits   | 8 bits | 8 bits  |
      +-------------+------------+--------+---------+
      |                                             |
      |<--------------- packetSize ---------------->|
  */

  //Attach netID and control byte to end of packet
  outgoingPacket[packetSize - 2] = settings.netID;
  outgoingPacket[packetSize - 1] = *(uint8_t *)&responseTrailer;

  //Display the packet contents
  if (settings.printPktData || settings.debugTransmit)
  {
    petWDT();
    systemPrint("TX: Control");
    systemPrint(" 0x");
    systemPrintln(*(uint8_t *)&receiveTrailer, HEX);
    if (*(uint8_t *)&receiveTrailer)
    {
      if (receiveTrailer.resend)                systemPrintln("    0x01: resend");
      if (receiveTrailer.ack)                   systemPrintln("    0x02: ack");
      if (receiveTrailer.remoteCommand)         systemPrintln("    0x04: remoteCommand");
      if (receiveTrailer.remoteCommandResponse) systemPrintln("    0x08: remoteCommandResponse");
      if (receiveTrailer.train)                 systemPrintln("    0x10: train");
    }
    petWDT();
    systemPrint("TX: Packet data ");
    systemPrint(packetSize);
    systemPrint(" (0x");
    systemPrint(packetSize, HEX);
    systemPrintln(") bytes");
    petWDT();
    if (settings.printPktData)
      dumpBuffer(outgoingPacket, packetSize);
  }

  //Apply AES and whitening only to new packets, not resends
  if (responseTrailer.resend == 0)
  {
    if (settings.encryptData == true)
      encryptBuffer(outgoingPacket, packetSize);

    if (settings.dataScrambling == true)
      radioComputeWhitening(outgoingPacket, packetSize);
  }

  //Display the transmitted packet bytes
  if (settings.printPktData || settings.debugTransmit)
  {
    petWDT();
    systemPrint("TX: Data ");
    systemPrint(packetSize);
    systemPrint(" (0x");
    systemPrint(packetSize, HEX);
    systemPrintln(") bytes");
    petWDT();
    if (settings.printPktData)
      dumpBuffer(outgoingPacket, packetSize);
  }

  //If we are trainsmitting at high data rates the receiver is often not ready for new data. Pause for a few ms (measured with logic analyzer).
  if (settings.airSpeed == 28800 || settings.airSpeed == 38400)
    delay(2);

  setRadioFrequency(false); //Return home before every transmission

  //Display the transmit frequency
  if (settings.debugTransmit)
  {
    systemPrint("TX: Transmitting @ ");
    systemPrint(channels[channelNumber], 3);
    systemPrintln(" MHz");
  }

  int state = radio.startTransmit(outgoingPacket, packetSize);
  if (state == RADIOLIB_ERR_NONE)
  {
    if (timeToHop) hopChannel();

    packetAirTime = calcAirTime(packetSize); //Calculate packet air size while we're transmitting in the background
    uint16_t responseDelay = packetAirTime / responseDelayDivisor; //Give the receiver a bit of wiggle time to respond
    packetAirTime += responseDelay;

    packetSent++;

    if (settings.debugTransmit)
    {
      systemPrint("TX: PacketAirTime ");
      systemPrintln(packetAirTime);
      systemPrint("TX: responseDelay: ");
      systemPrintln(responseDelay);
    }

    if (timeToHop) hopChannel();
  }
  else
  {
    if (settings.debugTransmit || settings.printTxErrors)
      systemPrintln("Error: TX");
  }

  packetTimestamp = millis(); //Move timestamp even if error
}

//ISR when DIO0 goes low
//Called when transmission is complete or when RX is received
void dio0ISR(void)
{
  transactionComplete = true;
}

//ISR when DIO1 goes low
//Called when FhssChangeChannel interrupt occurs (at regular HoppingPeriods)
//We do not use SX based channel hopping, and instead use a synchronized hardware timer
void dio1ISR(void)
{
  clearDIO1 = true;
}

//Given spread factor, bandwidth, coding rate and number of bytes, return total Air Time in ms for packet
uint16_t calcAirTime(uint8_t bytesToSend)
{
  float tSym = calcSymbolTime();
  float tPreamble = (settings.radioPreambleLength + 4.25) * tSym;
  float p1 = (8 * bytesToSend - 4 * settings.radioSpreadFactor + 28 + 16 * 1 - 20 * 0) / (4.0 * (settings.radioSpreadFactor - 2 * 0));
  p1 = ceil(p1) * settings.radioCodingRate;
  if (p1 < 0) p1 = 0;
  uint16_t payloadBytes = 8 + p1;
  float tPayload = payloadBytes * tSym;
  float tPacket = tPreamble + tPayload;

  return ((uint16_t)ceil(tPacket));
}

//Given spread factor and bandwidth, return symbol time
float calcSymbolTime()
{
  float tSym = pow(2, settings.radioSpreadFactor) / settings.radioBandwidth;
  return (tSym);
}

//Given spread factor, bandwidth, coding rate and frame size, return most bytes we can push per second
uint16_t calcMaxThroughput()
{
  uint8_t mostFramesPerSecond = 1000 / calcAirTime(settings.frameSize);
  uint16_t mostBytesPerSecond = settings.frameSize * mostFramesPerSecond;

  return (mostBytesPerSecond);
}

uint16_t myRandSeed;
bool myRandBit;

//Generate unique hop table based on radio settings
void generateHopTable()
{
  if (channels != NULL) free(channels);
  channels = (float *)malloc(settings.numberOfChannels * sizeof(float));

  float channelSpacing = (settings.frequencyMax - settings.frequencyMin) / (float)(settings.numberOfChannels + 2);

  //Keep away from edge of available spectrum
  float operatingMinFreq = settings.frequencyMin + (channelSpacing / 2);

  //Pre populate channel list
  for (int x = 0 ; x < settings.numberOfChannels ; x++)
    channels[x] = operatingMinFreq + (x * channelSpacing);

  //Feed random number generator with our specific platform settings
  //Use settings that must be identical to have a functioning link.
  //For example, we do not use coding rate because two radios can communicate with different coding rate values
  myRandSeed = settings.airSpeed + settings.netID + settings.pointToPoint + settings.encryptData
               + settings.dataScrambling
               + (uint16_t)settings.frequencyMin + (uint16_t)settings.frequencyMax
               + settings.numberOfChannels + settings.frequencyHop + settings.maxDwellTime
               + (uint16_t)settings.radioBandwidth + settings.radioSpreadFactor;

  if (settings.encryptData == true)
  {
    for (uint8_t x = 0 ; x < sizeof(settings.encryptionKey) ; x++)
      myRandSeed += settings.encryptionKey[x];
  }

  //'Randomly' shuffle list based on our specific seed
  shuffle(channels, settings.numberOfChannels);

  //Verify the AES IV length
  if (AES_IV_BYTES != gcm.ivSize())
  {
    systemPrint("ERROR - Wrong AES IV size in bytes, please set AES_IV_BYTES = ");
    systemPrintln(gcm.ivSize());
    while (1)
      petWDT();
  }

  //Verify the AES key length
  if (AES_KEY_BYTES != gcm.keySize())
  {
    systemPrint("ERROR - Wrong AES key size in bytes, please set AES_KEY_BYTES = ");
    systemPrintln(gcm.keySize());
    while (1)
      petWDT();
  }

  //Set new initial values for AES using settings based random seed
  for (uint8_t x = 0 ; x < sizeof(AESiv) ; x++)
    AESiv[x] = myRand();

  if ((settings.debug == true) || (settings.debugRadio == true))
  {
    systemPrint("channelSpacing: ");
    systemPrintln(channelSpacing, 3);

    systemPrintln("Channel table:");
    for (int x = 0 ; x < settings.numberOfChannels ; x++)
    {
      systemPrint(x);
      systemPrint(": ");
      systemPrint(channels[x], 3);
      systemPrintln();
    }

    systemPrint("AES IV:");
    for (uint8_t i = 0 ; i < sizeof(AESiv) ; i++)
    {
      systemPrint(" 0x");
      systemPrint(AESiv[i], HEX);
    }
    systemPrintln();
  }
}

//Array shuffle from http://benpfaff.org/writings/clc/shuffle.html
void shuffle(float *array, uint8_t n)
{
  for (uint8_t i = 0; i < n - 1; i++)
  {
    uint8_t j = myRand() % n;
    float t = array[j];
    array[j] = array[i];
    array[i] = t;
  }
}

//Simple lfsr randomizer. Needed because we cannot guarantee how random()
//will respond on different Arduino platforms. ie Uno acts diffrently from ESP32.
//We don't need 'truly random', we need repeatable across platforms.
uint16_t myRand()
{
  myRandBit = ((myRandSeed >> 0) ^ (myRandSeed >> 2) ^ (myRandSeed >> 3) ^ (myRandSeed >> 5) ) & 1;
  return myRandSeed = (myRandSeed >> 1) | (myRandBit << 15);
}

//If we have lost too many packets
//or have not had a successful heart beat within a determined amount of time
//the link is considered lost
bool linkLost()
{
  if (packetsLost > MAX_LOST_PACKET_BEFORE_LINKLOST)
    return (true);
  return (false);
}

//Move to the next channel
//This is called when the FHSS interrupt is received
//at the beginning and during of a transmission or reception
void hopChannel()
{
  timeToHop = false;
  triggerEvent(TRIGGER_FREQ_CHANGE);

  channelNumber++;
  channelNumber %= settings.numberOfChannels;

  //Select the new frequency
  float frequency;
  if (settings.autoTuneFrequency == true)
  {
    if (radioState == RADIO_LINKED_RECEIVING_STANDBY || radioState == RADIO_LINKED_ACK_WAIT
        || radioState == RADIO_BROADCASTING_RECEIVING_STANDBY) //Only adjust frequency on RX. Not TX.
      frequency = channels[channelNumber] - frequencyCorrection;
    else
      frequency = channels[channelNumber];
  }
  else
    frequency = channels[channelNumber];

  radio.setFrequency(frequency);

  //Print the frequency if requested
  if (settings.printFrequency)
  {
    systemPrintTimestamp();
    systemPrint(frequency, 3);
    systemPrintln(" MHz");
  }
}

//Returns true if the radio indicates we have an ongoing reception
//Bit 0: Signal Detected indicates that a valid LoRa preamble has been detected
//Bit 1: Signal Synchronized indicates that the end of Preamble has been detected, the modem is in lock
//Bit 3: Header Info Valid toggles high when a valid Header (with correct CRC) is detected
bool receiveInProcess()
{
  //triggerEvent(TRIGGER_RECEIVE_IN_PROCESS_START);

  uint8_t radioStatus = radio.getModemStatus();
  if (radioStatus & 0b1011) return (true); //If any bits are set there is a receive in progress
  return false;

  //A remote unit may have started transmitting but this unit has not received enough preamble to detect it.
  //Wait X * symbol time for clear air.
  //This was found by sending two nearly simultaneous packets and using a logic analyzer to establish the point at which
  //the 'Signal Detected' bit goes high.
  uint8_t clearAirDelay = calcSymbolTime() * 18;
  while (clearAirDelay-- > 0)
  {
    radioStatus = radio.getModemStatus();
    if (radioStatus & 0b1011) return (true); //If any bits are set there is a receive in progress
    if (timeToHop) hopChannel(); //If the channelTimer has expired, move to next frequency
    delay(1);
  }

  //triggerEvent(TRIGGER_RECEIVE_IN_PROCESS_END);
  return (false); //No receive in process
}

//Convert the user's requested dBm to what the radio should be set to, to hit that power level
//3 is lowest allowed setting using SX1276+RadioLib
uint8_t covertdBmToSetting(uint8_t userSetting)
{
  switch (userSetting)
  {
    case 14: return (2); break;
    case 15: return (3); break;
    case 16: return (4); break;
    case 17: return (5); break;
    case 18: return (6); break;
    case 19: return (7); break;
    case 20: return (7); break;
    case 21: return (8); break;
    case 22: return (9); break;
    case 23: return (10); break;
    case 24: return (11); break;
    case 25: return (12); break;
    case 26: return (13); break;
    case 27: return (20); break;
    case 28: return (20); break;
    case 29: return (20); break;
    case 30: return (20); break;
    default: return (3); break;
  }
}

#ifdef  RADIOLIB_LOW_LEVEL
uint8_t readSX1276Register(uint8_t reg)
{
  return radio._mod->SPIreadRegister(reg);
}

//Print the SX1276 LoRa registers
void printSX1276Registers ()
{
  //Define the valid LoRa registers
  const uint8_t valid_regs [16] =
  {
    0xc2, 0xff, 0xff, 0xff, 0x7f, 0x97, 0xcb, 0x0e,
    0x07, 0x28, 0x00, 0x08, 0x1e, 0x00, 0x01, 0x00
  };

  systemPrint("Registers:");
  for (uint8_t i = 0; i < (sizeof(valid_regs) * 8); i++)
  {
    //Only read and print the valid registers
    if (valid_regs[i >> 3] & (1 << (i & 7)))
    {
      systemPrint("    0x");
      systemPrint(i, HEX);
      systemPrint(": 0x");
      systemPrintln(readSX1276Register(i), HEX);
    }
  }
}

#endif  //RADIOLIB_LOW_LEVEL
