//Determine the type of packet received and print data as necessary
//The LoRa radio handles CRC and FHSS for us so we presume the packet was meant for us
//We check the packet netID for validity
//If the packet is a short/ping packet, ack
//If the packet is marked as resend, check if it's a duplicate
//Move any data to incomingPacket buffer
PacketType identifyPacketType()
{
  uint8_t incomingBuffer[MAX_PACKET_SIZE];
  radio.readData(incomingBuffer, MAX_PACKET_SIZE);

  //Receive the data packet
  uint8_t receivedBytes = radio.getPacketLength();

  if (settings.dataScrambling == true)
    radioComputeWhitening(incomingBuffer, receivedBytes);

  if (settings.encryptData == true)
    decryptBuffer(incomingBuffer, receivedBytes);

  LRS_DEBUG_PRINT("\n\rReceived bytes: ");
  LRS_DEBUG_PRINTLN(receivedBytes);

  //All packets must include the 2-byte control header
  if (receivedBytes < 2)
  {
    LRS_DEBUG_PRINTLN("Bad packet");
    return (PACKET_BAD);
  }

  /*
        |<--------------- data --------------->|
        |                                      |

        +-------------------------+------------+--------+---------+
        |                         | SF6 length | NET ID | Control |
        |                         |   8 bits   | 8 bits | 8 bits  |
        +-------------------------+------------+--------+---------+

        |                                                         |
        |<-------------------- receivedBytes -------------------->|
  */

  //Display the control header
  LRS_DEBUG_PRINT("ProcessPacket NetID: 0x");
  if (incomingBuffer[receivedBytes - 2] < 0x10) LRS_DEBUG_PRINT("0");
  LRS_DEBUG_PRINT(incomingBuffer[receivedBytes - 2], HEX);

  LRS_DEBUG_PRINT(" Control: 0x");
  if (incomingBuffer[receivedBytes - 1] < 0x10) LRS_DEBUG_PRINT("0");
  LRS_DEBUG_PRINT(incomingBuffer[receivedBytes - 1], HEX);
  LRS_DEBUG_PRINTLN();

  //Pull out control header
  uint8_t receivedNetID = incomingBuffer[receivedBytes - 2];
  *(uint8_t *)&receiveTrailer = incomingBuffer[receivedBytes - 1];

  //SF6 requires an implicit header which means there is no dataLength in the header
  //Instead, we manually store it 3 bytes from the end (before NetID)
  if (settings.radioSpreadFactor == 6)
  {
    //We've either received a control packet (2 bytes) or a data packet
    if (receivedBytes > 2)
    {
      receivedBytes = incomingBuffer[receivedBytes - 3]; //Obtain actual packet data length
      receivedBytes -= 1; //Remove the manual packetSize byte from consideration
    }
    LRS_DEBUG_PRINT("SF6 Received bytes: ");
    LRS_DEBUG_PRINTLN(receivedBytes);
  }

  receivedBytes -= 2; //Remove control bytes

  if (receivedNetID != settings.netID && settings.pointToPoint == true)
  {
    LRS_DEBUG_PRINT("NetID mismatch: ");
    LRS_DEBUG_PRINTLN(receivedNetID);
    return (PACKET_NETID_MISMATCH);
  }

  //----------
  //Handle ACKs and duplicate packets
  //----------

  if (receiveTrailer.ack == 1 && receiveTrailer.remoteCommand == 0 && receiveTrailer.remoteCommandResponse == 0)
  {
    LRS_DEBUG_PRINTLN("RX: Ack packet");
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
        LRS_DEBUG_PRINTLN("Duplicate received. Acking again.");
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
      LRS_DEBUG_PRINTLN("RX: Training Control Packet");
      return (PACKET_TRAINING_PING);
    }

    //If this packet is marked as a remote command, it's either an ack or a zero length packet (not known)
    else if (receiveTrailer.remoteCommand == 1)
    {
      if (receiveTrailer.ack == 1)
      {
        LRS_DEBUG_PRINTLN("RX: Command Ack");
        return (PACKET_COMMAND_ACK);
      }

      LRS_DEBUG_PRINTLN("RX: Unknown Command");
      return (PACKET_BAD);
    }

    //If this packet is marked as a remote command response, it's either an ack or a zero length packet (not known)
    else if (receiveTrailer.remoteCommandResponse == 1)
    {
      if (receiveTrailer.ack == 1)
      {
        LRS_DEBUG_PRINTLN("RX: Command Response Ack");
        return (PACKET_COMMAND_RESPONSE_ACK);
      }

      LRS_DEBUG_PRINTLN("RX: Unknown Response Command");
      return (PACKET_BAD);
    }

    //Not training, not command packet, just a ping
    LRS_DEBUG_PRINTLN("RX: Control Packet");
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
    LRS_DEBUG_PRINTLN("RX: Training Data");
    return (PACKET_TRAINING_DATA);
  }

  else if (receiveTrailer.remoteCommand == 1)
  {
    //New data from remote
    LRS_DEBUG_PRINTLN("RX: Command Data");
    return (PACKET_COMMAND_DATA);
  }

  else if (receiveTrailer.remoteCommandResponse == 1)
  {
    //New response data from remote
    LRS_DEBUG_PRINTLN("RX: Command Response Data");
    return (PACKET_COMMAND_RESPONSE_DATA);
  }

  LRS_DEBUG_PRINTLN("RX: Data");
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
    if (radio.implicitHeader(255) != RADIOLIB_ERR_NONE)
      success = false;
  }
  else
  {
    if (radio.explicitHeader() != RADIOLIB_ERR_NONE)
      success = false;
  }

  radio.setDio0Action(dio0ISR); //Called when transmission is finished
  radio.setDio1Action(dio1ISR); //Called after a transmission has started, so we can move to next freq

  if (pin_rxen != 255)
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
  uint16_t responseDelay = controlPacketAirTime / settings.responseDelayDivisor; //Give the receiver a bit of wiggle time to respond
  controlPacketAirTime += responseDelay;

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
  }

  if (success == false)
  {
    reportERROR();
    systemPrintln("Radio init failed. Check settings.");
  }
  LRS_DEBUG_PRINTLN("Radio configured");
}

//Set radio frequency
void setRadioFrequency(bool rxAdjust)
{
  float frequency;

  frequency = channels[radio.getFHSSChannel()];
  if (rxAdjust)
    frequency -= frequencyCorrection;
  if (settings.printFrequency)
  {
    systemPrint(frequency);
    systemPrintln(" MHz");
  }
  radio.setFrequency(frequency);
}

void returnToReceiving()
{
  setRadioFrequency(settings.autoTuneFrequency);

  timeToHop = false;

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
      radio.implicitHeader(255);
      state = radio.startReceive(255); //Expect a full data packet
      triggerEvent(TRIGGER_RTR_255BYTE);
    }
  }

  if (state != RADIOLIB_ERR_NONE) {
    LRS_DEBUG_PRINT("Receive failed: ");
    LRS_DEBUG_PRINTLN(state);
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

  LRS_DEBUG_PRINT("TX: Ping ");
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
    outgoingPacket[255 - 3] = packetSize + 1;
    packetSize = 255; //We're now going to transmit 255 bytes
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

  LRS_DEBUG_PRINT("TX: Data ");
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
    outgoingPacket[255 - 3] = packetSize + 1;
    packetSize = 255; //We're now going to transmit 255 bytes
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

  LRS_DEBUG_PRINT("TX: Resend ");
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

  LRS_DEBUG_PRINT("TX: Ack ");
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

  LRS_DEBUG_PRINT("TX: Training Ping ");
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

  LRS_DEBUG_PRINT("TX: Training Data ");
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

  LRS_DEBUG_PRINT("TX: Command Ack ");
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

  LRS_DEBUG_PRINT("TX: Command Data ");
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
    outgoingPacket[255 - 3] = packetSize + 1;
    packetSize = 255; //We're now going to transmit 255 bytes
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

  LRS_DEBUG_PRINT("TX: Command Response Ack ");
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

  LRS_DEBUG_PRINT("TX: Command Response Data ");
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
    outgoingPacket[255 - 3] = packetSize + 1;
    packetSize = 255; //We're now going to transmit 255 bytes
  }

  expectingAck = true; //We expect destination to ack
  sendPacket();
}

//Push the outgoing packet to the air
void sendPacket()
{
  /*
      +---  ...  ---+--------+---------+
      |    Data     | NET ID | Trailer |
      |   n bytes   | 8 bits | 8 bits  |
      +-------------+--------+---------+
      |                                |
      |<--------- packetSize --------->|
  */

  //Attach netID and control byte to end of packet
  outgoingPacket[packetSize - 2] = settings.netID;
  outgoingPacket[packetSize - 1] = *(uint8_t *)&responseTrailer;

  //Apply AES and whitening only to new packets, not resends
  if (responseTrailer.resend == 0)
  {
    if (settings.encryptData == true)
      encryptBuffer(outgoingPacket, packetSize);

    if (settings.dataScrambling == true)
      radioComputeWhitening(outgoingPacket, packetSize);
  }

  //If we are trainsmitting at high data rates the receiver is often not ready for new data. Pause for a few ms (measured with logic analyzer).
  if (settings.airSpeed == 28800 || settings.airSpeed == 38400)
    delay(2);

  setRadioFrequency(false); //Return home before every transmission

  LRS_DEBUG_PRINT("Transmitting @ ");
  LRS_DEBUG_PRINTLN(channels[radio.getFHSSChannel()], 3);

  int state = radio.startTransmit(outgoingPacket, packetSize);
  if (state == RADIOLIB_ERR_NONE)
  {
    if (timeToHop) hopChannel();

    packetAirTime = calcAirTime(packetSize); //Calculate packet air size while we're transmitting in the background
    uint16_t responseDelay = packetAirTime / settings.responseDelayDivisor; //Give the receiver a bit of wiggle time to respond
    packetAirTime += responseDelay;

    packetSent++;

    LRS_DEBUG_PRINT("PacketAirTime: ");
    LRS_DEBUG_PRINTLN(packetAirTime);
    LRS_DEBUG_PRINT("responseDelay: ");
    LRS_DEBUG_PRINTLN(responseDelay);

    if (timeToHop) hopChannel();
  }
  else
  {
    LRS_DEBUG_PRINTLN("Error: TX");
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
void dio1ISR(void)
{
  timeToHop = true;
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

  //Set new initial values for AES using settings based random seed
  for (uint8_t x = 0 ; x < 12 ; x++)
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
    for (uint8_t i = 0 ; i < 12 ; i++)
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
  radio.clearFHSSInt();
  timeToHop = false;

  if (settings.autoTuneFrequency == true)
  {
    if (radioState == RADIO_LINKED_RECEIVING_STANDBY || radioState == RADIO_LINKED_ACK_WAIT
        || radioState == RADIO_BROADCASTING_RECEIVING_STANDBY) //Only adjust frequency on RX. Not TX.
      radio.setFrequency(channels[radio.getFHSSChannel()] - frequencyCorrection);
    else
      radio.setFrequency(channels[radio.getFHSSChannel()]);
  }
  else
    radio.setFrequency(channels[radio.getFHSSChannel()]);
}

//Returns true if the radio indicates we have an ongoing reception
//Bit 0: Signal Detected indicates that a valid LoRa preamble has been detected
//Bit 1: Signal Synchronized indicates that the end of Preamble has been detected, the modem is in lock
//Bit 3: Header Info Valid toggles high when a valid Header (with correct CRC) is detected
bool receiveInProcess()
{
  uint8_t radioStatus = radio.getModemStatus();
  if ((radioStatus & 0b1) == 1) return (true); //If bit 0 is set, forget the other bits, there is a receive in progress
  return (false); //No receive in process

  //If bit 0 is cleared, there is likely no receive in progress, but we need to confirm it
  //The radio will clear this bit before the radio triggers the receiveComplete ISR so we often have a race condition.

  //If we are using FHSS then check channel freq. This is reset to 0 upon receive completion.
  //If radio has jumped back to channel 0 then we can confirm a transaction is complete.
  if (settings.frequencyHop == true)
  {
    if (radio.getFHSSChannel() == 0)
      return (false); //Receive not in process

    if (transactionComplete == false)
      return (true); //Receive still in process
  }
  else
  {
    //If we're not using FHSS, use small delay.
    delay(5); //Small wait before checking RX complete ISR again
    if (transactionComplete == false)
      return (true); //Receive still in process
  }

  return (false); //No receive in process

  //if ((radioStatus & 0b1) == 0) return (false); //If bit 0 is cleared, there is no receive in progress
  //return (true); //If bit 0 is set, forget the other bits, there is a receive in progress
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
