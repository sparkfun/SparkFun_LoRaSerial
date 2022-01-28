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
  uint8_t receivedBytes = radio.getPacketLength();

  LRS_DEBUG_PRINT(F("Received bytes: "));
  LRS_DEBUG_PRINTLN(receivedBytes);

  LRS_DEBUG_PRINT(F("ProcessPacket NetID: 0x"));
  if (incomingBuffer[receivedBytes - 2] < 0x10) LRS_DEBUG_PRINT(F("0"));
  LRS_DEBUG_PRINT(incomingBuffer[receivedBytes - 2], HEX);

  LRS_DEBUG_PRINT(F(" Control: 0x"));
  if (incomingBuffer[receivedBytes - 1] < 0x10) LRS_DEBUG_PRINT(F("0"));
  LRS_DEBUG_PRINT(incomingBuffer[receivedBytes - 1], HEX);
  LRS_DEBUG_PRINTLN();

  if (receivedBytes < 2)
  {
    LRS_DEBUG_PRINTLN(F("Bad packet"));
    return (PROCESS_BAD_PACKET);
  }

  //Pull out control header
  uint8_t receivedNetID = incomingBuffer[receivedBytes - 2];
  memcpy(&receiveTrailer, &incomingBuffer[receivedBytes - 1], 1);

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
    LRS_DEBUG_PRINT(F("SF6 Received bytes: "));
    LRS_DEBUG_PRINTLN(receivedBytes);
  }

  receivedBytes -= 2; //Remove control bytes

  if (receivedNetID != settings.netID)
  {
    LRS_DEBUG_PRINT(F("NetID mismatch: "));
    LRS_DEBUG_PRINTLN(receivedNetID);
    return (PROCESS_NETID_MISMATCH);
  }

  if (receiveTrailer.ack == 1)
  {
    LRS_DEBUG_PRINTLN(F("RX: Ack packet"));
    return (PROCESS_ACK_PACKET);
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
        LRS_DEBUG_PRINTLN(F("Duplicate received. Acking again."));
        return (PROCESS_DUPLICATE_PACKET); //It's a duplicate. Ack then ignore
      }
    }
    else
    {
      //We've received a resend but it's different from the lastPacket
      //Go ahead and print it
    }
  }

  //We have empty data packet, this is a control packet used for pinging/scanning
  if (receivedBytes == 0)
  {
    LRS_DEBUG_PRINTLN(F("RX: Ping packet"));
    return (PROCESS_CONTROL_PACKET);
  }

  //Update lastPacket details with current packet
  memcpy(lastPacket, incomingBuffer, receivedBytes);
  lastPacketSize = receivedBytes;

  LRS_DEBUG_PRINTLN(F("RX: Data packet"));
  return (PROCESS_DATA_PACKET);
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
        if (settings.debug == true)
        {
          Serial.print(F("Unknown airSpeed: "));
          Serial.println(settings.airSpeed);
        }
        break;
    }
  }

  if (radio.setFrequency(channels[0]) == RADIOLIB_ERR_INVALID_FREQUENCY)
    success = false;

  // Set output power (accepted range is -3 - 17 dBm)
  // NOTE: 20 dBm value allows high power operation, but transmission duty cycle MUST NOT exceed 1%
  if (radio.setOutputPower(settings.radioBroadcastPower_dbm) == RADIOLIB_ERR_INVALID_OUTPUT_POWER)
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
  uint16_t hoppingPeriod = 400.0 / calcSymbolTime(); //Limit FHSS dwell time to 400ms max. / automatically floors number
  if (hoppingPeriod > 254) hoppingPeriod = 254; //Limit to 8 bits. SF6 does not work with period of 255.
  if (settings.frequencyHop == false) hoppingPeriod = 0; //Disable
  if (radio.setFHSSHoppingPeriod(hoppingPeriod) != RADIOLIB_ERR_NONE)
    success = false;

  controlPacketAirTime = calcAirTime(2); //Used for response timeout during RADIO_ACK_WAIT
  uint16_t responseDelay = controlPacketAirTime / settings.responseDelayDivisor; //Give the receiver a bit of wiggle time to respond
  controlPacketAirTime += responseDelay;

  controlPacketAirTime += 10;

  if (settings.debug == true)
  {
    Serial.print(F("Freq: "));
    Serial.println(channels[0]);
    Serial.print(F("radioBandwidth: "));
    Serial.println(settings.radioBandwidth);
    Serial.print(F("radioSpreadFactor: "));
    Serial.println(settings.radioSpreadFactor);
    Serial.print(F("radioCodingRate: "));
    Serial.println(settings.radioCodingRate);
    Serial.print(F("radioSyncWord: "));
    Serial.println(settings.radioSyncWord);
    Serial.print(F("radioPreambleLength: "));
    Serial.println(settings.radioPreambleLength);
    Serial.print(F("calcSymbolTime: "));
    Serial.println(calcSymbolTime());
    Serial.print(F("HoppingPeriod: "));
    Serial.println(hoppingPeriod);
    Serial.print(F("controlPacketAirTime: "));
    Serial.println(controlPacketAirTime);
  }

  if (success == false)
  {
    reportERROR();
    if (settings.debug == true)
    {
      Serial.println(F("Radio init failed. Check settings."));
    }
  }
  LRS_DEBUG_PRINTLN(F("Radio online"));
}

void returnToReceiving()
{
  digitalWrite(pin_activityLED, LOW);
  currentChannel = 0; //Return home before receiving

  if (settings.autoTuneFrequency == true)
    radio.setFrequency(channels[currentChannel] - frequencyCorrection);
  else
    radio.setFrequency(channels[currentChannel]);

  radio.clearFHSSInt();
  hopsCompleted = 0; //Reset to detect in progress reception
  timeToHop = false;

  int state;
  if (settings.radioSpreadFactor > 6)
  {
    state = radio.startReceive();
  }
  else
  {
    if (expectingAck)
    {
      radio.implicitHeader(2);
      state = radio.startReceive(2); //Expect a control packet
      if (settings.debug == true)
      {
        digitalWrite(pin_trigger, LOW);
        delayMicroseconds(50);
        digitalWrite(pin_trigger, HIGH);
      }
      expectingAck = false; //Do not return to this receiving configuration if something goes wrong
    }
    else
    {
      radio.implicitHeader(255);
      state = radio.startReceive(255); //Expect a full data packet
      if (settings.debug == true)
      {
        digitalWrite(pin_trigger, LOW);
        delayMicroseconds(250);
        digitalWrite(pin_trigger, HIGH);
      }
    }
  }

  if (state != RADIOLIB_ERR_NONE) {
    LRS_DEBUG_PRINT(F("Receive failed: "));
    LRS_DEBUG_PRINTLN(state);
  }
}

//Create short packet of 2 control bytes - query remote radio for proof of life (ack)
void sendPingPacket()
{
  LRS_DEBUG_PRINT(F("TX: Ping "));
  responseTrailer.ack = 0; //This is not an ACK to a previous transmission
  responseTrailer.resend = 0; //This is not a resend
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

//Create short packet of 2 control bytes - do not expect ack
void sendAckPacket()
{
  LRS_DEBUG_PRINT(F("TX: Ack "));
  responseTrailer.ack = 1; //This is an ACK to a previous reception
  responseTrailer.resend = 0; //This is not a resend
  packetSize = 2;
  packetSent = 0; //Reset the number of times we've sent this packet
  expectingAck = false; //We do not expect destination to ack
  sendPacket();
}

//Create packet of current data + control bytes - expect ACK from recipient
void sendDataPacket()
{
  LRS_DEBUG_PRINT(F("TX: Data "));
  responseTrailer.ack = 0; //This is not an ACK to a previous transmission
  responseTrailer.resend = 0; //This is not a resend
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
  LRS_DEBUG_PRINT(F("TX: Resend "));
  responseTrailer.ack = 0; //This is not an ACK to a previous transmission
  responseTrailer.resend = 1; //This is a resend
  //packetSize += 2; //Don't adjust the packet size
  //packetSent = 0; //Don't reset
  expectingAck = true; //We expect destination to ack
  sendPacket();
}

//Push the outgoing packet to the air
void sendPacket()
{
  //Attach netID and control byte to end of packet
  outgoingPacket[packetSize - 2] = settings.netID;
  memcpy(&outgoingPacket[packetSize - 1], &responseTrailer, 1);

  digitalWrite(pin_activityLED, HIGH);

  currentChannel = 0; //Return home before every transmission
  radio.setFrequency(channels[currentChannel]); //Do not adjust frequency on TX, only RX.
  LRS_DEBUG_PRINTLN(channels[currentChannel], 3);
  radio.clearFHSSInt();
  int state = radio.startTransmit(outgoingPacket, packetSize);
  if (state == RADIOLIB_ERR_NONE)
  {
    packetAirTime = calcAirTime(packetSize); //Calculate packet air size while we're transmitting in the background
    uint16_t responseDelay = packetAirTime / settings.responseDelayDivisor; //Give the receiver a bit of wiggle time to respond
    packetAirTime += responseDelay;

    packetSent++;

    LRS_DEBUG_PRINT(F("PacketAirTime: "));
    LRS_DEBUG_PRINTLN(packetAirTime);
    LRS_DEBUG_PRINT(F("responseDelay: "));
    LRS_DEBUG_PRINTLN(responseDelay, 3);
  }
  else
  {
    LRS_DEBUG_PRINTLN(F("Error: TX"));
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
  //  Serial.print("Channel Spacing: ");
  //  Serial.println(channelSpacing, 3);

  //Keep away from edge of available spectrum
  float operatingMinFreq = settings.frequencyMin + (float)(channelSpacing / 2);
  //  Serial.print("operatingMinFreq: ");
  //  Serial.println(operatingMinFreq, 3);

  //Pre populate channel list
  for (int x = 0 ; x < settings.numberOfChannels ; x++)
    channels[x] = operatingMinFreq + (x * channelSpacing);

  //Feed random number generator with our specific platform settings
  //Use settings that must be identical to have a functioning link.
  //For example, we do not use CodingRate because two radios can communicate with different values
  myRandSeed = settings.netID + settings.airSpeed + (uint16_t)settings.radioBandwidth + settings.radioSpreadFactor;
  //  Serial.print("myRandSeed: ");
  //  Serial.println(myRandSeed);

  //'Randomly' shuffle list based on our specific seed
  shuffle(channels, settings.numberOfChannels);

  if (settings.debug == true)
  {
    Serial.println(F("Channel table:"));
    for (int x = 0 ; x < settings.numberOfChannels ; x++)
    {
      Serial.print(x);
      Serial.print(F(": "));
      Serial.print(channels[x], 3);
      Serial.println();
    }

    //Serial.print("size of channel table: ");
    //Serial.println(sizeof(channels));
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
  currentChannel++;
  currentChannel %= settings.numberOfChannels;

  //  LRS_DEBUG_PRINT("HopChannel: currentChannel: ");
  //  LRS_DEBUG_PRINT(currentChannel);
  //  LRS_DEBUG_PRINT(" radioChannel: ");
  //  LRS_DEBUG_PRINTLN(radio.getFHSSChannel());

  if (settings.autoTuneFrequency == true)
  {
    if (radioState == RADIO_RECEIVING_STANDBY || radioState == RADIO_ACK_WAIT) //Only adjust frequency on RX. Not TX.
      radio.setFrequency(channels[currentChannel] - frequencyCorrection);
    else
      radio.setFrequency(channels[currentChannel]);
  }
  else
    radio.setFrequency(channels[currentChannel]);

  hopsCompleted++;
  radio.clearFHSSInt();
  timeToHop = false;
}

//Returns true if the radio indicates we have an ongoing reception
//Bit 0: Signal Detected indicates that a valid LoRa preamble has been detected
//Bit 1: Signal Synchronized indicates that the end of Preamble has been detected, the modem is in lock
//Bit 3: Header Info Valid toggles high when a valid Header (with correct CRC) is detected
bool receiveInProcess()
{
  uint8_t radioStatus = radio.getModemStatus();
  if ((radioStatus & 0b1) == 0) return (false); //If bit 0 is cleared, there is no receive in progress
  return (true); //If bit 0 is set, forget the other bits, there is a receive in progress
}
