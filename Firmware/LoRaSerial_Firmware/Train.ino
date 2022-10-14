//Select the training protocol
void selectTraining(bool defaultTraining)
{
  if (settings.protocolVersion >= 2)
  {
    if (settings.pointToPoint)
      beginTrainingPointToPoint(defaultTraining);
    else
    {
      if (settings.trainingServer)
        beginTrainingServer();
      else
        beginTrainingClient();
    }
  }
  else if (settings.protocolVersion == 1)
    beginTraining(defaultTraining);
  else
  {
    //Handle unknown future versions
    systemPrint("Unknown protocol version: ");
    systemPrintln(settings.protocolVersion);
    while (1)
      petWDT();
  }
}

/*
      beginTraining
            |
            | Save settings
            |
            V
    RADIO_TRAINING_TRANSMITTING
            |
            V
    RADIO_TRAINING_ACK_WAIT --------------.
            |                             |
            V                             |
    RADIO_TRAINING_RECEIVING_HERE_FIRST   |
            |                             |
            +<----------------------------’
            |
            V
    RADIO_TRAINING_RECEIVED_PACKET
            |
            V
       endTraining


    beginTraining

      1. Disable point-to-point
      2. Disable frequency hopping
      3. Reduce power to minimum
      4. Generate HOP table
      5. Compute channel spacing
      6. Set training frequency
      7. Configure the radio
      8. Send training ping
      9. Set state RADIO_TRAINING_TRANSMITTING

    endTraining

      1. Restore original settings
      2. Update encryption key
      3. Set net ID
*/

void beginTraining(bool defaultTraining)
{
  if ((settings.debug == true) || (settings.debugTraining == true))
  {
    systemPrint("Begin ");
    if (defaultTraining)
      systemPrint("default ");
    systemPrintln("point-to-point training");
  }

  //Save the parameters
  if (defaultTraining)
    originalSettings = defaultSettings; //Upon completion we will return to default settings
  else
    originalSettings = settings; //Make copy of current settings

  //Change to known training frequency based on available freq and current major firmware version
  //This will allow different minor versions to continue to train to each other
  //During training use default radio settings. This ensures both radios are at known good settings.
  settings = defaultSettings; //Move to default settings

  //Disable hopping
  settings.frequencyHop = false;

  //Disable NetID checking
  settings.pointToPoint = false;
  settings.verifyRxNetID = false;

  //Debug training if requested
  if (originalSettings.debugTraining)
  {
    settings.debugTraining = originalSettings.debugTraining;
    settings.printPktData = originalSettings.printPktData;
    settings.printRfData = originalSettings.printRfData;
  }

  //Turn power as low as possible. We assume two units will be near each other.
  settings.radioBroadcastPower_dbm = 14;

  generateHopTable(); //Generate frequency table based on current settings

  configureRadio(); //Setup radio with settings

  //Move to frequency that is not part of the hop table
  //In normal operation we move 1/2 a channel away from min. In training, we move a full channel away + major firmware version.
  float channelSpacing = (settings.frequencyMax - settings.frequencyMin) / (float)(settings.numberOfChannels + 2);
  float trainFrequency = settings.frequencyMin + (channelSpacing * (FIRMWARE_VERSION_MAJOR % settings.numberOfChannels));

  channels[0] = trainFrequency; //Inject this frequency into the channel table

  //Transmit general ping packet to see if anyone else is sitting on the training channel
  //Send special packet with train = 1, then wait for response
  sendTrainingPingPacket();

  //Recalculate packetAirTime because we need to wait not for a 2-byte response, but a 19 byte response
  packetAirTime = calcAirTime(sizeof(trainEncryptionKey) + sizeof(trainNetID) + 2);

  //Reset cylon variables
  startCylonLEDs();

  changeState(RADIO_TRAINING_TRANSMITTING);
}

//Upon successful exchange of keys, go back to original settings
void endTraining(bool newTrainingAvailable)
{
  petWDT();
  settings = originalSettings; //Return to original radio settings

  //Apply new netID and AES if available
  if (newTrainingAvailable)
  {
    if (lastPacketSize == sizeof(settings.encryptionKey) + 1) //Error check, should be AES key + NetID
    {
      //Move training data into settings
      for (uint8_t x = 0 ; x < sizeof(settings.encryptionKey); x++)
        settings.encryptionKey[x] = lastPacket[x];

      settings.netID = lastPacket[lastPacketSize - 1]; //Last spot in array is netID

      if ((settings.debug == true) || (settings.debugTraining == true))
      {
        systemPrint("New ID: ");
        systemPrintln(settings.netID);

        systemPrint("New Key: ");
        for (uint8_t i = 0 ; i < 16 ; i++)
        {
          systemPrint(settings.encryptionKey[i], HEX);
          systemPrint(" ");
        }
        systemPrintln();
      }
    }
    else
    {
      //If the packet was marked as training but was not valid training data, then give up. Return to normal radio mode with pre-existing settings.
    }
  }
  else
  {
    //We transmitted the training data, move the local training data into settings
    for (uint8_t x = 0 ; x < sizeof(settings.encryptionKey); x++)
      settings.encryptionKey[x] = trainEncryptionKey[x];

    settings.netID = trainNetID; //Last spot in array is netID
  }

  petWDT();
  recordSystemSettings();

  petWDT();
  generateHopTable(); //Generate frequency table based on current settings

  configureRadio(); //Setup radio with settings

  returnToReceiving();

  //Blink LEDs to indicate training success
  setRSSI(0b0000);
  delayWDT(100);

  setRSSI(0b1001);
  delayWDT(500);

  setRSSI(0b0110);
  delayWDT(500);

  setRSSI(0b1111);
  delayWDT(500);

  setRSSI(0b0000);
  delayWDT(500);

  setRSSI(0b1111);
  delayWDT(1500);

  setRSSI(0);
  delayWDT(2000);

  changeState(RADIO_RESET);

  sentFirstPing = false; //Send ping as soon as we exit

  systemPrintln("LINK TRAINED");
}

//Generate new netID/AES key to share
//We assume the user needs to maintain their settings (airSpeed, numberOfChannels, freq min/max, bandwidth/spread/hop)
//but need to be on a different netID/AES key.
void generateTrainingSettings()
{
  if ((settings.debug == true) || (settings.debugTraining == true))
    systemPrintln("Generate New Training Settings");

  //Seed random number based on RF noise. We use Arduino random() because platform specific generation does not matter
  randomSeed(radio.randomByte());

  //Generate new NetID
  trainNetID = random(0, 256); //Inclusive, exclusive

  //Generate new AES Key. User may not be using AES but we still need both radios to have the same key in case they do enable AES.
  for (int x = 0 ; x < 16 ; x++)
    trainEncryptionKey[x] = random(0, 256); //Inclusive, exclusive

  //We do not generate new AES Initial Values here. Those are generated during generateHopTable() based on the unit's settings.

  if ((settings.debug == true) || (settings.debugTraining == true))
  {
    systemPrint("Select new NetID: ");
    systemPrintln(trainNetID);

    systemPrint("Select new Encryption Key:");
    for (uint8_t i = 0 ; i < 16 ; i++)
    {
      systemPrint(" ");
      systemPrint(trainEncryptionKey[i], HEX);
    }
    systemPrintln();
  }
}

//Start the cylon LEDs
void startCylonLEDs()
{
  trainCylonNumber = 0b0001;
  trainCylonDirection = -1;
}

//Update the cylon LEDs
void updateCylonLEDs()
{
  if ( (millis() - lastTrainBlink) > 75) //Blink while unit waits in training state
  {
    lastTrainBlink = millis();

    //Cylon the RSSI LEDs
    setRSSI(trainCylonNumber);

    if (trainCylonNumber == 0b1000 || trainCylonNumber == 0b0001)
      trainCylonDirection *= -1; //Change direction

    if (trainCylonDirection > 0)
      trainCylonNumber <<= 1;
    else
      trainCylonNumber >>= 1;
  }
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
//V2 Point-To-Point Training
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

void beginTrainingPointToPoint(bool defaultTraining)
{
  if (defaultTraining)
  {
    settings = defaultSettings; //Upon completion we will return to default settings
    systemPrint("Default ");
  }
  systemPrintln("Point-to-point training");

  //Common initialization
  commonTrainingInitialization();

  //Transmit general ping packet to see if anyone else is sitting on the training channel
  xmitDatagramP2PTrainingPing();

  //Set the next state
  changeState(RADIO_P2P_TRAINING_WAIT_PING_DONE);
}

void endPointToPointTraining(bool saveParams)
{
  memcpy(&settings, &originalSettings, sizeof(settings));
  if (saveParams)
    recordSystemSettings();
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
//V2 Multi-Point Client/Server Training
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

void beginTrainingClient()
{
  systemPrintln("Multipoint client training");

  //Common initialization
  commonTrainingInitialization();

  //Transmit client ping to the training server
  xmitDatagramMpTrainingPing();

  //Set the next state
  changeState(RADIO_MP_WAIT_TX_TRAINING_PING_DONE);
}

void beginTrainingServer()
{
  trainingServerRunning = true;

  //Display the values to be used for the client/server training
  systemPrintln("Multipoint server training");
  systemPrintln("Using:");
  systemPrint("  netID: ");
  systemPrintln(settings.netID);
  systemPrint("  Training key: ");
  displayEncryptionKey(settings.trainingKey);
  systemPrintln();

  //Common initialization
  commonTrainingInitialization();
  settings.trainingServer = true;         //52: Operate as the training server

  //Start the receive operation
  returnToReceiving();

  //Set the next state
  changeState(RADIO_MP_WAIT_FOR_TRAINING_PING);
}

//Perform the common training initialization
void commonTrainingInitialization()
{
  //Save the current settings
  originalSettings = settings;

  //Use common radio settings between the client and server for training
  settings = defaultSettings;
  settings.pointToPoint = false;          // 3: Disable netID checking
  settings.encryptData = true;            // 4: Enable packet encryption
  settings.dataScrambling = true;         // 6: Scramble the data
  settings.radioBroadcastPower_dbm = 14;  // 7: Minimum, assume radios are near each other
  settings.frequencyHop = false;          //11: Stay on the training frequency
  settings.printParameterName = true;     //28: Print the parameter names
  settings.verifyRxNetID = false;         //37: Disable netID checking
  settings.enableCRC16 = true;            //49: Use CRC-16
  memcpy(&settings.trainingKey, &originalSettings.trainingKey, AES_KEY_BYTES); //56: Common training key

  //Determine the components of the frame header and trailer
  selectHeaderAndTrailerBytes();

  //Debug training if requested
  if (originalSettings.debugTraining)
  {
    settings.debug = originalSettings.debug;
    settings.displayPacketQuality = originalSettings.displayPacketQuality;
    settings.printParameterName = originalSettings.printParameterName;
    settings.printFrequency = originalSettings.printFrequency;

    settings.debugRadio = originalSettings.debugRadio;
    settings.debugStates = originalSettings.debugStates;
    settings.debugTraining = originalSettings.debugTraining;
    settings.debugTrigger = originalSettings.debugTrigger;

    settings.printRfData = originalSettings.printRfData;
    settings.printPktData = originalSettings.printPktData;
    settings.triggerWidth = originalSettings.triggerWidth;
    settings.triggerWidthIsMultiplier = originalSettings.triggerWidthIsMultiplier;

    settings.triggerEnable = originalSettings.triggerEnable;
    settings.triggerEnable2 = originalSettings.triggerEnable2;
    settings.debugReceive = originalSettings.debugReceive;
    settings.debugTransmit = originalSettings.debugTransmit;
    settings.printTxErrors = originalSettings.printTxErrors;

    settings.printTimestamp = originalSettings.printTimestamp;
    settings.debugDatagrams = originalSettings.debugDatagrams;
    settings.displayRealMillis = originalSettings.displayRealMillis;
  }

  //Reset cylon variables
  startCylonLEDs();

  petWDT();
  generateHopTable(); //Generate frequency table based on current settings

  //Select the training frequency, a multiple of channels down from the maximum
  petWDT();
  float channelSpacing = (settings.frequencyMax - settings.frequencyMin) / (float)(settings.numberOfChannels + 2);
  float trainFrequency = settings.frequencyMax - (channelSpacing * (FIRMWARE_VERSION_MAJOR % settings.numberOfChannels));
  originalChannel = channels[0];      //Remember the original channel
  channels[0] = trainFrequency;       //Inject this frequency into the channel table

  //Use only the first channel of the previously allocated channel table
  petWDT();
  configureRadio(); //Setup radio with settings
}

//Upon successful exchange of parameters, switch to the new radio settings
void endClientServerTraining(uint8_t event)
{
  triggerEvent(event);
  settings = originalSettings; //Return to original radio settings

  if (settings.debugTraining)
    displayParameters();

  if (!settings.trainingServer)
  {
    //Record the new client settings
    petWDT();
    recordSystemSettings();

    systemPrint("Link trained from ");
    systemPrintUniqueID(trainingPartnerID);
    systemPrintln();
  }

  //Done with training
  trainingServerRunning = false;

  //Reboot the radio with the new parameters
  petWDT();
  systemFlush();
  systemReset();
}
