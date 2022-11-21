//Select the training protocol
void selectTraining(bool defaultTraining)
{
  if (settings.operatingMode == MODE_POINT_TO_POINT)
    beginTrainingPointToPoint(defaultTraining);
  else
  {
    if (settings.trainingServer)
      beginTrainingServer();
    else
      beginTrainingClient();
  }
}

//Generate new netID/AES key to share
//We assume the user needs to maintain their settings (airSpeed, numberOfChannels, freq min/max, bandwidth/spread/hop)
//but need to be on a different netID/AES key.
void generateTrainingSettings()
{
  if ((settings.debug == true) || (settings.debugTraining == true))
  {
    systemPrintln("Generate New Training Settings");
    outputSerialData(true);
  }

  //Seed random number based on RF noise. We use Arduino random() because platform specific generation does not matter
  randomSeed(radio.randomByte());

  //Generate new NetID
  settings.netID = random(0, 256); //Inclusive, exclusive

  //Generate new AES Key. User may not be using AES but we still need both radios to have the same key in case they do enable AES.
  for (int x = 0 ; x < 16 ; x++)
    settings.encryptionKey[x] = random(0, 256); //Inclusive, exclusive

  //We do not generate new AES Initial Values here. Those are generated during generateHopTable() based on the unit's settings.

  if ((settings.debug == true) || (settings.debugTraining == true))
  {
    systemPrint("Select new NetID: ");
    systemPrintln(settings.netID);

    systemPrint("Select new Encryption Key:");
    for (uint8_t i = 0 ; i < 16 ; i++)
    {
      systemPrint(" ");
      systemPrint(settings.encryptionKey[i], HEX);
    }
    systemPrintln();
    outputSerialData(true);
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

  //Generate new netID and encryption key
  generateTrainingSettings();

  //Common initialization
  commonTrainingInitialization();

  //Transmit general ping packet to see if anyone else is sitting on the training channel
  if (xmitDatagramP2PTrainingPing() == true)
  {
    trainingTimer = millis();

    //Set the next state
    changeState(RADIO_P2P_TRAINING_WAIT_PING_DONE);
  }
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
  if (xmitDatagramMpTrainingPing() == true)
  {
    trainingTimer = millis();

    //Set the next state
    changeState(RADIO_MP_WAIT_TX_TRAINING_PING_DONE);
  }
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
  settings.operatingMode = MODE_DATAGRAM; // 3: Use datagrams
  settings.encryptData = true;            // 4: Enable packet encryption
  settings.dataScrambling = true;         // 6: Scramble the data
  settings.radioBroadcastPower_dbm = 14;  // 7: Minimum, assume radios are near each other
  settings.frequencyHop = false;          //11: Stay on the training frequency
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
    settings.printFrequency = originalSettings.printFrequency;

    settings.debugRadio = originalSettings.debugRadio;
    settings.debugStates = originalSettings.debugStates;
    settings.debugTraining = originalSettings.debugTraining;

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
  convertAirSpeedToSettings(); //Update the settings based upon the air speed

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
  {
    displayParameters(0, settings.copyDebug || settings.copyTriggers);
    outputSerialData(true);
  }

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
