//Select the training protocol
void selectTraining()
{
  //If we are training via button, and in P2P mode, and Server is not set
  //we will need these settings if we exit training
  memcpy(&originalEncryptionKey, &settings.encryptionKey, AES_KEY_BYTES);
  originalNetID = settings.netID;
  originalServer = settings.server;

  if (settings.server)
    beginTrainingServer();
  else
    beginTrainingClient();
}

//Generate new netID/AES key to share
//We assume the user needs to maintain their settings (airSpeed, numberOfChannels, freq min/max, bandwidth/spread/hop)
//but need to be on a different netID/AES key.
void generateRandomKeysID()
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

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
//Client/Server Training
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//Start the training in client mode
void beginTrainingClient()
{
  //Common initialization
  commonTrainingInitialization();

  //Transmit client ping to the training server
  if (xmitDatagramTrainingPing() == true)
    //Set the next state
    changeState(RADIO_TRAIN_WAIT_TX_PING_DONE);
  else
    changeState(RADIO_TRAIN_WAIT_RX_RADIO_PARAMETERS);
  trainingTimer = millis();
}

//Start the training in server mode
void beginTrainingServer()
{
  trainingServerRunning = true;

  //Record the settings used for training
  petWDT();
  recordSystemSettings();

  //Display the values to be used for the client/server training
  systemPrintln("Server training parameters");
  systemPrint("  Training netID: ");
  systemPrintln(settings.netID);
  systemPrint("  Training key: ");
  displayEncryptionKey(settings.trainingKey);
  systemPrintln();

  //Common initialization
  commonTrainingInitialization();
  settings.server = true;         //52: Operate as the training server

  systemPrintln("Server radio protocol parameters");
  systemPrint("  netID: ");
  systemPrintln(tempSettings.netID);
  systemPrint("  Encryption key: ");
  displayEncryptionKey(tempSettings.encryptionKey);
  systemPrintln();
  outputSerialData(true);

  //Start the receive operation
  returnToReceiving();

  //Set the next state
  changeState(RADIO_TRAIN_WAIT_FOR_PING);
}

//Perform the common training initialization
void commonTrainingInitialization()
{
  //Save the current settings
  tempSettings = settings;

  //Use common radio settings between the client and server for training
  settings = defaultSettings;
  settings.dataScrambling = true;           //Scramble the data
  settings.enableCRC16 = true;              //Use CRC-16
  settings.encryptData = true;              //Enable packet encryption
  settings.frequencyHop = false;            //Stay on the training frequency
  settings.netID = 'T';                     //NetID for training
  settings.operatingMode = MODE_MULTIPOINT; //Use datagrams
  settings.radioBroadcastPower_dbm = 14;    //Minimum, assume radios are near each other
  settings.selectLedUse = LEDS_CYLON;       //Display the CYLON pattern on the LEDs
  settings.verifyRxNetID = true;            //Disable netID checking
  memcpy(&settings.trainingKey, &tempSettings.trainingKey, AES_KEY_BYTES); //56: Common training key

  //Determine the components of the frame header and trailer
  selectHeaderAndTrailerBytes();

  //Debug training if requested
  if (tempSettings.debugTraining)
  {
    settings.selectLedUse = tempSettings.selectLedUse;
    //Ignore copyDebug
    settings.debug = tempSettings.debug;
    settings.debugDatagrams = tempSettings.debugDatagrams;
    settings.debugNvm = tempSettings.debugNvm;
    settings.debugRadio = tempSettings.debugRadio;
    settings.debugReceive = tempSettings.debugReceive;
    settings.debugSerial = tempSettings.debugSerial;
    settings.debugStates = tempSettings.debugStates;
    settings.debugTraining = tempSettings.debugTraining;
    settings.debugTransmit = tempSettings.debugTransmit;
    settings.printPacketQuality = tempSettings.printPacketQuality;
    settings.displayRealMillis = tempSettings.displayRealMillis;
    settings.printAckNumbers = tempSettings.printAckNumbers;
    settings.printFrequency = tempSettings.printFrequency;
    settings.printLinkUpDown = tempSettings.printLinkUpDown;
    settings.printPktData = tempSettings.printPktData;
    settings.printRfData = tempSettings.printRfData;
    settings.printTimestamp = tempSettings.printTimestamp;
    settings.printTxErrors = tempSettings.printTxErrors;

    //Ignore copyTriggers
    settings.triggerEnable = tempSettings.triggerEnable;
    settings.triggerEnable2 = tempSettings.triggerEnable2;
    settings.triggerWidth = tempSettings.triggerWidth;
    settings.triggerWidthIsMultiplier = tempSettings.triggerWidthIsMultiplier;
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
  settings = tempSettings; //Return to original radio settings

  if (settings.debugTraining)
  {
    displayParameters(0, settings.copyDebug || settings.copyTriggers);
    outputSerialData(true);
  }

  if (settings.server == false)
  {
    //Record the new client settings
    petWDT();
    recordSystemSettings();

    systemPrint("Link trained from ");
    systemPrintUniqueID(trainingPartnerID);
    systemPrintln();

    outputSerialData(true);
  }

  //Done with training
  trainingServerRunning = false;

  //Reboot the radio with the new parameters
  petWDT();
  systemFlush();
  systemReset();
}
