//Select the training protocol
void selectTraining()
{
  if (tempSettings.server)
    beginTrainingServer();
  else
    beginTrainingClient();
}

//Generate new netID/AES key to share
//We assume the user needs to maintain their settings (airSpeed, numberOfChannels, freq min/max, bandwidth/spread/hop)
//but need to be on a different netID/AES key.
void generateRandomKeysID(Settings * radioSettings)
{
  if ((settings.debug == true) || (settings.debugTraining == true))
  {
    systemPrintln("Generate New Training Settings");
    outputSerialData(true);
  }

  //Seed random number based on RF noise. We use Arduino random() because platform specific generation does not matter
  randomSeed(radio.randomByte());

  //Generate new NetID
  radioSettings->netID = random(0, 256); //Inclusive, exclusive

  //Generate new AES Key. User may not be using AES but we still need both radios to have the same key in case they do enable AES.
  for (int x = 0 ; x < 16 ; x++)
    radioSettings->encryptionKey[x] = random(0, 256); //Inclusive, exclusive

  //We do not generate new AES Initial Values here. Those are generated during generateHopTable() based on the unit's settings.

  if ((settings.debug == true) || (settings.debugTraining == true))
  {
    systemPrint("Select new NetID: ");
    systemPrintln(radioSettings->netID);

    systemPrint("Select new Encryption Key:");
    for (uint8_t i = 0 ; i < 16 ; i++)
    {
      systemPrint(" ");
      systemPrint(radioSettings->encryptionKey[i], HEX);
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

  //Transmit FIND_PARTNER to the training server
  if (xmitDatagramTrainingFindPartner() == true)
    //Set the next state
    changeState(RADIO_TRAIN_WAIT_TX_FIND_PARTNER_DONE);
  else
    changeState(RADIO_TRAIN_WAIT_RX_RADIO_PARAMETERS);
}

//Start the training in server mode
void beginTrainingServer()
{
  petWDT();
  trainingServerRunning = true;

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
  systemPrintln(trainingSettings.netID);
  systemPrint("  Encryption key: ");
  displayEncryptionKey(trainingSettings.encryptionKey);
  systemPrintln();
  outputSerialData(true);

  //Start the receive operation
  returnToReceiving();

  //Set the next state
  changeState(RADIO_TRAIN_WAIT_FOR_FIND_PARTNER);
}

//Perform the common training initialization
void commonTrainingInitialization()
{
  //Use common radio settings between the client and server for training
  settings = defaultSettings;
  settings.dataScrambling = true;           //Scramble the data
  settings.enableCRC16 = true;              //Use CRC-16
  settings.encryptData = true;              //Enable packet encryption
  memcpy(&settings.encryptionKey, &trainingSettings.trainingKey, AES_KEY_BYTES); //Common encryption key
  settings.frequencyHop = false;            //Stay on the training frequency
  settings.maxResends = 1;                  //Don't waste time retransmitting
  settings.netID = 'T';                     //NetID for training
  settings.operatingMode = MODE_MULTIPOINT; //Use datagrams
  settings.radioBroadcastPower_dbm = trainingSettings.trainingTxPower_dbm;    //Minimum, assume radios are near each other
  settings.selectLedUse = LEDS_CYLON;       //Display the CYLON pattern on the LEDs
  settings.verifyRxNetID = true;            //Disable netID checking

  //Determine the components of the frame header and trailer
  selectHeaderAndTrailerBytes();

  //Debug training if requested
  if (trainingSettings.debugTraining)
  {
    settings.selectLedUse = trainingSettings.selectLedUse;
    //Ignore copyDebug
    settings.debug = trainingSettings.debug;
    settings.debugDatagrams = trainingSettings.debugDatagrams;
    settings.debugHeartbeat = trainingSettings.debugHeartbeat;
    settings.debugNvm = trainingSettings.debugNvm;
    settings.debugRadio = trainingSettings.debugRadio;
    settings.debugReceive = trainingSettings.debugReceive;
    settings.debugSerial = trainingSettings.debugSerial;
    settings.debugStates = trainingSettings.debugStates;
    settings.debugSync = trainingSettings.debugSync;
    settings.debugTraining = trainingSettings.debugTraining;
    settings.debugTransmit = trainingSettings.debugTransmit;
    settings.displayRealMillis = trainingSettings.displayRealMillis;
    settings.printAckNumbers = trainingSettings.printAckNumbers;
    settings.printChannel = trainingSettings.printChannel;
    settings.printFrequency = trainingSettings.printFrequency;
    settings.printLinkUpDown = trainingSettings.printLinkUpDown;
    settings.printPacketQuality = trainingSettings.printPacketQuality;
    settings.printPktData = trainingSettings.printPktData;
    settings.printRfData = trainingSettings.printRfData;
    settings.printTimestamp = trainingSettings.printTimestamp;
    settings.printTxErrors = trainingSettings.printTxErrors;

    //Ignore copyTriggers
    settings.triggerEnable = trainingSettings.triggerEnable;
    settings.triggerEnable2 = trainingSettings.triggerEnable2;
    settings.triggerWidth = trainingSettings.triggerWidth;
    settings.triggerWidthIsMultiplier = trainingSettings.triggerWidthIsMultiplier;
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
  settings = trainingSettings; //Return to original radio settings

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
  commandReset();
}
