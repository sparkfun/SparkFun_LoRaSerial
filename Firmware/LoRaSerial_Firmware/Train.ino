
void beginTraining()
{
  originalSettings = settings; //Make copy of current settings

  moveToTrainingFreq();
}

void beginDefaultTraining()
{
  Settings defaultSettings;
  originalSettings = defaultSettings; //Upon completion we will return to default settings

  moveToTrainingFreq();
}

//Upon successful exchange of keys, go back to original settings
void endTraining(bool newTrainingAvailable)
{
  settings = originalSettings; //Return to original radio settings

  //Apply new netID and AES if available
  if (newTrainingAvailable)
  {
    if (lastPacketSize == sizeof(settings.encryptionKey) + 1) //Error check, should be AES key + NetID
    {
      //Move training data into settings
      for (int x = 0 ; x < sizeof(settings.encryptionKey); x++)
        settings.encryptionKey[x] = lastPacket[x];

      settings.netID = lastPacket[lastPacketSize - 1]; //Last spot in array is netID

      if (settings.debug == true)
      {
        systemPrint("New Key: ");
        for (uint8_t i = 0 ; i < 16 ; i++)
        {
          if (settings.encryptionKey[i] < 0x10) systemPrint("0");
          systemPrint(settings.encryptionKey[i], HEX);
          systemPrint(" ");
        }
        systemPrintln();

        systemPrint("New ID: ");
        systemPrintln(settings.netID);
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
    for (int x = 0 ; x < sizeof(settings.encryptionKey); x++)
      settings.encryptionKey[x] = trainEncryptionKey[x];

    settings.netID = trainNetID; //Last spot in array is netID
  }

  recordSystemSettings();

  generateHopTable(); //Generate frequency table based on current settings

  configureRadio(); //Setup radio with settings

  returnToReceiving();

  if (settings.pointToPoint == true)
    changeState(RADIO_NO_LINK_RECEIVING_STANDBY);
  else
    changeState(RADIO_BROADCASTING_RECEIVING_STANDBY);

  systemPrintln("LINK TRAINED");
}

//Change to known training frequency based on available freq and current major firmware version
//This will allow different minor versions to continue to train to each other
//Send special packet with train = 1, then wait for response
void moveToTrainingFreq()
{
  //During training use default radio settings. This ensures both radios are at known good settings.
  Settings defaultSettings;
  settings = defaultSettings; //Move to default settings

  //Disable hopping
  settings.frequencyHop = false;

  //Disable NetID checking
  settings.pointToPoint = false;

  generateHopTable(); //Generate frequency table based on current settings

  configureRadio(); //Setup radio with settings

  //Move to frequency that is not part of the hop table
  //In normal operation we move 1/2 a channel away from min. In training, we move a full channel away + major firmware version.
  float channelSpacing = (settings.frequencyMax - settings.frequencyMin) / (float)(settings.numberOfChannels + 2);
  float trainFrequency = settings.frequencyMin + (channelSpacing * (FIRMWARE_VERSION_MAJOR % settings.numberOfChannels));

  channels[0] = trainFrequency; //Inject this frequency into the channel table

  //Transmit general ping packet to see if anyone else is sitting on the training channel
  sendTrainingPingPacket();

  //Recalculate packetAirTime because we need to wait not for a 2-byte response, but a 19 byte response
  packetAirTime = calcAirTime(sizeof(trainEncryptionKey) + sizeof(trainNetID) + 2);

  changeState(RADIO_TRAINING_TRANSMITTING);
}

//Generate new netID/AES key to share
//We assume the user needs to maintain their settings (airSpeed, numberOfChannels, freq min/max, bandwidth/spread/hop)
//but need to be on a different netID/AES key.
void generateTrainingSettings()
{
  LRS_DEBUG_PRINTLN("Generate New Training Settings");

  //Seed random number based on RF noise. We use Arduino random() because platform specific generation does not matter
  randomSeed(radio.randomByte());

  //Generate new NetID
  trainNetID = random(0, 256); //Inclusive, exclusive

  //Generate new AES Key. User may not be using AES but we still need both radios to have the same key in case they do enable AES.
  for (int x = 0 ; x < 16 ; x++)
    trainEncryptionKey[x] = random(0, 256); //Inclusive, exclusive

  //We do not generate new AES Initial Values here. Those are generated during generateHopTable() based on the unit's settings.

  if (settings.debug == true)
  {
    systemPrint(F("trainNetID: "));
    systemPrintln(trainNetID);

    systemPrint(F("trainEncryptionKey:"));
    for (uint8_t i = 0 ; i < 16 ; i++)
    {
      systemPrint(" 0x");
      if (trainEncryptionKey[i] < 0x10) systemPrint("0");
      systemPrint(trainEncryptionKey[i], HEX);
    }
    systemPrintln();
  }
}
