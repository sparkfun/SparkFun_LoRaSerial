
void beginTraining()
{
  LRS_DEBUG_PRINTLN("Begin training");

  originalSettings = settings; //Make copy of current settings

  moveToTrainingFreq();
}

void beginDefaultTraining()
{
  LRS_DEBUG_PRINTLN("Begin default training");

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

  recordSystemSettings();

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

  if (settings.pointToPoint == true)
    changeState(RADIO_NO_LINK_RECEIVING_STANDBY);
  else
    changeState(RADIO_BROADCASTING_RECEIVING_STANDBY);

  sentFirstPing = false; //Send ping as soon as we exit

  systemPrintln("LINK TRAINED");
}

/*
      beginTraining                beginDefaultTraining
            | Save current settings          | Save default settings
            V                                |
            +<-------------------------------’
            |
            V
    moveToTrainingFreq
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


    moveToTrainingFreq

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
  sendTrainingPingPacket();

  //Recalculate packetAirTime because we need to wait not for a 2-byte response, but a 19 byte response
  packetAirTime = calcAirTime(sizeof(trainEncryptionKey) + sizeof(trainNetID) + 2);

  //Reset cylon variables
  startCylonLEDs();

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

