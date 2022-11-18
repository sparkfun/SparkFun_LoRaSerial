//Apply settings to radio
//Called after begin() and once user exits from command interface
void configureRadio()
{
  float frequency;
  bool success = true;

  frequency = channels[0];
  if (radio.setFrequency(frequency) == RADIOLIB_ERR_INVALID_FREQUENCY)
    success = false;

  //Print the frequency if requested
  if (settings.printFrequency)
  {
    systemPrintTimestamp();
    systemPrint(frequency);
    systemPrintln(" MHz");
  }

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

  if (radio.setCRC(true) == RADIOLIB_ERR_INVALID_CRC_CONFIGURATION) //Enable hardware CRC
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

  radio.setDio0Action(transactionCompleteISR); //Called when transmission is finished
  radio.setDio1Action(hopISR); //Called after a transmission has started, so we can move to next freq

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

  //Precalculate the ACK packet time
  ackAirTime = calcAirTime(headerBytes + ACK_BYTES + trailerBytes); //Used for response timeout during ACK
  if (settings.radioSpreadFactor == 6)
    ackAirTime = calcAirTime(MAX_PACKET_SIZE);

  if ((settings.debug == true) || (settings.debugRadio == true))
  {
    systemPrint("Freq: ");
    systemPrintln(channels[0], 3);
    systemPrint("radioBandwidth: ");
    systemPrintln(settings.radioBandwidth, 3);
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

//Update the settings based upon the airSpeed value
void convertAirSpeedToSettings()
{
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
}

//Set radio frequency
void setRadioFrequency(bool rxAdjust)
{
  float frequency;

  frequency = channels[channelNumber];
  if (rxAdjust)
    frequency -= frequencyCorrection;
  radio.setFrequency(frequency);

  //Print the frequency if requested
  if (settings.printFrequency)
  {
    systemPrintTimestamp();
    systemPrint(frequency);
    systemPrintln(" MHz");
  }
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
    if (expectingAck && (settings.operatingMode == MODE_POINT_TO_POINT))
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

  //Reset RSSI measurements
  rssi = 0;
  hopCount = 0;
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
  uint8_t mostFramesPerSecond = 1000 / calcAirTime(MAX_PACKET_SIZE);
  uint16_t mostBytesPerSecond = maxDatagramSize * mostFramesPerSecond;

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
  myRandSeed = settings.airSpeed
               + settings.netID
               + settings.operatingMode
               + settings.encryptData
               + settings.dataScrambling
               + (uint16_t)settings.frequencyMin
               + (uint16_t)settings.frequencyMax
               + settings.numberOfChannels
               + settings.frequencyHop
               + settings.maxDwellTime
               + (uint16_t)settings.radioBandwidth
               + settings.radioSpreadFactor
               + settings.verifyRxNetID
               + settings.overheadTime
               + settings.enableCRC16
               + settings.clientPingRetryInterval;

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
    waitForever();
  }

  //Verify the AES key length
  if (AES_KEY_BYTES != gcm.keySize())
  {
    systemPrint("ERROR - Wrong AES key size in bytes, please set AES_KEY_BYTES = ");
    systemPrintln(gcm.keySize());
    waitForever();
  }

  //Set new initial values for AES using settings based random seed
  for (uint8_t x = 0 ; x < sizeof(AESiv) ; x++)
    AESiv[x] = myRand();

  if ((settings.debug == true) || (settings.debugRadio == true))
  {
    petWDT();
    systemPrint("channelSpacing: ");
    systemPrintln(channelSpacing, 3);

    systemPrintln("Channel table:");
    for (int x = 0 ; x < settings.numberOfChannels ; x++)
    {
      petWDT();
      systemPrint(x);
      systemPrint(": ");
      systemPrint(channels[x], 3);
      systemPrintln();
    }

    systemPrint("AES IV:");
    for (uint8_t i = 0 ; i < sizeof(AESiv) ; i++)
    {
      petWDT();
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

//Move to the next channel
//This is called when the FHSS interrupt is received
//at the beginning and during of a transmission or reception

void hopChannel()
{
  hopChannel(true); //Move forward
}

void hopChannelReverse()
{
  hopChannel(false); //Move backward
}

void hopChannel(bool moveForwardThroughTable)
{
  timeToHop = false;
  triggerEvent(TRIGGER_FREQ_CHANGE);

  if (moveForwardThroughTable)
  {
    channelNumber++;
    channelNumber %= settings.numberOfChannels;
  }
  else
  {
    if (channelNumber == 0) channelNumber = settings.numberOfChannels;
    channelNumber--;
  }

  //Select the new frequency
  float frequency;
  if (settings.autoTuneFrequency == true)
  {
    if (radioStateTable[radioState].rxState)
      frequency = channels[channelNumber] - frequencyCorrection;
    else
      frequency = channels[channelNumber];
  }
  else
    frequency = channels[channelNumber];

  radio.setFrequency(frequency);
  //triggerFrequency(frequency);

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

//ISR when DIO0 goes low
//Called when transmission is complete or when RX is received
void transactionCompleteISR(void)
{
  transactionComplete = true;
}

//ISR when DIO1 goes low
//Called when FhssChangeChannel interrupt occurs (at regular HoppingPeriods)
//We do not use SX based channel hopping, and instead use a synchronized hardware timer
//We use the hop ISR to measure RSSI during reception
void hopISR(void)
{
  hop = true;
}

//As we complete linkup, different airspeeds exit at different rates
//We adjust the initial clock setup as needed
int16_t getLinkupOffset()
{
  partialTimer = true; //Mark timer so that it runs only once with less than dwell time

  int linkupOffset = 0;
  
  switch (settings.airSpeed)
  {
    default:
      break;
    case (40):
      linkupOffset = 0;
      break;
    case (150):
      linkupOffset = 0;
      break;
    case (400):
      linkupOffset = 0;
      break;
    case (1200):
      linkupOffset = 0;
      break;
    case (2400):
      linkupOffset = 0;
      break;
    case (4800):
      linkupOffset = 0;
      break;
    case (9600):
      linkupOffset = 0;
      break;
    case (19200):
      linkupOffset = 0;
      break;
    case (28800):
      linkupOffset = 6;
      break;
    case (38400):
      linkupOffset = 6;
      break;
  }

  return (settings.maxDwellTime - getReceiveCompletionOffset() - linkupOffset); //Reduce the default window by the offset
}
