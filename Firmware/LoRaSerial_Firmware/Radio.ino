//Apply settings to radio
//Called after begin() and once user exits from command interface
bool configureRadio()
{
  bool success = true;

  radioCallHistory[RADIO_CALL_configureRadio] = millis();

  channelNumber = 0;
  if (!setRadioFrequency(false))
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
  uint16_t hoppingPeriod = settings.maxDwellTime / calcSymbolTimeMsec(); //Limit FHSS dwell time to 400ms max. / automatically floors number
  if (hoppingPeriod > 255) hoppingPeriod = 255; //Limit to 8 bits.
  if (settings.frequencyHop == false) hoppingPeriod = 0; //Disable
  if (radio.setFHSSHoppingPeriod(hoppingPeriod) != RADIOLIB_ERR_NONE)
    success = false;

  //Precalculate the packet times
  ackAirTime = calcAirTimeMsec(headerBytes + CHANNEL_TIMER_BYTES + trailerBytes); //Used for response timeout during ACK
  systemDescriptionAirTime = calcAirTimeMsec(headerBytes + P2P_SYNC_CLOCKS_BYTES + trailerBytes); //Used for response timeout during 3-way handshake
  maxPacketAirTime = calcAirTimeMsec(MAX_PACKET_SIZE);

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
    systemPrint("calcSymbolTimeMsec: ");
    systemPrint(calcSymbolTimeMsec(), 3);
    systemPrintln(" mSec");
    systemPrint("HoppingPeriod: ");
    systemPrintln(hoppingPeriod);
    systemPrint("ackAirTime: ");
    systemPrintln(ackAirTime);
    outputSerialData(true);
  }

  if (success == false)
    systemPrintln("Radio init failed. Check settings.");
  else if ((settings.debug == true) || (settings.debugRadio == true))
    systemPrintln("Radio configured");
  outputSerialData(true);
  return success;
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
          outputSerialData(true);
        }
        break;
    }
  }
}

//Set radio frequency
bool setRadioFrequency(bool rxAdjust)
{
  float previousFrequency;
  static uint8_t previousChannelNumber;

  radioCallHistory[RADIO_CALL_setRadioFrequency] = millis();

  //Determine the frequency to use
  previousFrequency = radioFrequency;
  radioFrequency = channels[channelNumber];
  if (rxAdjust && settings.autoTuneFrequency)
    radioFrequency -= frequencyCorrection;

  //Set the new frequency
  if (radio.setFrequency(radioFrequency) == RADIOLIB_ERR_INVALID_FREQUENCY)
    return false;

  if (settings.debugSync)
    triggerFrequency(radioFrequency);
  else
    triggerEvent(TRIGGER_FREQ_CHANGE);

  //Print the frequency if requested
  if (settings.printChannel && (previousChannelNumber != channelNumber))
  {
    systemPrintTimestamp();
    systemPrint("CH");
    systemPrintln(channelNumber);
    outputSerialData(true);
  }
  if (settings.printFrequency && (previousFrequency != radioFrequency))
  {
    systemPrintTimestamp();
    systemPrint(channelNumber);
    systemPrint(": ");
    systemPrint(radioFrequency, 3);
    systemPrint(" MHz, Ch 0 in ");
    systemPrint(mSecToChannelZero());
    systemPrintln(" mSec");
    outputSerialData(true);
  }
  previousChannelNumber = channelNumber;
  return true;
}

//Place the radio in receive mode
void returnToReceiving()
{
  radioCallHistory[RADIO_CALL_returnToReceiving] = millis();

  if (receiveInProcess() == true) return; //Do not touch the radio if it is already receiving

  int state;
  if (settings.radioSpreadFactor > 6)
  {
    state = radio.startReceive();
  }
  else
  {
    if (settings.operatingMode == MODE_POINT_TO_POINT)
    {
      radio.implicitHeader(sf6ExpectedSize);
      state = radio.startReceive(sf6ExpectedSize); //Set the size we expect to see

      if (sf6ExpectedSize < MAX_PACKET_SIZE)
        triggerEvent(TRIGGER_RTR_SHORT_PACKET);
      else
        triggerEvent(TRIGGER_RTR_255BYTE);

      sf6ExpectedSize = MAX_PACKET_SIZE; //Always return to expecing a full data packet
    }
  }

  if (state != RADIOLIB_ERR_NONE) {
    startReceiveFailureMillis = rcvTimeMillis;
    startReceiveFailureState = state;
    if ((settings.debug == true) || (settings.debugRadio == true))
    {
      systemPrint("Receive failed: ");
      systemPrintln(state);
      outputSerialData(true);
    }
  }
}

//Given spread factor, bandwidth, coding rate and number of bytes, return total Air Time in ms for packet
float calcAirTimeUsec(uint8_t bytesToSend)
{
  radioCallHistory[RADIO_CALL_calcAirTimeUsec] = millis();

  float tSymUsec = calcSymbolTimeUsec();

  //See Synchronization section
  float tPreambleUsec = (settings.radioPreambleLength + 2 + 2.25) * tSymUsec;

  // Rb = SF * (1 / ((2^SF) / B) = (SF * B) / 2^SF bit rate
  //
  float p1 = (8 * bytesToSend - 4 * settings.radioSpreadFactor + 28 + 16 * 1 - 20 * 0) / (4.0 * (settings.radioSpreadFactor - 2 * 0));
  p1 = ceil(p1) * settings.radioCodingRate;
  if (p1 < 0) p1 = 0;
  uint16_t payloadBytes = 8 + p1;
  float tPayloadUsec = payloadBytes * tSymUsec;
  float tPacketUsec = tPreambleUsec + tPayloadUsec;
  return tPacketUsec;
}

//Return the frame air time in milliseconds
uint16_t calcAirTimeMsec(uint8_t bytesToSend)
{
  return ((uint16_t)ceil(calcAirTimeUsec(bytesToSend) / 1000.));
}

//Given spread factor and bandwidth, return symbol time
float calcSymbolTimeUsec()
{
  //The following documentation is from https://en.wikipedia.org/wiki/LoRa
  //
  //Each symbol is represented by a cyclic shifted chirp over the frequency
  //interval (f0-B/2,f0+B/2) where f0 is the center frequency and B the
  //bandwidth of the signal (in Hertz).
  //The spreading factor (SF) is a selectable radio parameter from 5 to 12 and
  //represents the number of bits sent per symbol and in addition determines
  //how much the information is spread over time.
  //There are M=2^SF different initial frequencies of the cyclic shifted chirp
  //(the instantaneous frequency is linearly increased and wrapped to f0-B/2
  //when it reaches the maximum frequency f0+B/2.
  //The symbol rate is determined by Rs=B/(2^SF.
  //LoRa can trade off data rate for sensitivity (assuming a fixed channel
  //bandwidth B) by selecting the SF, i.e. the amount of spread used.
  //A lower SF corresponds to a higher data rate but a worse sensitivity, a
  //higher SF implies a better sensitivity but a lower data rate.

  //The LoRa PHY is described in https://wirelesspi.com/understanding-lora-phy-long-range-physical-layer/
  //
  // SF = spread factor = number of bits per symbol
  //
  // M = 2^SF = number of samples per symbol
  //
  // B = Bandwidth of the input signal
  //
  //From the sampling theorem:
  //
  // Fs >= 2 * B for real frequencies
  // Fs >= B for complex frequencies
  //
  //Since this is a complex frequency and we want to use the lowest possible
  //sampling frequency, we have
  //
  // Fs = B = 1 / Ts
  //
  // Ts = 1 / B
  //
  // Tm = M / B = Symbol time
  //
  // F1 = B / M = frequency step used within the bandwidth
  //
  // Fm = m * F1, m = 0 - (M-1)  Frequency starting and end points
  //
  // u = B / Tm = Chirp rate
  //
  // Example: SF = 2
  //    M = 2 ^ 2 = 4
  //
  //Each symbol time (Tm), the signal changes frequencies from f0 - B/2 to f0 + B/2.
  //Each symbol has a different starting (ending) frequency, but scans the entire
  //bandwidth.
  //
  //     ___________________________ f0 + B/2
  //             /|  /|  /|  /|
  //            / | / | / |   |  /
  //           /  |/  |   | / | /
  //     _____/___|___|/__|/__|/_____
  //                                  f0 - B/2
  //   Symbol | 00 | 01 | 10 | 11 |
  //          |    |    |    |    |
  //          0   Tm   2Tm  3Tm  4Tm
  //
  //The signal is e^(j* [(u*pi*t^2)+(2*pi*Fm*t)+phi])
  //
  //I: The real portion of the signal, cos((u*pi*t^2)+(2*pi*Fm*t)+phi)
  //Q: The imaginary portion of the signal, j*sin((u*pi*t^2)+(2*pi*Fm*t)+phi)
  //
  //Decoding the signal is done by down chirping, multiplying the following
  //phase locked signal with the broadcast signal and putting it through a low
  //pass filter.
  //
  //     ___________________________ f0 + B/2
  //          |\   |\   |\   |\
  //          | \  | \  | \  | \
  //          |  \ |  \ |  \ |  \
  //     _____|___\|___\|___\|___\__
  //                                  f0 - B/2
  //          |    |    |    |    |
  //          0   Tm   2Tm  3Tm  4Tm
  //
  //The resulting signal is effectively added and offset to produce a constant
  //frequency associated with the symbol over the symbol time.
  //
  //          0   Tm   2Tm  3Tm  4Tm
  //          |    |    |    |    |
  //                         _____    B
  //                    _____
  //               _____
  //          _____
  //                                  0
  //          |    |    |    |    |
  //          0   Tm   2Tm  3Tm  4Tm
  //
  //    SF           11      10       9       8       7       6
  //    M          2048    1024     512     256     128      64
  //    B KHz      62.5    62.5     125     125     500     500
  //    Tm uSec   65536   32768    8192    4096     512     256
  //    Symbols   15.25   30.52   122.1   244.1    1953    3906
  //
  //Compute time in microseconds using bandwidth in kHz
  float tSymUsec = (pow(2, settings.radioSpreadFactor) * 1000.) / settings.radioBandwidth;
  return (tSymUsec);
}

//Return symbol time in milliseconds
float calcSymbolTimeMsec()
{
  return calcSymbolTimeUsec() / 1000.;
}

//Given spread factor, bandwidth, coding rate and frame size, return most bytes we can push per second
uint16_t calcMaxThroughput()
{
  uint8_t mostFramesPerSecond = 1000 / maxPacketAirTime;
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
               + settings.clientFindPartnerRetryInterval;

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

    petWDT();

    systemPrint("NetID: ");
    systemPrintln(settings.netID);

    systemPrint("AES Key: ");
    for (uint8_t i = 0 ; i < AES_KEY_BYTES ; i++)
      systemPrint(settings.encryptionKey[i], HEX);
    systemPrintln();

    systemPrint("AES IV: ");
    for (uint8_t i = 0 ; i < AES_IV_BYTES ; i++)
      systemPrint(AESiv[i], HEX);
    systemPrintln();

    outputSerialData(true);
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
  hopChannel(true, 1); //Move forward
}

//Hop to the previous channel in the frequency list
void hopChannelReverse()
{
  hopChannel(false, 1); //Move backward
}

//Set the next radio frequency given the hop direction and frequency table
void hopChannel(bool moveForwardThroughTable, uint8_t channelCount)
{
  radioCallHistory[RADIO_CALL_hopChannel] = millis();

  timeToHop = false;

  if (moveForwardThroughTable)
    channelNumber += channelCount;
  else
    channelNumber += settings.numberOfChannels - channelCount;
  channelNumber %= settings.numberOfChannels;

  //Select the new frequency
  setRadioFrequency(radioStateTable[radioState].rxState);
  blinkChannelHopLed(true);
}

//Determine the time in milliseconds when channel zero is reached again
unsigned long mSecToChannelZero()
{
  unsigned long nextChannelZeroTimeInMillis;
  uint16_t remainingChannels;

  //Compute the time remaining at the current channel
  nextChannelZeroTimeInMillis = channelTimerMsec - (millis() - channelTimerStart);

  //Compute the time associated with the additional channels
  remainingChannels = settings.numberOfChannels - 1 - channelNumber;
  if (remainingChannels > 0)
    nextChannelZeroTimeInMillis += remainingChannels * settings.maxDwellTime;
  return nextChannelZeroTimeInMillis;
}

//Returns true if the radio indicates we have an ongoing reception
bool receiveInProcess()
{
  //triggerEvent(TRIGGER_RECEIVE_IN_PROCESS_START);

  uint8_t radioStatus = radio.getModemStatus();
  if (radioStatus & RECEIVE_IN_PROCESS_MASK)
  {
    //If any bits are set there is a receive in progress
    if ((lastModemStatus & RECEIVE_IN_PROCESS_MASK) == 0)
      lastReceiveInProcessTrue = millis();
    return (true);
  }
  lastModemStatus = radioStatus;
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
//Read a register from the SX1276 chip
uint8_t readSX1276Register(uint8_t reg)
{
  radioCallHistory[RADIO_CALL_readSX1276Register] = millis();

  return radio._mod->SPIreadRegister(reg);
}

//SX1276 LoRa Register Names
const char * const sx1276RegisterNames[] =
{
  NULL,           "RegOpMode",      NULL,                 NULL,                 // 0 -  3
  NULL,           NULL,             "RegFrfMsb",          "RegFrfMid",          // 4 -  7
  "RegFrfLsb",    "RegPaConfig",    "RegPaRamp",          "RegOcp",             // 8 -  b
  "RegLna",       "RegFifoAddrPtr", "RegFifoTxBaseAddr",  "regFifoRxBaseAddr",  // c -  f

  "FifoRxCurrentAddr", "RegIrqFlagsMask", "RegIrqFlags",  "RegRxNbBytes",       //10 - 13
  "RegRxHeaderCntValueMsb", "RegRxHeaderCntValueLsb", "RegRxPacketCntValueMsb", "RegRxPacketCntValueLsb", //14 - 17
  "RegModemStat", "RegPktSnrValue", "RegPktRssiValue",    "RegRssiValue",       //18 - 1b
  "RegHopChannel", "RegModemConfig1", "RegModemConfig2",   "RegSymbTimeoutLsb", //1c - 1f

  "RegPreambleMsb", "RegPreambleLsb", "RegPayloadLength", "RegMaxPayloadLength",//20 - 23
  "RegHopPeriod", "RegFifoRxByteAddr", "RegModemConfig3", NULL,                 //24 - 27
  "RegFeiMsb",    "RegFeiMid",        "RegFeiLsb",        NULL,                 //28 - 2b
  "RegRssiWideband", NULL,              NULL,             "RegIfFreq1",         //2c - 2f

  "RegIfFreq2",   "ReqDetectOptimize",  NULL,             "ReqInvertIQ",        //30 - 33
  NULL,           NULL,         "RegHighBwOpimize1",  "RegDetectionThreshold",  //34 - 37
  NULL,           "RegSyncWord",      "RegHighBwOptimize2", "RegInvertIQ2",     //38 - 3b
  NULL,           NULL,               NULL,               NULL,                 //3c - 3f

  "RegDioMapping1", "RegDioMapping2", "RegVersion",       NULL,                 //40 - 43
  NULL,           NULL,               NULL,               NULL,                 //44 - 47
  NULL,           NULL,               NULL,               "RegTcxo",            //48 - 4b
  NULL,           "RegPaDac",         NULL,               NULL,                 //4C - 4F

  NULL,           NULL,               NULL,               NULL,                 //50 - 53
  NULL,           NULL,               NULL,               NULL,                 //54 - 57
  NULL,           NULL,               NULL,               "RegFormerTemp",      //58 - 5b
  NULL,           NULL,               NULL,               NULL,                 //5c - 5f

  NULL,           "RegAgcRef",        "RegAgcThresh1",    "RegAgcThresh2",      //60 - 63
  "RegAgcThresh3", NULL,              NULL,               NULL,                 //64 - 67
  NULL,           NULL,               NULL,               NULL,                 //68 - 6b
  NULL,           NULL,               NULL,               NULL,                 //6c - 6f

  "RegPII",                                                                     //70
};

//Print the SX1276 LoRa registers
void printSX1276Registers()
{
  radioCallHistory[RADIO_CALL_printSX1276Registers] = millis();

  petWDT();
  systemPrintln("SX1276 Registers:");
  systemPrintln("     Reg Value  Name");
  for (uint8_t i = 0; i < (sizeof(sx1276RegisterNames) / sizeof(sx1276RegisterNames[0])); i++)
  {
    //Only read and print the valid registers
    if (sx1276RegisterNames[i])
    {
      systemPrint("    0x");
      systemPrint(i, HEX);
      systemPrint(": 0x");
      systemPrint(readSX1276Register(i), HEX);
      systemPrint(", ");
      systemPrintln(sx1276RegisterNames[i]);
      outputSerialData(true);
      petWDT();
    }
  }
}

#endif  //RADIOLIB_LOW_LEVEL

//ISR when DIO0 goes low
//Called when transmission is complete or when RX is received
void transactionCompleteISR(void)
{
  transactionCompleteMicros = micros();
  radioCallHistory[RADIO_CALL_transactionCompleteISR] = millis();
  triggerEvent(TRIGGER_TRANSACTION_COMPLETE);

  //Start the channel timer if requested
  if (startChannelTimerPending)
  {
    startChannelTimer(); //transactionCompleteISR, upon TX or RX of SYNC_CLOCKS
    startChannelTimerPending = false; //transactionCompleteISR, upon TX or RX of SYNC_CLOCKS
  }

  //Signal the state machine that a receive or transmit has completed
  transactionComplete = true;
}

//ISR when DIO1 goes low
//Called when FhssChangeChannel interrupt occurs (at regular HoppingPeriods)
//We do not use SX based channel hopping, and instead use a synchronized hardware timer
//We use the hop ISR to measure RSSI during reception
void hopISR(void)
{
  radioCallHistory[RADIO_CALL_hopISR] = millis();

  hop = true;
}

//We clear the hop ISR just to make logic analyzer data cleaner
void updateHopISR()
{
  if (hop) //Clear hop ISR as needed
  {
    hop = false;
    radio.clearFHSSInt(); //Clear the interrupt
  }
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//Table for use in calculating the software CRC-16
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

const uint16_t crc16Table[256] =
{
  0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
  0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
  0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
  0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
  0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
  0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
  0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
  0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
  0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
  0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
  0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
  0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
  0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
  0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
  0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
  0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
  0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
  0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
  0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
  0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
  0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
  0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
  0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
  0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
  0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
  0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
  0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
  0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
  0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
  0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
  0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
  0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//Point-To-Point: Bring up the link
//
//A three way handshake is used to get both systems to agree that data can flow in both
//directions. This handshake is also used to synchronize the HOP timer and zero the ACKs.
/*
                System A                 System B

                 RESET                     RESET
                   |                         |
         Channel 0 |                         | Channel 0
                   V                         V
       .----> P2P_NO_LINK               P2P_NO_LINK
       |           | Tx FIND_PARTNER         |
       | Timeout   |                         |
       |           V                         |
       | P2P_WAIT_TX_FIND_PARTNER_DONE       |
       |           |                         |
       |           | Tx Complete - - - - - > | Rx FIND_PARTNER
       |           |   Start Rx              |
       |           |   MAX_PACKET_SIZE       |
       |           V                         V
       `---- P2P_WAIT_SYNC_CLOCKS            +<----------------------------.
                   |                         | Tx SYNC_CLOCKS              |
                   |                         V                             |
                   |              P2P_WAIT_TX_SYNC_CLOCKS_DONE             |
                   |                         |                             |
    Rx SYNC_CLOCKS | < - - - - - - - - - - - | Tx Complete                 |
                   |                         |   Start Rx                  |
                   |                         |   MAX_PACKET_SIZE           |
                   |                         |                             |
                   V                         V         Timeout             |
  .--------------->+                P2P_WAIT_ZERO_ACKS ------------------->+
  |                |                         |                             ^
  |                | TX ZERO_ACKS            |                             |
  |                |                         |                             |
  |                V                         |                             |
  |    P2P_WAIT_TX_ZERO_ACKS_DONE            |                             |
  |                | Tx Complete - - - - - > | Rx ZERO_ACKS                |
  | Stop           |   Start HOP timer       |   Start HOP Timer           | Stop
  | HOP            |   Start Rx              |   Start Rx                  | HOP
  | Timer          |   MAX_PACKET_SIZE       |   MAX_PACKET_SIZE           | Timer
  |                |   Zero ACKs             |   Zero ACKs                 |
  |       Rx       |                         |                             |
  |    SYNC_CLOCKS V                         V         Rx FIND_PARTNER     |
  `---------- P2P_LINK_UP               P2P_LINK_UP -----------------------’
                   |                         |
                   | Rx Data                 | Rx Data
                   |                         |
                   V                         V

  Two timers are in use:
    datagramTimer:  Set at end of transmit, measures ACK timeout
    heartbeatTimer: Set upon entry to P2P_NO_LINK, measures time to send next FIND_PARTNER
*/
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//First packet in the three way handshake to bring up the link
bool xmitDatagramP2PFindPartner()
{
  uint8_t * startOfData;

  unsigned long currentMillis = millis();
  radioCallHistory[RADIO_CALL_xmitDatagramP2PFindPartner] = currentMillis;

  startOfData = endOfTxData;
  memcpy(endOfTxData, &currentMillis, sizeof(currentMillis));
  endOfTxData += sizeof(unsigned long);

  /*
                                              endOfTxData ---.
                                                             |
                                                             V
      +----------+---------+----------+------------+---------+----------+
      | Optional |         | Optional | Optional   |         |          |
      | NET ID   | Control | C-Timer  | SF6 Length | Millis  | Trailer  |
      | 8 bits   | 8 bits  | 2 bytes  | 8 bits     | 4 bytes | n Bytes  |
      +----------+---------+----------+------------+---------+----------+
  */

  //Verify the data length
  if ((endOfTxData - startOfData) != P2P_FIND_PARTNER_BYTES)
  {
    systemPrintln("ERROR - Fix the P2P_FIND_PARTNER_BYTES value!");
    outputSerialData(true);
    waitForever();
  }

  txControl.datagramType = DATAGRAM_FIND_PARTNER;
  return (transmitDatagram());
}

//Second packet in the three way handshake to bring up the link
//SYNC_CLOCKS packet sent by server in response the FIND_PARTNER, includes the
//channel number.  During discovery scanning, it's possible for the client to
//get the SYNC_CLOCKS but be on an adjacent channel.  The channel number
//ensures that the client gets the next hop correct.
bool xmitDatagramP2PSyncClocks()
{
  uint8_t * startOfData;

  unsigned long currentMillis = millis();
  radioCallHistory[RADIO_CALL_xmitDatagramP2PSyncClocks] = currentMillis;

  startOfData = endOfTxData;
  memcpy(endOfTxData, &channelNumber, sizeof(channelNumber));
  endOfTxData += sizeof(channelNumber);

  memcpy(endOfTxData, &currentMillis, sizeof(currentMillis));
  endOfTxData += sizeof(unsigned long);

  /*
                                                        endOfTxData ---.
                                                                       |
                                                                       V
      +----------+---------+----------+------------+---------+---------+----------+
      | Optional |         | Optional | Optional   | Channel |         |          |
      | NET ID   | Control | C-Timer  | SF6 Length | Number  | Millis  | Trailer  |
      | 8 bits   | 8 bits  | 2 bytes  | 8 bits     | 1 byte  | 4 bytes | n Bytes  |
      +----------+---------+----------+------------+---------+---------+----------+
  */

  //Verify the data length
  if ((endOfTxData - startOfData) != P2P_SYNC_CLOCKS_BYTES)
  {
    systemPrintln("ERROR - Fix the P2P_SYNC_CLOCKS_BYTES value!");
    outputSerialData(true);
    waitForever();
  }

  txControl.datagramType = DATAGRAM_SYNC_CLOCKS;
  return (transmitDatagram());
}

//Last packet in the three way handshake to bring up the link
bool xmitDatagramP2PZeroAcks()
{
  uint8_t * startOfData;

  unsigned long currentMillis = millis();
  radioCallHistory[RADIO_CALL_xmitDatagramP2PZeroAcks] = currentMillis;

  startOfData = endOfTxData;
  memcpy(endOfTxData, &currentMillis, sizeof(currentMillis));
  endOfTxData += sizeof(unsigned long);

  /*
                                              endOfTxData ---.
                                                             |
                                                             V
      +----------+---------+----------+------------+---------+----------+
      | Optional |         | Optional | Optional   |         |          |
      | NET ID   | Control | C-Timer  | SF6 Length | Millis  | Trailer  |
      | 8 bits   | 8 bits  | 2 bytes  | 8 bits     | 4 bytes | n Bytes  |
      +----------+---------+----------+------------+---------+----------+
  */

  //Verify the data length
  if ((endOfTxData - startOfData) != P2P_ZERO_ACKS_BYTES)
  {
    systemPrintln("ERROR - Fix the P2P_ZERO_ACKS_BYTES value!");
    outputSerialData(true);
    waitForever();
  }

  txControl.datagramType = DATAGRAM_ZERO_ACKS;
  return (transmitDatagram());
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Point-to-Point Data Exchange
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Send a command datagram to the remote system
bool xmitDatagramP2PCommand()
{
  radioCallHistory[RADIO_CALL_xmitDatagramP2PCommand] = millis();

  /*
                                                  endOfTxData ---.
                                                                 |
                                                                 V
      +----------+---------+----------+------------+---  ...  ---+----------+
      | Optional |         | Optional | Optional   |             | Optional |
      | NET ID   | Control | C-Timer  | SF6 Length |   Data      | Trailer  |
      | 8 bits   | 8 bits  | 2 bytes  | 8 bits     |   n bytes   | n Bytes  |
      +----------+---------+----------+------------+-------------+----------+
  */

  txControl.datagramType = DATAGRAM_REMOTE_COMMAND;
  return (transmitDatagram());
}

//Send a command response datagram to the remote system
bool xmitDatagramP2PCommandResponse()
{
  radioCallHistory[RADIO_CALL_xmitDatagramP2PCommandResponse] = millis();

  /*
                                                  endOfTxData ---.
                                                                 |
                                                                 V
      +----------+---------+----------+------------+---  ...  ---+----------+
      | Optional |         | Optional | Optional   |             | Optional |
      | NET ID   | Control | C-Timer  | SF6 Length |   Data      | Trailer  |
      | 8 bits   | 8 bits  | 2 bytes  | 8 bits     |   n bytes   | n Bytes  |
      +----------+---------+----------+------------+-------------+----------+
  */

  txControl.datagramType = DATAGRAM_REMOTE_COMMAND_RESPONSE;
  return (transmitDatagram());
}

//Send a data datagram to the remote system
bool xmitDatagramP2PData()
{
  radioCallHistory[RADIO_CALL_xmitDatagramP2PData] = millis();

  /*
                                                  endOfTxData ---.
                                                                 |
                                                                 V
      +----------+---------+----------+------------+---  ...  ---+----------+
      | Optional |         | Optional | Optional   |             | Optional |
      | NET ID   | Control | C-Timer  | SF6 Length |   Data      | Trailer  |
      | 8 bits   | 8 bits  | 2 bytes  | 8 bits     |   n bytes   | n Bytes  |
      +----------+---------+----------+------------+-------------+----------+
  */

  txControl.datagramType = DATAGRAM_DATA;
  return (transmitDatagram());
}

//Heartbeat packet to keep the link up
bool xmitDatagramP2PHeartbeat()
{
  uint8_t * startOfData;

  unsigned long currentMillis = millis();
  radioCallHistory[RADIO_CALL_xmitDatagramP2PHeartbeat] = currentMillis;

  startOfData = endOfTxData;
  memcpy(endOfTxData, &currentMillis, sizeof(currentMillis));
  endOfTxData += sizeof(currentMillis);

  /*
                                              endOfTxData ---.
                                                             |
                                                             V
      +----------+---------+----------+------------+---------+----------+
      | Optional |         | Optional | Optional   |         | Optional |
      | NET ID   | Control | C-Timer  | SF6 Length | Millis  | Trailer  |
      | 8 bits   | 8 bits  | 2 bytes  | 8 bits     | 4 bytes | n Bytes  |
      +----------+---------+----------+------------+---------+----------+
  */

  //Verify the data length
  if ((endOfTxData - startOfData) != P2P_HEARTBEAT_BYTES)
  {
    systemPrintln("ERROR - Fix the P2P_HEARTBEAT_BYTES value!");
    outputSerialData(true);
    waitForever();
  }

  txControl.datagramType = DATAGRAM_HEARTBEAT;
  return (transmitDatagram());
}

//Create short packet - do not expect ack
bool xmitDatagramP2PAck()
{
  uint8_t * startOfData;

  unsigned long currentMillis = millis();
  radioCallHistory[RADIO_CALL_xmitDatagramP2PAck] = currentMillis;

  startOfData = endOfTxData;
  memcpy(endOfTxData, &currentMillis, sizeof(currentMillis));
  endOfTxData += sizeof(currentMillis);

  /*
                                              endOfTxData ---.
                                                             |
                                                             V
      +----------+---------+----------+------------+---------+----------+
      | Optional |         | Optional | Optional   |         | Optional |
      | NET ID   | Control | C-Timer  | SF6 Length | Millis  | Trailer  |
      | 8 bits   | 8 bits  | 2 bytes  | 8 bits     | 4 bytes | n Bytes  |
      +----------+---------+----------+------------+---------+----------+
  */

  //Verify the data length
  if ((endOfTxData - startOfData) != P2P_ACK_BYTES)
  {
    systemPrintln("ERROR - Fix the P2P_ACK_BYTES value!");
    outputSerialData(true);
    waitForever();
  }

  txControl.datagramType = DATAGRAM_DATA_ACK;
  return (transmitDatagram());
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Multi-Point Data Exchange
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Send a data datagram to the remote system, including sync data
bool xmitDatagramMpData()
{
  radioCallHistory[RADIO_CALL_xmitDatagramMpData] = millis();

  /*
                                                  endOfTxData ---.
                                                                 |
                                                                 V
      +----------+---------+----------+------------+---  ...  ---+----------+
      | Optional |         | Optional | Optional   |             | Optional |
      | NET ID   | Control | C-Timer  | SF6 Length |   Data      | Trailer  |
      | 8 bits   | 8 bits  | 2 bytes  | 8 bits     |   n bytes   | n Bytes  |
      +----------+---------+----------+------------+-------------+----------+
  */

  txControl.datagramType = DATAGRAM_DATA;
  return (transmitDatagram());
}

//Heartbeat packet to sync other units in multipoint mode
bool xmitDatagramMpHeartbeat()
{
  uint8_t * startOfData;

  radioCallHistory[RADIO_CALL_xmitDatagramMpHeartbeat] = millis();

  startOfData = endOfTxData;

  /*
                                    endOfTxData ---.
                                                   |
                                                   V
      +----------+---------+----------+------------+----------+
      | Optional |         | Optional | Optional   | Optional |
      | NET ID   | Control | C-Timer  | SF6 Length | Trailer  |
      | 8 bits   | 8 bits  | 2 bytes  | 8 bits     | n Bytes  |
      +----------+---------+----------+------------+----------+
  */

  //Verify the data length
  if ((endOfTxData - startOfData) != MP_HEARTBEAT_BYTES)
  {
    systemPrintln("ERROR - Fix the MP_HEARTBEAT_BYTES value!");
    outputSerialData(true);
    waitForever();
  }

  txControl.datagramType = DATAGRAM_HEARTBEAT;
  return (transmitDatagram());
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//Multi-Point Client Training
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Build the FIND_PARTNER packet used for training
bool xmitDatagramTrainingFindPartner()
{
  radioCallHistory[RADIO_CALL_xmitDatagramTrainingFindPartner] = millis();

  //Add the source (server) ID
  memcpy(endOfTxData, myUniqueId, UNIQUE_ID_BYTES);
  endOfTxData += UNIQUE_ID_BYTES;

  /*
                                                endOfTxData ---.
                                                               |
                                                               V
      +----------+---------+----------+------------+-----------+----------+
      | Optional |         | Optional | Optional   |           | Optional |
      | NET ID   | Control | C-Timer  | SF6 Length | Client ID | Trailer  |
      | 8 bits   | 8 bits  | 2 bytes  | 8 bits     | 16 Bytes  | n Bytes  |
      +----------+---------+----------+------------+-----------+----------+
  */

  txControl.datagramType = DATAGRAM_TRAINING_FIND_PARTNER;
  return (transmitDatagram());
}

//Build the client ACK packet used for training
bool xmitDatagramTrainingAck(uint8_t * serverID)
{
  radioCallHistory[RADIO_CALL_xmitDatagramTrainingAck] = millis();

  //Add the destination (server) ID
  memcpy(endOfTxData, serverID, UNIQUE_ID_BYTES);
  endOfTxData += UNIQUE_ID_BYTES;

  //Add the source (client) ID
  memcpy(endOfTxData, myUniqueId, UNIQUE_ID_BYTES);
  endOfTxData += UNIQUE_ID_BYTES;

  /*
                                                endOfTxData ---.
                                                               |
                                                               V
      +----------+---------+----------+------------+-----------+----------+
      | Optional |         | Optional | Optional   |           | Optional |
      | NET ID   | Control | C-Timer  | SF6 Length | Client ID | Trailer  |
      | 8 bits   | 8 bits  | 2 bytes  | 8 bits     | 16 Bytes  | n Bytes  |
      +----------+---------+----------+------------+-----------+----------+
  */

  txControl.datagramType = DATAGRAM_TRAINING_ACK;
  return (transmitDatagram());
}

//Copy the training parameters received from the server into the settings structure
//that will eventually be written into the NVM
void updateRadioParameters(uint8_t * rxData)
{
  Settings params;

  //Get the parameters
  memcpy(&params, rxData, sizeof(params));

  //Update the radio parameters
  tempSettings.airSpeed = params.airSpeed;
  tempSettings.autoTuneFrequency = params.autoTuneFrequency;
  tempSettings.radioBandwidth = params.radioBandwidth;
  tempSettings.radioCodingRate = params.radioCodingRate;
  tempSettings.frequencyHop = params.frequencyHop;
  tempSettings.frequencyMax = params.frequencyMax;
  tempSettings.frequencyMin = params.frequencyMin;
  tempSettings.maxDwellTime = params.maxDwellTime;
  tempSettings.numberOfChannels = params.numberOfChannels;
  tempSettings.radioPreambleLength = params.radioPreambleLength;
  tempSettings.radioSpreadFactor = params.radioSpreadFactor;
  tempSettings.radioSyncWord = params.radioSyncWord;
  tempSettings.radioBroadcastPower_dbm = params.radioBroadcastPower_dbm;

  //Update the radio protocol parameters
  tempSettings.dataScrambling = params.dataScrambling;
  tempSettings.enableCRC16 = params.enableCRC16;
  tempSettings.encryptData = params.encryptData;
  memcpy(tempSettings.encryptionKey, params.encryptionKey, sizeof(tempSettings.encryptionKey));
  tempSettings.serialTimeoutBeforeSendingFrame_ms = params.serialTimeoutBeforeSendingFrame_ms;
  tempSettings.heartbeatTimeout = params.heartbeatTimeout;
  tempSettings.maxResends = params.maxResends;
  tempSettings.netID = params.netID;
  tempSettings.operatingMode = params.operatingMode;
  tempSettings.overheadTime = params.overheadTime;
  tempSettings.server = params.server;
  tempSettings.verifyRxNetID = params.verifyRxNetID;
  tempSettings.framesToYield = params.framesToYield;

  //Update the debug parameters
  if (params.copyDebug)
  {
    tempSettings.debug = params.debug;
    tempSettings.copyDebug = params.copyDebug;
    tempSettings.debug = params.debug;
    tempSettings.debugDatagrams = params.debugDatagrams;
    tempSettings.debugHeartbeat = params.debugHeartbeat;
    tempSettings.debugNvm = params.debugNvm;
    tempSettings.debugRadio = params.debugRadio;
    tempSettings.debugReceive = params.debugReceive;
    tempSettings.debugStates = params.debugStates;
    tempSettings.debugSync = params.debugSync;
    tempSettings.debugTraining = params.debugTraining;
    tempSettings.debugTransmit = params.debugTransmit;
    tempSettings.debugSerial = params.debugSerial;
    tempSettings.printPacketQuality = params.printPacketQuality;
    tempSettings.displayRealMillis = params.displayRealMillis;
    tempSettings.printAckNumbers = params.printAckNumbers;
    tempSettings.printFrequency = params.printFrequency;
    tempSettings.printLinkUpDown = params.printLinkUpDown;
    tempSettings.printPktData = params.printPktData;
    tempSettings.printRfData = params.printRfData;
    tempSettings.printTimestamp = params.printTimestamp;
    tempSettings.printTxErrors = params.printTxErrors;
    tempSettings.selectLedUse = params.selectLedUse;
  }

  //Update the serial parameters
  if (params.copySerial)
  {
    tempSettings.copySerial = params.copySerial;
    tempSettings.echo = params.echo;
    tempSettings.flowControl = params.flowControl;
    tempSettings.invertCts = params.invertCts;
    tempSettings.invertRts = params.invertRts;
    tempSettings.serialSpeed = params.serialSpeed;
    tempSettings.usbSerialWait = params.usbSerialWait;
  }

  //Update the training values
  tempSettings.clientFindPartnerRetryInterval = params.clientFindPartnerRetryInterval;
  //The trainingKey is already the same
  tempSettings.trainingTimeout = params.trainingTimeout;

  //Update the trigger parameters
  if (params.copyTriggers)
  {
    tempSettings.copyTriggers = params.copyTriggers;
    tempSettings.triggerEnable = params.triggerEnable;
    tempSettings.triggerEnable2 = params.triggerEnable2;
    tempSettings.triggerWidth = params.triggerWidth;
    tempSettings.triggerWidthIsMultiplier = params.triggerWidthIsMultiplier;
  }
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//Server Training
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Build the server parameters packet used for training
bool xmitDatagramTrainRadioParameters(const uint8_t * clientID)
{
  Settings params;

  radioCallHistory[RADIO_CALL_xmitDatagramTrainRadioParameters] = millis();

  //Initialize the radio parameters
  memcpy(&params, &tempSettings, sizeof(settings));
  params.server = false;

  //Add the destination (client) ID
  memcpy(endOfTxData, clientID, UNIQUE_ID_BYTES);
  endOfTxData += UNIQUE_ID_BYTES;

  //Add the source (server) ID
  memcpy(endOfTxData, myUniqueId, UNIQUE_ID_BYTES);
  endOfTxData += UNIQUE_ID_BYTES;

  //Add the radio parameters
  memcpy(endOfTxData, &params, sizeof(params));
  endOfTxData += sizeof(params);

  /*
                                                                          endOfTxData ---.
                                                                                         |
                                                                                         V
      +----------+---------+----------+------------+-----------+-----------+---  ...  ---+----------+
      | Optional |         | Optional | Optional   |           |           | Radio       | Optional |
      | NET ID   | Control | C-Timer  | SF6 Length | Client ID | Server ID | Parameters  | Trailer  |
      | 8 bits   | 8 bits  | 2 bytes  | 8 bits     | 16 Bytes  | 16 Bytes  | n bytes     | n Bytes  |
      +----------+---------+----------+------------+-----------+-----------+-------------+----------+
  */

  txControl.datagramType = DATAGRAM_TRAINING_PARAMS;
  return (transmitDatagram());
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//Virtual Circuit frames
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Broadcast a datagram to all of the VCs
bool xmitVcDatagram()
{
  radioCallHistory[RADIO_CALL_xmitVcDatagram] = millis();

  /*
                                                                   endOfTxData ---.
                                                                                  |
                                                                                  V
      +----------+---------+----------+------------+----------+---------+---------+----------+
      | Optional |         | Optional | Optional   |          |         |         | Optional |
      | NET ID   | Control | C-Timer  | SF6 Length | DestAddr | SrcAddr | Data    | Trailer  |
      | 8 bits   | 8 bits  | 2 bytes  | 8 bits     | 8 bits   | 8 bits  | n Bytes | n Bytes  |
      +----------+---------+----------+------------+----------+---------+---------+----------+
  */

  txControl.datagramType = DATAGRAM_DATAGRAM;
  txControl.ackNumber = 0;
  return (transmitDatagram());
}

//Broadcast a HEARTBEAT to all of the VCs
bool xmitVcHeartbeat()
{
  return xmitVcHeartbeat(VC_IGNORE_TX, myUniqueId);
}

bool xmitVcHeartbeat(int8_t addr, uint8_t * id)
{
  uint32_t currentMillis = millis();
  uint8_t * startOfData;
  uint8_t * txData;

  radioCallHistory[RADIO_CALL_xmitVcHeartbeat] = currentMillis;

  startOfData = endOfTxData;

  //Build the VC header
  txData = endOfTxData;
  *endOfTxData++ = 0; //Reserve for length
  *endOfTxData++ = VC_BROADCAST;
  *endOfTxData++ = addr;

  //Add this radio's unique ID
  memcpy(endOfTxData, id, UNIQUE_ID_BYTES);
  endOfTxData += UNIQUE_ID_BYTES;

  //Add the current time for timestamp synchronization
  memcpy(endOfTxData, &currentMillis, sizeof(currentMillis));
  endOfTxData += sizeof(currentMillis);

  //Set the length field
  *txData = (uint8_t)(endOfTxData - txData);

  /*
                                                                                         endOfTxData ---.
                                                                                                        |
                                                                                                        V
      +----------+---------+----------+------------+----------+----------+---------+----------+---------+----------+
      | Optional |         | Optional | Optional   |          |          |         |          |         | Optional |
      | NET ID   | Control | C-Timer  | SF6 Length |  Length  | DestAddr | SrcAddr | Src ID   | millis  | Trailer  |
      | 8 bits   | 8 bits  | 2 bytes  | 8 bits     |  8 bits  | 8 bits   | 8 bits  | 16 Bytes | 4 Bytes | n Bytes  |
      +----------+---------+----------+------------+----------+----------+---------+----------+---------+----------+
  */

  //Verify the data length
  if ((endOfTxData - startOfData) != VC_HEARTBEAT_BYTES)
  {
    systemPrintln("ERROR - Fix the VC_HEARTBEAT_BYTES value!");
    outputSerialData(true);
    waitForever();
  }

  txControl.datagramType = DATAGRAM_VC_HEARTBEAT;
  txControl.ackNumber = 0;

  //Select a random for the next heartbeat
  setVcHeartbeatTimer();
  return (transmitDatagram());
}

//Build the ACK frame
bool xmitVcAckFrame(int8_t destVc)
{
  VC_RADIO_MESSAGE_HEADER * vcHeader;

  radioCallHistory[RADIO_CALL_xmitVcAckFrame] = millis();

  vcHeader = (VC_RADIO_MESSAGE_HEADER *)endOfTxData;
  vcHeader->length = VC_RADIO_HEADER_BYTES;
  vcHeader->destVc = destVc;
  vcHeader->srcVc = myVc;
  endOfTxData += VC_RADIO_HEADER_BYTES;

  /*
                                                         endOfTxData ---.
                                                                        |
                                                                        V
      +----------+---------+----------+------------+----------+---------+----------+
      | Optional |         | Optional | Optional   |          |         | Optional |
      | NET ID   | Control | C-Timer  | SF6 Length | DestAddr | SrcAddr | Trailer  |
      | 8 bits   | 8 bits  | 2 bytes  | 8 bits     | 8 bits   | 8 bits  | n Bytes  |
      +----------+---------+----------+------------+----------+---------+----------+
  */

  //Finish building the ACK frame
  txControl.datagramType = DATAGRAM_DATA_ACK;
  return (transmitDatagram());
}

//Build and transmit the UNKNOWN_ACKS frame, first frame in 3-way ACKs handshake
bool xmitVcUnknownAcks(int8_t destVc)
{
  VC_RADIO_MESSAGE_HEADER * vcHeader;

  radioCallHistory[RADIO_CALL_xmitVcUnknownAcks] = millis();

  vcHeader = (VC_RADIO_MESSAGE_HEADER *)endOfTxData;
  vcHeader->length = VC_RADIO_HEADER_BYTES;
  vcHeader->destVc = destVc;
  vcHeader->srcVc = myVc;
  endOfTxData += VC_RADIO_HEADER_BYTES;

  /*
                                                         endOfTxData ---.
                                                                        |
                                                                        V
      +----------+---------+----------+------------+----------+---------+----------+
      | Optional |         | Optional | Optional   |          |         | Optional |
      | NET ID   | Control | C-Timer  | SF6 Length | DestAddr | SrcAddr | Trailer  |
      | 8 bits   | 8 bits  | 2 bytes  | 8 bits     | 8 bits   | 8 bits  | n Bytes  |
      +----------+---------+----------+------------+----------+---------+----------+
  */

  txControl.datagramType = DATAGRAM_VC_UNKNOWN_ACKS;
  return (transmitDatagram());
}

//Build and transmit the SYNC_ACKS frame, second frame in 3-way ACKs handshake
bool xmitVcSyncAcks(int8_t destVc)
{
  VC_RADIO_MESSAGE_HEADER * vcHeader;

  radioCallHistory[RADIO_CALL_xmitVcSyncAcks] = millis();

  vcHeader = (VC_RADIO_MESSAGE_HEADER *)endOfTxData;
  vcHeader->length = VC_RADIO_HEADER_BYTES;
  vcHeader->destVc = destVc;
  vcHeader->srcVc = myVc;
  endOfTxData += VC_RADIO_HEADER_BYTES;

  if(settings.debugSync)
  {
    systemPrint("    channelNumber: ");
    systemPrintln(channelNumber);
    outputSerialData(true);
  }

  /*
                                                         endOfTxData ---.
                                                                        |
                                                                        V
      +----------+---------+----------+------------+----------+---------+----------+
      | Optional |         | Optional | Optional   |          |         | Optional |
      | NET ID   | Control | C-Timer  | SF6 Length | DestAddr | SrcAddr | Trailer  |
      | 8 bits   | 8 bits  | 2 bytes  | 8 bits     | 8 bits   | 8 bits  | n Bytes  |
      +----------+---------+----------+------------+----------+---------+----------+
  */

  txControl.datagramType = DATAGRAM_VC_SYNC_ACKS;
  return (transmitDatagram());
}

//Build and transmit the ZERO_ACKS frame, last frame in 3-way ACKs handshake
bool xmitVcZeroAcks(int8_t destVc)
{
  VC_RADIO_MESSAGE_HEADER * vcHeader;

  radioCallHistory[RADIO_CALL_xmitVcZeroAcks] = millis();

  vcHeader = (VC_RADIO_MESSAGE_HEADER *)endOfTxData;
  vcHeader->length = VC_RADIO_HEADER_BYTES;
  vcHeader->destVc = destVc;
  vcHeader->srcVc = myVc;
  endOfTxData += VC_RADIO_HEADER_BYTES;

  /*
                                                         endOfTxData ---.
                                                                        |
                                                                        V
      +----------+---------+----------+------------+----------+---------+----------+
      | Optional |         | Optional | Optional   |          |         | Optional |
      | NET ID   | Control | C-Timer  | SF6 Length | DestAddr | SrcAddr | Trailer  |
      | 8 bits   | 8 bits  | 2 bytes  | 8 bits     | 8 bits   | 8 bits  | n Bytes  |
      +----------+---------+----------+------------+----------+---------+----------+
  */

  txControl.datagramType = DATAGRAM_VC_ZERO_ACKS;
  return (transmitDatagram());
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//Datagram reception
//
//Security or lack there of:
//
//The comments below outline the radio parameters and how they can be determined by
//listening to the radio traffic.  The required parameters are followed by one or
//more step numbers indicating the step which determines the parameter's value.  The
//comments further describe the steps that can be taken to reduce the attack surface
//for the LoRaSerial network.
//
//The following settings define the LoRaSerial radio network:
//
//    AirSpeed - 3, 4, 5
//    AutoTune - This is receive only
//    Bandwidth - 3
//    CodingRate - 5
//    FrequencyHop - 1
//    FrequencyMax - 1
//    FrequencyMin - 1
//    MaxDwellTime - 1
//    NumberOfChannels - 1
//    PreambleLength - 4
//    SpreadFactor - 4
//    SyncWord - 4
//    TxPower
//
//  Protocol settings:
//
//    DataScrambling - 6
//    EnableCRC16 - 8
//    EncryptData - 7
//    EncryptionKey - 7
//    FramesToYield
//    HeartBeatTimeout - 12
//    MaxResends - 14
//    NetID - 9
//    OperatingMode - 10, 11
//    Server - 10, 11
//    VerifyRxNetID - 9
//
//Attempting to attack a network of LoRaSerial radios would be done with the following
//steps:
//
// 1. Locate the frequencies that the network is using
//    a. Listen for traffic on a specific channel to determine if the network is using
//       that channel
//    b. Repeat across all of the channels in the available frequency band
// 2. Once the frequencies are identified, attempt to determine the channel sequence
//    by monitoring when traffic occurs on each of the identified frequencies, this
//    allows the attacker to construct the hop table for the network
// 3. Look at the bandwidth utilized by the radio network.  The signal for each
//    symbol is a ramp that spans the bandwidth selected for use
// 4. Using a receiver that uses the opposite ramp to generate the sub frequencies
//    within the bandwidth to decode the symbols.  By monitoring the network traffic
//    it is possible to determine the spreadfactor since there are 2^SF sub frequencies
//    that will be used within the bandwidth
// 5. Now that the spread factor is known, the next step is to determine the coding
//    rate used for forward error correction.  Here 4 data bits are converted into
//    between 5 to 8 bits in the transmitted frame
// 6. Look at the signal for multiple transmissions, does this signal have a DC offset?
//    The data scrambling setting is false if a DC offset is present, or is true when
//    no DC offset is present
// 7. Next step is breaking the encryption key
// 8. After the encryption key is obtained, it is possible determine if the link is
//    using software CRC by computing the CRC-16 value on the first n-2 bytes and
//    comparing that with the last 2 bytes
// 9. Determine if the link is using a network ID value, by checking the first byte
//    in each of the transmitted frames
// 10. Determine if the virtual circuit header is contained in the transmitted frames
// 11. Determine which set of data frames are being transmitted
// 12. Determine the maximum interval between HEARTBEAT frames to roughly determine
//     the HeartbeatTimeout value
// 13. Look for HEARTBEAT frames and FIND_PARTNER frames.  A FIND_PARTNER frame
//     is sent following a link failure which occurs after three HEARTBEAT timeout
//     periods.
// 14. The MaxResends value can be estimated by the maximum number of data
//     retransmissions done prior to the link failure.  A large number of link
//     failures will need to be collected from a very radio near the network.
//
//How do you prevent the attacks on a LoRaSerial radio network:
//
// 1. Don't advertize the network.  Reduce the TxPower value to the minimum needed
//    to successfully operate the network
//
// 2. Encrypt the data on the network.  Don't use the factory default encryption key.
//    Select a new encryption key and and train all of the radios in the network with
//    this key.  This will prevent attacks from any groups that are not able to break
//    the encryption.  However the radio network is vulnerable to well funded or
//    dedicated groups that are able to break the encryption.
//
// 3. Change the encryption key. When another network is using the same encryption key
//    and same network ID traffic from the other network is likely to be received.
//    Selecting another encryption key will avoid this behavior.  Also the encryption
//    key may be tested by setting a radio to MODE_MULTIPOINT, disabling VerifyRxNetID
//    and disabling EnableCRC16.  Both settings of DataScrambling will need to be tested.
//
// 4. Use a network ID.  This won't prevent attacks, but it prevents the reception
//    of frames from other networks use the same encryption key.
//
// 5. Use software CRC-16.  This won't prevent attacks but will eliminate most
//    frames from networks using the same encryption key that don't use a network
//    ID.  Occasionally, when they get lucky, our network ID matches the first byte
//    of their transmitted frame.  The software CRC-16 will most likely cause this
//    frame to be discarded.
//
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Determine the type of datagram received
PacketType rcvDatagram()
{
  uint8_t ackNumber;
  PacketType datagramType;
  uint8_t receivedNetID;
  CONTROL_U8 rxControl;
  VIRTUAL_CIRCUIT * vc;
  VC_RADIO_MESSAGE_HEADER * vcHeader;

  radioCallHistory[RADIO_CALL_rcvDatagram] = millis();

  //Acknowledge the receive interrupt
  transactionComplete = false;

  //Save the receive time
  rcvTimeMillis = millis();

  //Get the IRQ flags
  irqFlags = radio.getIRQFlags();

  //Get the received datagram
  framesReceived++;
  int state = radio.readData(incomingBuffer, MAX_PACKET_SIZE);

  printPacketQuality(); //Display the RSSI, SNR and frequency error values

  if (state == RADIOLIB_ERR_NONE)
  {
    rxSuccessMillis = rcvTimeMillis;
    triggerEvent(TRIGGER_RX_SPI_DONE);
    rssi = radio.getRSSI();
  }
  else
  {
    rxFailureMillis = rcvTimeMillis;
    rxFailureState = state;
    if (state == RADIOLIB_ERR_CRC_MISMATCH)
    {
      if (settings.debug || settings.debugDatagrams || settings.debugReceive)
      {
        systemPrintln("Receive CRC error!");
        outputSerialData(true);
      }
      badCrc++;
      returnToReceiving(); //Return to listening
      return (DATAGRAM_CRC_ERROR);
    }
    else
    {
      if (settings.debug || settings.debugDatagrams || settings.debugReceive)
      {
        systemPrint("Receive error: ");
        systemPrintln(state);
        outputSerialData(true);
      }
      returnToReceiving(); //Return to listening
      badFrames++;
      return (DATAGRAM_BAD);
    }
  }

  rxDataBytes = radio.getPacketLength();
  packetLength = rxDataBytes; //Total bytes received, used for calculating clock sync times in multi-point mode

  returnToReceiving(); //Immediately begin listening while we process new data

  rxData = incomingBuffer;
  vc = &virtualCircuitList[0];
  vcHeader = NULL;

  /*
      |<--------------------------- rxDataBytes --------------------------->|
      |                                                                     |
      +----------+---------+----------+------------+---  ...  ---+----------+
      | Optional |         | Optional | Optional   |             | Optional |
      | NET ID   | Control | C-Timer  | SF6 Length |   Data      | Trailer  |
      | 8 bits   | 8 bits  | 2 bytes  | 8 bits     |   n bytes   | n Bytes  |
      +----------+---------+----------+------------+-------------+----------+
      ^
      |
      '---- rxData
  */

  //Display the received data bytes
  if (settings.printRfData || settings.debugReceive)
  {
    systemPrintln("----------");
    systemPrintTimestamp();
    systemPrint("RX: ");
    systemPrint((settings.dataScrambling || settings.encryptData) ? "Encrypted " : "Unencrypted ");
    systemPrint("Frame ");
    systemPrint(rxDataBytes);
    systemPrint(" (0x");
    systemPrint(rxDataBytes, HEX);
    systemPrintln(") bytes");
    outputSerialData(true);
    if (timeToHop == true) //If the channelTimer has expired, move to next frequency
      hopChannel();
    petWDT();
    if (settings.printRfData && rxDataBytes)
      dumpBuffer(incomingBuffer, rxDataBytes);
    outputSerialData(true);
  }

  if (settings.dataScrambling == true)
    radioComputeWhitening(incomingBuffer, rxDataBytes);

  if (settings.encryptData == true)
  {
    decryptBuffer(incomingBuffer, rxDataBytes);
    if (timeToHop == true) //If the channelTimer has expired, move to next frequency
      hopChannel();
  }

  if (settings.debugReceive)
  {
    systemPrint("in: ");
    dumpBufferRaw(incomingBuffer, 14); //Print only the first few bytes when debugging packets
  }

  //Display the received data bytes
  if ((settings.dataScrambling || settings.encryptData)
      && (settings.printRfData || settings.debugReceive))
  {
    systemPrintTimestamp();
    systemPrint("RX: Unencrypted Frame ");
    systemPrint(rxDataBytes);
    systemPrint(" (0x");
    systemPrint(rxDataBytes, HEX);
    systemPrintln(") bytes");
    outputSerialData(true);
    if (timeToHop == true) //If the channelTimer has expired, move to next frequency
      hopChannel();
    petWDT();
    if (settings.printRfData && rxDataBytes)
      dumpBuffer(incomingBuffer, rxDataBytes);
    outputSerialData(true);
  }

  //All packets must include the control header
  if (rxDataBytes < minDatagramSize)
  {
    //Display the packet contents
    if (settings.printPktData || settings.debugDatagrams || settings.debugReceive)
    {
      systemPrintTimestamp();
      systemPrint("RX: Bad Frame ");
      systemPrint(rxDataBytes);
      systemPrint(" (0x");
      systemPrint(rxDataBytes, HEX);
      systemPrintln(") bytes");
      outputSerialData(true);
      if (timeToHop == true) //If the channelTimer has expired, move to next frequency
        hopChannel();
      petWDT();
      if (settings.printRfData && rxDataBytes)
        dumpBuffer(incomingBuffer, rxDataBytes);
      outputSerialData(true);
    }
    badFrames++;
    return (DATAGRAM_BAD);
  }

  /*
      |<---------------------- rxDataBytes ---------------------->|
      |                                                           |
      +----------+----------+----------+------------+---  ...  ---+----------+
      | Optional |          | Optional | Optional   |             | Optional |
      | NET ID   | Control  | C-Timer  | SF6 Length |   Data      | Trailer  |
      | 8 bits   | 8 bits   | 2 bytes  | 8 bits     |   n bytes   | n Bytes  |
      +----------+----------+----------+------------+-------------+----------+
      ^
      |
      '---- rxData
  */

  //Verify the netID if necessary
  if ((settings.operatingMode == MODE_POINT_TO_POINT) || settings.verifyRxNetID)
  {
    receivedNetID = *rxData++;
    if (receivedNetID != settings.netID)
    {
      if (settings.debugReceive || settings.debugDatagrams)
      {
        systemPrintTimestamp();
        systemPrint("RX: NetID ");
        systemPrint(receivedNetID);
        systemPrint(" (0x");
        systemPrint(receivedNetID, HEX);
        systemPrint(")");
        if (receivedNetID != settings.netID)
        {
          systemPrint(" expecting ");
          systemPrint(settings.netID);
        }
        systemPrintln();
        outputSerialData(true);
      }
      if (timeToHop == true) //If the channelTimer has expired, move to next frequency
        hopChannel();
      petWDT();
      if (settings.debugReceive && settings.printPktData && rxDataBytes)
        dumpBuffer(incomingBuffer, rxDataBytes);
      outputSerialData(true);
      netIdMismatch++;
      return (DATAGRAM_NETID_MISMATCH);
    }
  }

  //Process the trailer
  petWDT();
  if (settings.enableCRC16)
  {
    uint16_t crc;
    uint8_t * data;

    //Compute the CRC-16 value
    crc = 0xffff;
    if (timeToHop == true) //If the channelTimer has expired, move to next frequency
      hopChannel();
    for (data = incomingBuffer; data < &incomingBuffer[rxDataBytes - 2]; data++)
      crc = crc16Table[*data ^ (uint8_t)(crc >> (16 - 8))] ^ (crc << 8);
    if (timeToHop == true) //If the channelTimer has expired, move to next frequency
      hopChannel();
    if ((incomingBuffer[rxDataBytes - 2] != (crc >> 8))
        && (incomingBuffer[rxDataBytes - 1] != (crc & 0xff)))
    {
      //Display the packet contents
      if (settings.printPktData || settings.debugReceive || settings.debugDatagrams)
      {
        systemPrintTimestamp();
        systemPrint("RX: Bad CRC-16, received 0x");
        systemPrint(incomingBuffer[rxDataBytes - 2], HEX);
        systemPrint(incomingBuffer[rxDataBytes - 1], HEX);
        systemPrint(" expected 0x");
        systemPrintln(crc, HEX);
        outputSerialData(true);
        if (timeToHop == true) //If the channelTimer has expired, move to next frequency
          hopChannel();
        petWDT();
        if (settings.printRfData && rxDataBytes)
          dumpBuffer(incomingBuffer, rxDataBytes);
        outputSerialData(true);
      }
      badCrc++;
      return (DATAGRAM_CRC_ERROR);
    }
  }

  /*
      |<--------------------------- rxDataBytes ---------------------------->|
      |                                                                      |
      +----------+----------+----------+------------+---  ...  ---+----------+
      | Optional |          | Optional | Optional   |             | Optional |
      | NET ID   | Control  | C-Timer  | SF6 Length |    Data     | Trailer  |
      | 8 bits   | 8 bits   | 2 bytes  | 8 bits     |   n bytes   | n Bytes  |
      +----------+----------+----------+------------+-------------+----------+
                 ^
                 |
                 '---- rxData
  */

  //Get the control byte
  rxControl = *((CONTROL_U8 *)rxData++);
  datagramType = rxControl.datagramType;
  ackNumber = rxControl.ackNumber;

  if (settings.debugReceive)
    printControl(*((uint8_t *)&rxControl));

  if (datagramType >= MAX_DATAGRAM_TYPE)
  {
    if (settings.debugReceive || settings.debugDatagrams)
    {
      systemPrintTimestamp();
      systemPrint("RX: Invalid datagram type ");
      systemPrintln(datagramType);
      outputSerialData(true);
    }
    badFrames++;
    return (DATAGRAM_BAD);
  }

  //Ignore this frame is requested
  if (rxControl.ignoreFrame)
  {
    if (settings.debugReceive || settings.debugDatagrams)
    {
      systemPrintTimestamp();
      systemPrint("RX: Ignore this ");
      systemPrintln(datagramType);
      outputSerialData(true);
    }
    badFrames++;
    return (DATAGRAM_BAD);
  }

  //Display the CRC
  if (settings.enableCRC16 && settings.debugReceive)
  {
    systemPrintTimestamp();
    systemPrint("    CRC-16: 0x");
    systemPrint(incomingBuffer[rxDataBytes - 2], HEX);
    systemPrintln(incomingBuffer[rxDataBytes - 1], HEX);
    outputSerialData(true);
    if (timeToHop == true) //If the channelTimer has expired, move to next frequency
      hopChannel();
  }

  /*
      |<--------------------------- rxDataBytes ---------------------------->|
      |                                                                      |
      +----------+----------+----------+------------+---  ...  ---+----------+
      | Optional |          | Optional | Optional   |             | Optional |
      | NET ID   | Control  | C-Timer  | SF6 Length |    Data     | Trailer  |
      | 8 bits   | 8 bits   | 2 bytes  | 8 bits     |   n bytes   | n Bytes  |
      +----------+----------+----------+------------+-------------+----------+
                            ^
                            |
                            '---- rxData
  */
  //If hopping is enabled, sync data is located next within the header
  if (settings.frequencyHop == true)
  {
    memcpy(&msToNextHopRemote, rxData, sizeof(msToNextHopRemote));
    rxData += sizeof(msToNextHopRemote);

    //Display the channel timer
    if (settings.debugReceive)
    {
      systemPrint("    Channel Timer(ms): ");
      systemPrintln(msToNextHopRemote);
      outputSerialData(true);
      if (timeToHop == true) //If the channelTimer has expired, move to next frequency
        hopChannel();
    }
  }

  /*
      |<--------------------------- rxDataBytes ---------------------------->|
      |                                                                      |
      +----------+----------+----------+------------+---  ...  ---+----------+
      | Optional |          | Optional | Optional   |             | Optional |
      | NET ID   | Control  | C-Timer  | SF6 Length |    Data     | Trailer  |
      | 8 bits   | 8 bits   | 2 bytes  | 8 bits     |   n bytes   | n Bytes  |
      +----------+----------+----------+------------+-------------+----------+
                                       ^
                                       |
                                       '---- rxData
  */

  //Get the spread factor 6 length
  if (settings.radioSpreadFactor == 6)
  {
    if (rxDataBytes >= (*rxData + minDatagramSize))
      rxDataBytes = *rxData++;
    else
    {
      if (settings.debugReceive)
      {
        systemPrintTimestamp();
        systemPrint("Invalid SF6 length, received SF6 length ");
        systemPrint(*rxData);
        systemPrint(" > ");
        systemPrint((int)rxDataBytes - minDatagramSize);
        systemPrintln(" received bytes");
        outputSerialData(true);
        if (timeToHop == true) //If the channelTimer has expired, move to next frequency
          hopChannel();
      }
      badFrames++;
      return (DATAGRAM_BAD);
    }
    if (settings.debugTransmit)
    {
      systemPrintTimestamp();
      systemPrint("    SF6 Length: ");
      systemPrintln(rxDataBytes);
      outputSerialData(true);
      if (timeToHop == true) //If the channelTimer has expired, move to next frequency
        hopChannel();
    }
  }
  else
    rxDataBytes -= minDatagramSize;

  //Get the Virtual-Circuit header
  rxVcData = rxData;
  if (settings.operatingMode == MODE_VIRTUAL_CIRCUIT)
  {
    //Verify that the virtual circuit header is present
    if (rxDataBytes < 3)
    {
      if (settings.debugReceive || settings.debugDatagrams)
      {
        systemPrintTimestamp();
        systemPrint("Missing VC header bytes, received only ");
        systemPrint(rxDataBytes);
        systemPrintln(" bytes, expecting at least 3 bytes");
        outputSerialData(true);
        if (timeToHop == true) //If the channelTimer has expired, move to next frequency
          hopChannel();
      }
      badFrames++;
      return DATAGRAM_BAD;
    }

    //Parse the virtual circuit header
    vcHeader = (VC_RADIO_MESSAGE_HEADER *)rxData;
    rxDestVc = vcHeader->destVc;
    rxSrcVc = vcHeader->srcVc;
    rxVcData = &rxData[3];

    //Display the virtual circuit header
    if (settings.debugReceive)
    {
      systemPrintTimestamp();
      systemPrint("    VC Length: ");
      systemPrintln(vcHeader->length);
      systemPrint("    DestAddr: ");
      if (rxDestVc == VC_BROADCAST)
        systemPrintln("Broadcast");
      else
        systemPrintln(rxDestVc);
      systemPrint("    SrcAddr: ");
      if (rxSrcVc == VC_UNASSIGNED)
        systemPrintln("Unassigned");
      else
        systemPrintln(rxSrcVc);
      outputSerialData(true);
      if (timeToHop == true) //If the channelTimer has expired, move to next frequency
        hopChannel();
    }

    //Validate the source VC
    vc = NULL;
    if (rxSrcVc != VC_UNASSIGNED)
    {
      if ((uint8_t)rxSrcVc >= MAX_VC)
      {
        if (settings.debugReceive || settings.debugDatagrams)
        {
          systemPrintTimestamp();
          systemPrint("Invalid source VC: ");
          systemPrintln(rxSrcVc);
          outputSerialData(true);
          if (timeToHop == true) //If the channelTimer has expired, move to next frequency
            hopChannel();
          if (settings.printRfData && rxDataBytes)
            dumpBuffer(incomingBuffer, rxDataBytes);
          outputSerialData(true);
        }
        badFrames++;
        return DATAGRAM_BAD;
      }
      vc = &virtualCircuitList[rxSrcVc];
    }

    //Validate the length
    if (vcHeader->length != rxDataBytes)
    {
      if (settings.debugReceive || settings.debugDatagrams)
      {
        systemPrintTimestamp();
        systemPrint("Invalid VC length, received ");
        systemPrint(vcHeader->length);
        systemPrint(" expecting ");
        systemPrintln(rxDataBytes);
        outputSerialData(true);
      }
      if (timeToHop == true) //If the channelTimer has expired, move to next frequency
        hopChannel();
      if (vc)
        vc->badLength++;
      badFrames++;
      return DATAGRAM_BAD;
    }

    //Validate the destination VC
    if ((rxDestVc != VC_BROADCAST) && (rxDestVc != myVc))
    {
      if (settings.debugReceive || settings.debugDatagrams)
      {
        systemPrintTimestamp();
        systemPrint("Not my VC: ");
        systemPrintln(rxDestVc);
        outputSerialData(true);
        if (timeToHop == true) //If the channelTimer has expired, move to next frequency
          hopChannel();
        if (settings.printPktData && rxDataBytes)
          dumpBuffer(incomingBuffer, rxDataBytes);
        outputSerialData(true);
      }
      return DATAGRAM_NOT_MINE;
    }
  }

  //Verify the packet number last so that the expected datagram or ACK number can be updated
  if (vc && (settings.operatingMode != MODE_MULTIPOINT))
  {
    switch (datagramType)
    {
      default:
        break;

      case DATAGRAM_DATA_ACK:
        if (ackNumber != vc->txAckNumber)
        {
          if (settings.debugReceive || settings.debugDatagrams)
          {
            systemPrintTimestamp();
            systemPrint("Invalid ACK number, received ");
            systemPrint(ackNumber);
            systemPrint(" expecting ");
            systemPrintln(vc->txAckNumber);
            outputSerialData(true);
            if (timeToHop == true) //If the channelTimer has expired, move to next frequency
              hopChannel();
          }
          badFrames++;
          return (DATAGRAM_BAD);
        }

        //Set the next TX ACK number
        if (settings.printAckNumbers)
        {
          systemPrint("txAckNumber: ");
          systemPrint(vc->txAckNumber);
          systemPrint(" --> ");
        }
        vc->txAckNumber = (vc->txAckNumber + 1) & 3;
        if (settings.printAckNumbers)
        {
          systemPrintln(vc->txAckNumber);
          outputSerialData(true);
        }

        //Remember when the last datagram was received
        lastRxDatagram = rcvTimeMillis;
        break;

      case DATAGRAM_HEARTBEAT:
        if (settings.operatingMode == MODE_VIRTUAL_CIRCUIT)
          break;
        datagramType = validateDatagram(vc, datagramType, ackNumber, rxDataBytes);
        if (datagramType != DATAGRAM_HEARTBEAT)
          return datagramType;
        lastRxDatagram = rcvTimeMillis;
        break;

      case DATAGRAM_REMOTE_COMMAND:
        datagramType = validateDatagram(vc, datagramType, ackNumber, sizeof(commandRXBuffer)
                                        - availableRXCommandBytes());
        if (datagramType != DATAGRAM_REMOTE_COMMAND)
          return datagramType;
        lastRxDatagram = rcvTimeMillis;
        break;

      case DATAGRAM_REMOTE_COMMAND_RESPONSE:
        datagramType = validateDatagram(vc, datagramType, ackNumber, sizeof(serialTransmitBuffer)
                                        - availableTXBytes());
        if (datagramType != DATAGRAM_REMOTE_COMMAND_RESPONSE)
          return datagramType;
        lastRxDatagram = rcvTimeMillis;
        break;

      case DATAGRAM_DATA:
        datagramType = validateDatagram(vc, datagramType, ackNumber, sizeof(serialTransmitBuffer)
                                        - availableTXBytes());
        if (datagramType != DATAGRAM_DATA)
          return datagramType;
        lastRxDatagram = rcvTimeMillis;
        vc->messagesReceived++;
        break;
    }

    //Account for this frame
    vc->framesReceived++;
  }

  //If packet is good, check requestYield bit
  //If bit is set, this radio supresses transmissions for X number of frames
  //Datagram sender can both set and clear yield requests
  requestYield = rxControl.requestYield;
  if (requestYield)
  {
    triggerEvent(TRIGGER_RX_YIELD);
    yieldTimerStart = millis();
  }

  /*
                                                    |<-- rxDataBytes -->|
                                                    |                                                                      |
      +----------+----------+----------+------------+------  ...  ------+----------+
      | Optional |          | Optional | Optional   |                   | Optional |
      | NET ID   | Control  | C-Timer  | SF6 Length |       Data        | Trailer  |
      | 8 bits   | 8 bits   | 2 bytes  | 8 bits     |      n bytes      | n Bytes  |
      +----------+----------+----------+------------+-------------------+----------+
                                                    ^
                                                    |
                                                    '---- rxData
  */

  //Display the packet contents
  if (settings.printPktData || settings.debugReceive)
  {
    systemPrintTimestamp();
    systemPrint("RX: Datagram ");
    systemPrint(rxDataBytes);
    systemPrint(" (0x");
    systemPrint(rxDataBytes, HEX);
    systemPrintln(") bytes");
    outputSerialData(true);
    if (timeToHop == true) //If the channelTimer has expired, move to next frequency
      hopChannel();
    petWDT();
    if (settings.printPktData && rxDataBytes)
      dumpBuffer(rxData, rxDataBytes);
  }

  //Display the datagram type
  if (settings.debugDatagrams)
  {
    systemPrintTimestamp();
    systemPrint("RX: ");
    if (settings.operatingMode == MODE_VIRTUAL_CIRCUIT)
    {
      systemPrint((uint8_t)((rxDestVc == VC_BROADCAST) ? rxDestVc : rxDestVc & VCAB_NUMBER_MASK));
      systemPrint(" <-- ");
      systemPrint((uint8_t)((rxSrcVc == VC_UNASSIGNED) ? rxSrcVc : rxSrcVc & VCAB_NUMBER_MASK));
      systemWrite(' ');
    }
    systemPrint(radioDatagramType[datagramType]);
    switch (datagramType)
    {
      default:
        systemPrintln();
        break;

      case DATAGRAM_DATA:
      case DATAGRAM_DATA_ACK:
      case DATAGRAM_REMOTE_COMMAND:
      case DATAGRAM_REMOTE_COMMAND_RESPONSE:
      case DATAGRAM_HEARTBEAT:
        if (settings.operatingMode != MODE_MULTIPOINT)
        {
          systemPrint(" (");
          if (settings.operatingMode == MODE_VIRTUAL_CIRCUIT)
            systemPrint("VC ");
          systemPrint("ACK #");
          systemPrint(ackNumber);
          systemPrint(")");

          if (rxControl.requestYield)
            systemPrint(" (Y)");
        }
        systemPrintln();
        break;
    }
    outputSerialData(true);
  }
  if (timeToHop == true) //If the channelTimer has expired, move to next frequency
    hopChannel();

  //Process the packet
  datagramsReceived++;
  linkDownTimer = millis();

  //Blink the RX LED
  blinkRadioRxLed(true);
  return datagramType;
}

//Determine what PacketType value should be returned to the receiving code, options are:
// * Received datagramType
// * DATAGRAM_DUPLICATE
// * DATAGRAM_BAD
PacketType validateDatagram(VIRTUAL_CIRCUIT * vc, PacketType datagramType, uint8_t ackNumber, uint16_t freeBytes)
{
  if (ackNumber != vc->rmtTxAckNumber)
  {
    //Determine if this is a duplicate datagram
    if (ackNumber == ((vc->rmtTxAckNumber - 1) & 3))
    {
      if (settings.debugReceive || settings.debugDatagrams)
      {
        systemPrintTimestamp();
        systemPrint("Duplicate datagram received, ACK ");
        systemPrintln(ackNumber);
        outputSerialData(true);
        if (timeToHop == true) //If the channelTimer has expired, move to next frequency
          hopChannel();
      }
      linkDownTimer = millis();
      duplicateFrames++;
      return DATAGRAM_DUPLICATE;
    }

    //Not a duplicate
    if (settings.debugReceive || settings.debugDatagrams)
    {
      systemPrintTimestamp();
      systemPrint("Invalid datagram number, received ");
      systemPrint(ackNumber);
      systemPrint(" expecting ");
      systemPrintln(vc->rmtTxAckNumber);
      outputSerialData(true);
      if (timeToHop == true) //If the channelTimer has expired, move to next frequency
        hopChannel();
    }
    badFrames++;
    return DATAGRAM_BAD;
  }

  //Verify that there is sufficient space in the serialTransmitBuffer
  if (inCommandMode || ((sizeof(serialTransmitBuffer) - availableTXBytes()) < rxDataBytes))
  {
    if (settings.debugReceive || settings.debugDatagrams)
    {
      systemPrintTimestamp();
      systemPrintln("Insufficient space in the serialTransmitBuffer");
    }
    insufficientSpace++;

    //Apply back pressure to the other radio by dropping this packet and
    //forcing the other radio to retransmit the packet.
    badFrames++;
    return DATAGRAM_BAD;
  }

  //Receive this data packet and set the next expected datagram number
  if (settings.printAckNumbers)
  {
    systemPrint("rxAckNumber: ");
    systemPrint(vc->rxAckNumber);
    systemPrint(" --> ");
  }
  vc->rxAckNumber = vc->rmtTxAckNumber;
  if (settings.printAckNumbers)
  {
    systemPrintln(vc->rxAckNumber);
    systemPrint("rmtTxAckNumber: ");
    systemPrint(vc->rmtTxAckNumber);
    systemPrint(" --> ");
  }
  vc->rmtTxAckNumber = (vc->rmtTxAckNumber + 1) & 3;
  if (settings.printAckNumbers)
  {
    systemPrintln(vc->rmtTxAckNumber);
    outputSerialData(true);
  }
  return datagramType;
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//Datagram transmission
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Push the outgoing packet to the air
//Returns false if we could not start tranmission due to packet received or RX in process
bool transmitDatagram()
{
  uint8_t control;
  uint8_t * header;
  uint8_t length;
  int8_t srcVc;
  uint8_t * vcData;
  VIRTUAL_CIRCUIT * vc;
  VC_RADIO_MESSAGE_HEADER * vcHeader;

  if (timeToHop == true) //If the channelTimer has expired, move to next frequency
    hopChannel();

  //Remove some jitter by getting this time after the hopChannel
  txDatagramMicros = micros();
  radioCallHistory[RADIO_CALL_transmitDatagram] = millis();

  //Parse the virtual circuit header
  vc = &virtualCircuitList[0];
  vcHeader = NULL;
  if (settings.operatingMode == MODE_VIRTUAL_CIRCUIT)
  {
    vc = NULL;
    vcHeader = (VC_RADIO_MESSAGE_HEADER *)&outgoingPacket[headerBytes];
    txDestVc = vcHeader->destVc;
    srcVc = vcHeader->srcVc;
    if ((uint8_t)vcHeader->destVc <= MAX_VC)
    {
      vc = &virtualCircuitList[txDestVc];
      vc->messagesSent++;
    }
    vcData = (uint8_t *)&vcHeader[1];
  }

  //Determine the packet size
  datagramsSent++;
  txDatagramSize = endOfTxData - outgoingPacket;
  length = txDatagramSize - headerBytes;

  //Select the ACK number
  if (txControl.datagramType == DATAGRAM_DATA_ACK)
    txControl.ackNumber = vc->rxAckNumber;
  else
    txControl.ackNumber = vc->txAckNumber;

  //If we are ACK'ing data, and we have data to send ourselves, request that
  //the sender yield to give us an opportunity to send our data
  if ((txControl.datagramType == DATAGRAM_DATA_ACK) && availableRadioTXBytes())
  {
    triggerEvent(TRIGGER_TX_YIELD);
    txControl.requestYield = 1;
  }
  else
    txControl.requestYield = 0;

  //Print debug info as needed
  if (settings.debugDatagrams)
  {
    systemPrintTimestamp();
    systemPrint("TX: ");
    if (settings.operatingMode == MODE_VIRTUAL_CIRCUIT)
    {
      systemPrint((uint8_t)((srcVc == VC_UNASSIGNED) ? srcVc : srcVc & VCAB_NUMBER_MASK));
      systemPrint(" --> ");
      systemPrint((uint8_t)((txDestVc == VC_BROADCAST) ? txDestVc : txDestVc & VCAB_NUMBER_MASK));
      systemWrite(' ');
    }
    systemPrint(radioDatagramType[txControl.datagramType]);
    switch (txControl.datagramType)
    {
      default:
        systemPrintln();
        break;

      case DATAGRAM_DATA:
      case DATAGRAM_DATA_ACK:
      case DATAGRAM_REMOTE_COMMAND:
      case DATAGRAM_REMOTE_COMMAND_RESPONSE:
      case DATAGRAM_HEARTBEAT:
        if (settings.operatingMode != MODE_MULTIPOINT)
        {
          systemPrint(" (");
          if (settings.operatingMode == MODE_VIRTUAL_CIRCUIT)
            systemPrint("VC ");
          systemPrint("ACK #");
          systemPrint(txControl.ackNumber);
          systemPrint(")");

          if (txControl.requestYield)
            systemPrint(" (requestYield)");
        }
        systemPrintln();
        break;
    }
    outputSerialData(true);
    if (timeToHop == true) //If the channelTimer has expired, move to next frequency
      hopChannel();
  }

  /*
                                                   endOfTxData ---.
                                                                  |
                                                                  V
      +----------+----------+----------+------------+---  ...  ---+----------+
      | Optional |          | Optional |  Optional  |             | Optional |
      |  NET ID  | Control  | C-Timer  | SF6 Length |    Data     | Trailer  |
      |  8 bits  |  8 bits  | 2 bytes  |   8 bits   |   n bytes   | n Bytes  |
      +----------+----------+----------+------------+-------------+----------+
      |                                             |             |
      |                                             |<- Length -->|
      |<-------------------- txDatagramSize --------------------->|
  */

  //Display the packet contents
  if (settings.printPktData || settings.debugTransmit)
  {
    systemPrintTimestamp();
    systemPrint("TX: Datagram ");
    systemPrint(length);
    systemPrint(" (0x");
    systemPrint(length, HEX);
    systemPrintln(") bytes");
    if (timeToHop == true) //If the channelTimer has expired, move to next frequency
      hopChannel();
    petWDT();
    if (settings.printPktData)
      dumpBuffer(&endOfTxData[-length], length);
    outputSerialData(true);
  }

  //Build the datagram header
  header = outgoingPacket;
  if (headerBytes && settings.debugTransmit)
  {
    systemPrintTimestamp();
    systemPrint("TX: Header");
    systemPrint(headerBytes);
    systemPrint(" (0x");
    systemPrint(headerBytes);
    systemPrintln(") bytes");
    outputSerialData(true);
    if (timeToHop == true) //If the channelTimer has expired, move to next frequency
      hopChannel();
  }

  /*
      .------ Header                               endOfTxData ---.
      |                                                           |
      V                                                           V
      +----------+----------+----------+------------+---  ...  ---+----------+
      | Optional |          | Optional |  Optional  |             | Optional |
      |  NET ID  | Control  | C-Timer  | SF6 Length |    Data     | Trailer  |
      |  8 bits  |  8 bits  | 2 bytes  |   8 bits   |   n bytes   | n Bytes  |
      +----------+----------+----------+------------+-------------+----------+
      |                                             |             |
      |                                             |<- Length -->|
      |<-------------------- txDatagramSize --------------------->|
  */

  //Add the netID if necessary
  if ((settings.operatingMode == MODE_POINT_TO_POINT) || settings.verifyRxNetID)
  {
    *header++ = settings.netID;

    //Display the netID value
    if (settings.debugTransmit)
    {
      systemPrintTimestamp();
      systemPrint("    NetID: ");
      systemPrint(settings.netID);
      systemPrint(" (0x");
      systemPrint(settings.netID, HEX);
      systemPrintln(")");
      outputSerialData(true);
      if (timeToHop == true) //If the channelTimer has expired, move to next frequency
        hopChannel();
      petWDT();
    }
  }

  //Add the control byte
  control = *(uint8_t *)&txControl;
  *header++ = control;

  /*
                            .------ Header         endOfTxData ---.
                            |                                     |
                            V                                     V
      +----------+----------+----------+------------+---  ...  ---+----------+
      | Optional |          | Optional |  Optional  |             | Optional |
      |  NET ID  | Control  | C-Timer  | SF6 Length |    Data     | Trailer  |
      |  8 bits  |  8 bits  | 2 bytes  |   8 bits   |   n bytes   | n Bytes  |
      +----------+----------+----------+------------+-------------+----------+
      |                                             |             |
      |                                             |<- Length -->|
      |<-------------------- txDatagramSize --------------------->|
  */

  //Display the control value
  if (settings.debugTransmit)
    printControl(control);

  //Add the clock sync information
  if (settings.frequencyHop == true)
  {
    //Hake sure that the transmitted msToNextHop is in the range 0 - maxDwellTime
    if (timeToHop)
      hopChannel();

    //Measure the time to the next hop
    triggerEvent(TRIGGER_TX_LOAD_CHANNEL_TIMER_VALUE);
    txSetChannelTimerMicros = micros();
    unsigned long currentMillis = millis();
    uint16_t msToNextHop; //TX channel timer value
    if (channelTimerMsec)
      msToNextHop = channelTimerMsec - (currentMillis - channelTimerStart);
    else
      msToNextHop = settings.maxDwellTime;

    //Validate this value
    if (ENABLE_DEVELOPER && (!clockSyncReceiver))
    {
      if ((msToNextHop < 0) || (msToNextHop > settings.maxDwellTime))
      {
        int16_t channelTimer;
        uint8_t * data;

        systemPrint("TX Frame ");
        systemPrintln(radioDatagramType[txControl.datagramType]);
        data = outgoingPacket;
        if ((settings.operatingMode == MODE_POINT_TO_POINT) || settings.verifyRxNetID)
        {
          systemPrint("    Net Id: ");
          systemPrint(*data);
          systemPrint(" (0x");
          systemPrint(*data++, HEX);
          systemPrintln(")");
        }
        printControl(*data++);
        memcpy(&channelTimer, data, sizeof(channelTimer));
        data += sizeof(channelTimer);
        systemPrint("    Channel Timer(ms): ");
        systemPrintln(channelTimer);

        systemPrint("ERROR: Invalid msToNextHop value, ");
        systemPrintln(msToNextHop);

        //Set a valid value
        msToNextHop = settings.maxDwellTime;
      }
      else if (settings.debugSync)
      {
        switch (txControl.datagramType)
        {
          case DATAGRAM_DATA_ACK:
          case DATAGRAM_SYNC_CLOCKS:
            systemPrint("TX msToNextHop: ");
            systemPrint(msToNextHop);
            systemPrintln(" mSec");
          break;
        }
      }
    } //ENABLE_DEVELOPER

    //Set the time in the frame
    memcpy(header, &msToNextHop, sizeof(msToNextHop));
    header += sizeof(msToNextHop); //aka CHANNEL_TIMER_BYTES

    if (settings.debugTransmit)
    {
      systemPrintTimestamp();
      systemPrint("    Channel Timer(ms): ");
      systemPrintln(msToNextHop);
      outputSerialData(true);
      if (timeToHop == true) //If the channelTimer has expired, move to next frequency
        hopChannel();
    }
  }
  else
    txSetChannelTimerMicros = micros();

  /*
                          Header ------.           endOfTxData ---.
                                       |                          |
                                       V                          V
      +----------+----------+----------+------------+---  ...  ---+----------+
      | Optional |          | Optional |  Optional  |             | Optional |
      |  NET ID  | Control  | C-Timer  | SF6 Length |    Data     | Trailer  |
      |  8 bits  |  8 bits  | 2 bytes  |   8 bits   |   n bytes   | n Bytes  |
      +----------+----------+----------+------------+-------------+----------+
      |                                             |             |
      |                                             |<- Length -->|
      |<-------------------- txDatagramSize --------------------->|
  */

  //Add the spread factor 6 length if required
  if (settings.radioSpreadFactor == 6)
  {
    *header++ = length;

    //Send either a short ACK or full length packet
    switch (txControl.datagramType)
    {
      default:
        txDatagramSize = MAX_PACKET_SIZE - trailerBytes; //We're now going to transmit a full size datagram
        break;

      case DATAGRAM_FIND_PARTNER:
      case DATAGRAM_SYNC_CLOCKS:
      case DATAGRAM_ZERO_ACKS:
        txDatagramSize = headerBytes + CLOCK_MILLIS_BYTES; //Short packet is 5 + 4
        break;

      case DATAGRAM_DATA_ACK:
        txDatagramSize = headerBytes; //Short ACK packet is 5
        break;
    }

    radio.implicitHeader(txDatagramSize); //Set header size so that hardware CRC is calculated correctly

    endOfTxData = &outgoingPacket[txDatagramSize];
    if (settings.debugTransmit)
    {
      systemPrintTimestamp();
      systemPrint("    SF6 Length: ");
      systemPrintln(length);
      systemPrint("    SF6 TX Header Size: ");
      systemPrintln(txDatagramSize);
      outputSerialData(true);
      if (timeToHop == true) //If the channelTimer has expired, move to next frequency
        hopChannel();
    }
  }

  /*                                               endOfTxData ---.
                                       Header ------.             |
                                                    |             |
                                                    V             V
      +----------+----------+----------+------------+---  ...  ---+----------+
      | Optional |          | Optional |  Optional  |             | Optional |
      |  NET ID  | Control  | C-Timer  | SF6 Length |    Data     | Trailer  |
      |  8 bits  |  8 bits  | 2 bytes  |   8 bits   |   n bytes   | n Bytes  |
      +----------+----------+----------+------------+-------------+----------+
      |                                             |             |
      |                                             |<- Length -->|
      |<-------------------- txDatagramSize --------------------->|
  */

  //Verify the Virtual-Circuit length
  if (settings.debugTransmit && (settings.operatingMode == MODE_VIRTUAL_CIRCUIT))
  {
    systemPrintTimestamp();
    systemPrint("    Length: ");
    systemPrintln(vcHeader->length);
    systemPrint("    DestAddr: ");
    if (txDestVc == VC_BROADCAST)
      systemPrintln("Broadcast");
    else
      systemPrintln(txDestVc);
    systemPrint("    SrcAddr: ");
    if (srcVc == VC_UNASSIGNED)
      systemPrintln("Unassigned");
    else
      systemPrintln(srcVc);
    outputSerialData(true);
    if (timeToHop == true) //If the channelTimer has expired, move to next frequency
      hopChannel();
  }

  /*
                                                   endOfTxData ---.
                                                                  |
                                                                  V
      +----------+----------+----------+------------+---  ...  ---+----------+
      | Optional |          | Optional |  Optional  |             | Optional |
      |  NET ID  | Control  | C-Timer  | SF6 Length |    Data     | Trailer  |
      |  8 bits  |  8 bits  | 2 bytes  |   8 bits   |   n bytes   | n Bytes  |
      +----------+----------+----------+------------+-------------+----------+
      |                                                           |
      |<-------------------- txDatagramSize --------------------->|
  */

  //Add the datagram trailer
  if (settings.enableCRC16)
  {
    uint16_t crc;
    uint8_t * txData;

    //Compute the CRC-16 value
    crc = 0xffff;
    if (timeToHop == true) //If the channelTimer has expired, move to next frequency
      hopChannel();
    for (txData = outgoingPacket; txData < endOfTxData; txData++)
      crc = crc16Table[*txData ^ (uint8_t)(crc >> (16 - 8))] ^ (crc << 8);
    *endOfTxData++ = (uint8_t)(crc >> 8);
    *endOfTxData++ = (uint8_t)(crc & 0xff);
  }
  txDatagramSize += trailerBytes;
  if (timeToHop == true) //If the channelTimer has expired, move to next frequency
    hopChannel();

  //Display the trailer
  if (trailerBytes && settings.debugTransmit)
  {
    systemPrintTimestamp();
    systemPrint("TX: Trailer ");
    systemPrint(trailerBytes);
    systemPrint(" (0x");
    systemPrint(trailerBytes);
    systemPrintln(") bytes");
    outputSerialData(true);
    if (timeToHop == true) //If the channelTimer has expired, move to next frequency
      hopChannel();

    //Display the CRC
    if (settings.enableCRC16 && (settings.printPktData || settings.debugReceive))
    {
      systemPrintTimestamp();
      systemPrint("    CRC-16: 0x");
      systemPrint(endOfTxData[-2], HEX);
      systemPrintln(endOfTxData[-1], HEX);
      outputSerialData(true);
      if (timeToHop == true) //If the channelTimer has expired, move to next frequency
        hopChannel();
    }
  }

  /*
                                                   endOfTxData ---.
                                                                  |
                                                                  V
      +----------+----------+----------+------------+---  ...  ---+----------+
      | Optional |          | Optional |  Optional  |             | Optional |
      |  NET ID  | Control  | C-Timer  | SF6 Length |    Data     | Trailer  |
      |  8 bits  |  8 bits  | 2 bytes  |   8 bits   |   n bytes   | n Bytes  |
      +----------+----------+----------+------------+-------------+----------+
      |                                                           |
      |<-------------------- txDatagramSize --------------------->|
  */

  //Display the transmitted packet bytes
  if (settings.printRfData || settings.debugTransmit)
  {
    systemPrintTimestamp();
    systemPrint("TX: Unencrypted Frame ");
    systemPrint(txDatagramSize);
    systemPrint(" (0x");
    systemPrint(txDatagramSize, HEX);
    systemPrintln(") bytes");
    outputSerialData(true);
    if (timeToHop == true) //If the channelTimer has expired, move to next frequency
      hopChannel();
    petWDT();
    if (settings.printRfData)
      dumpBuffer(outgoingPacket, txDatagramSize);
    outputSerialData(true);
  }

  //Print raw packet bytes before encryption
  if (settings.debugTransmit)
  {
    systemPrint("out: ");
    dumpBufferRaw(outgoingPacket, 14);
  }

  //Encrypt the datagram
  if (settings.encryptData == true)
  {
    encryptBuffer(outgoingPacket, txDatagramSize);
    if (timeToHop == true) //If the channelTimer has expired, move to next frequency
      hopChannel();
  }

  //Scramble the datagram
  if (settings.dataScrambling == true)
    radioComputeWhitening(outgoingPacket, txDatagramSize);

  //Display the transmitted packet bytes
  if ((settings.printRfData || settings.debugTransmit)
      && (settings.encryptData || settings.dataScrambling))
  {
    systemPrintTimestamp();
    systemPrint("TX: Encrypted Frame ");
    systemPrint(txDatagramSize);
    systemPrint(" (0x");
    systemPrint(txDatagramSize, HEX);
    systemPrintln(") bytes");
    outputSerialData(true);
    if (timeToHop == true) //If the channelTimer has expired, move to next frequency
      hopChannel();
    petWDT();
    if (settings.printRfData)
      dumpBuffer(outgoingPacket, txDatagramSize);
    outputSerialData(true);
  }

  //If we are trainsmitting at high data rates the receiver is often not ready for new data. Pause for a few ms (measured with logic analyzer).
  if (settings.airSpeed == 28800 || settings.airSpeed == 38400)
    delay(2);

  //Reset the buffer data pointer for the next transmit operation
  endOfTxData = &outgoingPacket[headerBytes];

  //Compute the time needed for this frame. Part of ACK timeout.
  frameAirTime = calcAirTimeMsec(txDatagramSize);

  //Transmit this datagram
  frameSentCount = 0; //This is the first time this frame is being sent
  return (retransmitDatagram(vc));
}

//Print the control byte value
void printControl(uint8_t value)
{
  CONTROL_U8 * control = (CONTROL_U8 *)&value;

  systemPrintTimestamp();
  systemPrint("    Control: 0x");
  systemPrintln(value, HEX);

  if (timeToHop == true) //If the channelTimer has expired, move to next frequency
    hopChannel();

  systemPrintTimestamp();
  systemPrint("        ACK # ");
  systemPrintln(value & 3);

  if (timeToHop == true) //If the channelTimer has expired, move to next frequency
    hopChannel();

  systemPrintTimestamp();
  systemPrint("        datagramType ");
  if (control->datagramType < MAX_DATAGRAM_TYPE)
    systemPrintln(radioDatagramType[control->datagramType]);
  else
  {
    systemPrint("Unknown ");
    systemPrintln(control->datagramType);
  }

  if (control->requestYield)
  {
    systemPrintTimestamp();
    systemPrintln("        requestYield");
  }

  if (control->ignoreFrame)
  {
    systemPrintTimestamp();
    systemPrintln("        Ignore Frame");
  }

  outputSerialData(true);

  if (timeToHop == true) //If the channelTimer has expired, move to next frequency
    hopChannel();

  petWDT();
}

//The previous transmission was not received, retransmit the datagram
//Returns false if we could not start tranmission due to packet received or RX in process
bool retransmitDatagram(VIRTUAL_CIRCUIT * vc)
{
  radioCallHistory[RADIO_CALL_retransmitDatagram] = millis();

  /*
      +----------+----------+----------+------------+---  ...  ---+----------+
      | Optional |          | Optional | Optional   |             | Optional |
      | NET ID   | Control  | C-Timer  | SF6 Length |   Data      | Trailer  |
      | 8 bits   | 8 bits   | 2 bytes  | 8 bits     |   n bytes   | n Bytes  |
      +----------+----------+----------+------------+-------------+----------+
      |                                                                      |
      |<------------------------- txDatagramSize --------------------------->|
  */

  if (timeToHop == true) //If the channelTimer has expired, move to next frequency
    hopChannel();

  //Display the transmitted frame bytes
  if (frameSentCount && (settings.printRfData || settings.debugTransmit))
  {
    systemPrintTimestamp();
    systemPrint("TX: Retransmit ");
    systemPrint((settings.encryptData || settings.dataScrambling) ? "Encrypted " : "Unencrypted ");
    systemPrint("Frame ");
    systemPrint(txDatagramSize);
    systemPrint(" (0x");
    systemPrint(txDatagramSize, HEX);
    systemPrintln(") bytes");
    outputSerialData(true);
    if (timeToHop == true) //If the channelTimer has expired, move to next frequency
      hopChannel();
    petWDT();
    if (settings.printRfData)
      dumpBuffer(outgoingPacket, txDatagramSize);
    outputSerialData(true);
  }

  //Transmit this frame
  if (timeToHop == true) //If the channelTimer has expired, move to next frequency
    hopChannel();

  frameAirTime = calcAirTimeMsec(txDatagramSize); //Calculate frame air size while we're transmitting in the background
  uint16_t responseDelay = frameAirTime / responseDelayDivisor; //Give the receiver a bit of wiggle time to respond

  //Drop this datagram if the receiver is active
  if (
    (receiveInProcess() == true)
    || (transactionComplete == true)
    || (
      //If we are in VC mode, and destination is not broadcast,
      //and the destination circuit is offline
      //and we are not scanning for servers
      //then don't transmit
      (settings.operatingMode == MODE_VIRTUAL_CIRCUIT)
      && (txDestVc != VC_BROADCAST)
      && (virtualCircuitList[txDestVc & VCAB_NUMBER_MASK].vcState == VC_STATE_LINK_DOWN)
      && (txDestVc != VC_UNASSIGNED)
      && (radioState != RADIO_DISCOVER_SCANNING)
    )
  )
  {
    triggerEvent(TRIGGER_TRANSMIT_CANCELED);
    if (settings.debugReceive || settings.debugDatagrams)
    {
      systemPrint("TX failed: ");
      if (transactionComplete)
        systemPrintln("RXTC");
      else if (receiveInProcess())
        systemPrintln("RXIP");
      else
        systemPrintln("VC link down");
      outputSerialData(true);
    }
    return (false); //Do not start transmit while RX is or has occured
  }
  else
  {
    //Move the data to the radio over SPI
    int state = radio.startTransmit(outgoingPacket, txDatagramSize);

    if (state == RADIOLIB_ERR_NONE)
    {
      xmitTimeMillis = millis();
      triggerEvent(TRIGGER_TX_SPI_DONE);
      txSuccessMillis = xmitTimeMillis;
      frameSentCount++;
      if (vc)
        vc->framesSent++;
      framesSent++;
      if (settings.debugTransmit)
      {
        systemPrintTimestamp();
        systemPrint("TX: frameAirTime ");
        systemPrint(frameAirTime);
        systemPrintln(" mSec");
        outputSerialData(true);
        if (timeToHop == true) //If the channelTimer has expired, move to next frequency
          hopChannel();

        systemPrintTimestamp();
        systemPrint("TX: responseDelay ");
        systemPrint(responseDelay);
        systemPrintln(" mSec");
        outputSerialData(true);
        if (timeToHop == true) //If the channelTimer has expired, move to next frequency
          hopChannel();
      }
    }
    else
    {
      txFailureMillis = millis();
      txFailureState = state;
      if (settings.debugTransmit)
      {
        systemPrintTimestamp();
        systemPrint("TX: Transmit error, state ");
        systemPrintln(state);
        outputSerialData(true);
      }
    }
  }

  frameAirTime += responseDelay;
  datagramTimer = millis(); //Move timestamp even if error

  //Compute the interval before a retransmission occurs in milliseconds,
  //this value increases with each transmission
  retransmitTimeout = random(ackAirTime, frameAirTime + ackAirTime)
                    * (frameSentCount + 1) * 3 / 2;

  //BLink the TX LED
  blinkRadioTxLed(true);

  return (true); //Transmission has started
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//Transmit Ignored Frames
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

bool getTxTime(bool (*transmitFrame)(), uint32_t * txFrameUsec, const char * frameName)
{
  bool txStatus;
  uint32_t transmitDelay;

#define IGNORE_TRANSMIT_DELAY_MSEC    100

  //Transmit the frame
  transmitDelay = 0;
  do
  {
    //Delay between retries
    if (transmitDelay)
      delay(transmitDelay);
    transmitDelay += IGNORE_TRANSMIT_DELAY_MSEC;

    //Fail transmission after 5 attempts
    if (transmitDelay > (5 * IGNORE_TRANSMIT_DELAY_MSEC))
    {
      if (settings.debugSync)
      {
        systemPrintTimestamp();
        systemPrint("TX ignore ");
        systemPrint(frameName);
        systemPrintln(" failed!");
        outputSerialData(true);
      }
      return false;
    }

    //Attempt to transmit the requested frame
    txControl.ignoreFrame = true;
    txStatus = transmitFrame();
    txControl.ignoreFrame = false;
  } while (!txStatus);

  //Wait for transmit completion
  while (!transactionComplete)
    petWDT();
  transactionComplete = false;

  //Compute the transmit time
  *txFrameUsec = transactionCompleteMicros - txSetChannelTimerMicros;
  if (settings.debugSync)
  {
    systemPrintTimestamp();
    systemPrint("TX ");
    systemPrint(frameName);
    systemPrint(": ");
    systemPrint(((float)*txFrameUsec) / 1000., 5);
    systemPrintln(" mSec");
    outputSerialData(true);
  }
  return true;
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Use the maximum dwell setting to start the timer that indicates when to hop channels
void startChannelTimer()
{
  startChannelTimer(settings.maxDwellTime);
}

//Use the specified value to start the timer that indicates when to hop channels
void startChannelTimer(int16_t startAmount)
{
  radioCallHistory[RADIO_CALL_startChannelTimer] = millis();

  channelTimer.disableTimer();
  channelTimer.setInterval_MS(startAmount, channelTimerHandler);
  digitalWrite(pin_hop_timer, channelNumber & 1);
  reloadChannelTimer = (startAmount != settings.maxDwellTime);
  timeToHop = false;
  channelTimerStart = millis(); //startChannelTimer - ISR updates value
  channelTimerMsec = startAmount; //startChannelTimer - ISR updates value
  channelTimer.enableTimer();
  triggerEvent(TRIGGER_HOP_TIMER_START);
}

//Stop the channel (hop) timer
void stopChannelTimer()
{
  radioCallHistory[RADIO_CALL_stopChannelTimer] = millis();

  //Turn off the channel timer
  channelTimer.disableTimer();
  digitalWrite(pin_hop_timer, channelNumber & 1);
  channelTimerMsec = 0; //Indicate that the timer is off

  triggerEvent(TRIGGER_HOP_TIMER_STOP);
  timeToHop = false;
}

//Given the remote unit's number of ms before its next hop,
//adjust our own channelTimer interrupt to be synchronized with the remote unit
void syncChannelTimer(uint32_t frameAirTimeUsec)
{
  int16_t adjustment;
  uint8_t caseNumber;
  unsigned long currentMillis;
  int8_t delayedHopCount;
  uint16_t frameAirTimeMsec;
  int16_t lclHopTimeMsec;
  uint16_t msToNextHop;
  int16_t rmtHopTimeMsec;

  currentMillis = millis();
  radioCallHistory[RADIO_CALL_syncChannelTimer] = currentMillis;

  if (!clockSyncReceiver) return; //syncChannelTimer
  if (settings.frequencyHop == false) return;

  //msToNextHopRemote is obtained during rcvDatagram() and is in the range of
  //0 - settings.maxDwellTime
  //Validate this range
  if (ENABLE_DEVELOPER)
  {
    if ((msToNextHopRemote < 0) || (msToNextHopRemote > settings.maxDwellTime))
    {
      int16_t channelTimer;
      uint8_t * data;

      systemPrintln("RX Frame");
      dumpBuffer(incomingBuffer, headerBytes + rxDataBytes + trailerBytes);

      data = incomingBuffer;
      if ((settings.operatingMode == MODE_POINT_TO_POINT) || settings.verifyRxNetID)
      {
        systemPrint("    Net Id: ");
        systemPrint(*data);
        systemPrint(" (0x");
        systemPrint(*data++, HEX);
        systemPrintln(")");
      }
      printControl(*data++);
      memcpy(&channelTimer, data, sizeof(channelTimer));
      data += sizeof(channelTimer);
      systemPrint("    Channel Timer(ms): ");
      systemPrint(channelTimer);
      systemPrint(" (0x");
      systemPrint(channelTimer, HEX);
      systemPrintln(")");

      systemPrint("ERROR: Invalid msToNextHopRemote value, ");
      systemPrintln(msToNextHopRemote);
      return;
    }
  } //ENABLE_DEVELOPER

  //----------------------------------------------------------------------
  // Enter the critical section
  //----------------------------------------------------------------------

  //Synchronize with the hardware timer
  channelTimer.disableTimer();

  //When timeToHop is true, a hop is required to match the hops indicated by
  //the channelTimerStart value.  Delay this hop to avoid adding unaccounted
  //delay.  After the channel timer is restarted, perform this hop because
  //the channelTimerStart value indicated that it was done.  The channel
  //timer update will add only microseconds to when the hop is done.
  delayedHopCount = timeToHop ? 1 : 0;

  //The radios are using the same frequencies since the frame was successfully
  //received.  The goal is to adjust the channel timer to fire in close proximity
  //to the firing of the remote sysstem's channel timer.  The following cases
  //need to be identified:
  //
  // 1. Both systems are on the same channel
  //      Adjust channel timer value
  // 2. Remote system is on next channel (remote hopped, local did not)
  //      Immediate local hop
  //      Adjust channel timer value
  // 3. Remote system on previous channel (local hopped, remote did not)
  //      Extend channel timer value by maxDwellTime
  //
  //For low transmission rates, the transmission may have spanned multiple hops
  //and all of the frequencies must have matched for the frame to be received
  //successfully.  Therefore the above cases hold for low frequencies as well.
  //
  //Compute the remote system's channel timer firing time offset in milliseconds
  //using the channel timer value and the adjustments for transmit and receive
  //time (time of flight)
  frameAirTimeMsec = (frameAirTimeUsec + TX_TO_RX_USEC + micros() - transactionCompleteMicros) / 1000;
  rmtHopTimeMsec = msToNextHopRemote - frameAirTimeMsec;

  //Compute the when the local system last hopped
  lclHopTimeMsec = currentMillis - channelTimerStart;
  adjustment = 0;

#define REMOTE_SYSTEM_LIKELY_TO_HOP_MSEC    2
#define CHANNEL_TIMER_SYNC_MSEC   (frameAirTimeMsec + REMOTE_SYSTEM_LIKELY_TO_HOP_MSEC)

  //Determine if the remote has hopped or is likely to hop very soon
  if (rmtHopTimeMsec <= REMOTE_SYSTEM_LIKELY_TO_HOP_MSEC)
  {
    //Adjust the next channel timer value
    adjustment += settings.maxDwellTime;

    //Determine if the local system has just hopped
    if (((unsigned long)lclHopTimeMsec) <= CHANNEL_TIMER_SYNC_MSEC)
    {
      caseNumber = 1;
      //Case 1 above, both systems using the same channel
      //
      //Remote system
      //
      //  channelTimerStart                                        Channel timer fires
      //    |------...-------------------------------------------->|
      //                                     Channel Timer value   |
      //                                 |<----------------------->|
      //                                 |                         | rmtHopTimeMsec
      //                                 |                         |<--|
      //                                 |                             |
      //                                 |                           Current Time
      //                                 |<------------>|<------------>|
      //                                    txTimeUsec     rxTimeUsec  |
      //                                                               |
      //Local system                                                   |
      //                                                lclHopTimeMsec |
      //                                                      |------->|
      //                                                      |        | New timer value
      //                                                      |    |<--|----------------...-->|
      //                                                      |----------------...-->|
      //                                      channelTimerStart           Channel timer fires
      //
      //No hop is necessary
    }
    else
    {
      caseNumber = 2;
      //Case 2 above, the local system did not hop
      //
      //Remote system
      //
      //  channelTimerStart            Channel timer fires                                    Channel timer fires
      //    |...---------------------->|------...-------------------------------------------->|
      //           Channel Timer value |
      //                       |<----->|
      //                       |       |   rmtHopTimeMsec
      //                       |       |<--------------------|
      //                       |                           Current Time
      //                       |<------------>|<------------>|
      //                          txTimeUsec     rxTimeUsec  |
      //                                                     |
      //                                                     |
      //Local system                                         |
      //                                      Computed value |     New timer value
      //                               |<--------------------|------...----------------------->|
      //                                                     |
      //                            lclHopTimeMsec           |
      //               |------------------------------------>|
      //               |------...-------------------------------------------->|
      //       channelTimerStart                                     Channel timer fires
      //
      //A hop is necessary to get to a common channel
      delayedHopCount += 1;
    }
  }
  else
  {
    caseNumber = 3;
    //Case 3, both systems using the same channel
    //
    //  channelTimerStart                                        Channel timer fires
    //    |------...-------------------------------------------->|
    //                                Channel Timer value        |
    //                       |<--------------------------------->|
    //                       |                                   |
    //                       |                    rmtHopTimeMsec |
    //                       |                             |---->|
    //                       |                       Current Time
    //                       |<------------>|<------------>|
    //                          txTimeUsec     rxTimeUsec  |
    //                                                     |
    //Local system                                         |
    //                                                     | New timer value
    //                                                     |---->|
    //               |------...-------------------------------------->|
    //             channelTimerStart                       |          Channel timer fires
    //                                                     |--------->|
    //                                                    lclHopTimeMsec
    //
    //No hop is necessary
  }

  //Compute the next hop time
  msToNextHop = rmtHopTimeMsec + adjustment;

  //When the ISR fires, reload the channel timer with settings.maxDwellTime
  reloadChannelTimer = true;

  //Log the previous clock sync
  clockSyncData[clockSyncIndex].msToNextHop = msToNextHop;
  clockSyncData[clockSyncIndex].frameAirTimeMsec = frameAirTimeMsec;
  clockSyncData[clockSyncIndex].msToNextHopRemote = msToNextHopRemote;
  clockSyncData[clockSyncIndex].adjustment = adjustment;
  clockSyncData[clockSyncIndex].delayedHopCount = delayedHopCount;
  clockSyncData[clockSyncIndex].lclHopTimeMsec = lclHopTimeMsec;
  clockSyncData[clockSyncIndex].timeToHop = timeToHop;
  clockSyncIndex += 1;
  if(clockSyncIndex >= (sizeof(clockSyncData) / sizeof(CLOCK_SYNC_DATA)) ) clockSyncIndex = 0;

  //Restart the channel timer
  timeToHop = false;
  channelTimer.setInterval_MS(msToNextHop, channelTimerHandler); //Adjust our hardware timer to match our mate's
  digitalWrite(pin_hop_timer, ((channelNumber + delayedHopCount) % settings.numberOfChannels) & 1);
  channelTimerStart = currentMillis;
  channelTimerMsec = msToNextHop; //syncChannelTimer update
  channelTimer.enableTimer();

  //----------------------------------------------------------------------
  // Leave the critical section
  //----------------------------------------------------------------------

  //Trigger after adjustments to timer to avoid skew during debug
  triggerEvent(TRIGGER_SYNC_CHANNEL_TIMER);

  //Hop if the timer fired prior to disabling the timer, resetting the channelTimerStart value
  if (delayedHopCount)
    hopChannel(true, delayedHopCount);

  //Display the channel sync timer calculations
  if (settings.debugSync)
  {
    systemPrint("Case #");
    systemPrint(caseNumber);
    systemPrint(", ");
    systemPrint(delayedHopCount);
    systemPrint(" Hops, ");
    systemPrint(msToNextHopRemote);
    systemPrint(" Nxt Hop - ");
    systemPrint(frameAirTimeMsec);
    systemPrint(" (TX + RX)");
    if (adjustment)
    {
      systemPrint(" + ");
      systemPrint(adjustment);
      systemPrint(" Adj");
    }
    systemPrint(" = ");
    systemPrint(msToNextHop);
    systemPrintln(" mSec");
  }
}

//This function resets the heartbeat time and re-rolls the random time
//Call when something has happened (ACK received, etc) where clocks have been sync'd
//Short/long times set to avoid two radios attempting to xmit heartbeat at same time
//Those who send an ACK have short time to next heartbeat. Those who send a heartbeat or data have long time to next heartbeat.
void setHeartbeatShort()
{
  heartbeatTimer = millis();
  radioCallHistory[RADIO_CALL_setHeartbeatShort] = heartbeatTimer;

  heartbeatRandomTime = random(settings.heartbeatTimeout * 2 / 10, settings.heartbeatTimeout / 2); //20-50%

  //Slow datarates can have significant ack transmission times
  //Add the amount of time it takes to send an ack
  heartbeatRandomTime += frameAirTime + ackAirTime + settings.overheadTime + getReceiveCompletionOffset();
}

void setHeartbeatLong()
{
  heartbeatTimer = millis();
  radioCallHistory[RADIO_CALL_setHeartbeatLong] = heartbeatTimer;

  heartbeatRandomTime = random(settings.heartbeatTimeout * 8 / 10, settings.heartbeatTimeout); //80-100%

  //Slow datarates can have significant ack transmission times
  //Add the amount of time it takes to send an ack
  heartbeatRandomTime += frameAirTime + ackAirTime + settings.overheadTime + getReceiveCompletionOffset();
}

//Only the server sends heartbeats in multipoint mode
//Not random, just the straight timeout
void setHeartbeatMultipoint()
{
  heartbeatTimer = millis();
  radioCallHistory[RADIO_CALL_setHeartbeatMultipoint] = heartbeatTimer;

  heartbeatRandomTime = settings.heartbeatTimeout;

  //Slow datarates can have significant ack transmission times
  //Add the amount of time it takes to send an ack
  heartbeatRandomTime += frameAirTime + ackAirTime + settings.overheadTime + getReceiveCompletionOffset();
}

//Determine the delay for the next VC HEARTBEAT
void setVcHeartbeatTimer()
{
  long deltaMillis;

  heartbeatTimer = millis();
  radioCallHistory[RADIO_CALL_setVcHeartbeatTimer] = heartbeatTimer;

  /*
     The goal of this routine is to randomize the placement of the HEARTBEAT
     messages, allowing traffic to flow normally.  However since clients are
     waiting in channel zero (0) for a HEARTBEAT, the last couple of invervals
     are adjusted for the server to ensure that a HEARTBEAT is sent in channel
     zero.

     dwellTime: 400 mSec
     heartbeatTimeout: 3000 mSec
     50% heartbeatTimeout: 1500 mSec

     channel     38  39  40  41  42  43  44  45  46  47  48  49   0   1   2
     dwellTime    |   |   |   |   |   |   |   |   |   |   |   |   |   |   |
     seconds    |    .    |    .    |    .    |    .    |    .    |    .    |
     case 1: > 4.5, Use random, then remaining
                    ^--------------^^^^^^^^^^^^^^^^---------------^
     case 2: 4.5 >= X >= 3, Use half, then second half
                     ^^^^^^^^^^^^^^^^-------^^^^^^^^--------------^
     case 3: X < 3, Use remaining
                                     ^----------------------------^
  */
  petWDT();

  //Determine the delay before channel zero is reached
  deltaMillis = mSecToChannelZero();

  //Determine the delay before the next HEARTBEAT frame
  if ((!settings.server) || (deltaMillis > ((3 * settings.heartbeatTimeout) / 2))
      || (deltaMillis <= 0))
    //Use the random interval: 50% - 100%
    heartbeatRandomTime = random(settings.heartbeatTimeout / 2,
                                 settings.heartbeatTimeout);
  else if (deltaMillis >= settings.heartbeatTimeout)
    heartbeatRandomTime = deltaMillis / 2;
  else
    heartbeatRandomTime = deltaMillis + VC_DELAY_HEARTBEAT_MSEC;

  //Display the next HEARTBEAT time interval
  if (settings.debugHeartbeat)
  {
    systemPrint("mSecToChannelZero: ");
    systemPrintln(deltaMillis);
    systemPrint("heartbeatRandomTime: ");
    systemPrintln(heartbeatRandomTime);
  }
}

//Conversion table from radio status value into a status string
const I16_TO_STRING radioStatusCodes[] =
{
  {RADIOLIB_ERR_NONE, "RADIOLIB_ERR_NONE"},
  {RADIOLIB_ERR_UNKNOWN, "RADIOLIB_ERR_UNKNOWN"},
  {RADIOLIB_ERR_CHIP_NOT_FOUND, "RADIOLIB_ERR_CHIP_NOT_FOUND"},
  {RADIOLIB_ERR_MEMORY_ALLOCATION_FAILED, "RADIOLIB_ERR_MEMORY_ALLOCATION_FAILED"},
  {RADIOLIB_ERR_PACKET_TOO_LONG, "RADIOLIB_ERR_PACKET_TOO_LONG"},
  {RADIOLIB_ERR_TX_TIMEOUT, "RADIOLIB_ERR_TX_TIMEOUT"},
  {RADIOLIB_ERR_RX_TIMEOUT, "RADIOLIB_ERR_RX_TIMEOUT"},
  {RADIOLIB_ERR_CRC_MISMATCH, "RADIOLIB_ERR_CRC_MISMATCH"},
  {RADIOLIB_ERR_INVALID_BANDWIDTH, "RADIOLIB_ERR_INVALID_BANDWIDTH"},
  {RADIOLIB_ERR_INVALID_SPREADING_FACTOR, "RADIOLIB_ERR_INVALID_SPREADING_FACTOR"},
  {RADIOLIB_ERR_INVALID_CODING_RATE, "RADIOLIB_ERR_INVALID_CODING_RATE"},
  {RADIOLIB_ERR_INVALID_BIT_RANGE, "RADIOLIB_ERR_INVALID_BIT_RANGE"},
  {RADIOLIB_ERR_INVALID_FREQUENCY, "RADIOLIB_ERR_INVALID_FREQUENCY"},
  {RADIOLIB_ERR_INVALID_OUTPUT_POWER, "RADIOLIB_ERR_INVALID_OUTPUT_POWER"},
  {RADIOLIB_PREAMBLE_DETECTED, "RADIOLIB_PREAMBLE_DETECTED"},
  {RADIOLIB_CHANNEL_FREE, "RADIOLIB_CHANNEL_FREE"},
  {RADIOLIB_ERR_SPI_WRITE_FAILED, "RADIOLIB_ERR_SPI_WRITE_FAILED"},
  {RADIOLIB_ERR_INVALID_CURRENT_LIMIT, "RADIOLIB_ERR_INVALID_CURRENT_LIMIT"},
  {RADIOLIB_ERR_INVALID_PREAMBLE_LENGTH, "RADIOLIB_ERR_INVALID_PREAMBLE_LENGTH"},
  {RADIOLIB_ERR_INVALID_GAIN, "RADIOLIB_ERR_INVALID_GAIN"},
  {RADIOLIB_ERR_WRONG_MODEM, "RADIOLIB_ERR_WRONG_MODEM"},
  {RADIOLIB_ERR_INVALID_NUM_SAMPLES, "RADIOLIB_ERR_INVALID_NUM_SAMPLES"},
  {RADIOLIB_ERR_INVALID_RSSI_OFFSET, "RADIOLIB_ERR_INVALID_RSSI_OFFSET"},
  {RADIOLIB_ERR_INVALID_ENCODING, "RADIOLIB_ERR_INVALID_ENCODING"},
  {RADIOLIB_ERR_LORA_HEADER_DAMAGED, "RADIOLIB_ERR_LORA_HEADER_DAMAGED"},
};

//Return the status string matching the status value, return NULL if not found
const char * getRadioStatusCode(int status)
{
  for (int index = 0; index < sizeof(radioStatusCodes) / sizeof(radioStatusCodes[0]); index++)
  {
    if (radioStatusCodes[index].value == status)
      return radioStatusCodes[index].string;
  }
  return NULL;
}

//Conversion table from radio call value into a name string
const I16_TO_STRING radioCallName[] =
{
  {RADIO_CALL_configureRadio, "configureRadio"},
  {RADIO_CALL_setRadioFrequency, "setRadioFrequency"},
  {RADIO_CALL_returnToReceiving, "returnToReceiving"},
  {RADIO_CALL_calcAirTimeUsec, "calcAirTimeUsec"},
  {RADIO_CALL_xmitDatagramP2PFindPartner, "xmitDatagramP2PFindPartner"},
  {RADIO_CALL_xmitDatagramP2PSyncClocks, "xmitDatagramP2PSyncClocks"},
  {RADIO_CALL_xmitDatagramP2PZeroAcks, "xmitDatagramP2PZeroAcks"},
  {RADIO_CALL_xmitDatagramP2PCommand, "xmitDatagramP2PCommand"},
  {RADIO_CALL_xmitDatagramP2PCommandResponse, "xmitDatagramP2PCommandResponse"},
  {RADIO_CALL_xmitDatagramP2PData, "xmitDatagramP2PData"},
  {RADIO_CALL_xmitDatagramP2PHeartbeat, "xmitDatagramP2PHeartbeat"},
  {RADIO_CALL_xmitDatagramP2PAck, "xmitDatagramP2PAck"},
  {RADIO_CALL_xmitDatagramMpData, "xmitDatagramMpData"},
  {RADIO_CALL_xmitDatagramMpHeartbeat, "xmitDatagramMpHeartbeat"},
  {RADIO_CALL_xmitDatagramTrainingFindPartner, "xmitDatagramTrainingFindPartner"},
  {RADIO_CALL_xmitDatagramTrainingAck, "xmitDatagramTrainingAck"},
  {RADIO_CALL_xmitDatagramTrainRadioParameters, "xmitDatagramTrainRadioParameters"},
  {RADIO_CALL_xmitVcDatagram, "xmitVcDatagram"},
  {RADIO_CALL_xmitVcHeartbeat, "xmitVcHeartbeat"},
  {RADIO_CALL_xmitVcAckFrame, "xmitVcAckFrame"},
  {RADIO_CALL_xmitVcUnknownAcks, "xmitVcUpdateAcks"},
  {RADIO_CALL_xmitVcSyncAcks, "xmitVcSyncAcks"},
  {RADIO_CALL_xmitVcZeroAcks, "xmitVcZeroAcks"},
  {RADIO_CALL_rcvDatagram, "rcvDatagram"},
  {RADIO_CALL_transmitDatagram, "transmitDatagram"},
  {RADIO_CALL_retransmitDatagram, "retransmitDatagram"},
  {RADIO_CALL_startChannelTimer, "startChannelTimer"},
  {RADIO_CALL_stopChannelTimer, "stopChannelTimer"},
  {RADIO_CALL_syncChannelTimer, "syncChannelTimer"},
  {RADIO_CALL_setHeartbeatShort, "setHeartbeatShort"},
  {RADIO_CALL_setHeartbeatLong, "setHeartbeatLong"},
  {RADIO_CALL_setHeartbeatMultipoint, "setHeartbeatMultipoint"},
  {RADIO_CALL_setVcHeartbeatTimer, "setVcHeartbeatTimer"},
  {RADIO_CALL_hopChannel, "hopChannel"},

  //Insert new values before this line
  {RADIO_CALL_hopISR, "hopISR"},
  {RADIO_CALL_transactionCompleteISR, "transactionCompleteISR"},
  {RADIO_CALL_channelTimerHandler, "channelTimerHandler"},
#ifdef  RADIOLIB_LOW_LEVEL
  {RADIO_CALL_readSX1276Register, "readSX1276Register"},
  {RADIO_CALL_printSX1276Registers, "printSX1276Registers"},
#endif  //RADIOLIB_LOW_LEVEL
};

//Verify the RADIO_CALLS enum against the radioCallName
bool verifyRadioCallNames()
{
  bool valid;

  valid = ((sizeof(radioCallName) / sizeof(radioCallName[0])) == RADIO_CALL_MAX);
  if (!valid)
    systemPrintln("ERROR - Please update the radioCallName");
  return valid;
}

//Convert a radio call value into a string, return NULL if not found
const char * getRadioCall(uint8_t radioCall)
{
  for (int index = 0; index < sizeof(radioCallName) / sizeof(radioCallName[0]); index++)
  {
    if (radioCallName[index].value == radioCall)
      return radioCallName[index].string;
  }
  return NULL;
}

//Display the radio call history
void displayRadioCallHistory()
{
  uint8_t index;
  uint8_t sortOrder[RADIO_CALL_MAX];
  const char * string;
  uint8_t temp;

  //Set the default sort order
  petWDT();
  for (index = 0; index < RADIO_CALL_MAX; index++)
    sortOrder[index] = index;

  //Perform a bubble sort
  for (index = 0; index < RADIO_CALL_MAX; index++)
    for (int x = index + 1; x < RADIO_CALL_MAX; x++)
      if (radioCallHistory[sortOrder[index]] > radioCallHistory[sortOrder[x]])
      {
        temp = sortOrder[index];
        sortOrder[index] = sortOrder[x];
        sortOrder[x] = temp;
      }

  //Display the radio call history
  for (index = 0; index < RADIO_CALL_MAX; index++)
    if (radioCallHistory[sortOrder[index]])
    {
      systemPrint("        ");
      systemPrintTimestamp(radioCallHistory[sortOrder[index]] + timestampOffset);
      string = getRadioCall(sortOrder[index]);
      if (string)
      {
        systemPrint(": ");
        systemPrint(string);
      }
      systemPrintln();
    }
  petWDT();
}
