//Apply settings to radio
//Called after begin() and once user exits from command interface
void configureRadio()
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
  uint16_t hoppingPeriod = settings.maxDwellTime / calcSymbolTime(); //Limit FHSS dwell time to 400ms max. / automatically floors number
  if (hoppingPeriod > 255) hoppingPeriod = 255; //Limit to 8 bits.
  if (settings.frequencyHop == false) hoppingPeriod = 0; //Disable
  if (radio.setFHSSHoppingPeriod(hoppingPeriod) != RADIOLIB_ERR_NONE)
    success = false;

  //Precalculate the ACK packet time
  ackAirTime = calcAirTime(headerBytes + CHANNEL_TIMER_BYTES + trailerBytes); //Used for response timeout during ACK

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
    outputSerialData(true);
  }

  if (success == false)
  {
    reportERROR();
    systemPrintln("Radio init failed. Check settings.");
  }
  if ((settings.debug == true) || (settings.debugRadio == true))
  {
    systemPrintln("Radio configured");
    outputSerialData(true);
  }
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
  radioCallHistory[RADIO_CALL_setRadioFrequency] = millis();

  //Determine the frequency to use
  radioFrequency = channels[channelNumber];
  if (rxAdjust && settings.autoTuneFrequency)
    radioFrequency -= frequencyCorrection;

  //Set the new frequency
  if (radio.setFrequency(radioFrequency) == RADIOLIB_ERR_INVALID_FREQUENCY)
    return false;

  if (settings.debugSync)
    triggerFrequency(radioFrequency);

  //Determine the time in milliseconds when channel zero is reached again
  nextChannelZeroTimeInMillis = millis() + ((settings.numberOfChannels - channelNumber) * settings.maxDwellTime);

  //Print the frequency if requested
  if (settings.printFrequency)
  {
    systemPrintTimestamp();
    systemPrint(channelNumber);
    systemPrint(": ");
    systemPrint(radioFrequency, 3);
    systemPrint(" MHz, Ch 0 in ");
    systemPrint(nextChannelZeroTimeInMillis - millis());
    systemPrintln(" mSec");
    outputSerialData(true);
  }
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
    if ((settings.debug == true) || (settings.debugRadio == true))
    {
      systemPrint("Receive failed: ");
      systemPrintln(state);
      outputSerialData(true);
    }
  }
}

//Given spread factor, bandwidth, coding rate and number of bytes, return total Air Time in ms for packet
uint16_t calcAirTime(uint8_t bytesToSend)
{
  radioCallHistory[RADIO_CALL_calcAirTime] = millis();

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
  hopChannel(true); //Move forward
}

//Hop to the previous channel in the frequency list
void hopChannelReverse()
{
  hopChannel(false); //Move backward
}

//Set the next radio frequency given the hop direction and frequency table
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
  setRadioFrequency(radioStateTable[radioState].rxState);
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
//Read a register from the SX1276 chip
uint8_t readSX1276Register(uint8_t reg)
{
  radioCallHistory[RADIO_CALL_readSX1276Register] = millis();

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

  radioCallHistory[RADIO_CALL_printSX1276Registers] = millis();

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
  radioCallHistory[RADIO_CALL_transactionCompleteISR] = millis();

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
      linkupOffset = 0;
      break;
    case (38400):
      linkupOffset = 0;
      break;
  }

  return (settings.maxDwellTime - getReceiveCompletionOffset() - linkupOffset); //Reduce the default window by the offset
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
//Point-To-Point Training
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Ping the other radio in the point-to-point configuration
bool xmitDatagramP2PTrainingPing()
{
  radioCallHistory[RADIO_CALL_xmitDatagramP2PTrainingPing] = millis();

  /*
            endOfTxData ---.
                           |
                           V
      +---------+----------+
      |         |          |
      | Control | Trailer  |
      | 8 bits  | n Bytes  |
      +---------+----------+
  */

  txControl.datagramType = DATAGRAM_P2P_TRAINING_PING;
  return (transmitDatagram());
}

//Build the parameters packet used for training
bool xmitDatagramP2pTrainingParams()
{
  Settings params;

  radioCallHistory[RADIO_CALL_xmitDatagramP2pTrainingParams] = millis();

  //Initialize the radio parameters
  memcpy(&params, &originalSettings, sizeof(settings));
  params.operatingMode = MODE_POINT_TO_POINT;

  //Add the radio parameters
  memcpy(endOfTxData, &params, sizeof(params));
  endOfTxData += sizeof(params);

  /*
                          endOfTxData ---.
                                         |
                                         V
      +----------+---------+---  ...  ---+----------+
      | Optional |         |   Radio     | Optional |
      |  NET ID  | Control | Parameters  | Trailer  |
      |  8 bits  | 8 bits  |   n bytes   | n Bytes  |
      +----------+---------+-------------+----------+
  */

  txControl.datagramType = DATAGRAM_P2P_TRAINING_PARAMS;
  return (transmitDatagram());
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//Point-To-Point: Bring up the link
//
//A three way handshake is used to get both systems to agree that data can flow in both
//directions.  This handshake is also used to synchronize the HOP timer.
/*
                    System A                 System B

                     RESET                     RESET
                       |                         |
             Channel 0 |                         | Channel 0
                       V                         V
           .----> P2P_NO_LINK               P2P_NO_LINK
           |           | Tx PING                 |
           | Timeout   |                         |
           |           V                         |
           | P2P_WAIT_TX_PING_DONE               |
           |           |                         |
           |           | Tx Complete - - - - - > | Rx PING
           |           |   Start Rx              |
           |           |   MAX_PACKET_SIZE       |
           |           V                         V
           `---- P2P_WAIT_ACK_1                  +<----------------------.
                       |                         | Tx PING ACK1          |
                       |                         V                       |
                       |              P2P_WAIT_TX_ACK_1_DONE             |
                       |                         |                       |
          Rx PING ACK1 | < - - - - - - - - - - - | Tx Complete           |
                       |                         |   Start Rx            |
                       |                         |   MAX_PACKET_SIZE     |
                       |                         |                       |
                       V                         V         Timeout       |
           .---------->+                   P2P_WAIT_ACK_2 -------------->+
           |   TX PING |                         |                       ^
           |      ACK2 |                         |                       |
           |           V                         |                       |
           | P2P_WAIT_TX_ACK_2_DONE              |                       |
           |           | Tx Complete - - - - - > | Rx PING ACK2          |
           | Stop      |   Start HOP timer       |   Start HOP Timer     | Stop
           | HOP       |   Start Rx              |   Start Rx            | HOP
           | Timer     |   MAX_PACKET_SIZE       |   MAX_PACKET_SIZE     | Timer
           |           |                         |                       |
           | Rx        |                         |                       |
           | PING ACK  V                         V         Rx PING       |
           `----- P2P_LINK_UP               P2P_LINK_UP -----------------’
                       |                         |
                       | Rx Data                 | Rx Data
                       |                         |
                       V                         V

  Two timers are in use:
    datagramTimer:  Set at end of transmit, measures ACK timeout
    heartbeatTimer: Set upon entry to P2P_NO_LINK, measures time to send next PING
*/
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//First packet in the three way handshake to bring up the link
bool xmitDatagramP2PPing()
{
  radioCallHistory[RADIO_CALL_xmitDatagramP2PPing] = millis();

  unsigned long currentMillis = millis();
  memcpy(endOfTxData, &currentMillis, sizeof(currentMillis));
  endOfTxData += sizeof(unsigned long);

  /*
                    endOfTxData ---.
                                   |
                                   V
      +--------+---------+---------+----------+
      |        |         |         |          |
      | NET ID | Control | Millis  | Trailer  |
      | 8 bits | 8 bits  | 4 bytes | n Bytes  |
      +--------+---------+---------+----------+
  */

  txControl.datagramType = DATAGRAM_PING;
  return (transmitDatagram());
}

//Second packet in the three way handshake to bring up the link
bool xmitDatagramP2PAck1()
{
  radioCallHistory[RADIO_CALL_xmitDatagramP2PAck1] = millis();

  unsigned long currentMillis = millis();
  memcpy(endOfTxData, &currentMillis, sizeof(currentMillis));
  endOfTxData += sizeof(unsigned long);

  /*
                    endOfTxData ---.
                                   |
                                   V
      +--------+---------+---------+----------+
      |        |         |         | Optional |
      | NET ID | Control | Millis  | Trailer  |
      | 8 bits | 8 bits  | 4 bytes | n Bytes  |
      +--------+---------+---------+----------+
  */

  txControl.datagramType = DATAGRAM_ACK_1;
  return (transmitDatagram());
}

//Last packet in the three way handshake to bring up the link
bool xmitDatagramP2PAck2()
{
  radioCallHistory[RADIO_CALL_xmitDatagramP2PAck2] = millis();

  unsigned long currentMillis = millis();
  memcpy(endOfTxData, &currentMillis, sizeof(currentMillis));
  endOfTxData += sizeof(unsigned long);

  /*
                    endOfTxData ---.
                                   |
                                   V
      +--------+---------+---------+----------+
      |        |         |         |          |
      | NET ID | Control | Millis  | Trailer  |
      | 8 bits | 8 bits  | 4 bytes | n Bytes  |
      +--------+---------+---------+----------+
  */

  txControl.datagramType = DATAGRAM_ACK_2;
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
      +--------+---------+---  ...  ---+----------+
      |        |         |             | Optional |
      | NET ID | Control |    Data     | Trailer  |
      | 8 bits | 8 bits  |   n bytes   | n Bytes  |
      +--------+---------+-------------+----------+
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
      +--------+---------+---  ...  ---+----------+
      |        |         |             | Optional |
      | NET ID | Control |    Data     | Trailer  |
      | 8 bits | 8 bits  |   n bytes   | n Bytes  |
      +--------+---------+-------------+----------+
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
      +--------+---------+---  ...  ---+----------+
      |        |         |             | Optional |
      | NET ID | Control |    Data     | Trailer  |
      | 8 bits | 8 bits  |   n bytes   | n Bytes  |
      +--------+---------+-------------+----------+
  */

  txControl.datagramType = DATAGRAM_DATA;
  return (transmitDatagram());
}

//Heartbeat packet to keep the link up
bool xmitDatagramP2PHeartbeat()
{
  radioCallHistory[RADIO_CALL_xmitDatagramP2PHeartbeat] = millis();

  unsigned long currentMillis = millis();
  memcpy(endOfTxData, &currentMillis, sizeof(currentMillis));
  endOfTxData += sizeof(currentMillis);

  /*
                    endOfTxData ---.
                                   |
                                   V
      +--------+---------+---------+----------+
      |        |         |         | Optional |
      | NET ID | Control | Millis  | Trailer  |
      | 8 bits | 8 bits  | 4 Bytes | n Bytes  |
      +--------+---------+---------+----------+
  */

  txControl.datagramType = DATAGRAM_HEARTBEAT;
  return (transmitDatagram());
}

//Create short packet of 2 control bytes - do not expect ack
bool xmitDatagramP2PAck()
{
  int ackLength;

  radioCallHistory[RADIO_CALL_xmitDatagramP2PAck] = millis();

  uint8_t * ackStart = endOfTxData;
  uint16_t msToNextHop = settings.maxDwellTime - (millis() - timerStart);
  memcpy(endOfTxData, &msToNextHop, sizeof(msToNextHop));
  endOfTxData += sizeof(msToNextHop);

  //Verify the ACK length
  ackLength = endOfTxData - ackStart;
  if (ackLength != CLOCK_SYNC_BYTES)
  {
    systemPrint("ERROR - Please define CLOCK_SYNC_BYTES = ");
    systemPrintln(ackLength);
    waitForever();
  }

  /*
                     endOfTxData ---.
                                    |
                                    V
      +--------+---------+----------+----------+
      |        |         | Channel  | Optional |
      | NET ID | Control |  Timer   | Trailer  |
      | 8 bits | 8 bits  | 2 bytes  | n Bytes  |
      +--------+---------+----------+----------+
  */

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

  uint16_t msToNextHop = settings.maxDwellTime - (millis() - timerStart);
  memcpy(endOfTxData, &msToNextHop, sizeof(msToNextHop));
  endOfTxData += sizeof(msToNextHop);

  /*
                                   endOfTxData ---.
                                                  |
                                                  V
      +--------+---------+---  ...  ---+----------+----------+
      |        |         |             | Channel  | Optional |
      | NET ID | Control |    Data     |  Timer   | Trailer  |
      | 8 bits | 8 bits  |   n bytes   | 2 bytes  | n Bytes  |
      +--------+---------+-------------+----------+----------+
  */



  txControl.datagramType = DATAGRAM_DATA;
  return (transmitDatagram());
}

//Heartbeat packet to sync other units in multipoint mode
bool xmitDatagramMpHeartbeat()
{
  radioCallHistory[RADIO_CALL_xmitDatagramMpHeartbeat] = millis();

  uint16_t msToNextHop = settings.maxDwellTime - (millis() - timerStart);
  memcpy(endOfTxData, &msToNextHop, sizeof(msToNextHop));
  endOfTxData += sizeof(msToNextHop);

  /*
                     endOfTxData ---.
                                    |
                                    V
      +--------+---------+----------+----------+
      |        |         | Channel  | Optional |
      | NET ID | Control |  Timer   | Trailer  |
      | 8 bits | 8 bits  | 2 bytes  | n Bytes  |
      +--------+---------+----------+----------+
  */

  txControl.datagramType = DATAGRAM_HEARTBEAT;
  return (transmitDatagram());
}

//Ack packet sent by server in response the client ping, includes sync data and channel number
//During Multipoint scanning, it's possible for the client to get an ack but be 500kHz off
//The channel Number ensures that the client gets the next hop correct
bool xmitDatagramMpAck()
{
  radioCallHistory[RADIO_CALL_xmitDatagramMpAck] = millis();

  memcpy(endOfTxData, &channelNumber, sizeof(channelNumber));
  endOfTxData += sizeof(channelNumber);

  uint16_t msToNextHop = settings.maxDwellTime - (millis() - timerStart);
  memcpy(endOfTxData, &msToNextHop, sizeof(msToNextHop));
  endOfTxData += sizeof(msToNextHop);


  /*
                               endOfTxData ---.
                                              |
                                              V
      +--------+---------+---------+----------+----------+
      |        |         | Channel | Channel  | Optional |
      | NET ID | Control | Number  |  Timer   | Trailer  |
      | 8 bits | 8 bits  | 1 byte  | 2 bytes  | n Bytes  |
      +--------+---------+---------+----------+----------+
  */

  txControl.datagramType = DATAGRAM_ACK_1;
  return (transmitDatagram());
}

//Ping packet sent during scanning
bool xmitDatagramMpPing()
{
  radioCallHistory[RADIO_CALL_xmitDatagramMpPing] = millis();

  /*
          endOfTxData ---.
                         |
                         V
      +--------+---------+----------+
      |        |         | Optional |
      | NET ID | Control | Trailer  |
      | 8 bits | 8 bits  | n Bytes  |
      +--------+---------+----------+
  */

  txControl.datagramType = DATAGRAM_PING;
  return (transmitDatagram());
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//Multi-Point Client Training
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Build the client ping packet used for training
bool xmitDatagramTrainingPing()
{
  radioCallHistory[RADIO_CALL_xmitDatagramTrainingPing] = millis();

  //Add the source (server) ID
  memcpy(endOfTxData, myUniqueId, UNIQUE_ID_BYTES);
  endOfTxData += UNIQUE_ID_BYTES;

  /*
                        endOfTxData ---.
                                       |
                                       V
      +----------+---------+-----------+----------+
      | Optional |         |           | Optional |
      |  NET ID  | Control | Client ID | Trailer  |
      |  8 bits  | 8 bits  |  16 Bytes | n Bytes  |
      +----------+---------+-----------+----------+
  */

  txControl.datagramType = DATAGRAM_TRAINING_PING;
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
      +----------+---------+-----------+-----------+----------+
      | Optional |         |           |           | Optional |
      |  NET ID  | Control | Server ID | Client ID | Trailer  |
      |  8 bits  | 8 bits  |  16 Bytes |  16 Bytes | n Bytes  |
      +----------+---------+-----------+-----------+----------+
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
  originalSettings.airSpeed = params.airSpeed;
  originalSettings.autoTuneFrequency = params.autoTuneFrequency;
  originalSettings.radioBandwidth = params.radioBandwidth;
  originalSettings.radioCodingRate = params.radioCodingRate;
  originalSettings.frequencyHop = params.frequencyHop;
  originalSettings.frequencyMax = params.frequencyMax;
  originalSettings.frequencyMin = params.frequencyMin;
  originalSettings.maxDwellTime = params.maxDwellTime;
  originalSettings.numberOfChannels = params.numberOfChannels;
  originalSettings.radioPreambleLength = params.radioPreambleLength;
  originalSettings.radioSpreadFactor = params.radioSpreadFactor;
  originalSettings.radioSyncWord = params.radioSyncWord;
  originalSettings.radioBroadcastPower_dbm = params.radioBroadcastPower_dbm;

  //Update the radio protocol parameters
  originalSettings.dataScrambling = params.dataScrambling;
  originalSettings.enableCRC16 = params.enableCRC16;
  originalSettings.encryptData = params.encryptData;
  memcpy(originalSettings.encryptionKey, params.encryptionKey, sizeof(originalSettings.encryptionKey));
  originalSettings.serialTimeoutBeforeSendingFrame_ms = params.serialTimeoutBeforeSendingFrame_ms;
  originalSettings.heartbeatTimeout = params.heartbeatTimeout;
  originalSettings.maxResends = params.maxResends;
  originalSettings.netID = params.netID;
  originalSettings.operatingMode = params.operatingMode;
  originalSettings.overheadTime = params.overheadTime;
  originalSettings.server = params.server;
  originalSettings.verifyRxNetID = params.verifyRxNetID;

  //Update the debug parameters
  if (params.copyDebug)
  {
    originalSettings.debug = params.debug;
    originalSettings.copyDebug = params.copyDebug;
    originalSettings.debug = params.debug;
    originalSettings.debugDatagrams = params.debugDatagrams;
    originalSettings.debugHeartbeat = params.debugHeartbeat;
    originalSettings.debugNvm = params.debugNvm;
    originalSettings.debugRadio = params.debugRadio;
    originalSettings.debugReceive = params.debugReceive;
    originalSettings.debugStates = params.debugStates;
    originalSettings.debugSync = params.debugSync;
    originalSettings.debugTraining = params.debugTraining;
    originalSettings.debugTransmit = params.debugTransmit;
    originalSettings.debugSerial = params.debugSerial;
    originalSettings.displayPacketQuality = params.displayPacketQuality;
    originalSettings.displayRealMillis = params.displayRealMillis;
    originalSettings.printAckNumbers = params.printAckNumbers;
    originalSettings.printFrequency = params.printFrequency;
    originalSettings.printLinkUpDown = params.printLinkUpDown;
    originalSettings.printPktData = params.printPktData;
    originalSettings.printRfData = params.printRfData;
    originalSettings.printTimestamp = params.printTimestamp;
    originalSettings.printTxErrors = params.printTxErrors;
    originalSettings.selectLedUse = params.selectLedUse;
  }

  //Update the serial parameters
  if (params.copySerial)
  {
    originalSettings.copySerial = params.copySerial;
    originalSettings.echo = params.echo;
    originalSettings.flowControl = params.flowControl;
    originalSettings.invertCts = params.invertCts;
    originalSettings.invertRts = params.invertRts;
    originalSettings.serialSpeed = params.serialSpeed;
    originalSettings.usbSerialWait = params.usbSerialWait;
  }

  //Update the training values
  originalSettings.clientPingRetryInterval = params.clientPingRetryInterval;
  //The trainingKey is already the same
  originalSettings.trainingTimeout = params.trainingTimeout;

  //Update the trigger parameters
  if (params.copyTriggers)
  {
    originalSettings.copyTriggers = params.copyTriggers;
    originalSettings.triggerEnable = params.triggerEnable;
    originalSettings.triggerEnable2 = params.triggerEnable2;
    originalSettings.triggerWidth = params.triggerWidth;
    originalSettings.triggerWidthIsMultiplier = params.triggerWidthIsMultiplier;
  }
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//Multi-Point Server Training
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Build the server parameters packet used for training
bool xmitDatagramTrainRadioParameters(const uint8_t * clientID)
{
  Settings params;

  radioCallHistory[RADIO_CALL_xmitDatagramTrainRadioParameters] = millis();

  //Initialize the radio parameters
  memcpy(&params, &originalSettings, sizeof(settings));
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
      +----------+---------+-----------+-----------+---  ...  ---+----------+
      | Optional |         |           |           |   Radio     | Optional |
      |  NET ID  | Control | Client ID | Server ID | Parameters  | Trailer  |
      |  8 bits  | 8 bits  |  16 Bytes |  16 Bytes |   n bytes   | n Bytes  |
      +----------+---------+-----------+-----------+-------------+----------+
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
      +----------+---------+--------+----------+---------+---------+----------+
      | Optional |         |        |          |         |         | Optional |
      |  NET ID  | Control | Length | DestAddr | SrcAddr |  Data   | Trailer  |
      |  8 bits  | 8 bits  | 8 bits |  8 bits  | 8 bits  | n Bytes | n Bytes  |
      +----------+---------+--------+----------+---------+---------+----------+
  */

  txControl.datagramType = DATAGRAM_DATAGRAM;
  txControl.ackNumber = 0;
  return (transmitDatagram());
}

//Broadcast a HEARTBEAT to all of the VCs
bool xmitVcHeartbeat(int8_t addr, uint8_t * id)
{
  uint32_t currentMillis = millis();
  uint8_t * txData;

  radioCallHistory[RADIO_CALL_xmitVcHeartbeat] = currentMillis;

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

  //Add the channel timer for HOP synchronization
  uint16_t msToNextHop = settings.maxDwellTime - (millis() - timerStart);
  memcpy(endOfTxData, &msToNextHop, sizeof(msToNextHop));
  endOfTxData += sizeof(msToNextHop);

  //Set the length field
  *txData = (uint8_t)(endOfTxData - txData);

  /*
                                                                          endOfTxData ---.
                                                                                         |
                                                                                         V
      +----------+---------+--------+----------+---------+----------+---------+----------+----------+
      | Optional |         |        |          |         |          |         | Channel  | Optional |
      |  NET ID  | Control | Length | DestAddr | SrcAddr |  Src ID  | millis  |  Timer   | Trailer  |
      |  8 bits  | 8 bits  | 8 bits |  8 bits  | 8 bits  | 16 Bytes | 4 Bytes | 2 bytes  || n Bytes  |
      +----------+---------+--------+----------+---------+----------+---------+----------+----------+
  */

  txControl.datagramType = DATAGRAM_VC_HEARTBEAT;
  txControl.ackNumber = 0;

  //Determine the time that it took to pass this frame to the radio
  //This time is used to adjust the time offset
  vcTxHeartbeatMillis = millis() - currentMillis;

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
  vcHeader->length = VC_RADIO_HEADER_BYTES + CHANNEL_TIMER_BYTES;
  vcHeader->destVc = destVc;
  vcHeader->srcVc = myVc;
  endOfTxData += VC_RADIO_HEADER_BYTES;

  /*
                                        endOfTxData ---.
                                                       |
                                                       V
      +--------+---------+--------+----------+---------+----------+----------+
      |        |         |        |          |         | Channel  | Optional |
      | NET ID | Control | Length | DestAddr | SrcAddr |  Timer   | Trailer  |
      | 8 bits | 8 bits  | 8 bits |  8 bits  | 8 bits  | 2 bytes  | n Bytes  |
      +--------+---------+--------+----------+---------+----------+----------+
  */

  //Finish building the ACK frame
  return xmitDatagramP2PAck();
}

bool xmitVcPing(int8_t destVc)
{
  VC_RADIO_MESSAGE_HEADER * vcHeader;

  radioCallHistory[RADIO_CALL_xmitVcPing] = millis();

  vcHeader = (VC_RADIO_MESSAGE_HEADER *)endOfTxData;
  vcHeader->length = VC_RADIO_HEADER_BYTES;
  vcHeader->destVc = destVc;
  vcHeader->srcVc = myVc;
  endOfTxData += VC_RADIO_HEADER_BYTES;

  /*
                                        endOfTxData ---.
                                                       |
                                                       V
      +--------+---------+--------+----------+---------+----------+
      |        |         |        |          |         | Optional |
      | NET ID | Control | Length | DestAddr | SrcAddr | Trailer  |
      | 8 bits | 8 bits  | 8 bits |  8 bits  | 8 bits  | n Bytes  |
      +--------+---------+--------+----------+---------+----------+
  */

  txControl.datagramType = DATAGRAM_PING;
  return (transmitDatagram());
}

bool xmitVcAck1(int8_t destVc)
{
  VC_RADIO_MESSAGE_HEADER * vcHeader;

  radioCallHistory[RADIO_CALL_xmitVcAck1] = millis();

  vcHeader = (VC_RADIO_MESSAGE_HEADER *)endOfTxData;
  vcHeader->length = VC_RADIO_HEADER_BYTES;
  vcHeader->destVc = destVc;
  vcHeader->srcVc = myVc;
  endOfTxData += VC_RADIO_HEADER_BYTES;

  /*
                                        endOfTxData ---.
                                                       |
                                                       V
      +--------+---------+--------+----------+---------+----------+
      |        |         |        |          |         | Optional |
      | NET ID | Control | Length | DestAddr | SrcAddr | Trailer  |
      | 8 bits | 8 bits  | 8 bits |  8 bits  | 8 bits  | n Bytes  |
      +--------+---------+--------+----------+---------+----------+
  */

  txControl.datagramType = DATAGRAM_ACK_1;
  return (transmitDatagram());
}

bool xmitVcAck2(int8_t destVc)
{
  VC_RADIO_MESSAGE_HEADER * vcHeader;

  radioCallHistory[RADIO_CALL_xmitVcAck2] = millis();

  vcHeader = (VC_RADIO_MESSAGE_HEADER *)endOfTxData;
  vcHeader->length = VC_RADIO_HEADER_BYTES;
  vcHeader->destVc = destVc;
  vcHeader->srcVc = myVc;
  endOfTxData += VC_RADIO_HEADER_BYTES;

  /*
                                        endOfTxData ---.
                                                       |
                                                       V
      +--------+---------+--------+----------+---------+----------+
      |        |         |        |          |         | Optional |
      | NET ID | Control | Length | DestAddr | SrcAddr | Trailer  |
      | 8 bits | 8 bits  | 8 bits |  8 bits  | 8 bits  | n Bytes  |
      +--------+---------+--------+----------+---------+----------+
  */

  txControl.datagramType = DATAGRAM_ACK_2;
  return (transmitDatagram());
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//Datagram reception
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

  //Get the received datagram
  framesReceived++;
  int state = radio.readData(incomingBuffer, MAX_PACKET_SIZE);

  rssi = radio.getRSSI();
  printPacketQuality(); //Display the RSSI, SNR and frequency error values

  if (state == RADIOLIB_ERR_NONE)
  {
    rxSuccessMillis = rcvTimeMillis;
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
      |<---------------------- rxDataBytes ---------------------->|
      |                                                           |
      +----------+----------+------------+---  ...  ---+----------+
      | Optional |          |  Optional  |             | Optional |
      |  NET ID  | Control  | SF6 Length |    Data     | Trailer  |
      |  8 bits  |  8 bits  |   8 bits   |   n bytes   | n Bytes  |
      +----------+----------+------------+-------------+----------+
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
      +----------+----------+------------+---  ...  ---+----------+
      | Optional |          |  Optional  |             | Optional |
      |  NET ID  | Control  | SF6 Length |    Data     | Trailer  |
      |  8 bits  |  8 bits  |   8 bits   |   n bytes   | n Bytes  |
      +----------+----------+------------+-------------+----------+
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
      |<---------------------- rxDataBytes ---------------------->|
      |                                                           |
      +----------+----------+------------+---  ...  ---+----------+
      | Optional |          |  Optional  |             | Optional |
      |  NET ID  | Control  | SF6 Length |    Data     | Trailer  |
      |  8 bits  |  8 bits  |   8 bits   |   n bytes   | n Bytes  |
      +----------+----------+------------+-------------+----------+
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
      |<---------------------- rxDataBytes ---------------------->|
      |                                                           |
      +----------+----------+------------+---  ...  ---+----------+
      | Optional |          |  Optional  |             | Optional |
      |  NET ID  | Control  | SF6 Length |    Data     | Trailer  |
      |  8 bits  |  8 bits  |   8 bits   |   n bytes   | n Bytes  |
      +----------+----------+------------+-------------+----------+
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

  /*
                                         |<-- rxDataBytes -->|
                                         |                   |
      +----------+----------+------------+------  ...  ------+----------+
      | Optional |          |  Optional  |                   | Optional |
      |  NET ID  | Control  | SF6 Length |       Data        | Trailer  |
      |  8 bits  |  8 bits  |   8 bits   |      n bytes      | n Bytes  |
      +----------+----------+------------+-------------------+----------+
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
  if (settings.selectLedUse == LEDS_RADIO_USE)
    digitalWrite(ALT_LED_RX_DATA, LED_ON);
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

  radioCallHistory[RADIO_CALL_transmitDatagram] = millis();

  if (timeToHop == true) //If the channelTimer has expired, move to next frequency
    hopChannel();

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

  //Process the packet
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
      +----------+----------+------------+---  ...  ---+----------+
      | Optional |          |  Optional  |             | Optional |
      |  NET ID  | Control  | SF6 Length |    Data     | Trailer  |
      |  8 bits  |  8 bits  |   8 bits   |   n bytes   | n Bytes  |
      +----------+----------+------------+-------------+----------+
      |                                  |             |
      |                                  |<- Length -->|
      |<--------- txDatagramSize --------------------->|
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

  //Display the control value
  if (settings.debugTransmit)
    printControl(control);

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

      case DATAGRAM_PING:
      case DATAGRAM_ACK_1:
      case DATAGRAM_ACK_2:
        txDatagramSize = headerBytes + CLOCK_MILLIS_BYTES; //Short packet is 3 + 4
        break;

      case DATAGRAM_DATA_ACK:
        txDatagramSize = headerBytes + CLOCK_SYNC_BYTES; //Short ACK packet is 3 + 2
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
      +----------+----------+------------+---  ...  ---+
      | Optional |          |  Optional  |             |
      |  NET ID  | Control  | SF6 Length |    Data     |
      |  8 bits  |  8 bits  |   8 bits   |   n bytes   |
      +----------+----------+------------+-------------+
      |                                                |
      |<--------------- txDatagramSize --------------->|
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
      +----------+----------+------------+---  ...  ---+----------+
      | Optional |          |  Optional  |             | Optional |
      |  NET ID  | Control  | SF6 Length |    Data     | Trailer  |
      |  8 bits  |  8 bits  |   8 bits   |   n bytes   | n Bytes  |
      +----------+----------+------------+-------------+----------+
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

  //Print before encryption
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
  frameAirTime = calcAirTime(txDatagramSize);

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
      +----------+----------+------------+---  ...  ---+----------+
      | Optional |          |  Optional  |             | Optional |
      |  NET ID  | Control  | SF6 Length |    Data     | Trailer  |
      |  8 bits  |  8 bits  |   8 bits   |   n bytes   | n Bytes  |
      +----------+----------+------------+-------------+----------+
      |                                                           |
      |<-------------------- txDatagramSize --------------------->|
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

  //Drop this datagram if the receiver is active
  frameAirTime = calcAirTime(txDatagramSize); //Calculate frame air size while we're transmitting in the background
  uint16_t responseDelay = frameAirTime / responseDelayDivisor; //Give the receiver a bit of wiggle time to respond
  if ((receiveInProcess() == true) || (transactionComplete == true)
      || ((settings.operatingMode == MODE_VIRTUAL_CIRCUIT) && (txDestVc != VC_BROADCAST)
          && (virtualCircuitList[txDestVc & VCAB_NUMBER_MASK].vcState == VC_STATE_LINK_DOWN)))
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

    int state = radio.startTransmit(outgoingPacket, txDatagramSize);

    if (state == RADIOLIB_ERR_NONE)
    {
      xmitTimeMillis = millis();
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
  retransmitTimeout = random(ackAirTime, frameAirTime + ackAirTime); //Wait this number of ms between retransmits. Increases with each re-transmit.

  //BLink the RX LED
  if (settings.selectLedUse == LEDS_RADIO_USE)
    digitalWrite(ALT_LED_TX_DATA, LED_ON);

  return (true); //Tranmission has started
}

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
  channelTimer.enableTimer();
  timerStart = millis(); //ISR normally takes care of this but allow for correct ACK sync before first ISR
  triggerEvent(TRIGGER_HOP_TIMER_START);
}

//Stop the channel (hop) timer
void stopChannelTimer()
{
  radioCallHistory[RADIO_CALL_stopChannelTimer] = millis();

  channelTimer.disableTimer();
  triggerEvent(TRIGGER_HOP_TIMER_STOP);
}

//Given the remote unit's number of ms before its next hop,
//adjust our own channelTimer interrupt to be synchronized with the remote unit
void syncChannelTimer()
{
  int16_t msToNextHopRemote; //Can become negative
  memcpy(&msToNextHopRemote, &rxVcData[rxDataBytes - 2], sizeof(msToNextHopRemote));

  radioCallHistory[RADIO_CALL_syncChannelTimer] = millis();
  //If the sync arrived in an ACK, we know how long that packet took to transmit
  //Calculate the packet airTime based on the size of data received
  msToNextHopRemote -= calcAirTime(packetLength);


  //Different airspeeds complete the transmitComplete ISR at different rates
  //We adjust the clock setup as needed
  switch (settings.airSpeed)
  {
    default:
      break;
    case (40):
      msToNextHopRemote -= getReceiveCompletionOffset();
      break;
    case (150):
      msToNextHopRemote -= 145;
      break;
    case (400):
      msToNextHopRemote -= getReceiveCompletionOffset();
      break;
    case (1200):
      msToNextHopRemote -= getReceiveCompletionOffset();
      break;
    case (2400):
      msToNextHopRemote -= getReceiveCompletionOffset();
      break;
    case (4800):
      msToNextHopRemote -= 2;
      break;
    case (9600):
      break;
    case (19200):
      msToNextHopRemote -= 2;
      break;
    case (28800):
      msToNextHopRemote -= 2;
      break;
    case (38400):
      msToNextHopRemote -= 3;
      break;
  }

  int16_t msToNextHopLocal = settings.maxDwellTime - (millis() - timerStart);

  //Precalculate large/small time amounts
  uint16_t smallAmount = settings.maxDwellTime / 8;
  uint16_t largeAmount = settings.maxDwellTime - smallAmount;

  int16_t msToNextHop = msToNextHopRemote; //By default, we will adjust our clock to match our mate's

  bool resetHop = false; //The hop ISR may occur while we are forcing a hop (case A and C). Reset timeToHop as needed.

  //Below are the edge cases that occur when a hop occurs near ACK reception

  //msToNextHopLocal is small and msToNextHopRemote is negative (and small)
  //If we are about to hop (msToNextHopLocal is small), and a msToNextHopRemote comes in negative (and small) then the remote has hopped
  //Then hop now, and adjust our clock to the remote's next hop (msToNextHopRemote + dwellTime)
  if (msToNextHopLocal < smallAmount && (msToNextHopRemote <= 0 && msToNextHopRemote >= (smallAmount * -1)))
  {
    hopChannel();
    msToNextHop = msToNextHopRemote + settings.maxDwellTime;
    resetHop = true; //We moved channels. Don't allow the ISR to move us again until after we've updated the timer.
  }

  //msToNextHopLocal is large and msToNextHopRemote is negative
  //If we just hopped (msToNextHopLocal is large), and msToNextHopRemote comes in negative then the remote has hopped
  //No need to hop. Adjust our clock to the remote's next hop (msToNextHopRemote + dwellTime)
  else if (msToNextHopLocal > largeAmount && msToNextHopRemote <= 0)
  {
    msToNextHop = msToNextHopRemote + settings.maxDwellTime;
  }

  //msToNextHopLocal is small and msToNextHopRemote is large
  //If we are about to hop (msToNextHopLocal is small), and a msToNextHopRemote comes in large then the remote has hopped recently
  //Then hop now, and adjust our clock to the remote's next hop (msToNextHopRemote)
  else if (msToNextHopLocal < smallAmount && msToNextHopRemote > largeAmount)
  {
    hopChannel();
    msToNextHop = msToNextHopRemote;
    resetHop = true; //We moved channels. Don't allow the ISR to move us again until after we've updated the timer.
  }

  //msToNextHopLocal is large and msToNextHopRemote is large
  //If we just hopped (msToNextHopLocal is large), and a msToNextHopRemote comes in large then the remote has hopped
  //Then adjust our clock to the remote's next hop (msToNextHopRemote)
  else if (msToNextHopLocal > largeAmount && msToNextHopRemote > largeAmount)
  {
    msToNextHop = msToNextHopRemote;
  }

  //msToNextHopLocal is large and msToNextHopRemote is small
  //If we just hopped (msToNextHopLocal is large), and a msToNextHopRemote comes in small then the remote is about to hop
  //Then adjust our clock to the remote's next hop (msToNextHopRemote + dwellTime)
  else if (msToNextHopLocal > largeAmount && msToNextHopRemote < smallAmount)
  {
    msToNextHop = msToNextHopRemote + settings.maxDwellTime;
  }

  //msToNextHopLocal is small and msToNextHopRemote is small
  //If we are about to hop (msToNextHopLocal is small), and a msToNextHopRemote comes in small then the remote is about to hop
  //Then adjust our clock to the remote's next hop (msToNextHopRemote)
  else if (msToNextHopLocal < smallAmount && msToNextHopRemote < smallAmount)
  {
    msToNextHop = msToNextHopRemote;

    //If we have a negative remote hop time that is larger than a dwell time then the remote has hopped again
    //This is seen at lower air speeds
    //Hop now, and adjust our clock to the remote's next hop (msToNextHopRemote + dwellTime)
    if (msToNextHop < (settings.maxDwellTime * -1)) //-402 < -400
    {
      hopChannel();
      msToNextHop += settings.maxDwellTime;
      resetHop = true; //We moved channels. Don't allow the ISR to move us again until after we've updated the timer.
    }
  }

  //Insure against negative timer values
  while (msToNextHop < 0)
    msToNextHop += settings.maxDwellTime;

  if (settings.debugSync)
  {
    systemPrint("msToNextHopRemote: ");
    systemPrint(msToNextHopRemote);
    systemPrint(" msToNextHopLocal: ");
    systemPrint(msToNextHopLocal);
    systemPrint(" msToNextHop: ");
    systemPrint(msToNextHop);
    systemPrintln();
  }

  partialTimer = true;
  channelTimer.disableTimer();
  channelTimer.setInterval_MS(msToNextHop, channelTimerHandler); //Adjust our hardware timer to match our mate's

  if (resetHop) //We moved channels. Don't allow the ISR to move us again until after we've updated the timer.
    timeToHop = false;

  channelTimer.enableTimer();

  triggerEvent(TRIGGER_SYNC_CHANNEL); //Trigger after adjustments to timer to avoid skew during debug
  if (settings.debugSync)
    outputSerialData(true);
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
  deltaMillis = nextChannelZeroTimeInMillis - heartbeatTimer;
  if (deltaMillis <= 0)
  {
    nextChannelZeroTimeInMillis = heartbeatTimer + ((settings.numberOfChannels - channelNumber) * settings.maxDwellTime);
    deltaMillis = nextChannelZeroTimeInMillis - heartbeatTimer;
  }

  //Determine the delay before the next HEARTBEAT frame
  if ((!settings.server) || (deltaMillis > ((3 * settings.heartbeatTimeout) / 2))
      || (deltaMillis <= 0))
    //Use the random interval: 50% - 100%
    heartbeatRandomTime = random(settings.heartbeatTimeout / 2,
                                 settings.heartbeatTimeout);
  else if (deltaMillis >= settings.heartbeatTimeout)
    heartbeatRandomTime = deltaMillis / 2;
  else
    heartbeatRandomTime = deltaMillis;

  //Display the next HEARTBEAT time interval
  if (settings.debugHeartbeat)
  {
    systemPrint("deltaMillis: ");
    systemPrintln(deltaMillis);
    systemPrint("heartbeatRandomTime: ");
    systemPrintln(heartbeatRandomTime);
    outputSerialData(true);
    petWDT();
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
  {RADIO_CALL_calcAirTime, "calcAirTime"},
  {RADIO_CALL_xmitDatagramP2PTrainingPing, "xmitDatagramP2PTrainingPing"},
  {RADIO_CALL_xmitDatagramP2pTrainingParams, "xmitDatagramP2pTrainingParams"},
  {RADIO_CALL_xmitDatagramP2PPing, "xmitDatagramP2PPing"},
  {RADIO_CALL_xmitDatagramP2PAck1, "xmitDatagramP2PAck1"},
  {RADIO_CALL_xmitDatagramP2PAck2, "xmitDatagramP2PAck2"},
  {RADIO_CALL_xmitDatagramP2PCommand, "xmitDatagramP2PCommand"},
  {RADIO_CALL_xmitDatagramP2PCommandResponse, "xmitDatagramP2PCommandResponse"},
  {RADIO_CALL_xmitDatagramP2PData, "xmitDatagramP2PData"},
  {RADIO_CALL_xmitDatagramP2PHeartbeat, "xmitDatagramP2PHeartbeat"},
  {RADIO_CALL_xmitDatagramP2PAck, "xmitDatagramP2PAck"},
  {RADIO_CALL_xmitDatagramMpData, "xmitDatagramMpData"},
  {RADIO_CALL_xmitDatagramMpHeartbeat, "xmitDatagramMpHeartbeat"},
  {RADIO_CALL_xmitDatagramMpAck, "xmitDatagramMpAck"},
  {RADIO_CALL_xmitDatagramMpPing, "xmitDatagramMpPing"},
  {RADIO_CALL_xmitDatagramTrainingPing, "xmitDatagramTrainingPing"},
  {RADIO_CALL_xmitDatagramTrainingAck, "xmitDatagramTrainingAck"},
  {RADIO_CALL_xmitDatagramTrainRadioParameters, "xmitDatagramTrainRadioParameters"},
  {RADIO_CALL_xmitVcDatagram, "xmitVcDatagram"},
  {RADIO_CALL_xmitVcHeartbeat, "xmitVcHeartbeat"},
  {RADIO_CALL_xmitVcAckFrame, "xmitVcAckFrame"},
  {RADIO_CALL_xmitVcPing, "xmitVcPing"},
  {RADIO_CALL_xmitVcAck1, "xmitVcAck1"},
  {RADIO_CALL_xmitVcAck2, "xmitVcAck2"},
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
  //Insert new values before this line
  {RADIO_CALL_hopISR, "hopISR"},
  {RADIO_CALL_transactionCompleteISR, "transactionCompleteISR"},
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
      systemPrintTimestamp(radioCallHistory[sortOrder[index]]);
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
