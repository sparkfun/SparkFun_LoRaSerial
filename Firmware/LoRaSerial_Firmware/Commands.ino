//To add a new ATSxx command:
//Add an entry to the "commands" table below

//To add a new commnd prefix such as AT or RT
//Add an entry to the "prefix" table below

//----------------------------------------
//  Data structures
//----------------------------------------

enum {
  TYPE_BOOL = 0,
  TYPE_FLOAT,
  TYPE_KEY,
  TYPE_SPEED_AIR,
  TYPE_SPEED_SERIAL,
  TYPE_U8,
  TYPE_U16,
  TYPE_U32,
};

typedef bool (* VALIDATION_ROUTINE)(void * value, uint32_t valMin, uint32_t valMax);

typedef struct _COMMAND_ENTRY
{
  uint8_t number;
  uint32_t minValue;
  uint32_t maxValue;
  uint8_t digits;
  uint8_t type;
  VALIDATION_ROUTINE validate;
  const char * name;
  void * setting;
} COMMAND_ENTRY;

typedef bool (* COMMAND_ROUTINE)(const char * commandString);
typedef struct
{
  const char * prefix;
  COMMAND_ROUTINE processCommand;
} COMMAND_PREFIX;

//----------------------------------------
//  Command prefix routines
//----------------------------------------

bool commandAT(const char * commandString)
{
  //'AT'
  if (commandLength == 2)
    reportOK();

  //ATI, ATO, ATZ commands
  else if (commandLength == 3)
  {
    switch (commandString[2])
    {
      case ('?'): //Display the command help
        systemPrintln("Command summary:");
        systemPrintln("  AT? - Print the command summary");
        systemPrintln("  ATF - Enter training mode and return to factory defaults");
        systemPrintln("  ATG - Generate new netID and encryption key");
        systemPrintln("  ATI - Display the radio version");
        systemPrintln("  ATI? - Display the information commands");
        systemPrintln("  ATIn - Display system information");
        systemPrintln("  ATO - Exit command mode");
        systemPrintln("  ATSn=xxx - Set parameter n's value to xxx");
        systemPrintln("  ATSn? - Print parameter n's current value");
        systemPrintln("  ATT - Enter training mode");
        systemPrintln("  ATX - Stop the training server");
        systemPrintln("  ATZ - Reboot the radio");
        systemPrintln("  AT&F - Restore factory settings");
        systemPrintln("  AT&W - Save current settings to NVM");
        break;
      case ('F'): //Enter training mode and return to factory defaults
        reportOK();
        selectTraining(true);
        break;
      case ('G'): //Generate a new netID and encryption key
        generateTrainingSettings();
        reportOK();
        break;
      case ('I'):
        //Shows the radio version
        reportOK();
        systemPrint("SparkFun LoRaSerial ");
        systemPrint(platformPrefix);
        systemPrint(" v");
        systemPrint(FIRMWARE_VERSION_MAJOR);
        systemPrint(".");
        systemPrintln(FIRMWARE_VERSION_MINOR);
        break;
      case ('O'): //Exit command mode
        if (printerEndpoint == PRINT_TO_RF)
        {
          //If we are pointed at the RF link, send ok and wait for response ACK before applying settings
          reportOK();
        }
        else
        {
          //Apply settings and return
          generateHopTable(); //Generate freq with new settings
          configureRadio(); //Apply any new settings

          setRSSI(0); //Turn off LEDs
          inCommandMode = false; //Return to printing normal RF serial data
          reportOK();
          changeState(RADIO_RESET);
        }
        break;
      case ('T'): //Enter training mode
        reportOK();
        selectTraining(false);
        break;
      case ('X'): //Stop the training server
        if (trainingServerRunning && (!settings.pointToPoint) && settings.trainingServer)
        {
          endClientServerTraining(TRIGGER_TRAINING_SERVER_STOPPED);
          reportOK();
        }
        else
          reportERROR();
        break;
      case ('Z'): //Reboots the radio
        reportOK();
        systemFlush();
        systemReset();
        break;
      default:
        return false;
    }
  }

  //ATIx commands
  else if (commandString[2] == 'I' && commandLength == 4)
  {
    switch (commandString[3])
    {
      case ('?'): //ATI? - Display the information commands
        systemPrintln("  ATI0 - Show user settable parameters");
        systemPrintln("  ATI1 - Show board variant");
        systemPrintln("  ATI2 - Show firmware version");
        systemPrintln("  ATI3 - Display RSSI value");
        systemPrintln("  ATI4 - Get random byte from RSSI");
        systemPrintln("  ATI5 - Show max possible bytes per second");
        systemPrintln("  ATI6 - Display AES key");
        systemPrintln("  ATI7 - Show current FHSS channel");
        systemPrintln("  ATI8 - Display unique ID");
        systemPrintln("  ATI9 - Display the total datagrams sent");
        systemPrintln("  ATI10 - Display the total datagrams received");
        systemPrintln("  ATI11 - Display the total frames sent");
        systemPrintln("  ATI12 - Display the total frames received");
        systemPrintln("  ATI13 - Display the total bad frames received");
        systemPrintln("  ATI14 - Display the total duplicate frames received");
        systemPrintln("  ATI15 - Display the total lost TX frames");
        systemPrintln("  ATI16 - Display the maximum datagram size");
        systemPrintln("  ATI17 - Display the total link failures");
        break;
      case ('0'): //ATI0 - Show user settable parameters
        displayParameters();
        break;
      case ('1'): //ATI1 - Show board variant
        systemPrint("SparkFun LoRaSerial ");
        systemPrint(platformPrefix);
        systemPrint("\r\n");
        break;
      case ('2'): //ATI2 - Show firmware version
        systemPrint(FIRMWARE_VERSION_MAJOR);
        systemPrint(".");
        systemPrintln(FIRMWARE_VERSION_MINOR);
        break;
      case ('3'): //ATI3 - Display latest RSSI
        systemPrintln(radio.getRSSI());
        break;
      case ('4'): //ATI4 - Get random byte from RSSI
        systemPrintln(radio.randomByte());
        break;
      case ('5'): //ATI5 - Show max possible bytes per second
        systemPrintln(calcMaxThroughput());
        break;
      case ('6'): //ATI6 - Display AES key
        for (uint8_t i = 0 ; i < 16 ; i++)
          systemPrint(settings.encryptionKey[i], HEX);
        systemPrintln();
        break;
      case ('7'): //ATI7 - Show current FHSS channel
        systemPrintln(channelNumber);
        break;
      case ('8'): //ATI8 - Display the unique ID
        systemPrintUniqueID(myUniqueId);
        systemPrintln();
        break;
      case ('9'): //ATI9 - Display the toal datagrams sent
        systemPrint("Total datagrams sent: ");
        systemPrintln(datagramsSent);
        break;
      default:
        return false;
    }
  }
  else if ((commandString[2] == 'I') && (commandString[3] == '1') && (commandLength == 5))
  {
    switch (commandString[4])
    {
      default:
        return false;
      case ('0'): //ATI10 - Display the total datagrams received
        systemPrint("Total datagrams received: ");
        systemPrintln(datagramsReceived);
        break;
      case ('1'): //ATI11 - Display the total frames received
        systemPrint("Total frames sent: ");
        systemPrintln(framesReceived);
        break;
      case ('2'): //ATI12 - Display the total frames received
        systemPrint("Total frames sent: ");
        systemPrintln(framesReceived);
        break;
      case ('3'): //ATI13 - Display the total bad frames received
        systemPrint("Total bad frames received: ");
        systemPrintln(badFrames);
        break;
      case ('4'): //ATI14 - Display the total duplicate frames received
        systemPrint("Total duplicate frames received: ");
        systemPrintln(duplicateFrames);
        break;
      case ('5'): //ATI15 - Display the total lost TX frames
        systemPrint("Total lost TX frames: ");
        systemPrintln(lostFrames);
        break;
      case ('6'): //ATI16 - Display the maximum datagram size
        systemPrint("Maximum datagram size: ");
        systemPrintln(maxDatagramSize);
        break;
      case ('7'): //ATI17 - Display the total link failures
        systemPrint("Total link failures: ");
        systemPrintln(linkFailures);
        break;
    }
  }

  //AT&x commands
  else if (commandString[2] == '&')
  {
    //&W and &F
    if (commandLength == 4)
    {
      switch (commandString[3])
      {
        case ('W'): //AT&W - Write parameters to the flash memory
          {
            recordSystemSettings();
            reportOK();
          }
          break;
        case ('F'): //AT&F - Restore default parameters
          {
            settings = defaultSettings; //Overwrite all current settings with defaults
            recordSystemSettings();
            reportOK();
          }
          break;
        default:
          return false;
      }
    }
  }
  else
    return false;
  return true;
}

//Send the AT command over RF link
bool sendRemoteCommand(const char * commandString)
{
  //We cannot send a command if not linked
  if (isLinked() == false)
    return false;

  //Move this command into the transmit buffer
  for (int x = 0 ; x < commandLength ; x++)
  {
    commandTXBuffer[commandTXHead++] = commandString[x];
    commandTXHead %= sizeof(commandTXBuffer);
  }
  remoteCommandResponse = false;
  return true;
}

//----------------------------------------
//  Command prefix table
//----------------------------------------

const COMMAND_PREFIX prefixTable[] = {
  {"ATS", commandSet},
  {"AT", commandAT},
  {"RT", sendRemoteCommand},
};

const int prefixCount = sizeof(prefixTable) / sizeof(prefixTable[0]);

//----------------------------------------
//  Command processing routine
//----------------------------------------

//Check to see if a valid command has been received
void checkCommand()
{
  char * commandString;
  int index;
  int prefixLength;
  bool success;

  //Verify the command length
  commandString = trimCommand(); //Remove any leading whitespace
  commandString[commandLength] = '\0'; //Terminate buffer
  if (commandLength < 2) //Too short
    reportERROR();

  //Locate the correct processing routine for the command prefix
  success = false;
  for (index = 0; index < prefixCount; index++)
  {
    //Locate the prefix
    prefixLength = strlen(prefixTable[index].prefix);
    if (strncmp(commandString, prefixTable[index].prefix, prefixLength) != 0)
      continue;

    //Process the command
    success = prefixTable[index].processCommand(commandString);
    break;
  }

  //Print the command failure
  if (!success)
    reportERROR();

  commandLength = 0; //Get ready for next command
}

void reportOK()
{
  systemPrintln("OK");
}

void reportERROR()
{
  systemPrintln("ERROR");
}

//Remove any preceeding or following whitespace chars
char * trimCommand()
{
  char * commandString = commandBuffer;

  while (isspace(*commandString))
  {
    commandString++;
    commandLength--;
  }
  return commandString;
}

//----------------------------------------
//  Data validation routines
//----------------------------------------

bool valBandwidth (void * value, uint32_t valMin, uint32_t valMax)
{
  double doubleSettingValue = *(double *)value;

  UNUSED(valMin);
  UNUSED(valMax);

  if ((settings.airSpeed != 0) && (doubleSettingValue != 0))
  {
    systemPrintln("AirSpeed is overriding");
    return false;
  }

  //Some doubles get rounded incorrectly
  return ((settings.airSpeed != 0)
          || ((doubleSettingValue * 100) == 1040)
          || (doubleSettingValue == 15.6)
          || ((doubleSettingValue * 100) == 2080)
          || (doubleSettingValue == 31.25)
          || (doubleSettingValue == 41.7)
          || (doubleSettingValue == 62.5)
          || (doubleSettingValue == 125.0)
          || (doubleSettingValue == 250.0)
          || (doubleSettingValue == 500.0));
}

bool valFreqMax (void * value, uint32_t valMin, uint32_t valMax)
{
  double doubleSettingValue = *(double *)value;

  UNUSED(valMin);

  return ((doubleSettingValue >= settings.frequencyMin) && (doubleSettingValue <= (double)valMax));
}

bool valFreqMin (void * value, uint32_t valMin, uint32_t valMax)
{
  double doubleSettingValue = *(double *)value;

  UNUSED(valMax);

  return ((doubleSettingValue >= (double)valMin) && (doubleSettingValue <= settings.frequencyMax));
}

bool valInt (void * value, uint32_t valMin, uint32_t valMax)
{
  uint32_t settingValue = *(uint32_t *)value;

  return ((settingValue >= valMin) && (settingValue <= valMax));
}

bool valKey (void * value, uint32_t valMin, uint32_t valMax)
{
  unsigned int length;
  char * str = (char *)value;
  char * strEnd;

  UNUSED(valMin);
  UNUSED(valMax);

  //Validate the length of the encryption key
  length = strlen(str);
  if (length != (2 * sizeof(settings.encryptionKey)))
    return false;

  //Validate the characters in the encryption key
  strEnd = &str[length];
  if (strlen(str) == length)
  {
    while (str < strEnd)
    {
      if (charToHex(*str) < 0)
        break;
      str++;
    }
    if (str >= strEnd)
      return true;
  }
  return false;
}

bool valOverride (void * value, uint32_t valMin, uint32_t valMax)
{
  uint32_t settingValue = *(uint32_t *)value;

  if (settings.airSpeed != 0)
  {
    systemPrintln("AirSpeed is overriding");
    return false;
  }

  return ((settingValue >= valMin) && (settingValue <= valMax));
}

bool valSpeedAir (void * value, uint32_t valMin, uint32_t valMax)
{
  bool valid;
  uint32_t settingValue = *(uint32_t *)value;

  UNUSED(valMin);
  UNUSED(valMax);

  valid = ((settingValue == 0)
           || (settingValue == 40)
           || (settingValue == 150)
           || (settingValue == 400)
           || (settingValue == 1200)
           || (settingValue == 2400)
           || (settingValue == 4800)
           || (settingValue == 9600)
           || (settingValue == 19200)
           || (settingValue == 28800)
           || (settingValue == 38400));
  if (valid && (settings.airSpeed == 0) && (settingValue != 0))
    systemPrintln("Warning: AirSpeed override of bandwidth, spread factor, and coding rate");
  return valid;
}

bool valSpeedSerial (void * value, uint32_t valMin, uint32_t valMax)
{
  uint32_t settingValue = *(uint32_t *)value;

  UNUSED(valMin);
  UNUSED(valMax);

  return ((settingValue == 2400)
          || (settingValue == 4800)
          || (settingValue == 9600)
          || (settingValue == 14400)
          || (settingValue == 19200)
          || (settingValue == 38400)
          || (settingValue == 57600)
          || (settingValue == 115200));
}

//----------------------------------------
//  Command table
//----------------------------------------

const COMMAND_ENTRY commands[] =
{ //#, min, max, digits,   type,            validation,        name,                setting addr
  {0,   0,   0,      0, TYPE_SPEED_SERIAL, valSpeedSerial, "SerialSpeed",          &settings.serialSpeed},
  {1,   0,   0,      0, TYPE_SPEED_AIR,    valSpeedAir,    "AirSpeed",             &settings.airSpeed},
  {2,   0, 255,      0, TYPE_U8,           valInt,         "netID",                &settings.netID},
  {3,   0,   1,      0, TYPE_BOOL,         valInt,         "PointToPoint",         &settings.pointToPoint},
  {4,   0,   1,      0, TYPE_BOOL,         valInt,         "EncryptData",          &settings.encryptData},

  {5,   0,   0,      0, TYPE_KEY,          valKey,         "EncryptionKey",        &settings.encryptionKey},
  {6,   0,   1,      0, TYPE_BOOL,         valInt,         "DataScrambling",       &settings.dataScrambling},
  {7,  14,  30,      0, TYPE_U8,           valInt,         "TxPower",              &settings.radioBroadcastPower_dbm},
  {8, 902,   0,      3, TYPE_FLOAT,        valFreqMin,     "FrequencyMin",         &settings.frequencyMin},
  {9,   0, 928,      3, TYPE_FLOAT,        valFreqMax,     "FrequencyMax",         &settings.frequencyMax},

  {10,   1, 255,     0, TYPE_U8,           valInt,         "NumberOfChannels",     &settings.numberOfChannels},
  {11,   0,   1,     0, TYPE_BOOL,         valInt,         "FrequencyHop",         &settings.frequencyHop},
  {12,  10, 65535,   0, TYPE_U16,          valInt,         "MaxDwellTime",         &settings.maxDwellTime},
  {13,   0,   0,     2, TYPE_FLOAT,        valBandwidth,   "Bandwidth",            &settings.radioBandwidth},
  {14,   6,  12,     0, TYPE_U8,           valOverride,    "SpreadFactor",         &settings.radioSpreadFactor},

  {15,   5,   8,     0, TYPE_U8,           valOverride,    "CodingRate",           &settings.radioCodingRate},
  {16,   0, 255,     0, TYPE_U8,           valInt,         "SyncWord",             &settings.radioSyncWord},
  {17,   6, 65535,   0, TYPE_U16,          valInt,         "PreambleLength",       &settings.radioPreambleLength},
  {19,  10, 2000,    0, TYPE_U16,          valInt,         "FrameTimeout",         &settings.serialTimeoutBeforeSendingFrame_ms},

  {20,    0,   1,    0, TYPE_BOOL,         valInt,         "Debug",                &settings.debug},
  {21,    0,   1,    0, TYPE_BOOL,         valInt,         "Echo",                 &settings.echo},
  {22,  250, 65535,  0, TYPE_U16,          valInt,         "HeartBeatTimeout",     &settings.heartbeatTimeout},
  {23,    0,   1,    0, TYPE_BOOL,         valInt,         "FlowControl",          &settings.flowControl},
  {24,    0,   1,    0, TYPE_BOOL,         valInt,         "AutoTune",             &settings.autoTuneFrequency},

  {25,    0,   1,    0, TYPE_BOOL,         valInt,         "DisplayPacketQuality", &settings.displayPacketQuality},
  {26,    0, 255,    0, TYPE_U8,           valInt,         "MaxResends",           &settings.maxResends},
  {27,    0,   1,    0, TYPE_BOOL,         valInt,         "SortParametersByName", &settings.sortParametersByName},
  {28,    0,   1,    0, TYPE_BOOL,         valInt,         "PrintParameterName",   &settings.printParameterName},
  {29,    0,   1,    0, TYPE_BOOL,         valInt,         "PrintFrequency",       &settings.printFrequency},

  {30,    0,   1,    0, TYPE_BOOL,         valInt,         "DebugRadio",           &settings.debugRadio},
  {31,    0,   1,    0, TYPE_BOOL,         valInt,         "DebugStates",          &settings.debugStates},
  {32,    0,   1,    0, TYPE_BOOL,         valInt,         "DebugTraining",        &settings.debugTraining},
  {33,    0,   1,    0, TYPE_BOOL,         valInt,         "DebugTrigger",         &settings.debugTrigger},
  {34,    0,   1,    0, TYPE_BOOL,         valInt,         "UsbSerialWait",        &settings.usbSerialWait},

  {35,    0,   1,    0, TYPE_BOOL,         valInt,         "PrintRfData",          &settings.printRfData},
  {36,    0,   1,    0, TYPE_BOOL,         valInt,         "PrintPktData",         &settings.printPktData},
  {37,    0,   1,    0, TYPE_BOOL,         valInt,         "VerifyRxNetID",        &settings.verifyRxNetID},
  {38,    1, 255,    0, TYPE_U8,           valInt,         "TriggerWidth",         &settings.triggerWidth},
  {39,    0,   1,    0, TYPE_BOOL,         valInt,         "TriggerWidthIsMultiplier", &settings.triggerWidthIsMultiplier},

  {40,    0, 0xffffffff, 0, TYPE_U32,      valInt,         "TriggerEnable: 31-0",  &settings.triggerEnable},
  {41,    0, 0xffffffff, 0, TYPE_U32,      valInt,         "TriggerEnable2: 63-32", &settings.triggerEnable2},
  {42,    0,   1,    0, TYPE_BOOL,         valInt,         "DebugReceive",         &settings.debugReceive},
  {43,    0,   1,    0, TYPE_BOOL,         valInt,         "DebugTransmit",        &settings.debugTransmit},
  {44,    0,   1,    0, TYPE_BOOL,         valInt,         "PrintTxErrors",        &settings.printTxErrors},

  {45,    2,   2,    0, TYPE_U8,           valInt,         "protocolVersion",      &settings.protocolVersion},
  {46,    0,   1,    0, TYPE_BOOL,         valInt,         "PrintTimestamp",       &settings.printTimestamp},
  {47,    0,   1,    0, TYPE_BOOL,         valInt,         "DebugDatagrams",       &settings.debugDatagrams},
  {48,    0, 1000,   0, TYPE_U16,          valInt,         "OverHeadtime",         &settings.overheadTime},
  {49,    0,   1,    0, TYPE_BOOL,         valInt,         "EnableCRC16",          &settings.enableCRC16},

  {50,    0,   1,    0, TYPE_BOOL,         valInt,         "DisplayRealMillis",    &settings.displayRealMillis},
  {51,    0,   1,    0, TYPE_BOOL,         valInt,         "TrainingServer",       &settings.trainingServer},
  {52,    1, 255,    0, TYPE_U8,           valInt,         "ClientRetryInterval",  &settings.clientPingRetryInterval},
  {53,    0,   1,    0, TYPE_BOOL,         valInt,         "CopyDebug",            &settings.copyDebug},
  {54,    0,   1,    0, TYPE_BOOL,         valInt,         "CopySerial",           &settings.copySerial},

  {55,    0,   1,    0, TYPE_BOOL,         valInt,         "CopyTriggers",         &settings.copyTriggers},
  {56,    0,   0,    0, TYPE_KEY,          valKey,         "TrainingKey",          &settings.trainingKey},
  {57,    0,   1,    0, TYPE_BOOL,         valInt,         "PrintLinkUpDown",      &settings.printLinkUpDown},

  //Define any user parameters starting at 255 decrementing towards 0
};

const int commandCount = sizeof(commands) / sizeof(commands[0]);

//----------------------------------------
//  ATSxx routines
//----------------------------------------

const char * commandGetNumber(const char * buffer, uint32_t * value)
{
  int number;

  //Assume an invalid number
  number = -1;
  if ((*buffer >= '0') && (*buffer <= '9'))
  {
    //Get the number
    number = 0;
    while ((*buffer >= '0') && (*buffer <= '9'))
      number = (number * 10) + *buffer++ - '0';
  }

  //Return the command number and the pointer to the next character
  *value = number;
  return buffer;
}

void commandDisplay(uint8_t number, bool printName)
{
  const COMMAND_ENTRY * command;
  const COMMAND_ENTRY * commandEnd;

  //Locate the command
  command = &commands[0];
  commandEnd = &commands[commandCount];
  while (command < commandEnd)
    if (command->number == number)
      break;
    else
      command++;

  //Verify the command number
  if (command >= commandEnd)
    return;

  //Print the setting name
  if (printName)
  {
    systemPrint(command->name);
    systemPrint("=");
  }

  //Print the setting value
  switch (command->type)
  {
    case TYPE_BOOL:
      systemPrint((uint8_t)(*(bool *)(command->setting)));
      break;
    case TYPE_FLOAT:
      systemPrint(*((float *)(command->setting)), command->digits);
      break;
    case TYPE_KEY:
      displayEncryptionKey((uint8_t *)(command->setting));
      break;
    case TYPE_U8:
      systemPrint(*(uint8_t *)(command->setting));
      break;
    case TYPE_U16:
      systemPrint(*(uint16_t *)(command->setting));
      break;
    case TYPE_SPEED_AIR:
    case TYPE_SPEED_SERIAL:
    case TYPE_U32:
      systemPrint(*(uint32_t *)(command->setting));
      break;
  }
  systemPrintln();
}

//Set or display the command
bool commandSet(const char * commandString)
{
  const char * buffer;
  const COMMAND_ENTRY * command;
  double doubleSettingValue;
  int index;
  uint32_t number;
  uint32_t settingValue;
  bool valid;

  do {
    //Validate the command number
    buffer = commandGetNumber(&commandString[3], &number);
    for (index = 0; index < commandCount; index++)
      if (number == commands[index].number)
        break;
    if (index >= commandCount)
      break;
    command = &commands[index];

    //Is this a display request
    if (strcmp(buffer, "?") == 0)
    {
      commandDisplay(command->number, settings.printParameterName);
      return true;
    }

    //Make sure the command has the proper syntax
    if (*buffer++ != '=')
      break;

    //Get the value
    doubleSettingValue = strtod(buffer, NULL);
    settingValue = doubleSettingValue;

    //Validate and set the value
    valid = false;
    switch (command->type)
    {
      case TYPE_BOOL:
        valid = command->validate((void *)&settingValue, command->minValue, command->maxValue);
        if (valid)
          *(bool *)(command->setting) = (bool)settingValue;
        break;
      case TYPE_FLOAT:
        valid = command->validate((void *)&doubleSettingValue, command->minValue, command->maxValue);
        if (valid)
          *((float *)(command->setting)) = doubleSettingValue;
        break;
      case TYPE_KEY:
        valid = command->validate((void *)buffer, command->minValue, command->maxValue);
        if (valid)
          for (uint32_t x = 0; x < (2 * AES_KEY_BYTES); x += 2)
            ((uint8_t *)command->setting)[x / 2] = charHexToDec(buffer[x], buffer[x + 1]);
        break;
      case TYPE_SPEED_AIR:
      case TYPE_SPEED_SERIAL:
      case TYPE_U32:
        valid = command->validate((void *)&settingValue, command->minValue, command->maxValue);
        if (valid)
          *(uint32_t *)(command->setting) = settingValue;
        break;
      case TYPE_U8:
        valid = command->validate((void *)&settingValue, command->minValue, command->maxValue);
        if (valid)
          *(uint8_t *)(command->setting) = (uint8_t)settingValue;
        break;
      case TYPE_U16:
        valid = command->validate((void *)&settingValue, command->minValue, command->maxValue);
        if (valid)
          *(uint16_t *)(command->setting) = (uint16_t)settingValue;
        break;
    }
    if (valid == false)
      break;

    //Display the parameter if requested
    if (settings.printParameterName)
      commandDisplay(command->number, true);

    //The parameter was successfully set
    reportOK();
    return true;
  } while (0);

  //Report the error
  return false;
}

//Display the encryption key
void displayEncryptionKey(uint8_t * key)
{
  for (uint8_t index = 0 ; index < sizeof(settings.encryptionKey) ; index++)
    systemPrint(key[index], HEX);
}

//Show current settings in user friendly way
void displayParameters()
{
  int index;
  uint8_t sortOrder[commandCount];
  uint8_t temp;
  int x;

  //Set the default sort order
  for (index = 0; index < commandCount; index++)
    sortOrder[index] = index;

  //Perform a bubble sort if requested
  if (settings.sortParametersByName)
    for (index = 0; index < commandCount; index++)
      for (x = index + 1; x < commandCount; x++)
        if (stricmp(commands[sortOrder[index]].name, commands[sortOrder[x]].name) > 0)
        {
          temp = sortOrder[index];
          sortOrder[index] = sortOrder[x];
          sortOrder[x] = temp;
        }

  //Print the parameters
  for (index = 0; index < commandCount; index++)
  {
    petWDT(); //Printing may take longer than WDT at 9600, so pet the WDT.

    if (printerEndpoint == PRINT_TO_RF)
      systemPrint("R"); //If someone is asking for our settings over RF, respond with 'R' style settings
    else
      systemPrint("A");

    systemPrint("TS");
    systemPrint(commands[sortOrder[index]].number);
    systemPrint(":");
    commandDisplay(commands[sortOrder[index]].number, true);
  }
}
