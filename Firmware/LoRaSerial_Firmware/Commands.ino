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

typedef bool (* COMMAND_ROUTINE)(const char * commandString);
typedef struct
{
  const char * prefix;
  COMMAND_ROUTINE processCommand;
} COMMAND_PREFIX;

//----------------------------------------
//  Command prefix routines
//----------------------------------------

//Process the AT commands
bool commandAT(const char * commandString)
{
  uint32_t delayMillis;
  const char * string;
  unsigned long timer;
  VIRTUAL_CIRCUIT * vc = &virtualCircuitList[cmdVc];

  //'AT'
  if (commandLength == 2)
    return true;

  //AT?, ATA, ATB, ATC, ATG, ATI, ATO, ATZ commands
  if (commandLength == 3)
  {
    switch (commandString[2])
    {
      default:
        return false;

      case ('?'): //Display the command help
        systemPrintln("Command summary:");
        systemPrintln("  AT? - Print the command summary");
        systemPrintln("  ATA - Get the current VC status");
        systemPrintln("  ATB - Break the link");
        systemPrintln("  ATC - Establish VC connection for data");
        systemPrintln("  ATD - Display the debug settings");
        systemPrintln("  ATF - Restore factory settings");
        systemPrintln("  ATG - Generate new netID and encryption key");
        systemPrintln("  ATI - Display the radio version");
        systemPrintln("  ATI? - Display the information commands");
        systemPrintln("  ATIn - Display system information");
        systemPrintln("  ATO - Exit command mode");
        systemPrintln("  ATP - Display probe trigger settings");
        systemPrintln("  ATR - Display radio settings");
        systemPrintln("  ATS - Display the serial settings");
        systemPrintln("  ATT - Enter training mode");
        systemPrintln("  ATV - Display virtual circuit settings");
        systemPrintln("  ATW - Save current settings to NVM");
        systemPrintln("  ATZ - Reboot the radio");
        systemPrintln("  AT-Param=xxx - Set parameter's value to xxx by name (Param)");
        systemPrintln("  AT-Param? - Print parameter's current value by name (Param)");
        systemPrintln("  AT-? - Display the setting values");
        return true;

      case ('A'): //ATA - Get the current VC status
        for (int index = 0; index < MAX_VC; index++)
          vcSendPcStateMessage(index, virtualCircuitList[index].vcState);
        return true;

      case ('B'): //ATB - Break the link

        //Compute the time delay
        delayMillis = (VC_LINK_BREAK_MULTIPLIER + 2) * settings.heartbeatTimeout;

        //Warn the user of the delay
        systemPrint("Delaying ");
        systemPrint(delayMillis / 1000);
        systemPrintln(" seconds to break the link");
        outputSerialData(true);

        //Flag the links as broken
        if (settings.operatingMode == MODE_VIRTUAL_CIRCUIT)
        {
          for (int i = 0; i < MAX_VC; i++)
            if ((virtualCircuitList[i].vcState != VC_STATE_LINK_DOWN) && (i != myVc))
              vcBreakLink(i);
        }
        else if (settings.operatingMode == MODE_POINT_TO_POINT)
        {
          breakLink();
          outputSerialData(true);
        }

        //Idle the system to break the link
        //This is required on the server system which does not request an VC number assignment
        timer = millis();
        while ((millis() - timer) < ((VC_LINK_BREAK_MULTIPLIER + 2) * settings.heartbeatTimeout))
          petWDT();

        //Display the end of the delay
        systemPrintln("Delay done");
        return true;

      case ('C'): //ATC - Establish VC connection for data
        if ((settings.operatingMode != MODE_VIRTUAL_CIRCUIT)
            || (vc->vcState != VC_STATE_LINK_ALIVE))
          return false;
        if (cmdVc == myVc)
          vcChangeState(cmdVc, VC_STATE_CONNECTED);
        else
          vcChangeState(cmdVc, VC_STATE_SEND_PING);
        return true;

      case ('F'): //ATF - Restore default parameters
        settings = defaultSettings; //Overwrite all current settings with defaults
        validateSettings(); //Modify defaults for each radio type (915, 868, 433, etc)
        recordSystemSettings();
        return true;

      case ('G'): //ATG - Generate a new netID and encryption key
        generateRandomKeysID();
        return true;

      case ('I'): //ATI
        //Shows the radio version
        systemPrint("SparkFun LoRaSerial ");
        systemPrint(platformPrefix);
        systemPrint(" v");
        systemPrint(FIRMWARE_VERSION_MAJOR);
        systemPrint(".");
        systemPrintln(FIRMWARE_VERSION_MINOR);
        return true;

      case ('O'): //ATO - Exit command mode
        if (printerEndpoint == PRINT_TO_RF)
          //If we are pointed at the RF link, send ok and wait for response ACK before applying settings
          return true;

        //Apply raio settings by entering the reset state
        inCommandMode = false; //Return to printing normal RF serial data
        changeState(RADIO_RESET);
        return true;

      case ('T'): //ATT - Enter training mode
        selectTraining();
        return true;

      case ('W'): //ATW - Write parameters to the flash memory
        recordSystemSettings();
        return true;

      case ('Z'): //ATZ - Reboots the radio
        reportOK();
        outputSerialData(true);
        systemFlush();
        systemReset();
        return true;
    }
  }

  //ATIx commands
  if (commandString[2] == 'I' && commandLength == 4)
  {
    switch (commandString[3])
    {
      default:
        return false;

      case ('?'): //ATI? - Display the information commands
        systemPrintln("  ATI0 - Show user settable parameters");
        systemPrintln("  ATI1 - Show board variant");
        systemPrintln("  ATI2 - Show firmware version");
        systemPrintln("  ATI3 - Display RSSI value");
        systemPrintln("  ATI4 - Get random byte from RSSI");
        systemPrintln("  ATI5 - Show max possible bytes per second");
        systemPrintln("  ATI6 - Display AES key");
        systemPrintln("  ATI7 - Show current FHSS channel");
        systemPrintln("  ATI8 - Display system unique ID");
        systemPrintln("  ATI9 - Display the maximum datagram size");
        systemPrintln("  ATI10 - Display radio metrics");
        systemPrintln("  ATI11 - Return myVc value");
        systemPrintln("  ATI12 - Display the VC details");
        systemPrintln("  ATI13 - Display the SX1276 registers");
        systemPrintln("  ATI14 - Dump the radioTxBuffer");
        return true;

      case ('0'): //ATI0 - Show user settable parameters
        displayParameters(0, true);
        return true;

      case ('1'): //ATI1 - Show board variant
        systemPrint("SparkFun LoRaSerial ");
        systemPrint(platformPrefix);
        systemPrint("\r\n");
        return true;

      case ('2'): //ATI2 - Show firmware version
        systemPrint(FIRMWARE_VERSION_MAJOR);
        systemPrint(".");
        systemPrintln(FIRMWARE_VERSION_MINOR);
        return true;

      case ('3'): //ATI3 - Display latest RSSI
        systemPrintln(radio.getRSSI());
        return true;

      case ('4'): //ATI4 - Get random byte from RSSI
        systemPrintln(radio.randomByte());
        return true;

      case ('5'): //ATI5 - Show max possible bytes per second
        systemPrintln(calcMaxThroughput());
        return true;

      case ('6'): //ATI6 - Display currentState
        displayState(radioState);
        return true;

      case ('7'): //ATI7 - Show current FHSS channel
        systemPrintln(channelNumber);
        return true;

      case ('8'): //ATI8 - Display the system's unique ID
        systemPrintUniqueID(myUniqueId);
        systemPrintln();
        return true;

      case ('9'): //ATI9 - Display the maximum datagram size
        systemPrint("Maximum datagram size: ");
        systemPrintln(maxDatagramSize);
        return true;
    }
  }
  if ((commandString[2] == 'I') && (commandString[3] == '1') && (commandLength == 5))
  {
    switch (commandString[4])
    {
      default:
        return false;

      case ('0'): //ATI10 - Display radio metrics
        systemPrint("Radio Metrics @ ");
        systemPrintTimestamp(millis());
        systemPrintln();
        if (settings.operatingMode == MODE_POINT_TO_POINT)
        {
          systemPrint("    Link Status: ");
          systemPrintln((radioState >= RADIO_P2P_LINK_UP) ? "Up" : "Down");
          systemPrint("        Last RX:  ");
          systemPrintTimestamp(lastRxDatagram);
          systemPrintln();
          systemPrint("        First RX: ");
          systemPrintTimestamp(lastLinkUpTime);
          systemPrintln();
          systemPrint("        Up Time:  ");
          systemPrintTimestamp(lastRxDatagram - lastLinkUpTime);
          systemPrintln();
        }
        systemPrintln("    Sent");
        systemPrint("        Datagrams: ");
        systemPrintln(datagramsSent);
        systemPrint("        Frames: ");
        systemPrintln(framesSent);
        systemPrint("        Lost Frames: ");
        systemPrintln(lostFrames);
        systemPrintln("    Received");
        systemPrint("        Frames: ");
        systemPrintln(framesReceived);
        systemPrint("        Datagrams: ");
        systemPrintln(datagramsReceived);
        systemPrint("        Bad CRC: ");
        systemPrintln(badCrc);
        systemPrint("        Bad Frames: ");
        systemPrintln(badFrames);
        systemPrint("        Duplicate Frames: ");
        systemPrintln(duplicateFrames);
        systemPrint("        Insufficient Space: ");
        systemPrintln(insufficientSpace);
        systemPrint("        Net ID Mismatch: ");
        systemPrintln(netIdMismatch);
        if (settings.operatingMode == MODE_POINT_TO_POINT)
        {
          vc = &virtualCircuitList[0];
          systemPrintln("    ACK Management");
          systemPrint("        Last RX ACK number: ");
          systemPrintln(vc->rxAckNumber);
          systemPrint("        Next RX ACK number: ");
          systemPrintln(vc->rmtTxAckNumber);
          systemPrint("        Last TX ACK number: ");
          systemPrintln(vc->txAckNumber);
          systemPrint("        transmitTimer: ");
          if (transmitTimer)
            systemPrintTimestamp(transmitTimer);
          else
            systemPrint("Not Running");
          systemPrintln();
        }
        else if (settings.operatingMode == MODE_VIRTUAL_CIRCUIT)
        {
          systemPrintln("    ACK Management");
          systemPrint("        vcAackTimer: ");
          if (vcAckTimer)
          {
            systemPrint("VC ");
            systemPrint(txDestVc);
            systemPrint(", ");
            systemPrintTimestamp(vcAckTimer);
            systemPrintln();
          }
          else
            systemPrintln("Not Running");
          systemPrint("    vcConnecting: ");
          systemPrint((int)vcConnecting, HEX);
          if (vcConnecting)
          {
            systemPrintln();
            for (int index = 0; index < MAX_VC; index++)
            {
              if (vcConnecting & (1 << index))
              {
                systemPrint("        VC ");
                if (index < 10)
                  systemWrite(' ');
                systemPrint(index);
                systemPrint(" State: ");
                systemPrintln(vcStateNames[virtualCircuitList[index].vcState]);
              }
            }
          }
          else
            systemPrintln("None");
        }
        systemPrintln("    Circular Buffers");
        systemPrint("        serialReceiveBuffer: rxHead: ");
        systemPrint(rxHead);
        systemPrint(", rxTail: ");
        systemPrint(rxTail);
        systemPrint(", ");
        systemPrint(availableRXBytes());
        systemPrintln(" bytes");
        systemPrint("        serialTransmitBuffer: txHead: ");
        systemPrint(txHead);
        systemPrint(", txTail: ");
        systemPrint(txTail);
        systemPrint(", ");
        systemPrint(availableTXBytes());
        systemPrintln(" bytes");
        systemPrint("        radioTxBuffer: radioTxHead: ");
        systemPrint(radioTxHead);
        systemPrint(", radioTxTail: ");
        systemPrint(radioTxTail);
        systemPrint(", ");
        systemPrint(availableRadioTXBytes());
        systemPrintln(" bytes");
        systemPrint("        commandRXBuffer: commandRXHead: ");
        systemPrint(commandRXHead);
        systemPrint(", commandRXTail: ");
        systemPrint(commandRXTail);
        systemPrint(", ");
        systemPrint(availableRXCommandBytes());
        systemPrintln(" bytes");
        systemPrint("        commandTXBuffer: commandTXHead: ");
        systemPrint(commandTXHead);
        systemPrint(", commandTXTail: ");
        systemPrint(commandTXTail);
        systemPrint(", ");
        systemPrint(availableTXCommandBytes());
        systemPrintln(" bytes");
        systemPrintln("    Radio");
        systemPrint("        Channel: ");
        systemPrintln(channelNumber);
        systemPrint("        Frequency: ");
        systemPrint(radioFrequency, 3);
        systemPrint(" MHz");
        if (radioFrequency != channels[channelNumber])
        {
          systemPrint(" = ");
          systemPrint(channels[channelNumber], 3);
          systemPrint(" MHz + ");
          systemPrint((radioFrequency - channels[channelNumber]) * 1000, 0);
          systemPrint(" KHz");
        }
        systemPrintln();
        systemPrint("        maxDwellTime: ");
        systemPrint(settings.maxDwellTime);
        systemPrintln(" mSec");
        if (channelNumber)
        {
          systemPrint("        Next Ch 0 time: ");
          systemPrintTimestamp(nextChannelZeroTimeInMillis);
          int hopCount = settings.numberOfChannels - channelNumber;
          systemPrint(", in ");
          systemPrint(hopCount);
          systemPrintln(" hops");
        }
        systemPrint("        Last Successful Transmit: ");
        if (txSuccessMillis)
        {
          systemPrintTimestamp(txSuccessMillis);
          systemPrintln();
        }
        else
          systemPrintln("None");
        systemPrint("        Last Transmit Failure: ");
        if (txFailureMillis)
        {
          systemPrintTimestamp(txFailureMillis);
          systemPrint(", Status: ");
          systemPrint(txFailureState);
          string = getRadioStatusCode(txFailureState);
          if (string)
          {
            systemPrint(", ");
            systemPrint(string);
          }
          systemPrintln();
        }
        else
          systemPrintln("None");
        systemPrint("        Last Successful Receive: ");
        if (rxSuccessMillis)
        {
          systemPrintTimestamp(rxSuccessMillis);
          systemPrintln();
        }
        else
          systemPrintln("None");
        systemPrint("        Last Receive Failure: ");
        if (rxFailureMillis)
        {
          systemPrintTimestamp(rxFailureMillis);
          systemPrint(", Status: ");
          systemPrint(rxFailureState);
          string = getRadioStatusCode(rxFailureState);
          if (string)
          {
            systemPrint(", ");
            systemPrint(string);
          }
          systemPrintln();
        }
        else
          systemPrintln("None");
        systemPrint("        transactionComplete: ");
        systemPrintln(transactionComplete ? "True" : "False");
        systemPrint("        receiveInProcess: ");
        systemPrintln(receiveInProcess() ? "True" : "False");
        systemPrintln("    Call History");
        displayRadioCallHistory();
        systemPrintln("    State History");
        displayRadioStateHistory();
        return true;

      case ('1'): //ATI11 - Return myVc value
        systemPrintln();
        systemPrint("myVc: ");
        systemPrintln(myVc);
        return true;

      case ('2'): //ATI12 - Display the VC details
        systemPrint("VC ");
        systemPrint(cmdVc);
        systemPrint(": ");
        if (!vc->valid)
        {
          systemPrint("Down, Not valid @ ");
          systemPrintTimestamp(millis());
          systemPrintln();
        }
        else
        {
          systemPrint(vcStateNames[vc->vcState]);
          systemPrint(" @ ");
          systemPrintTimestamp(millis());
          systemPrintln();
          systemPrint("    ID: ");
          systemPrintUniqueID(vc->uniqueId);
          systemPrintln(vc->valid ? " (Valid)" : " (Invalid)");
          systemPrintln("    Heartbeats");
          if (cmdVc == myVc)
          {
            systemPrint("        Next TX: ");
            systemPrintTimestamp(heartbeatTimer + heartbeatRandomTime);
            systemPrintln();
          }
          systemPrint("        Last:    ");
          systemPrintTimestamp(vc->lastTrafficMillis);
          systemPrintln();
          systemPrint("        First:   ");
          systemPrintTimestamp(vc->firstHeartbeatMillis);
          systemPrintln();
          systemPrint("        Up Time: ");
          systemPrintTimestamp(vc->lastTrafficMillis - vc->firstHeartbeatMillis);
          systemPrintln();
          systemPrintln("    Sent");
          systemPrint("        Frames: ");
          systemPrintln(vc->framesSent);
          systemPrint("        Messages: ");
          systemPrintln(vc->messagesSent);
          systemPrintln("    Received");
          systemPrint("        Frames: ");
          systemPrintln(vc->framesReceived);
          systemPrint("        Messages: ");
          systemPrintln(vc->messagesReceived);
          systemPrint("        Bad Lengths: ");
          systemPrintln(vc->badLength);
          systemPrint("        Link Failures: ");
          systemPrintln(linkFailures);
          systemPrintln("    ACK Management");
          systemPrint("        Last RX ACK number: ");
          systemPrintln(vc->rxAckNumber);
          systemPrint("        Next RX ACK number: ");
          systemPrintln(vc->rmtTxAckNumber);
          systemPrint("        Last TX ACK number: ");
          systemPrintln(vc->txAckNumber);
          if (txDestVc == cmdVc)
          {
            systemPrint("        vcAackTimer: ");
            if (vcAckTimer)
              systemPrintTimestamp(vcAckTimer);
            else
              systemPrint("Not Running");
            systemPrintln();
          }
        }
        return true;

      case ('3'): //ATI13 - Display the SX1276 registers
        printSX1276Registers();
        return true;

      case ('4'): //ATI14 - Dump the radioTxBuffer
        systemPrintln("radioTxBuffer:");
        dumpCircularBuffer(radioTxBuffer, radioTxTail, sizeof(radioTxBuffer), availableRadioTXBytes());
        return true;
    }
  }

  //Invalid command
  return false;
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
  {"ATD", commandDisplayDebug},
  {"ATP", commandDisplayProbe},
  {"ATR", commandDisplayRadio},
  {"ATS", commandDisplaySerial},
  {"ATV", commandDisplayVirtualCircuit},
  {"AT-?", commandDisplayAll},
  {"AT-", commandSetByName},
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
  success = false;
  commandString = trimCommand(); //Remove any leading whitespace

  //Remove trailing CR and LF
  while ((commandLength > 0) && ((commandString[commandLength - 1] == '\r')
                                 || (commandString[commandLength - 1] == '\n')))
    commandLength -= 1;

  //Upper case the command
  for (index = 0; index < commandLength; index++)
    commandBuffer[index] = toupper(commandBuffer[index]);

  commandString[commandLength] = '\0'; //Terminate buffer

  //Verify the command length
  if (commandLength >= 2)
  {
    //Locate the correct processing routine for the command prefix
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
  }

  //Print the command failure
  petWDT();
  if (success)
    reportOK();
  else
    systemPrintln("ERROR");
  outputSerialData(true);
  petWDT();

  commandLength = 0; //Get ready for next command
}

//Indicate successful command completion
void reportOK()
{
  systemPrintln("OK");
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

//Display all of the commands
bool commandDisplayAll(const char * commandString)
{
  displayParameters(0, false);
  return true;
}

//Display only the debugging commands
bool commandDisplayDebug(const char * commandString)
{
  displayParameters('D', false);
  return true;
}

//Display only the trigger probe commands
bool commandDisplayProbe(const char * commandString)
{
  displayParameters('P', false);
  return true;
}

//Display only the radio commands
bool commandDisplayRadio(const char * commandString)
{
  displayParameters('R', false);
  return true;
}

//Display only the serial commands
bool commandDisplaySerial(const char * commandString)
{
  displayParameters('S', false);
  return true;
}

//Display only the virtual circuit commands
bool commandDisplayVirtualCircuit(const char * commandString)
{
  displayParameters('V', false);
  return true;
}

//----------------------------------------
//  Data validation routines
//----------------------------------------

//Validate a bandwidth value
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

//Validate a maximum frequency value
bool valFreqMax (void * value, uint32_t valMin, uint32_t valMax)
{
  double doubleSettingValue = *(double *)value;

  UNUSED(valMin);

  return ((doubleSettingValue >= settings.frequencyMin) && (doubleSettingValue <= (double)valMax));
}

//Validate a minimum frequency value
bool valFreqMin (void * value, uint32_t valMin, uint32_t valMax)
{
  double doubleSettingValue = *(double *)value;

  UNUSED(valMax);

  return ((doubleSettingValue >= (double)valMin) && (doubleSettingValue <= settings.frequencyMax));
}

//Validate an integer value
bool valInt (void * value, uint32_t valMin, uint32_t valMax)
{
  uint32_t settingValue = *(uint32_t *)value;

  return ((settingValue >= valMin) && (settingValue <= valMax));
}

//Validate an encryption key value
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

//Determine if the AirSpeed value is overriding the parameter value
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

//Validate the AirSpeed value
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

//Validate the SerialSpeed value
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
{
  /*Debug parameters
    Ltr, All, min, max, digits,    type,         validation,     name,                   setting addr */
  {'D',   1,   0,   1,    0, TYPE_BOOL,         valInt,         "CopyDebug",            &settings.copyDebug},
  {'D',   1,   0,   1,    0, TYPE_BOOL,         valInt,         "Debug",                &settings.debug},
  {'D',   1,   0,   1,    0, TYPE_BOOL,         valInt,         "DebugDatagrams",       &settings.debugDatagrams},
  {'D',   1,   0,   1,    0, TYPE_BOOL,         valInt,         "DebugHeartbeat",       &settings.debugHeartbeat},
  {'D',   1,   0,   1,    0, TYPE_BOOL,         valInt,         "DebugNvm",             &settings.debugNvm},
  {'D',   1,   0,   1,    0, TYPE_BOOL,         valInt,         "DebugRadio",           &settings.debugRadio},
  {'D',   1,   0,   1,    0, TYPE_BOOL,         valInt,         "DebugReceive",         &settings.debugReceive},
  {'D',   1,   0,   1,    0, TYPE_BOOL,         valInt,         "DebugStates",          &settings.debugStates},
  {'D',   1,   0,   1,    0, TYPE_BOOL,         valInt,         "DebugSync",            &settings.debugSync},
  {'D',   1,   0,   1,    0, TYPE_BOOL,         valInt,         "DebugTraining",        &settings.debugTraining},
  {'D',   1,   0,   1,    0, TYPE_BOOL,         valInt,         "DebugTransmit",        &settings.debugTransmit},
  {'D',   1,   0,   1,    0, TYPE_BOOL,         valInt,         "DebugSerial",          &settings.debugSerial},
  {'D',   1,   0,   1,    0, TYPE_BOOL,         valInt,         "DisplayRealMillis",    &settings.displayRealMillis},
  {'D',   1,   0,   1,    0, TYPE_BOOL,         valInt,         "PrintAckNumbers",      &settings.printAckNumbers},
  {'D',   1,   0,   1,    0, TYPE_BOOL,         valInt,         "PrintFrequency",       &settings.printFrequency},
  {'D',   1,   0,   1,    0, TYPE_BOOL,         valInt,         "PrintLinkUpDown",      &settings.printLinkUpDown},
  {'D',   1,   0,   1,    0, TYPE_BOOL,         valInt,         "PrintPacketQuality",   &settings.printPacketQuality},
  {'D',   1,   0,   1,    0, TYPE_BOOL,         valInt,         "PrintPktData",         &settings.printPktData},
  {'D',   1,   0,   1,    0, TYPE_BOOL,         valInt,         "PrintRfData",          &settings.printRfData},
  {'D',   1,   0,   1,    0, TYPE_BOOL,         valInt,         "PrintTimestamp",       &settings.printTimestamp},
  {'D',   1,   0,   1,    0, TYPE_BOOL,         valInt,         "PrintTxErrors",        &settings.printTxErrors},
  {'D',   1,   0, 255,    0, TYPE_U8,           valInt,         "SelectLedUse",         &settings.selectLedUse},

  /*Radio parameters
    Ltr, All, min, max, digits,    type,         validation,     name,                   setting addr */
  {'R',   0,   0,   0,    0, TYPE_SPEED_AIR,    valSpeedAir,    "AirSpeed",             &settings.airSpeed},
  {'R',   0,   0,   1,    0, TYPE_BOOL,         valInt,         "AutoTune",             &settings.autoTuneFrequency},
  {'R',   0,   0,   0,    2, TYPE_FLOAT,        valBandwidth,   "Bandwidth",            &settings.radioBandwidth},
  {'R',   0,   5,   8,    0, TYPE_U8,           valOverride,    "CodingRate",           &settings.radioCodingRate},
  {'R',   0,   0,   1,    0, TYPE_BOOL,         valInt,         "FrequencyHop",         &settings.frequencyHop},
  {'R',   0,   0, 931,    3, TYPE_FLOAT,        valFreqMax,     "FrequencyMax",         &settings.frequencyMax},
  {'R',   0, 900,   0,    3, TYPE_FLOAT,        valFreqMin,     "FrequencyMin",         &settings.frequencyMin},
  {'R',   0,  10, 65535,  0, TYPE_U16,          valInt,         "MaxDwellTime",         &settings.maxDwellTime},
  {'R',   0,   1, 255,    0, TYPE_U8,           valInt,         "NumberOfChannels",     &settings.numberOfChannels},
  {'R',   0,   6, 65535,  0, TYPE_U16,          valInt,         "PreambleLength",       &settings.radioPreambleLength},
  {'R',   0,   6,  12,    0, TYPE_U8,           valOverride,    "SpreadFactor",         &settings.radioSpreadFactor},
  {'R',   0,   0, 255,    0, TYPE_U8,           valInt,         "SyncWord",             &settings.radioSyncWord},
  {'R',   0,  14,  30,    0, TYPE_U8,           valInt,         "TxPower",              &settings.radioBroadcastPower_dbm},

  /*Radio protocol parameters
    Ltr, All, min, max, digits,    type,         validation,     name,                   setting addr */
  {'R',   0,   0,   1,    0, TYPE_BOOL,         valInt,         "DataScrambling",       &settings.dataScrambling},
  {'R',   0,   0,   1,    0, TYPE_BOOL,         valInt,         "EnableCRC16",          &settings.enableCRC16},
  {'R',   0,   0,   1,    0, TYPE_BOOL,         valInt,         "EncryptData",          &settings.encryptData},
  {'R',   0,   0,   0,    0, TYPE_KEY,          valKey,         "EncryptionKey",        &settings.encryptionKey},
  {'R',   0,   0, 255,    0, TYPE_U8,           valInt,         "FramesToYield",        &settings.framesToYield},
  {'R',   0,  10, 2000,   0, TYPE_U16,          valInt,         "FrameTimeout",         &settings.serialTimeoutBeforeSendingFrame_ms},
  {'R',   0, 250, 65535,  0, TYPE_U16,          valInt,         "HeartBeatTimeout",     &settings.heartbeatTimeout},
  {'R',   0,   0, 255,    0, TYPE_U8,           valInt,         "MaxResends",           &settings.maxResends},
  {'R',   0,   0, 255,    0, TYPE_U8,           valInt,         "NetID",                &settings.netID},
  {'R',   0,   0,   2,    0, TYPE_U8,           valInt,         "OperatingMode",        &settings.operatingMode},
  {'R',   0,   0, 1000,   0, TYPE_U16,          valInt,         "OverHeadtime",         &settings.overheadTime},
  {'R',   0,   0,   1,    0, TYPE_BOOL,         valInt,         "Server",               &settings.server},
  {'R',   0,   0,   1,    0, TYPE_BOOL,         valInt,         "VerifyRxNetID",        &settings.verifyRxNetID},

  /*Serial parameters
    Ltr, All, min, max, digits,    type,         validation,     name,                   setting addr */
  {'S',   0,   0,   1,    0, TYPE_BOOL,         valInt,         "CopySerial",           &settings.copySerial},
  {'S',   0,   0,   1,    0, TYPE_BOOL,         valInt,         "Echo",                 &settings.echo},
  {'S',   0,   0,   1,    0, TYPE_BOOL,         valInt,         "FlowControl",          &settings.flowControl},
  {'S',   0,   0,   1,    0, TYPE_BOOL,         valInt,         "InvertCts",            &settings.invertCts},
  {'S',   0,   0,   1,    0, TYPE_BOOL,         valInt,         "InvertRts",            &settings.invertRts},
  {'S',   0,   0,   0,    0, TYPE_SPEED_SERIAL, valSpeedSerial, "SerialSpeed",          &settings.serialSpeed},
  {'S',   0,   0,   1,    0, TYPE_BOOL,         valInt,         "UsbSerialWait",        &settings.usbSerialWait},

  /*Training parameters
    Ltr, All, min, max, digits,    type,         validation,     name,                   setting addr */
  {'R',   0,   1, 255,    0, TYPE_U8,           valInt,      "ClientPingRetryInterval", &settings.clientPingRetryInterval},
  {'R',   0,   0,   0,    0, TYPE_KEY,          valKey,         "TrainingKey",          &settings.trainingKey},
  {'R',   0,   1, 255,    0, TYPE_U8,           valInt,         "TrainingTimeout",      &settings.trainingTimeout},

  /*Trigger parameters
    Ltr, All, min, max, digits,    type,         validation,     name,                   setting addr */
  {'P',   1,   0,   1,    0, TYPE_BOOL,         valInt,         "CopyTriggers",         &settings.copyTriggers},
  {'P',   1,   0, 0xffffffff, 0, TYPE_U32,      valInt,         "TriggerEnable_31-0",   &settings.triggerEnable},
  {'P',   1,   0, 0xffffffff, 0, TYPE_U32,      valInt,         "TriggerEnable_63-32",  &settings.triggerEnable2},
  {'P',   1,   1, 255,    0, TYPE_U8,           valInt,         "TriggerWidth",         &settings.triggerWidth},
  {'P',   1,   0,   1,    0, TYPE_BOOL,         valInt,     "TriggerWidthIsMultiplier", &settings.triggerWidthIsMultiplier},

  /*Virtual circuit parameters
    Ltr, All, min, max, digits,    type,         validation,     name,                   setting addr */
  {'V',   0,   0, MAX_VC - 1, 0, TYPE_U8,         valInt,         "CmdVC",                &cmdVc},
};

const int commandCount = sizeof(commands) / sizeof(commands[0]);

//----------------------------------------
//  ATSxx routines
//----------------------------------------

//Display a command
void commandDisplay(const COMMAND_ENTRY * command)
{
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

//Set or display the parameter by name
bool commandSetByName(const char * commandString)
{
  const char * buffer;
  const COMMAND_ENTRY * command;
  int index;
  int nameLength;
  int number;
  const char * param;
  int paramLength;
  int table;

  //Determine the parameter name length
  param = &commandString[3];
  buffer = param;
  paramLength = 0;
  while (*buffer && (*buffer != '=') && (*buffer != '?'))
  {
    buffer++;
    paramLength += 1;
  }

  command = NULL;
  for (index = 0; index < commandCount; index++)
  {
    nameLength = strlen(commands[index].name);
    if (nameLength == paramLength)
    {
      //Compare the parameter names
      if (strnicmp(param, commands[index].name, nameLength) == 0)
      {
        command = &commands[index];
        break;
      }
    }
  }

  //Verify that the parameter was found
  if (!command)
    //Report the error
    return false;

  //Process this command
  return commandSetOrDisplayValue(command, buffer);
}

//Set or display the command
bool commandSetOrDisplayValue(const COMMAND_ENTRY * command, const char * buffer)
{
  double doubleSettingValue;
  uint32_t settingValue;
  bool valid;

  do {
    //Is this a display request
    if (strcmp(buffer, "?") == 0)
    {
      commandDisplay(command);
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

    //The parameter was successfully set
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
void displayParameters(char letter, bool displayAll)
{
  int index;
  uint8_t sortOrder[commandCount];
  uint8_t temp;
  int x;

  //Set the default sort order
  for (index = 0; index < commandCount; index++)
    sortOrder[index] = index;

  //Perform a bubble sort if requested
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
    if (displayAll || (letter == commands[sortOrder[index]].letter)
        || ((letter == 0) && (!commands[sortOrder[index]].requireAll)))
    {
      petWDT(); //Printing may take longer than WDT at 9600, so pet the WDT.

      if (printerEndpoint == PRINT_TO_RF)
        systemPrint("R"); //If someone is asking for our settings over RF, respond with 'R' style settings
      else
        systemPrint("A");

      systemPrint("T-");
      systemPrint(commands[sortOrder[index]].name);
      systemPrint("=");
      commandDisplay(&commands[sortOrder[index]]);
    }
  }
}
