//To add a new command:
//Add the response to a query: ATS13?
//Add the data entry and validity check: ATS13=
//Add the setting to displayParameters
//Increase displayParameters loop value

//Check to see if a valid command has been received
void checkCommand()
{
  systemPrintln();

  if (commandLength < 2) //Too short
    reportERROR();

  //Check for 'AT'
  else if (isATcommand(commandBuffer) == false)
    reportERROR();

  //'AT'
  else if (commandLength == 2)
    reportOK();

  //ATI, ATO, ATZ commands
  else if (commandLength == 3)
  {
    switch (commandBuffer[2])
    {
      case ('I'):
        //Shows the radio version
        reportOK();
        systemPrint(F("SparkFun STR "));
        systemPrint(platformPrefix);
        systemPrint(F(" v"));
        systemPrint(FIRMWARE_VERSION_MAJOR);
        systemPrint(F("."));
        systemPrint(FIRMWARE_VERSION_MINOR);
        systemPrintln();
        break;
      case ('O'): //Exit command mode
        //If linked, send new settings to remote unit
        if (isLinked() == true)
        {
          //Todo check to see if there are new settings to transmit or not
          settingsDelivered = 0;
          sendCommandDataPacket(); //Send updated settings to remote
          changeState(RADIO_LINKED_COMMAND_TRANSMITTING);
        }
        else
        {
          //If not linked, apply settings and return
          generateHopTable(); //Generate freq with new settings
          configureRadio(); //Apply any new settings

          digitalWrite(pin_linkLED, LOW);
          digitalWrite(pin_activityLED, LOW);
          if (settings.pointToPoint == true)
            changeState(RADIO_NO_LINK_RECEIVING_STANDBY);
          else
            changeState(RADIO_BROADCASTING_RECEIVING_STANDBY);

          serialState = RADIO_SERIAL_PASSTHROUGH;

          reportOK();
        }
        break;
      case ('T'): //Enter training mode
        reportOK();
        beginTraining();
        break;
      case ('F'): //Enter training mode and return to factory defaults
        reportOK();
        beginDefaultTraining();
        break;
      case ('Z'): //Reboots the radio
        reportOK();
        systemFlush();
        systemReset();
        break;
      default:
        reportERROR();
        break;
    }
  }

  //ATIx commands
  else if (commandBuffer[2] == 'I' && commandLength == 4)
  {
    switch (commandBuffer[3])
    {
      case ('0'): //ATI0 - Show user settable parameters
        displayParameters('S');
        break;
      case ('1'): //ATI1 - Show board variant
        systemPrint(F("SparkFun STR "));
        systemPrintln(platformPrefix);
        break;
      case ('2'): //ATI2 - Show firmware version
        systemPrint(FIRMWARE_VERSION_MAJOR);
        systemPrint(F("."));
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
        {
          if (settings.encryptionKey[i] < 0x10) systemPrint("0");
          systemPrint(settings.encryptionKey[i], HEX);
          systemPrint(" ");
        }
        systemPrintln();
        break;
      case ('7'): //ATI7 - Show current FHSS channel
        systemPrintln(radio.getFHSSChannel());
        break;
      case ('8'): //ATI8 - Show all remote user settable parameters
        if (isLinked() == true)
          displayParameters('R');
        else
          reportERROR();
        break;
      default:
        reportERROR();
        break;
    }
  }

  //AT&x commands
  else if (commandBuffer[2] == '&')
  {
    //&W, &F, &T, and &T=RSSI/TDM
    if (commandLength == 4)
    {
      switch (commandBuffer[3])
      {
        case ('W'): //AT&W - Write parameters to the flash memory
          {
            recordSystemSettings();
            reportOK();
          }
          break;
        case ('F'): //AT&F - Restore default parameters
          {
            Settings defaultSettings; //Create new settings that will have all default values
            settings = defaultSettings; //Overwrite all current settings with defaults
            recordSystemSettings();
            reportOK();
          }
          break;
        default:
          reportERROR();
          break;
      }
    }
    else
    {
      //RSSI
      if (strcmp_P(commandBuffer, PSTR("AT&T=RSSI")) == 0) //Enable packet quality reporting
      {
        settings.displayPacketQuality = true;
        reportOK();
      }
      else
      {
        reportERROR();
      }
    }
  }

  //ATSn? and ATRn?
  else if ((commandBuffer[2] == 'S' || commandBuffer[2] == 'R') && (commandBuffer[4] == '?' || commandBuffer[5] == '?'))
  {
    if (commandBuffer[2] == 'R' && isLinked() == false)
    {
      reportERROR();
      return;
    }

    switch (commandBuffer[3])
    {
      case ('0'): //ATS0?
        if (commandBuffer[2] == 'S') systemPrintln(settings.serialSpeed);
        if (commandBuffer[2] == 'R') systemPrintln(remoteSettings.serialSpeed);
        break;
      case ('1'): //ATS1? and ATS1*?
        {
          switch (commandBuffer[4])
          {
            case ('?'): //ATS1?
              if (commandBuffer[2] == 'S') systemPrintln(settings.airSpeed);
              if (commandBuffer[2] == 'R') systemPrintln(remoteSettings.airSpeed);
              break;
            case ('0'): //ATS10?
              if (commandBuffer[2] == 'S') systemPrintln(settings.frequencyHop);
              if (commandBuffer[2] == 'R') systemPrintln(remoteSettings.frequencyHop);
              break;
            case ('1'): //ATS11?
              if (commandBuffer[2] == 'S') systemPrintln(settings.maxDwellTime);
              if (commandBuffer[2] == 'R') systemPrintln(remoteSettings.maxDwellTime);
              break;
            case ('2'): //ATS12?
              if (commandBuffer[2] == 'S') systemPrintln(settings.radioBandwidth);
              if (commandBuffer[2] == 'R') systemPrintln(remoteSettings.radioBandwidth);
              break;
            case ('3'): //ATS13?
              if (commandBuffer[2] == 'S') systemPrintln(settings.radioSpreadFactor);
              if (commandBuffer[2] == 'R') systemPrintln(remoteSettings.radioSpreadFactor);
              break;
            case ('4'): //ATS14?
              if (commandBuffer[2] == 'S') systemPrintln(settings.radioCodingRate);
              if (commandBuffer[2] == 'R') systemPrintln(remoteSettings.radioCodingRate);
              break;
            case ('5'): //ATS15?
              if (commandBuffer[2] == 'S') systemPrintln(settings.radioSyncWord);
              if (commandBuffer[2] == 'R') systemPrintln(remoteSettings.radioSyncWord);
              break;
            case ('6'): //ATS16?
              if (commandBuffer[2] == 'S') systemPrintln(settings.radioPreambleLength);
              if (commandBuffer[2] == 'R') systemPrintln(remoteSettings.radioPreambleLength);
              break;
            case ('7'): //ATS17?
              if (commandBuffer[2] == 'S') systemPrintln(settings.frameSize);
              if (commandBuffer[2] == 'R') systemPrintln(remoteSettings.frameSize);
              break;
            case ('8'): //ATS18?
              if (commandBuffer[2] == 'S') systemPrintln(settings.serialTimeoutBeforeSendingFrame_ms);
              if (commandBuffer[2] == 'R') systemPrintln(remoteSettings.serialTimeoutBeforeSendingFrame_ms);
              break;
            case ('9'): //ATS19?
              if (commandBuffer[2] == 'S') systemPrintln(settings.debug);
              if (commandBuffer[2] == 'R') systemPrintln(remoteSettings.debug);
              break;

            default:
              reportERROR();
              break;
          }
        }
        break;

      case ('2'): //ATS2? and ATS2*?
        {
          switch (commandBuffer[4])
          {
            case ('?'): //ATS2?
              if (commandBuffer[2] == 'S') systemPrintln(settings.netID);
              if (commandBuffer[2] == 'R') systemPrintln(remoteSettings.netID);
              break;
            case ('0'): //ATS20?
              if (commandBuffer[2] == 'S') systemPrintln(settings.echo);
              if (commandBuffer[2] == 'R') systemPrintln(remoteSettings.echo);
              break;
            case ('1'): //ATS21?
              if (commandBuffer[2] == 'S') systemPrintln(settings.heartbeatTimeout);
              if (commandBuffer[2] == 'R') systemPrintln(remoteSettings.heartbeatTimeout);
              break;
            case ('2'): //ATS22?
              if (commandBuffer[2] == 'S') systemPrintln(settings.flowControl);
              if (commandBuffer[2] == 'R') systemPrintln(remoteSettings.flowControl);
              break;
            case ('3'): //ATS23?
              if (commandBuffer[2] == 'S') systemPrintln(settings.autoTuneFrequency);
              if (commandBuffer[2] == 'R') systemPrintln(remoteSettings.autoTuneFrequency);
              break;
            case ('4'): //ATS24?
              if (commandBuffer[2] == 'S') systemPrintln(settings.displayPacketQuality);
              if (commandBuffer[2] == 'R') systemPrintln(remoteSettings.displayPacketQuality);
              break;
            case ('5'): //ATS25?
              if (commandBuffer[2] == 'S') systemPrintln(settings.maxResends);
              if (commandBuffer[2] == 'R') systemPrintln(remoteSettings.maxResends);
              break;

            default:
              reportERROR();
              break;
          }
        }
        break;

      case ('3'): //ATS3?
        if (commandBuffer[2] == 'S') systemPrintln(settings.pointToPoint);
        if (commandBuffer[2] == 'R') systemPrintln(remoteSettings.pointToPoint);
        break;
      case ('4'): //ATS4?
        if (commandBuffer[2] == 'S') systemPrintln(settings.encryptData);
        if (commandBuffer[2] == 'R') systemPrintln(remoteSettings.encryptData);
        break;
      case ('5'): //ATS5?
        if (commandBuffer[2] == 'S') systemPrintln(settings.dataScrambling);
        if (commandBuffer[2] == 'R') systemPrintln(remoteSettings.dataScrambling);
        break;
      case ('6'): //ATS6?
        if (commandBuffer[2] == 'S') systemPrintln(settings.radioBroadcastPower_dbm);
        if (commandBuffer[2] == 'R') systemPrintln(remoteSettings.radioBroadcastPower_dbm);
        break;
      case ('7'): //ATS7?
        if (commandBuffer[2] == 'S') systemPrintln(settings.frequencyMin);
        if (commandBuffer[2] == 'R') systemPrintln(remoteSettings.frequencyMin);
        break;
      case ('8'): //ATS8?
        if (commandBuffer[2] == 'S') systemPrintln(settings.frequencyMax);
        if (commandBuffer[2] == 'R') systemPrintln(remoteSettings.frequencyMax);
        break;
      case ('9'): //ATS9?
        if (commandBuffer[2] == 'S') systemPrintln(settings.numberOfChannels);
        if (commandBuffer[2] == 'R') systemPrintln(remoteSettings.numberOfChannels);
        break;

      default:
        reportERROR();
        break;
    }
  }

  //ATSn=X and ATRn=X and ATLn=X
  else if ((commandBuffer[2] == 'S' || commandBuffer[2] == 'R' || commandBuffer[2] == 'L') && (commandBuffer[4] == '=' || commandBuffer[5] == '='))
  {
    if ((commandBuffer[2] == 'R' || commandBuffer[2] == 'L') && isLinked() == false)
    {
      reportERROR();
      return;
    }

    char temp[30];
    strcpy(temp, commandBuffer); //strok modifies the original so we make a copy

    char *str = temp;

    char* ptr;
    str = strtok(str, "="); //Locate the location of the = sign
    str = strtok(NULL, "="); //Move pointer to end of line, after the first occurrence

    double doubleSettingValue = strtod(str, &ptr); //Pull out setting value
    int32_t settingValue = doubleSettingValue;

    switch (commandBuffer[3])
    {
      case ('0'): //ATS0=
        {
          if (settingValue == 2400
              || settingValue == 4800
              || settingValue == 9600
              || settingValue == 14400
              || settingValue == 19200
              || settingValue == 38400
              || settingValue == 57600
              || settingValue == 115200
             )
          {
            if (commandBuffer[2] == 'S' || commandBuffer[2] == 'L') settings.serialSpeed = settingValue;
            if (commandBuffer[2] == 'R' || commandBuffer[2] == 'L') remoteSettings.serialSpeed = settingValue;
            reportOK();
          }
          else
            reportERROR();
        }
        break;
      case ('1'): //ATS1= and ATS1*=
        {
          switch (commandBuffer[4])
          {
            case ('='): //ATS1=
              if (settingValue == 0
                  || settingValue == 40
                  || settingValue == 150
                  || settingValue == 400
                  || settingValue == 1200
                  || settingValue == 2400
                  || settingValue == 4800
                  || settingValue == 9600
                  || settingValue == 19200
                  || settingValue == 28800
                  || settingValue == 38400
                 )
              {
                if (settings.airSpeed == 0 && settingValue != 0) systemPrintln(F("Warning: AirSpeed override of bandwidth, spread factor, and coding rate"));

                if (commandBuffer[2] == 'S' || commandBuffer[2] == 'L') settings.airSpeed = settingValue;
                if (commandBuffer[2] == 'R' || commandBuffer[2] == 'L') remoteSettings.airSpeed = settingValue;

                //TODO - We need to update the values for spread, etc, but without touching the radio yet.
                //configureRadio(); //Update spread, bandwidth, and coding as needed
                reportOK();
              }
              else
                reportERROR();
              break;
            case ('0'): //ATS10=
              if (settingValue >= 0 && settingValue <= 1)
              {
                if (commandBuffer[2] == 'S' || commandBuffer[2] == 'L') settings.frequencyHop = settingValue;
                if (commandBuffer[2] == 'R' || commandBuffer[2] == 'L') remoteSettings.frequencyHop = settingValue;
                reportOK();
              }
              else
                reportERROR();
              break;
            case ('1'): //ATS11=
              if (settingValue >= 10 && settingValue <= 65535)
              {
                if (commandBuffer[2] == 'S' || commandBuffer[2] == 'L') settings.maxDwellTime = settingValue;
                if (commandBuffer[2] == 'R' || commandBuffer[2] == 'L') remoteSettings.maxDwellTime = settingValue;
                reportOK();
              }
              else
                reportERROR();
              break;
            case ('2'): //ATS12=
              if (settings.airSpeed != 0)
              {
                systemPrintln(F("AirSpeed is overriding"));
                reportERROR();
              }
              else
              {
                //Some doubles get rounded incorrectly
                if ( (doubleSettingValue * 100 == 1040)
                     || doubleSettingValue == 15.6
                     || (doubleSettingValue * 100) == 2080
                     || doubleSettingValue == 31.25
                     || doubleSettingValue == 41.7
                     || doubleSettingValue == 62.5
                     || doubleSettingValue == 125.0
                     || doubleSettingValue == 250.0
                     || doubleSettingValue == 500.0
                   )
                {
                  if (commandBuffer[2] == 'S' || commandBuffer[2] == 'L') settings.radioBandwidth = doubleSettingValue;
                  if (commandBuffer[2] == 'R' || commandBuffer[2] == 'L') remoteSettings.radioBandwidth = doubleSettingValue;
                  reportOK();
                }
                else
                  reportERROR();
              }
              break;
            case ('3'): //ATS13=
              if (settings.airSpeed != 0)
              {
                systemPrintln(F("AirSpeed is overriding"));
                reportERROR();
              }
              else
              {
                if (settingValue >= 6 && settingValue <= 12)
                {
                  if (commandBuffer[2] == 'S' || commandBuffer[2] == 'L') settings.radioSpreadFactor = settingValue;
                  if (commandBuffer[2] == 'R' || commandBuffer[2] == 'L') remoteSettings.radioSpreadFactor = settingValue;
                  reportOK();
                }
                else
                  reportERROR();
              }
              break;
            case ('4'): //ATS14=
              if (settings.airSpeed != 0)
              {
                systemPrintln(F("AirSpeed is overriding"));
                reportERROR();
              }
              else
              {
                if (settingValue >= 5 && settingValue <= 8)
                {
                  if (commandBuffer[2] == 'S' || commandBuffer[2] == 'L') settings.radioCodingRate = settingValue;
                  if (commandBuffer[2] == 'R' || commandBuffer[2] == 'L') remoteSettings.radioCodingRate = settingValue;
                  reportOK();
                }
                else
                  reportERROR();
              }
              break;
            case ('5'): //ATS15=
              if (settingValue >= 0 && settingValue <= 255)
              {
                if (commandBuffer[2] == 'S' || commandBuffer[2] == 'L') settings.radioSyncWord = settingValue;
                if (commandBuffer[2] == 'R' || commandBuffer[2] == 'L') remoteSettings.radioSyncWord = settingValue;
                reportOK();
              }
              else
                reportERROR();
              break;
            case ('6'): //ATS16=
              if (settingValue >= 6 && settingValue <= 65535)
              {
                if (commandBuffer[2] == 'S' || commandBuffer[2] == 'L') settings.radioPreambleLength = settingValue;
                if (commandBuffer[2] == 'R' || commandBuffer[2] == 'L') remoteSettings.radioPreambleLength = settingValue;
                reportOK();
              }
              else
                reportERROR();
              break;
            case ('7'): //ATS17=
              if (settingValue >= 16 && settingValue <= (256 - 2)) //NetID+Control trailer is 2 bytes
              {
                if (commandBuffer[2] == 'S' || commandBuffer[2] == 'L') settings.frameSize = settingValue;
                if (commandBuffer[2] == 'R' || commandBuffer[2] == 'L') remoteSettings.frameSize = settingValue;
                reportOK();
              }
              else
                reportERROR();
              break;
            case ('8'): //ATS18=
              if (settingValue >= 10 && settingValue <= 2000)
              {
                if (commandBuffer[2] == 'S' || commandBuffer[2] == 'L') settings.serialTimeoutBeforeSendingFrame_ms = settingValue;
                if (commandBuffer[2] == 'R' || commandBuffer[2] == 'L') remoteSettings.serialTimeoutBeforeSendingFrame_ms = settingValue;
                reportOK();
              }
              else
                reportERROR();
              break;
            case ('9'): //ATS19=
              if (settingValue >= 0 && settingValue <= 1)
              {
                if (commandBuffer[2] == 'S' || commandBuffer[2] == 'L') settings.debug = settingValue;
                if (commandBuffer[2] == 'R' || commandBuffer[2] == 'L') remoteSettings.debug = settingValue;
                reportOK();
              }
              else
                reportERROR();
              break;

            default:
              reportERROR();
              break;
          }
        }
        break;


      case ('2'): //ATS2= and ATS2*=
        {
          switch (commandBuffer[4])
          {
            case ('='): //ATS2=
              if (settingValue >= 0 && settingValue <= 255)
              {
                if (commandBuffer[2] == 'S' || commandBuffer[2] == 'L') settings.netID = settingValue;
                if (commandBuffer[2] == 'R' || commandBuffer[2] == 'L') remoteSettings.netID = settingValue;
                reportOK();
              }
              else
                reportERROR();
              break;
            case ('0'): //ATS20=
              if (settingValue >= 0 && settingValue <= 1)
              {
                if (commandBuffer[2] == 'S' || commandBuffer[2] == 'L') settings.echo = settingValue;
                if (commandBuffer[2] == 'R' || commandBuffer[2] == 'L') remoteSettings.echo = settingValue;
                reportOK();
              }
              else
                reportERROR();
              break;
            case ('1'): //ATS21=
              if (settingValue >= 250 && settingValue <= 65535)
              {
                if (commandBuffer[2] == 'S' || commandBuffer[2] == 'L') settings.heartbeatTimeout = settingValue;
                if (commandBuffer[2] == 'R' || commandBuffer[2] == 'L') remoteSettings.heartbeatTimeout = settingValue;
                reportOK();
              }
              else
                reportERROR();
              break;
            case ('2'): //ATS22=
              if (settingValue >= 0 && settingValue <= 1)
              {
                if (commandBuffer[2] == 'S' || commandBuffer[2] == 'L') settings.flowControl = settingValue;
                if (commandBuffer[2] == 'R' || commandBuffer[2] == 'L') remoteSettings.flowControl = settingValue;
                reportOK();
              }
              else
                reportERROR();
              break;
            case ('3'): //ATS23=
              if (settingValue >= 0 && settingValue <= 1)
              {
                if (commandBuffer[2] == 'S' || commandBuffer[2] == 'L') settings.autoTuneFrequency = settingValue;
                if (commandBuffer[2] == 'R' || commandBuffer[2] == 'L') remoteSettings.autoTuneFrequency = settingValue;
                reportOK();
              }
              else
                reportERROR();
              break;
            case ('4'): //ATS24=
              if (settingValue >= 0 && settingValue <= 1)
              {
                if (commandBuffer[2] == 'S' || commandBuffer[2] == 'L') settings.displayPacketQuality = settingValue;
                if (commandBuffer[2] == 'R' || commandBuffer[2] == 'L') remoteSettings.displayPacketQuality = settingValue;
                reportOK();
              }
              else
                reportERROR();
              break;
            case ('5'): //ATS25=
              if (settingValue >= 0 && settingValue <= 255)
              {
                if (commandBuffer[2] == 'S' || commandBuffer[2] == 'L') settings.maxResends = settingValue;
                if (commandBuffer[2] == 'R' || commandBuffer[2] == 'L') remoteSettings.maxResends = settingValue;
                reportOK();
              }
              else
                reportERROR();
              break;

            default:
              reportERROR();
              break;
          }
        }
        break;

      case ('3'): //ATS3=
        if (settingValue >= 0 && settingValue <= 1)
        {
          if (commandBuffer[2] == 'S' || commandBuffer[2] == 'L') settings.pointToPoint = settingValue;
          if (commandBuffer[2] == 'R' || commandBuffer[2] == 'L') remoteSettings.pointToPoint = settingValue;
          reportOK();
        }
        else
          reportERROR();
        break;
      case ('4'): //ATS4=
        if (settingValue >= 0 && settingValue <= 1)
        {
          if (commandBuffer[2] == 'S' || commandBuffer[2] == 'L') settings.encryptData = settingValue;
          if (commandBuffer[2] == 'R' || commandBuffer[2] == 'L') remoteSettings.encryptData = settingValue;
          reportOK();
        }
        else
          reportERROR();
        break;
      case ('5'): //ATS5=
        if (settingValue >= 0 && settingValue <= 1)
        {
          if (commandBuffer[2] == 'S' || commandBuffer[2] == 'L') settings.dataScrambling = settingValue;
          if (commandBuffer[2] == 'R' || commandBuffer[2] == 'L') remoteSettings.dataScrambling = settingValue;
          reportOK();
        }
        else
          reportERROR();
        break;
      case ('6'): //ATS6=
        if (settingValue >= 0 && settingValue <= 20)
        {
          if (commandBuffer[2] == 'S' || commandBuffer[2] == 'L') settings.radioBroadcastPower_dbm = settingValue;
          if (commandBuffer[2] == 'R' || commandBuffer[2] == 'L') remoteSettings.radioBroadcastPower_dbm = settingValue;
          reportOK();
        }
        else
          reportERROR();
        break;
      case ('7'): //ATS7=
        if (doubleSettingValue >= 902.0 && doubleSettingValue <= settings.frequencyMax)
        {
          if (commandBuffer[2] == 'S' || commandBuffer[2] == 'L') settings.frequencyMin = doubleSettingValue;
          if (commandBuffer[2] == 'R' || commandBuffer[2] == 'L') remoteSettings.frequencyMin = doubleSettingValue;
          reportOK();
        }
        else
          reportERROR();
        break;
      case ('8'): //ATS8=
        if (doubleSettingValue >= settings.frequencyMin && doubleSettingValue <= 928.0)
        {
          if (commandBuffer[2] == 'S' || commandBuffer[2] == 'L') settings.frequencyMax = doubleSettingValue;
          if (commandBuffer[2] == 'R' || commandBuffer[2] == 'L') remoteSettings.frequencyMax = doubleSettingValue;
          reportOK();
        }
        else
          reportERROR();
        break;
      case ('9'): //ATS9=
        if (settingValue >= 1 && settingValue <= 50)
        {
          if (commandBuffer[2] == 'S' || commandBuffer[2] == 'L') settings.numberOfChannels = settingValue;
          if (commandBuffer[2] == 'R' || commandBuffer[2] == 'L') remoteSettings.numberOfChannels = settingValue;
          reportOK();
        }
        else
          reportERROR();
        break;

      default:
        reportERROR();
        break;
    }

  } //End ATSn=X commands

  //Unknown command
  else
    reportERROR();

  commandLength = 0; //Get ready for next command
}

void reportOK()
{
  systemPrintln(F("OK"));
}

void reportERROR()
{
  systemPrintln(F("ERROR"));
}

//Check if AT appears in the correct position
bool isATcommand(char *buffer)
{
  if (buffer[0] == 'A')
    if (buffer[1] == 'T')
      return (true);
  return (false);
}

//Show current settings in user friendly way
void displayParameters(char parameterType)
{
  for (uint8_t x = 0 ; x <= 25 ; x++)
  {
    if (parameterType == 'S') systemPrint("S");
    if (parameterType == 'R') systemPrint("R");
    systemPrint(x);
    systemPrint(F(":"));

    //Name
    switch (x)
    {
      case (0):
        systemPrint(F("SerialSpeed"));
        break;
      case (1):
        systemPrint(F("AirSpeed"));
        break;
      case (2):
        systemPrint(F("netID"));
        break;
      case (3):
        systemPrint(F("PointToPoint"));
        break;
      case (4):
        systemPrint(F("EncryptData"));
        break;
      case (5):
        systemPrint(F("DataScrambling"));
        break;
      case (6):
        systemPrint(F("TxPower"));
        break;
      case (7):
        systemPrint(F("FrequencyMin"));
        break;
      case (8):
        systemPrint(F("FrequencyMax"));
        break;
      case (9):
        systemPrint(F("NumberOfChannels"));
        break;
      case (10):
        systemPrint(F("FrequencyHop"));
        break;
      case (11):
        systemPrint(F("MaxDwellTime"));
        break;
      case (12):
        systemPrint(F("Bandwidth"));
        break;
      case (13):
        systemPrint(F("SpreadFactor"));
        break;
      case (14):
        systemPrint(F("CodingRate"));
        break;
      case (15):
        systemPrint(F("SyncWord"));
        break;
      case (16):
        systemPrint(F("PreambleLength"));
        break;
      case (17):
        systemPrint(F("FrameSize"));
        break;
      case (18):
        systemPrint(F("FrameTimeout"));
        break;
      case (19):
        systemPrint(F("Debug"));
        break;
      case (20):
        systemPrint(F("Echo"));
        break;
      case (21):
        systemPrint(F("HeartBeatTimeout"));
        break;
      case (22):
        systemPrint(F("FlowControl"));
        break;
      case (23):
        systemPrint(F("AutoTune"));
        break;
      case (24):
        systemPrint(F("DisplayPacketQuality"));
        break;
      case (25):
        systemPrint(F("MaxResends"));
        break;
      default:
        systemPrint(F("Unknown"));
        break;
    }

    systemPrint(F("="));

    //Value
    switch (x)
    {
      case (0):
        if (parameterType == 'S') systemPrint(settings.serialSpeed);
        if (parameterType == 'R') systemPrint(remoteSettings.serialSpeed);
        break;
      case (1):
        if (parameterType == 'S') systemPrint(settings.airSpeed);
        if (parameterType == 'R') systemPrint(remoteSettings.airSpeed);
        break;
      case (2):
        if (parameterType == 'S') systemPrint(settings.netID);
        if (parameterType == 'R') systemPrint(remoteSettings.netID);
        break;
      case (3):
        if (parameterType == 'S') systemPrint(settings.pointToPoint);
        if (parameterType == 'R') systemPrint(remoteSettings.pointToPoint);
        break;
      case (4):
        if (parameterType == 'S') systemPrint(settings.encryptData);
        if (parameterType == 'R') systemPrint(remoteSettings.encryptData);
        break;
      case (5):
        if (parameterType == 'S') systemPrint(settings.dataScrambling);
        if (parameterType == 'R') systemPrint(remoteSettings.dataScrambling);
        break;
      case (6):
        if (parameterType == 'S') systemPrint(settings.radioBroadcastPower_dbm);
        if (parameterType == 'R') systemPrint(remoteSettings.radioBroadcastPower_dbm);
        break;
      case (7):
        if (parameterType == 'S') systemPrint(settings.frequencyMin, 3);
        if (parameterType == 'R') systemPrint(remoteSettings.frequencyMin, 3);
        break;
      case (8):
        if (parameterType == 'S') systemPrint(settings.frequencyMax, 3);
        if (parameterType == 'R') systemPrint(remoteSettings.frequencyMax, 3);
        break;
      case (9):
        if (parameterType == 'S') systemPrint(settings.numberOfChannels);
        if (parameterType == 'R') systemPrint(remoteSettings.numberOfChannels);
        break;
      case (10):
        if (parameterType == 'S') systemPrint(settings.frequencyHop);
        if (parameterType == 'R') systemPrint(remoteSettings.frequencyHop);
        break;
      case (11):
        if (parameterType == 'S') systemPrint(settings.maxDwellTime);
        if (parameterType == 'R') systemPrint(remoteSettings.maxDwellTime);
        break;
      case (12):
        if (parameterType == 'S') systemPrint(settings.radioBandwidth, 2);
        if (parameterType == 'R') systemPrint(remoteSettings.radioBandwidth, 2);
        break;
      case (13):
        if (parameterType == 'S') systemPrint(settings.radioSpreadFactor);
        if (parameterType == 'R') systemPrint(remoteSettings.radioSpreadFactor);
        break;
      case (14):
        if (parameterType == 'S') systemPrint(settings.radioCodingRate);
        if (parameterType == 'R') systemPrint(remoteSettings.radioCodingRate);
        break;
      case (15):
        if (parameterType == 'S') systemPrint(settings.radioSyncWord);
        if (parameterType == 'R') systemPrint(remoteSettings.radioSyncWord);
        break;
      case (16):
        if (parameterType == 'S') systemPrint(settings.radioPreambleLength);
        if (parameterType == 'R') systemPrint(remoteSettings.radioPreambleLength);
        break;
      case (17):
        if (parameterType == 'S') systemPrint(settings.frameSize);
        if (parameterType == 'R') systemPrint(remoteSettings.frameSize);
        break;
      case (18):
        if (parameterType == 'S') systemPrint(settings.serialTimeoutBeforeSendingFrame_ms);
        if (parameterType == 'R') systemPrint(remoteSettings.serialTimeoutBeforeSendingFrame_ms);
        break;
      case (19):
        if (parameterType == 'S') systemPrint(settings.debug);
        if (parameterType == 'R') systemPrint(remoteSettings.debug);
        break;
      case (20):
        if (parameterType == 'S') systemPrint(settings.echo);
        if (parameterType == 'R') systemPrint(remoteSettings.echo);
        break;
      case (21):
        if (parameterType == 'S') systemPrint(settings.heartbeatTimeout);
        if (parameterType == 'R') systemPrint(remoteSettings.heartbeatTimeout);
        break;
      case (22):
        if (parameterType == 'S') systemPrint(settings.flowControl);
        if (parameterType == 'R') systemPrint(remoteSettings.flowControl);
        break;
      case (23):
        if (parameterType == 'S') systemPrint(settings.autoTuneFrequency);
        if (parameterType == 'R') systemPrint(remoteSettings.autoTuneFrequency);
        break;
      case (24):
        if (parameterType == 'S') systemPrint(settings.displayPacketQuality);
        if (parameterType == 'R') systemPrint(remoteSettings.displayPacketQuality);
        break;
      case (25):
        if (parameterType == 'S') systemPrint(settings.maxResends);
        if (parameterType == 'R') systemPrint(remoteSettings.maxResends);
        break;
      default:
        systemPrint(F("0"));
        break;
    }

    systemPrintln();
  }
}
