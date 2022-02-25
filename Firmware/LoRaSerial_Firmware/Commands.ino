//To add a new command:
//Add the response to a query: ATS13?
//Add the data entry and validity check: ATS13=
//Add the setting to displayParameters
//Increase displayParameters loop value

//Respond to AT commands
void commandMode()
{
  char commandBuffer[30];

  systemPrintln(F("\r\nOK"));

  while (Serial.available()) Serial.read();
#if defined(ARDUINO_ARCH_SAMD)
  while (Serial1.available()) Serial1.read();
#endif

  while (true)
  {
    uint8_t commandLength = readLine(commandBuffer, sizeof(commandBuffer));
    if (commandLength >= 2)
    {
      for (uint8_t x = 0 ; x < commandLength ; x++)
        commandBuffer[x] = toupper(commandBuffer[x]);

      //Check if it's AT or RT
      if (isATcommand(commandBuffer))
      {
        //AT command
        if (commandLength == 2)
        {
          reportOK();
        }

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
              generateHopTable(); //Generate freq with new settings
              configureRadio(); //Apply any new settings

              digitalWrite(pin_linkLED, LOW);
              digitalWrite(pin_activityLED, LOW);
              if (settings.pointToPoint == true)
                changeState(RADIO_NO_LINK_RECEIVING_STANDBY);
              else
                changeState(RADIO_BROADCASTING_RECEIVING_STANDBY);

              reportOK();
              return;
              break;
            case ('T'): //Enter training mode
              reportOK();
              beginTraining();
              return;
              break;
            case ('F'): //Enter training mode and return to factory defaults
              reportOK();
              beginDefaultTraining();
              return;
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
              displayParameters();
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

        //ATSn? commands
        else if (commandBuffer[2] == 'S' && (commandBuffer[4] == '?' || commandBuffer[5] == '?'))
        {
          switch (commandBuffer[3])
          {
            case ('0'): //ATS0?
              systemPrintln(settings.serialSpeed);
              break;
            case ('1'): //ATS1? and ATS1*?
              {
                switch (commandBuffer[4])
                {
                  case ('?'): //ATS1?
                    systemPrintln(settings.airSpeed);
                    break;
                  case ('0'): //ATS10?
                    systemPrintln(settings.frequencyHop);
                    break;
                  case ('1'): //ATS11?
                    systemPrintln(settings.maxDwellTime);
                    break;
                  case ('2'): //ATS12?
                    systemPrintln(settings.radioBandwidth);
                    break;
                  case ('3'): //ATS13?
                    systemPrintln(settings.radioSpreadFactor);
                    break;
                  case ('4'): //ATS14?
                    systemPrintln(settings.radioCodingRate);
                    break;
                  case ('5'): //ATS15?
                    systemPrintln(settings.radioSyncWord);
                    break;
                  case ('6'): //ATS16?
                    systemPrintln(settings.radioPreambleLength);
                    break;
                  case ('7'): //ATS17?
                    systemPrintln(settings.frameSize);
                    break;
                  case ('8'): //ATS18?
                    systemPrintln(settings.serialTimeoutBeforeSendingFrame_ms);
                    break;
                  case ('9'): //ATS19?
                    systemPrintln(settings.debug);
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
                    systemPrintln(settings.netID);
                    break;
                  case ('0'): //ATS20?
                    systemPrintln(settings.echo);
                    break;
                  case ('1'): //ATS21?
                    systemPrintln(settings.heartbeatTimeout);
                    break;
                  case ('2'): //ATS22?
                    systemPrintln(settings.flowControl);
                    break;
                  case ('3'): //ATS23?
                    systemPrintln(settings.autoTuneFrequency);
                    break;
                  case ('4'): //ATS24?
                    systemPrintln(settings.displayPacketQuality);
                    break;
                  case ('5'): //ATS25?
                    systemPrintln(settings.maxResends);
                    break;

                  default:
                    reportERROR();
                    break;
                }
              }
              break;

            case ('3'): //ATS3?
              systemPrintln(settings.pointToPoint);
              break;
            case ('4'): //ATS4?
              systemPrintln(settings.encryptData);
              break;
            case ('5'): //ATS5?
              systemPrintln(settings.dataScrambling);
              break;
            case ('6'): //ATS6?
              systemPrintln(settings.radioBroadcastPower_dbm);
              break;
            case ('7'): //ATS7?
              systemPrintln(settings.frequencyMin);
              break;
            case ('8'): //ATS8?
              systemPrintln(settings.frequencyMax);
              break;
            case ('9'): //ATS9?
              systemPrintln(settings.numberOfChannels);
              break;

            default:
              reportERROR();
              break;
          }
        }

        //ATSn=X commands
        else if (commandBuffer[2] == 'S' && (commandBuffer[4] == '=' || commandBuffer[5] == '='))
        {
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
                  settings.serialSpeed = settingValue;
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
                      settings.airSpeed = settingValue;
                      configureRadio(); //Update spread, bandwidth, and coding as needed
                      reportOK();
                    }
                    else
                      reportERROR();
                    break;
                  case ('0'): //ATS10=
                    if (settingValue >= 0 && settingValue <= 1)
                    {
                      settings.frequencyHop = settingValue;
                      reportOK();
                    }
                    else
                      reportERROR();
                    break;
                  case ('1'): //ATS11=
                    if (settingValue >= 10 && settingValue <= 65535)
                    {
                      settings.maxDwellTime = settingValue;
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
                        settings.radioBandwidth = doubleSettingValue;
                        reportOK();
                      }
                      else
                        reportERROR();
                    }
                    break;
                  case ('3'): //ATS13=
                    if (settingValue >= 6 && settingValue <= 12)
                    {
                      settings.radioSpreadFactor = settingValue;
                      reportOK();
                    }
                    else
                      reportERROR();
                    break;
                  case ('4'): //ATS14=
                    if (settingValue >= 5 && settingValue <= 8)
                    {
                      settings.radioCodingRate = settingValue;
                      reportOK();
                    }
                    else
                      reportERROR();
                    break;
                  case ('5'): //ATS15=
                    if (settingValue >= 0 && settingValue <= 255)
                    {
                      settings.radioSyncWord = settingValue;
                      reportOK();
                    }
                    else
                      reportERROR();
                    break;
                  case ('6'): //ATS16=
                    if (settingValue >= 6 && settingValue <= 65535)
                    {
                      settings.radioPreambleLength = settingValue;
                      reportOK();
                    }
                    else
                      reportERROR();
                    break;
                  case ('7'): //ATS17=
                    if (settingValue >= 16 && settingValue <= (256 - 2)) //NetID+Control trailer is 2 bytes
                    {
                      settings.frameSize = settingValue;
                      reportOK();
                    }
                    else
                      reportERROR();
                    break;
                  case ('8'): //ATS18=
                    if (settingValue >= 10 && settingValue <= 2000)
                    {
                      settings.serialTimeoutBeforeSendingFrame_ms = settingValue;
                      reportOK();
                    }
                    else
                      reportERROR();
                    break;
                  case ('9'): //ATS19=
                    if (settingValue >= 0 && settingValue <= 1)
                    {
                      settings.debug = settingValue;
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
                      settings.netID = settingValue;
                      reportOK();
                    }
                    else
                      reportERROR();
                    break;
                  case ('0'): //ATS20=
                    if (settingValue >= 0 && settingValue <= 1)
                    {
                      settings.echo = settingValue;
                      reportOK();
                    }
                    else
                      reportERROR();
                    break;
                  case ('1'): //ATS21=
                    if (settingValue >= 250 && settingValue <= 65535)
                    {
                      settings.heartbeatTimeout = settingValue;
                      reportOK();
                    }
                    else
                      reportERROR();
                    break;
                  case ('2'): //ATS22=
                    if (settingValue >= 0 && settingValue <= 1)
                    {
                      settings.flowControl = settingValue;
                      reportOK();
                    }
                    else
                      reportERROR();
                    break;
                  case ('3'): //ATS23=
                    if (settingValue >= 0 && settingValue <= 1)
                    {
                      settings.autoTuneFrequency = settingValue;
                      reportOK();
                    }
                    else
                      reportERROR();
                    break;
                  case ('4'): //ATS24=
                    if (settingValue >= 0 && settingValue <= 1)
                    {
                      settings.displayPacketQuality = settingValue;
                      reportOK();
                    }
                    else
                      reportERROR();
                    break;
                  case ('5'): //ATS25=
                    if (settingValue >= 0 && settingValue <= 255)
                    {
                      settings.maxResends = settingValue;
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
                settings.pointToPoint = settingValue;
                reportOK();
              }
              else
                reportERROR();
              break;
            case ('4'): //ATS4=
              if (settingValue >= 0 && settingValue <= 1)
              {
                settings.encryptData = settingValue;
                reportOK();
              }
              else
                reportERROR();
              break;
            case ('5'): //ATS5=
              if (settingValue >= 0 && settingValue <= 1)
              {
                settings.dataScrambling = settingValue;
                reportOK();
              }
              else
                reportERROR();
              break;
            case ('6'): //ATS6=
              if (settingValue >= 0 && settingValue <= 20)
              {
                settings.radioBroadcastPower_dbm = settingValue;
                reportOK();
              }
              else
                reportERROR();
              break;
            case ('7'): //ATS7=
              if (doubleSettingValue >= 902.0 && doubleSettingValue <= settings.frequencyMax)
              {
                settings.frequencyMin = doubleSettingValue;
                reportOK();
              }
              else
                reportERROR();
              break;
            case ('8'): //ATS8=
              if (doubleSettingValue >= settings.frequencyMin && doubleSettingValue <= 928.0)
              {
                settings.frequencyMax = doubleSettingValue;
                reportOK();
              }
              else
                reportERROR();
              break;
            case ('9'): //ATS9=
              if (settingValue >= 1 && settingValue <= 50)
              {
                settings.numberOfChannels = settingValue;
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
        {
          reportERROR();
        }

      } //End isATcommand()
      else
      {
        reportERROR();
      }

    } //commandSize >= 2
    else
    {
      reportERROR();
    }
  } //while == true
} //End commandMode()

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

//Check if RT appears in the correct position
//bool isRTcommand()
//{
//  if (commandBuffer[0] == 'r' || commandBuffer[0] == 'R')
//    if (commandBuffer[1] == 't' || commandBuffer[1] == 'T')
//      return (true);
//  return (false);
//}

//Reads a line until the \n enter character is found
byte readLine(char* readBuffer, byte bufferLength)
{
  memset(readBuffer, 0, bufferLength); //Clear buffer

  byte readLength = 0;
  while (readLength < bufferLength - 1)
  {
#if defined(ARDUINO_ARCH_SAMD)
    while (!Serial.available() && !Serial1.available())
#else
    while (!Serial.available())
#endif
    {
      petWDT();
      delay(10);
    }

    byte c = systemRead();
    systemWrite(c);

    if (c == '\r') {
      systemPrintln();
      readBuffer[readLength] = '\0';
      break;
    }
    else if (c == '\n') {
      //Do nothing - ignore newlines
    }
    else {
      readBuffer[readLength] = c;
      ++readLength;
    }
  }

  return readLength;
}

//Show current settings in user friendly way
void displayParameters()
{
  for (uint8_t x = 0 ; x <= 25 ; x++)
  {
    systemPrint(F("S"));
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
        systemPrint(settings.serialSpeed);
        break;
      case (1):
        systemPrint(settings.airSpeed);
        break;
      case (2):
        systemPrint(settings.netID);
        break;
      case (3):
        systemPrint(settings.pointToPoint);
        break;
      case (4):
        systemPrint(settings.encryptData);
        break;
      case (5):
        systemPrint(settings.dataScrambling);
        break;
      case (6):
        systemPrint(settings.radioBroadcastPower_dbm);
        break;
      case (7):
        systemPrint(settings.frequencyMin, 3);
        break;
      case (8):
        systemPrint(settings.frequencyMax, 3);
        break;
      case (9):
        systemPrint(settings.numberOfChannels);
        break;
      case (10):
        systemPrint(settings.frequencyHop);
        break;
      case (11):
        systemPrint(settings.maxDwellTime);
        break;
      case (12):
        systemPrint(settings.radioBandwidth, 2);
        break;
      case (13):
        systemPrint(settings.radioSpreadFactor);
        break;
      case (14):
        systemPrint(settings.radioCodingRate);
        break;
      case (15):
        systemPrint(settings.radioSyncWord);
        break;
      case (16):
        systemPrint(settings.radioPreambleLength);
        break;
      case (17):
        systemPrint(settings.frameSize);
        break;
      case (18):
        systemPrint(settings.serialTimeoutBeforeSendingFrame_ms);
        break;
      case (19):
        systemPrint(settings.debug);
        break;
      case (20):
        systemPrint(settings.echo);
        break;
      case (21):
        systemPrint(settings.heartbeatTimeout);
        break;
      case (22):
        systemPrint(settings.flowControl);
        break;
      case (23):
        systemPrint(settings.autoTuneFrequency);
        break;
      case (24):
        systemPrint(settings.displayPacketQuality);
        break;
      case (25):
        systemPrint(settings.maxResends);
        break;
      default:
        systemPrint(F("0"));
        break;
    }

    systemPrintln();
  }
}
