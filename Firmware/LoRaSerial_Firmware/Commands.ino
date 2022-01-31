//To add a new command:
//Add the response to a query: ATS13?
//Add the data entry and validity check: ATS13=
//Add the setting to displayParameters
//Increase displayParameters loop value

//Respond to AT commands
void commandMode()
{
  char commandBuffer[30];
  Serial.println(F("\r\nOK"));

  while (Serial.available()) Serial.read();

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
              Serial.print(F("SparkFun STR "));
              Serial.print(platformPrefix);
              Serial.print(F(" v"));
              Serial.print(FIRMWARE_VERSION_MAJOR);
              Serial.print(F("."));
              Serial.print(FIRMWARE_VERSION_MINOR);
              Serial.println();
              break;
            case ('O'): //Exit command mode
              generateHopTable(); //Generate freq with new settings
              configureRadio(); //Apply any new settings
              reportOK();
              return;
              break;
            case ('Z'): //Reboots the radio
              reportOK(); //Does SiK reply with OK before reboot or not?
              Serial.flush();
#if defined(ARDUINO_ARCH_SAMD)
              NVIC_SystemReset();
#elif defined(ARDUINO_ARCH_ESP32)
              ESP.restart();
#endif
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
              Serial.print(F("SparkFun STR "));
              Serial.println(platformPrefix);
              break;
            case ('2'): //ATI2 - Show firmware version
              Serial.print(FIRMWARE_VERSION_MAJOR);
              Serial.print(F("."));
              Serial.println(FIRMWARE_VERSION_MINOR);
              break;
            case ('3'): //ATI3 - Display latest RSSI
              Serial.println(radio.getRSSI());
              break;
            case ('4'): //ATI4 - Get random byte from RSSI
              Serial.println(radio.randomByte());
              break;
            case ('5'): //ATI5 - Show max possible bytes per second
              Serial.println(calcMaxThroughput());
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
              case ('T'): //AT&T - Disable debugging report
                settings.displayPacketQuality = false;
                reportOK();
                break;
              default:
                reportERROR();
                break;
            }
          }
          else
          {
            //RSSI
            if (strcmp_P(commandBuffer, PSTR("AT&T=RSSI")) == 0) //Enable debugging report
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
              Serial.println(settings.serialSpeed);
              break;
            case ('1'):
              {
                switch (commandBuffer[4])
                {
                  case ('?'): //ATS1?
                    Serial.println(settings.airSpeed);
                    break;
                  case ('0'): //ATS10?
                    Serial.println(settings.radioPreambleLength);
                    break;
                  case ('1'): //ATS11?
                    Serial.println(settings.frameSize);
                    break;
                  case ('2'): //ATS12?
                    Serial.println(settings.serialTimeoutBeforeSendingFrame_ms);
                    break;
                  case ('3'): //ATS13?
                    Serial.println(settings.debug);
                    break;
                  case ('4'): //ATS14?
                    Serial.println(settings.echo);
                    break;
                  case ('5'): //ATS15?
                    Serial.println(settings.heartbeatTimeout);
                    break;
                  case ('6'): //ATS16?
                    Serial.println(settings.flowControl);
                    break;
                  case ('7'): //ATS17?
                    Serial.println(settings.frequencyHop);
                    break;
                  case ('8'): //ATS18?
                    Serial.println(settings.autoTuneFrequency);
                    break;
                  case ('9'): //ATS19?
                    Serial.println(settings.displayPacketQuality);
                    break;
                  default:
                    reportERROR();
                    break;
                }
              }
              break;
            case ('2'): //ATS2?
              Serial.println(settings.radioBroadcastPower_dbm);
              break;
            case ('3'): //ATS3?
              Serial.println(settings.frequencyMin);
              break;
            case ('4'): //ATS4?
              Serial.println(settings.frequencyMax);
              break;
            case ('5'): //ATS5?
              Serial.println(settings.numberOfChannels);
              break;
            case ('6'): //ATS6?
              Serial.println(settings.radioBandwidth);
              break;
            case ('7'): //ATS7?
              Serial.println(settings.radioSpreadFactor);
              break;
            case ('8'): //ATS8?
              Serial.println(settings.radioCodingRate);
              break;
            case ('9'): //ATS9?
              Serial.println(settings.radioSyncWord);
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
            case ('1'):
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
                      if (settings.airSpeed == 0 && settingValue != 0) Serial.println(F("Warning: AirSpeed override of bandwidth, spread factor, and coding rate"));
                      settings.airSpeed = settingValue;
                      configureRadio(); //Update spread, bandwidth, and coding as needed
                      reportOK();
                    }
                    else
                      reportERROR();
                    break;
                  case ('0'): //ATS10=
                    if (settingValue >= 6 && settingValue <= 65535)
                    {
                      settings.radioPreambleLength = settingValue;
                      reportOK();
                    }
                    else
                      reportERROR();
                    break;
                  case ('1'): //ATS11=
                    if (settingValue >= 16 && settingValue <= (256 - 2)) //NetID+Control trailer is 2 bytes
                    {
                      settings.frameSize = settingValue;
                      reportOK();
                    }
                    else
                      reportERROR();
                    break;
                  case ('2'): //ATS12=
                    if (settingValue >= 10 && settingValue <= 2000)
                    {
                      settings.serialTimeoutBeforeSendingFrame_ms = settingValue;
                      reportOK();
                    }
                    else
                      reportERROR();
                    break;
                  case ('3'): //ATS13=
                    if (settingValue >= 0 && settingValue <= 1)
                    {
                      settings.debug = settingValue;
                      reportOK();
                    }
                    else
                      reportERROR();
                    break;
                  case ('4'): //ATS14=
                    if (settingValue >= 0 && settingValue <= 1)
                    {
                      settings.echo = settingValue;
                      reportOK();
                    }
                    else
                      reportERROR();
                    break;
                  case ('5'): //ATS15=
                    if (settingValue >= 250 && settingValue <= 65535)
                    {
                      settings.heartbeatTimeout = settingValue;
                      reportOK();
                    }
                    else
                      reportERROR();
                    break;
                  case ('6'): //ATS16=
                    if (settingValue >= 0 && settingValue <= 1)
                    {
                      settings.flowControl = settingValue;
                      reportOK();
                    }
                    else
                      reportERROR();
                    break;
                  case ('7'): //ATS17=
                    if (settingValue >= 0 && settingValue <= 1)
                    {
                      settings.frequencyHop = settingValue;
                      reportOK();
                    }
                    else
                      reportERROR();
                    break;
                  case ('8'): //ATS18=
                    if (settingValue >= 0 && settingValue <= 1)
                    {
                      settings.autoTuneFrequency = settingValue;
                      reportOK();
                    }
                    else
                      reportERROR();
                    break;
                  case ('9'): //ATS19=
                    if (settingValue >= 0 && settingValue <= 1)
                    {
                      settings.displayPacketQuality = settingValue;
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
            case ('2'): //ATS2=
              if (settingValue >= 0 && settingValue <= 20)
              {
                settings.radioBroadcastPower_dbm = settingValue;
                reportOK();
              }
              else
                reportERROR();
              break;
            case ('3'): //ATS3=
              if (doubleSettingValue >= 902.0 && doubleSettingValue <= settings.frequencyMax)
              {
                settings.frequencyMin = doubleSettingValue;
                reportOK();
              }
              else
                reportERROR();
              break;
            case ('4'): //ATS4=
              if (doubleSettingValue >= settings.frequencyMin && doubleSettingValue <= 928.0)
              {
                settings.frequencyMax = doubleSettingValue;
                reportOK();
              }
              else
                reportERROR();
              break;
            case ('5'): //ATS5=
              if (settingValue >= 1 && settingValue <= 50)
              {
                settings.numberOfChannels = settingValue;
                reportOK();
              }
              else
                reportERROR();
              break;
            case ('6'): //ATS6=
              if (settings.airSpeed != 0)
              {
                Serial.println(F("AirSpeed is overriding"));
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
            case ('7'): //ATS7=
              if (settingValue >= 6 && settingValue <= 12)
              {
                settings.radioSpreadFactor = settingValue;
                reportOK();
              }
              else
                reportERROR();
              break;
            case ('8'): //ATS8=
              if (settingValue >= 5 && settingValue <= 8)
              {
                settings.radioCodingRate = settingValue;
                reportOK();
              }
              else
                reportERROR();
              break;
            case ('9'): //ATS9=
              if (settingValue >= 0 && settingValue <= 255)
              {
                settings.radioSyncWord = settingValue;
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
  Serial.println(F("OK"));
}

void reportERROR()
{
  Serial.println(F("ERROR"));
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
    while (!Serial.available());
    byte c = Serial.read();

    //toggleLED(stat1);

    // Only echo back if this is enabled
    if (settings.echo == true)
      Serial.write(c);

    if (c == '\r') {
      Serial.println();
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
  for (uint8_t x = 0 ; x <= 19 ; x++)
  {
    Serial.print(F("S"));
    Serial.print(x);
    Serial.print(F(":"));

    //Name
    switch (x)
    {
      case (0):
        Serial.print(F("SerialSpeed"));
        break;
      case (1):
        Serial.print(F("AirSpeed"));
        break;
      case (2):
        Serial.print(F("TxPower"));
        break;
      case (3):
        Serial.print(F("FrequencyMin"));
        break;
      case (4):
        Serial.print(F("FrequencyMax"));
        break;
      case (5):
        Serial.print(F("NumberOfChannels"));
        break;
      case (6):
        Serial.print(F("Bandwidth"));
        break;
      case (7):
        Serial.print(F("SpreadFactor"));
        break;
      case (8):
        Serial.print(F("CodingRate"));
        break;
      case (9):
        Serial.print(F("SyncWord"));
        break;
      case (10):
        Serial.print(F("PreambleLength"));
        break;
      case (11):
        Serial.print(F("FrameSize"));
        break;
      case (12):
        Serial.print(F("FrameTimeout"));
        break;
      case (13):
        Serial.print(F("Debug"));
        break;
      case (14):
        Serial.print(F("Echo"));
        break;
      case (15):
        Serial.print(F("HeartBeatTimeout"));
        break;
      case (16):
        Serial.print(F("FlowControl"));
        break;
      case (17):
        Serial.print(F("FrequencyHop"));
        break;
      case (18):
        Serial.print(F("AutoTune"));
        break;
      case (19):
        Serial.print(F("DisplayPacketQuality"));
        break;
      default:
        Serial.print(F("Unknown"));
        break;
    }

    Serial.print(F("="));

    //Value
    switch (x)
    {
      case (0):
        Serial.print(settings.serialSpeed);
        break;
      case (1):
        Serial.print(settings.airSpeed);
        break;
      case (2):
        Serial.print(settings.radioBroadcastPower_dbm);
        break;
      case (3):
        Serial.print(settings.frequencyMin, 3);
        break;
      case (4):
        Serial.print(settings.frequencyMax, 3);
        break;
      case (5):
        Serial.print(settings.numberOfChannels);
        break;
      case (6):
        Serial.print(settings.radioBandwidth);
        break;
      case (7):
        Serial.print(settings.radioSpreadFactor);
        break;
      case (8):
        Serial.print(settings.radioCodingRate);
        break;
      case (9):
        Serial.print(settings.radioSyncWord);
        break;
      case (10):
        Serial.print(settings.radioPreambleLength);
        break;
      case (11):
        Serial.print(settings.frameSize);
        break;
      case (12):
        Serial.print(settings.serialTimeoutBeforeSendingFrame_ms);
        break;
      case (13):
        Serial.print(settings.debug);
        break;
      case (14):
        Serial.print(settings.echo);
        break;
      case (15):
        Serial.print(settings.heartbeatTimeout);
        break;
      case (16):
        Serial.print(settings.flowControl);
        break;
      case (17):
        Serial.print(settings.frequencyHop);
        break;
      case (18):
        Serial.print(settings.autoTuneFrequency);
        break;
      case (19):
        Serial.print(settings.displayPacketQuality);
        break;
      default:
        Serial.print(F("0"));
        break;
    }

    Serial.println();
  }
}

//Toggle a pin. Used for logic analyzer debugging.
void triggerEvent(uint16_t triggerWidth)
{
  if (pin_trigger != 255)
  {
    if (settings.debug == true)
    {
      digitalWrite(pin_trigger, LOW);
      delayMicroseconds(triggerWidth);
      digitalWrite(pin_trigger, HIGH);
    }
  }
}
