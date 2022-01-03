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
              configureRadio(); //Apply any new settings
              reportOK();
              return;
              break;
            case ('Z'): //Reboots the radio
              reportOK(); //Does SiK reply with OK before reboot or not?
              Serial.flush();
#if defined(ARDUINO_AVR_UNO)
              Reset_AVR();
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
              case ('T'): //AT&T -
                Serial.println(F("AT&T"));
                reportOK();
                break;
              default:
                reportERROR();
                break;
            }
          }
          else
          {
            //RSSI and TDM
            if (strcmp_P(commandBuffer, PSTR("AT&T=RSSI")) == 0)
            {
              Serial.println(F("RSSI Debug"));
              reportOK();
            }
            else if (strcmp_P(commandBuffer, PSTR("AT&T=TDM")) == 0)
            {
              Serial.println(F("TDM Debug"));
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
                    Serial.println(settings.serialTimeoutBeforeSendingFrame_ms);
                    break;
                  case ('1'): //ATS11?
                    Serial.println(settings.debug);
                    break;
                  case ('2'): //ATS12?
                    Serial.println(settings.echo);
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
              Serial.println(settings.radioFrequency);
              break;
            case ('4'): //ATS4?
              Serial.println(settings.radioBandwidth);
              break;
            case ('5'): //ATS5?
              Serial.println(settings.radioSpreadFactor);
              break;
            case ('6'): //ATS6?
              Serial.println(settings.radioCodingRate);
              break;
            case ('7'): //ATS7?
              Serial.println(settings.radioSyncWord);
              break;
            case ('8'): //ATS8?
              Serial.println(settings.radioPreambleLength);
              break;
            case ('9'): //ATS9?
              Serial.println(settings.frameSize);
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
                        || settingValue == 90
                        || settingValue == 150
                        || settingValue == 300
                        || settingValue == 1200
                        || settingValue == 2400
                        || settingValue == 4800
                        || settingValue == 9600
                        || settingValue == 14400
                        || settingValue == 19200
                        || settingValue == 28800
                        || settingValue == 38400
                       )
                    {
                      if(settings.airSpeed == 0 && settingValue != 0) Serial.println(F("Warning: AirSpeed override of bandwidth, spread factor, and coding rate"));
                      settings.airSpeed = settingValue;
                      configureRadio(); //Update spread, bandwidth, and coding as needed
                      reportOK();
                    }
                    else
                      reportERROR();
                    break;
                  case ('0'): //ATS10=
                    if (settingValue >= 10 && settingValue <= 2000)
                    {
                      settings.serialTimeoutBeforeSendingFrame_ms = settingValue;
                      reportOK();
                    }
                    else
                      reportERROR();
                    break;
                  case ('1'): //ATS11=
                    if (settingValue >= 0 && settingValue <= 1)
                    {
                      settings.debug = settingValue;
                      reportOK();
                    }
                    else
                      reportERROR();
                    break;
                  case ('2'): //ATS12=
                    if (settingValue >= 0 && settingValue <= 1)
                    {
                      settings.echo = settingValue;
                      reportOK();
                    }
                    else
                      reportERROR();
                    break;
                  case ('3'): //ATS13=
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
              if (doubleSettingValue >= 137.0 && doubleSettingValue <= 1020.0)
              {
                settings.radioFrequency = doubleSettingValue;
                reportOK();
              }
              else
                reportERROR();
              break;
            case ('4'): //ATS4=
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
            case ('5'): //ATS5=
              if (settingValue >= 6 && settingValue <= 12)
              {
                settings.radioSpreadFactor = settingValue;
                reportOK();
              }
              else
                reportERROR();
              break;
            case ('6'): //ATS6=
              if (settingValue >= 5 && settingValue <= 8)
              {
                settings.radioCodingRate = settingValue;
                reportOK();
              }
              else
                reportERROR();
              break;
            case ('7'): //ATS7=
              if (settingValue >= 0 && settingValue <= 255)
              {
                settings.radioSyncWord = settingValue;
                reportOK();
              }
              else
                reportERROR();
              break;
            case ('8'): //ATS8=
              if (settingValue >= 6 && settingValue <= 65535)
              {
                settings.radioPreambleLength = settingValue;
                reportOK();
              }
              else
                reportERROR();
              break;
            case ('9'): //ATS9=
              if (settingValue >= 16 && settingValue <= 255)
              {
                settings.frameSize = settingValue;
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
  for (uint8_t x = 0 ; x <= 12 ; x++)
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
        Serial.print(F("Frequency"));
        break;
      case (4):
        Serial.print(F("Bandwidth"));
        break;
      case (5):
        Serial.print(F("SpreadFactor"));
        break;
      case (6):
        Serial.print(F("CodingRate"));
        break;
      case (7):
        Serial.print(F("SyncWord"));
        break;
      case (8):
        Serial.print(F("PreambleLength"));
        break;
      case (9):
        Serial.print(F("FrameSize"));
        break;
      case (10):
        Serial.print(F("FrameTimeout"));
        break;
      case (11):
        Serial.print(F("Debug"));
        break;
      case (12):
        Serial.print(F("Echo"));
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
        Serial.print(settings.radioFrequency, 1);
        break;
      case (4):
        Serial.print(settings.radioBandwidth);
        break;
      case (5):
        Serial.print(settings.radioSpreadFactor);
        break;
      case (6):
        Serial.print(settings.radioCodingRate);
        break;
      case (7):
        Serial.print(settings.radioSyncWord);
        break;
      case (8):
        Serial.print(settings.radioPreambleLength);
        break;
      case (9):
        Serial.print(settings.frameSize);
        break;
      case (10):
        Serial.print(settings.serialTimeoutBeforeSendingFrame_ms);
        break;
      case (11):
        Serial.print(settings.debug);
        break;
      case (12):
        Serial.print(settings.echo);
        break;
      default:
        Serial.print(F("0"));
        break;
    }

    Serial.println();
  }
}

//Given spread factor, bandwidth, coding rate and frame size, return most bytes we can push per second
//Formula from SX1726 datasheet. Reported values are close, but not exact to spreadsheet due to limited precision.
uint16_t calcMaxThroughput()
{
  float tSym = pow(2, settings.radioSpreadFactor) / settings.radioBandwidth;
  float tPreamble = (settings.radioPreambleLength + 4.25) * tSym;
  float p1 = (8 * settings.frameSize - 4 * settings.radioSpreadFactor + 28 + 16 * 1 - 20 * 0) / (4.0 * (settings.radioSpreadFactor - 2 * 0));
  p1 = ceil(p1) * settings.radioCodingRate;
  if (p1 < 0) p1 = 0;
  uint16_t payloadBytes = 8 + p1;
  float tPayload = payloadBytes * tSym;
  float tPacket = tPreamble + tPayload;
  uint8_t mostFramesPerSecond = 1000 / tPacket;
  uint16_t mostBytesPerSecond = settings.frameSize * mostFramesPerSecond;

  if (settings.debug == true)
  {
    Serial.print(F("tSym: "));
    Serial.println(tSym, 2);
    Serial.print(F("tPreamble: "));
    Serial.println(tPreamble, 2);
    Serial.print(F("payloadBytes: "));
    Serial.println(payloadBytes);
    Serial.print(F("tPayload: "));
    Serial.println(tPayload, 2);
    Serial.print(F("tPacket: "));
    Serial.println(tPacket, 2);
    Serial.print(F("mostFramesPerSecond: "));
    Serial.println(mostFramesPerSecond);
    Serial.print(F("mostBytesPerSecond: "));
    Serial.println(mostBytesPerSecond);
  }

  return (mostBytesPerSecond);
}
