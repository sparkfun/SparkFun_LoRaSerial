void systemPrint(const char* value)
{
  if (printerEndpoint == PRINT_TO_SERIAL)
  {
    arch.serialPrint(value);
  }
  else if (printerEndpoint == PRINT_TO_RF)
  {
    //Move these characters into the transmit buffer
    for (uint16_t x = 0 ; x < strlen(value) ; x++)
    {
      commandTXBuffer[commandTXHead++] = value[x];
      commandTXHead %= sizeof(commandTXBuffer);
    }
  }
}

void systemPrintln(const char* value)
{
  systemPrint(value);
  systemPrint("\r\n");
}

void systemPrint(int value)
{
  char temp[20];
  sprintf(temp, "%d", value);
  systemPrint(temp);
}

void systemPrint(int value, uint8_t printType)
{
  char temp[20];

  if (printType == HEX)
    sprintf(temp, "%08x", value);
  else if (printType == DEC)
    sprintf(temp, "%d", value);

  systemPrint(temp);
}

void systemPrintln(int value)
{
  systemPrint(value);
  systemPrint("\r\n");
}

void systemPrint(uint8_t value, uint8_t printType)
{
  char temp[20];

  if (printType == HEX)
    sprintf(temp, "%02X", value);
  else if (printType == DEC)
    sprintf(temp, "%d", value);

  systemPrint(temp);
}

void systemPrintln(uint8_t value, uint8_t printType)
{
  systemPrint(value, printType);
  systemPrint("\r\n");
}

void systemPrint(float value, uint8_t decimals)
{
  char temp[20];
  sprintf(temp, "%.*f", decimals, value);
  systemPrint(temp);
}

void systemPrintln(float value, uint8_t decimals)
{
  systemPrint(value, decimals);
  systemPrint("\r\n");
}

void systemPrint(double value, uint8_t decimals)
{
  char temp[300];
  sprintf(temp, "%.*g", decimals, value);
  systemPrint(temp);
}

void systemPrintln(double value, uint8_t decimals)
{
  systemPrint(value, decimals);
  systemPrint("\r\n");
}

void systemPrintln()
{
  systemPrint("\r\n");
}

void systemPrintTimestamp()
{
  unsigned int milliseconds;
  unsigned int seconds;
  unsigned int minutes;
  unsigned int hours;
  unsigned int days;
  unsigned int total;

  petWDT();
  if (settings.printTimestamp)
  {
    //Get the clock value
    milliseconds = millis();

    //Compute the values for display
    seconds = milliseconds / 1000;
    minutes = seconds / 60;
    hours = minutes / 60;
    days = hours / 24;

    total = days * 24;
    hours -= total;

    total = (total + hours) * 60;
    minutes -= total;

    total = (total + minutes) * 60;
    seconds -= total;

    total = (total + seconds) * 1000;
    milliseconds -= total;

    //Print the days
    if (days < 10) systemPrint(" ");
    if (days)
      systemPrint(days);
    else
      systemPrint(" ");
    systemPrint(" ");

    //Print the time
    if (hours < 10) systemPrint(" ");
    systemPrint(hours);
    systemPrint(":");
    if (minutes < 10) systemPrint("0");
    systemPrint(minutes);
    systemPrint(":");
    if (seconds < 10) systemPrint("0");
    systemPrint(seconds);
    systemPrint(".");
    if (milliseconds < 100) systemPrint("0");
    if (milliseconds < 10) systemPrint("0");
    systemPrint(milliseconds);
    systemPrint(": ");
    petWDT();
  }
}

void systemPrintUniqueID(uint8_t * uniqueID)
{
  int index;

  //Display in the same byte order as dump output
  for (index = 0; index < UNIQUE_ID_BYTES; index++)
    systemPrint(uniqueID[index], HEX);
}

void systemWrite(uint8_t value)
{
  arch.serialWrite(value);
}

void systemFlush()
{
  arch.serialFlush();
}

uint8_t systemRead()
{
  return (arch.serialRead());
}

//Check the train button and change state accordingly
void updateButton()
{
  if (trainBtn != NULL)
  {
    trainBtn->read();

    if (trainState == TRAIN_NO_PRESS && trainBtn->pressedFor(trainButtonTime))
    {
      trainState = TRAIN_PRESSED_2S;
      lastTrainBlink = millis();
    }
    else if (trainState == TRAIN_PRESSED_2S && trainBtn->wasReleased())
    {
      setRSSI(0b1111);

      beginTraining();

      trainState = TRAIN_NO_PRESS;
    }
    else if (trainState == TRAIN_PRESSED_2S && trainBtn->pressedFor(trainWithDefaultsButtonTime))
    {
      trainState = TRAIN_PRESSED_5S;
    }
    else if (trainState == TRAIN_PRESSED_5S && trainBtn->wasReleased())
    {
      setRSSI(0b1111);

      beginDefaultTraining();

      trainState = TRAIN_NO_PRESS;
    }

    //Blink LEDs according to our state while we wait for user to release button
    if (trainState == TRAIN_PRESSED_2S)
    {
      if (millis() - lastTrainBlink > 500) //Slow blink
      {
        Serial.println("Train Blinking LEDs");
        lastTrainBlink = millis();

        //Toggle RSSI LEDs
        if (digitalRead(pin_rssi1LED) == HIGH)
          setRSSI(0);
        else
          setRSSI(0b1111);
      }
    }
    else if (trainState == TRAIN_PRESSED_5S)
    {
      if (millis() - lastTrainBlink > 100) //Fast blink
      {
        lastTrainBlink = millis();

        //Toggle RSSI LEDs
        if (digitalRead(pin_rssi1LED) == HIGH)
          setRSSI(0);
        else
          setRSSI(0b1111);
      }
    }
  }
}

//Platform specific reset commands
void systemReset()
{
  arch.systemReset();
}

//Encrypt a given array in place
void encryptBuffer(uint8_t *bufferToEncrypt, uint8_t arraySize)
{
  gcm.setKey(settings.encryptionKey, gcm.keySize());
  gcm.setIV(AESiv, sizeof(AESiv));

  gcm.encrypt(bufferToEncrypt, bufferToEncrypt, arraySize);
}

//Decrypt a given array in place
void decryptBuffer(uint8_t *bufferToDecrypt, uint8_t arraySize)
{
  gcm.setKey(settings.encryptionKey, gcm.keySize());
  gcm.setIV(AESiv, sizeof(AESiv));

  gcm.decrypt(bufferToDecrypt, bufferToDecrypt, arraySize);
}


//IBM Whitening process from Semtech "Software Whitening and CRC on SX12xx Devices" app note
//Removes DC Bias from data with long strings of 1s or 0s
void radioComputeWhitening(uint8_t *buffer, uint16_t bufferSize)
{
  uint8_t WhiteningKeyMSB = 0x01;
  uint8_t WhiteningKeyLSB = 0xFF;
  uint8_t WhiteningKeyMSBPrevious = 0;

  for (uint16_t j = 0 ; j < bufferSize ; j++)
  {
    buffer[j] ^= WhiteningKeyLSB;

    for (uint8_t i = 0 ; i < 8 ; i++)
    {
      WhiteningKeyMSBPrevious = WhiteningKeyMSB;
      WhiteningKeyMSB = (WhiteningKeyLSB & 0x01) ^ ((WhiteningKeyLSB >> 5) & 0x01);
      WhiteningKeyLSB = ((WhiteningKeyLSB >> 1 ) & 0xFF) | ((WhiteningKeyMSBPrevious << 7) & 0x80);
    }
  }
}

//Toggle a pin. Used for logic analyzer debugging.
void triggerEvent(uint8_t triggerNumber)
{
  uint8_t triggerBitNumber;
  uint32_t triggerEnable;
  uint16_t triggerWidth;

  //Determine which trigger enable to use
  triggerBitNumber = triggerNumber;
  triggerEnable = settings.triggerEnable;
  if (triggerNumber >= 32)
  {
    triggerBitNumber -= 32;
    triggerEnable = settings.triggerEnable2;
  }

  //Determine if the trigger is enabled
  if ((pin_trigger != PIN_UNDEFINED) && (triggerEnable & (1 << triggerBitNumber)))
  {
    //Determine if the trigger pin is enabled
    if (pin_trigger != PIN_UNDEFINED)
    {
      if ((settings.debug == true) || (settings.debugTrigger == true))
      {
        //Determine the trigger pulse width
        triggerWidth = settings.triggerWidth;
        if (settings.triggerWidthIsMultiplier)
          triggerWidth *= (triggerNumber + 1);

        //Output the trigger pulse
        digitalWrite(pin_trigger, LOW);
        delayMicroseconds(triggerWidth);
        digitalWrite(pin_trigger, HIGH);
      }
    }
  }
}

//Copy the contents of settings struct to outgoing array
//Note: All variables in struct_settings must be fully cast (uint16_t, int32_t, etc, not int)
//so that we will have alignment between radios using different platforms (ie ESP32 vs SAMD)
void moveSettingsToPacket(Settings settings, uint8_t* packetBuffer)
{
  //Get a byte pointer that points to the beginning of the struct
  uint8_t *bytePtr = (uint8_t*)&settings;

  for (uint8_t x = 0 ; x < sizeof(settings) ; x++)
    packetBuffer[x] = bytePtr[x];
}

//Used to move an incoming packet into the remoteSettings temp buffer
void movePacketToSettings(Settings settings, uint8_t* packetBuffer)
{
  // Get a byte pointer that points to the beginning of the struct
  uint8_t *bytePtr = (uint8_t*)&settings;

  for (uint8_t x = 0 ; x < sizeof(settings) ; x++)
    bytePtr[x] = packetBuffer[x];
}

//Convert ASCII character to base 16
int8_t charToHex(char a)
{
  a = toupper(a);

  if ('0' <= a && a <= '9') a -= '0';
  else if ('A' <= a && a <= 'F') a = a - 'A' + 10;
  else a = -1;
  return a;
}

//Given two letters, convert to base 10
uint8_t charHexToDec(char a, char b)
{
  a = charToHex(a);
  b = charToHex(b);
  return ((a << 4) | b);
}

void dumpBuffer(uint8_t * data, int length)
{
  char byte[2];
  int bytes;
  uint8_t * dataEnd;
  uint8_t * dataStart;
  const int displayWidth = 16;
  int index;

  byte[1] = 0;
  dataStart = data;
  dataEnd = &data[length];
  while (data < dataEnd)
  {
    // Display the offset
    systemPrint("    0x");
    systemPrint((uint8_t)(data - dataStart), HEX);
    systemPrint(": ");

    // Determine the number of bytes to display
    bytes = dataEnd - data;
    if (bytes > displayWidth)
      bytes = displayWidth;

    // Display the data bytes in hex
    for (index = 0; index < bytes; index++)
    {
      systemPrint(" ");
      systemPrint(*data++, HEX);
      petWDT();
    }

    // Space over to the ASCII display
    for (; index < displayWidth; index++)
    {
      systemPrint("   ");
      petWDT();
    }
    systemPrint("  ");

    // Display the ASCII bytes
    data -= bytes;
    for (index = 0; index < bytes; index++) {
      byte[0] = *data++;
      systemPrint(((byte[0] <= ' ') || (byte[0] >= 0x7f)) ? "." : byte);
    }
    systemPrintln();
    petWDT();
  }
}

void updateRSSI()
{
  //RSSI must be above these negative numbers for LED to illuminate
  const int rssiLevelLow = -150;
  const int rssiLevelMed = -70;
  const int rssiLevelHigh = -50;
  const int rssiLevelMax = -20;

  int rssi = radio.getRSSI();

  //Set LEDs according to RSSI level
  if (rssi > rssiLevelLow)
    setRSSI(0b0001);
  if (rssi > rssiLevelMed)
    setRSSI(0b0011);
  if (rssi > rssiLevelHigh)
    setRSSI(0b0111);
  if (rssi > rssiLevelMax)
    setRSSI(0b1111);
}

void setRSSI(uint8_t ledBits)
{
  if (ledBits & 0b0001)
    digitalWrite(pin_rssi1LED, HIGH);
  else
    digitalWrite(pin_rssi1LED, LOW);

  if (ledBits & 0b0010)
    digitalWrite(pin_rssi2LED, HIGH);
  else
    digitalWrite(pin_rssi2LED, LOW);

  if (ledBits & 0b0100)
    digitalWrite(pin_rssi3LED, HIGH);
  else
    digitalWrite(pin_rssi3LED, LOW);

  if (ledBits & 0b1000)
    digitalWrite(pin_rssi4LED, HIGH);
  else
    digitalWrite(pin_rssi4LED, LOW);
}

void txLED(bool illuminate)
{
  if (pin_txLED != PIN_UNDEFINED)
  {
    if (illuminate == true)
      digitalWrite(pin_txLED, HIGH);
    else
      digitalWrite(pin_txLED, LOW);
  }
}

void rxLED(bool illuminate)
{
  if (pin_rxLED != PIN_UNDEFINED)
  {
    if (illuminate == true)
      digitalWrite(pin_rxLED, HIGH);
    else
      digitalWrite(pin_rxLED, LOW);
  }
}

int stricmp(const char * str1, const char * str2)
{
  char char1;
  char char2;

  //Do a case insensitive comparison between the two strings
  do {
    char1 = toupper(*str1++);
    char2 = toupper(*str2++);
  } while (char1 && (char1 == char2));

  //Return the difference between the two strings
  return char1 - char2;
}
