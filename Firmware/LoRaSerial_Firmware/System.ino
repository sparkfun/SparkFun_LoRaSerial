//Copy the string into the serialTransmitBuffer or command response buffer (commandTXBuffer)
void systemPrint(const char* string)
{
  uint16_t length;

  length = strlen(string);
  if (printerEndpoint == PRINT_TO_SERIAL)
  {
    for (uint16_t x = 0 ; x < length ; x++)
      serialOutputByte(string[x]);
  }
  else if (printerEndpoint == PRINT_TO_RF)
  {
    //Move these characters into the transmit buffer
    for (uint16_t x = 0 ; x < length ; x++)
    {
      commandTXBuffer[commandTXHead++] = string[x];
      commandTXHead %= sizeof(commandTXBuffer);
    }
  }
}

//Print a string with a carriage return and linefeed
void systemPrintln(const char* value)
{
  systemPrint(value);
  systemPrint("\r\n");
}

//Print an integer value
void systemPrint(int value)
{
  char temp[20];
  sprintf(temp, "%d", value);
  systemPrint(temp);
}

//Print an integer value as HEX or decimal
void systemPrint(int value, uint8_t printType)
{
  char temp[20];

  if (printType == HEX)
    sprintf(temp, "%08x", value);
  else if (printType == DEC)
    sprintf(temp, "%d", value);

  systemPrint(temp);
}

//Print an integer value with a carriage return and line feed
void systemPrintln(int value)
{
  systemPrint(value);
  systemPrint("\r\n");
}

//Print an 8-bit value as HEX or decimal
void systemPrint(uint8_t value, uint8_t printType)
{
  char temp[20];

  if (printType == HEX)
    sprintf(temp, "%02X", value);
  else if (printType == DEC)
    sprintf(temp, "%d", value);

  systemPrint(temp);
}

//Print an 8-bit value as HEX or decimal with a carriage return and linefeed
void systemPrintln(uint8_t value, uint8_t printType)
{
  systemPrint(value, printType);
  systemPrint("\r\n");
}

//Print a 16-bit value as HEX or decimal
void systemPrint(uint16_t value, uint8_t printType)
{
  char temp[20];

  if (printType == HEX)
    sprintf(temp, "%04X", value);
  else if (printType == DEC)
    sprintf(temp, "%d", value);

  systemPrint(temp);
}

//Print a 16-bit value as HEX or decimal with a carriage return and linefeed
void systemPrintln(uint16_t value, uint8_t printType)
{
  systemPrint(value, printType);
  systemPrint("\r\n");
}

//Print a floating point value with a specified number of decimal places
void systemPrint(float value, uint8_t decimals)
{
  char temp[20];
  sprintf(temp, "%.*f", decimals, value);
  systemPrint(temp);
}

//Print a floating point value with a specified number of decimal places and a
//carriage return and linefeed
void systemPrintln(float value, uint8_t decimals)
{
  systemPrint(value, decimals);
  systemPrint("\r\n");
}

//Print a double precision floating point value with a specified number of decimal
//places
void systemPrint(double value, uint8_t decimals)
{
  char temp[300];
  sprintf(temp, "%.*g", decimals, value);
  systemPrint(temp);
}

//Print a double precision floating point value with a specified number of decimal
//places and a carriage return and linefeed
void systemPrintln(double value, uint8_t decimals)
{
  systemPrint(value, decimals);
  systemPrint("\r\n");
}

//Print a carriage return and linefeed
void systemPrintln()
{
  systemPrint("\r\n");
}

//Print a timestamp value: days hours:minutes:seconds.milliseconds
void systemPrintTimestamp(unsigned int milliseconds)
{
  unsigned int seconds;
  unsigned int minutes;
  unsigned int hours;
  unsigned int days;
  unsigned int total;

  petWDT();

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
  petWDT();
}

//Print a timestamp value with an offset
void systemPrintTimestamp()
{
  unsigned int milliseconds;

  if (settings.printTimestamp)
  {
    //Get the clock value
    milliseconds = millis();

    //Offset the value for display
    if (!settings.displayRealMillis)
      milliseconds += timestampOffset;

    //Print the time
    systemPrintTimestamp(milliseconds);
    systemPrint(": ");
  }
}

//Print the unique ID value
void systemPrintUniqueID(uint8_t * uniqueID)
{
  int index;

  //Display in the same byte order as dump output
  for (index = 0; index < UNIQUE_ID_BYTES; index++)
    systemPrint(uniqueID[index], HEX);
}

//Output a byte to the serial port
void systemWrite(uint8_t value)
{
  serialOutputByte(value);
}

//Output a buffer of the specified length to the serial port
void systemWrite(uint8_t * buffer, uint16_t length)
{
  uint8_t * end;

  //Output the entire buffer ignoring contents
  end = &buffer[length];
  while (buffer < end)
    serialOutputByte(*buffer++);
}

//Ensure all serial output has been transmitted, FIFOs are empty
void systemFlush()
{
  arch.serialFlush();
}

//Read a byte from the serial port
uint8_t systemRead()
{
  return (arch.serialRead());
}

//Check the train button and change state accordingly
void updateButton()
{
  static TrainStates trainState = TRAIN_NO_PRESS;

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

      selectTraining();

      trainState = TRAIN_IN_PROCESS;
    }
    else if (trainState == TRAIN_IN_PROCESS && trainBtn->wasReleased())
    {
      //Exiting training
      setRSSI(0b0000); //Turn off LEDs

      //Reboot the radio
      petWDT();
      systemFlush();
      systemReset();
    }

    //Blink LEDs according to our state while we wait for user to release button
    if (trainState == TRAIN_PRESSED_2S)
    {
      if (millis() - lastTrainBlink > 500) //Slow blink
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

//Display any debug serial data and then loop forever
void waitForever()
{
  //Output the remaining serial data
  outputSerialData(true);

  //Empty the USB serial device
  systemFlush();

  //Wait forever
  while (1)
    petWDT();
}

//Encrypt a given array in place
void encryptBuffer(uint8_t *bufferToEncrypt, uint8_t arraySize)
{
  gcm.setKey(settings.encryptionKey, sizeof(settings.encryptionKey));
  gcm.setIV(AESiv, sizeof(AESiv));

  gcm.encrypt(bufferToEncrypt, bufferToEncrypt, arraySize);
}

//Decrypt a given array in place
void decryptBuffer(uint8_t *bufferToDecrypt, uint8_t arraySize)
{
  gcm.setKey(settings.encryptionKey, sizeof(settings.encryptionKey));
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
    if (timeToHop == true) //If the channelTimer has expired, move to next frequency
      hopChannel();
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

  //Determine if the trigger pin is enabled
  if (pin_trigger == PIN_UNDEFINED)
    return;

  //Determine which trigger enable to use
  triggerBitNumber = triggerNumber;
  triggerEnable = settings.triggerEnable;
  if (triggerNumber >= 32)
  {
    triggerBitNumber -= 32;
    triggerEnable = settings.triggerEnable2;
  }

  //Determine if the trigger is enabled
  if (triggerEnable & (1 << triggerBitNumber))
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

//Dump a buffer with offset from buffer start, 16 bytes per row, displaying hex and ASCII
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
      if (timeToHop == true) //If the channelTimer has expired, move to next frequency
        hopChannel();
      petWDT();
    }

    // Space over to the ASCII display
    for (; index < displayWidth; index++)
    {
      systemPrint("   ");
      if (timeToHop == true) //If the channelTimer has expired, move to next frequency
        hopChannel();
      petWDT();
    }
    systemPrint("  ");

    // Display the ASCII bytes
    data -= bytes;
    for (index = 0; index < bytes; index++) {
      byte[0] = *data++;
      systemPrint(((byte[0] < ' ') || (byte[0] >= 0x7f)) ? "." : byte);
    }
    systemPrintln();
    if (timeToHop == true) //If the channelTimer has expired, move to next frequency
      hopChannel();
    petWDT();
  }
}

//Dump a buffer assuming that it contains text
void dumpBufferRaw(uint8_t * data, int length)
{
  systemPrint("0x ");
  for (int x = 0 ; x < length ; x++)
  {
    systemPrint(data[x], HEX);
    systemPrint(" ");
  }
  systemPrintln();
}

//Dump a circular buffer with offset from buffer start, 16 bytes per row, displaying hex and ASCII
void dumpCircularBuffer(uint8_t * buffer, uint16_t tail, uint16_t bufferLength, int length)
{
  int bytes;
  uint8_t data;
  uint16_t delta;
  const int displayWidth = 16;
  uint16_t i;
  int index;
  uint16_t offset;

  offset = tail;
  while (length > 0)
  {
    //Display the offset
    systemPrint("    0x");
    systemPrint((uint16_t)(offset % bufferLength), HEX);
    systemPrint(": ");

    //Determine the number of bytes to display
    bytes = length;
    if (bytes > displayWidth)
      bytes = displayWidth;

    //Adjust for the offset
    delta = offset % displayWidth;
    if (delta)
    {
      bytes -= delta;
      for (index = 0; index < delta; index++)
      {
        systemPrint("   ");
        if (timeToHop == true) //If the channelTimer has expired, move to next frequency
          hopChannel();
        petWDT();
      }
    }

    //Display the data bytes in hex
    for (index = 0; index < bytes; index++)
    {
      systemWrite(' ');
      data = buffer[(offset + index) % bufferLength];
      systemPrint(data, HEX);
      if (timeToHop == true) //If the channelTimer has expired, move to next frequency
        hopChannel();
      petWDT();
    }

    //Space over to the ASCII display
    for (; (delta + index) < displayWidth; index++)
    {
      systemPrint("   ");
      if (timeToHop == true) //If the channelTimer has expired, move to next frequency
        hopChannel();
      petWDT();
    }
    systemPrint("  ");

    //Display the ASCII bytes
    for (index = 0; index < delta; index++)
      systemWrite(' ');
    for (index = 0; index < bytes; index++) {
      data = buffer[(offset + index) % bufferLength];
      systemWrite(((data < ' ') || (data >= 0x7f)) ? '.' : data);
    }
    systemPrintln();
    if (timeToHop == true) //If the channelTimer has expired, move to next frequency
      hopChannel();
    petWDT();
    offset += bytes;
    length -= bytes;
  }
}

//Compute the RSSI value
void updateRSSI()
{
  //Calculate the average RSSI if possible
  if (hopCount > 0)
    rssi /= hopCount;
  else
    rssi = radio.getRSSI();

  if (hopCount > 0)
  {
    //Reset RSSI measurements
    hopCount = 0;
    rssi = 0;
  }
}

//Update the state of the 4 green LEDs
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

//Set the serial TX (blue) LED value
void txLED(bool illuminate)
{
  if (settings.selectLedUse != LEDS_RSSI)
    return;
  if (pin_txLED != PIN_UNDEFINED)
  {
    if (illuminate == true)
      digitalWrite(pin_txLED, HIGH);
    else
      digitalWrite(pin_txLED, LOW);
  }
}

//Set the serial RX (yellow) LED value
void rxLED(bool illuminate)
{
  if (settings.selectLedUse != LEDS_RSSI)
    return;
  if (pin_rxLED != PIN_UNDEFINED)
  {
    if (illuminate == true)
      digitalWrite(pin_rxLED, HIGH);
    else
      digitalWrite(pin_rxLED, LOW);
  }
}

//Radio LED display
// Green1: Radio RX data received
// Green2: Receiving HEARTBEATs (link up)
// Green3: RSSI level
// Green4: Radio TX data sent
// Blue: Bad frame received
// Yellow: Bad CRC received
void radioLeds()
{
  uint32_t currentMillis;
  static uint32_t previousMillis;
  static uint32_t previousBadFrames;
  static uint32_t badFramesMillis;
  static uint32_t previousBadCrc;
  static uint32_t badCrcMillis;
  static int8_t rssiCount = 127;
  static int8_t rssiValue;

  //Turn off the RX LED to end the blink
  currentMillis = millis();
  if ((currentMillis - linkDownTimer) >= ALT_LED_BLINK_MILLIS)
    digitalWrite(ALT_LED_RX_DATA, LED_OFF);

  //Turn off the TX LED to end the blink
  currentMillis = millis();
  if ((currentMillis - datagramTimer) >= ALT_LED_BLINK_MILLIS)
    digitalWrite(ALT_LED_TX_DATA, LED_OFF);

  //Blink the bad frames LED
  if (badFrames != previousBadFrames)
  {
    previousBadFrames = badFrames;
    badFramesMillis = currentMillis;
    digitalWrite(ALT_LED_BAD_FRAMES, LED_ON);
  }
  else if (badFramesMillis && ((currentMillis - badFramesMillis) >= ALT_LED_BLINK_MILLIS))
  {
    badFramesMillis = 0;
    digitalWrite(ALT_LED_BAD_FRAMES, LED_OFF);
  }

  //Blink the bad CRC or duplicate frames LED
  if (settings.enableCRC16 && (badCrc != previousBadCrc))
  {
    previousBadCrc = badCrc;
    badCrcMillis = currentMillis;
    digitalWrite(ALT_LED_BAD_CRC, LED_ON);
  }
  if ((!settings.enableCRC16) && (duplicateFrames != previousBadCrc))
  {
    previousBadCrc = duplicateFrames;
    badCrcMillis = currentMillis;
    digitalWrite(ALT_LED_BAD_CRC, LED_ON);
  }
  else if (badCrcMillis && ((currentMillis - badCrcMillis) >= ALT_LED_BLINK_MILLIS))
  {
    badCrcMillis = 0;
    digitalWrite(ALT_LED_BAD_CRC, LED_OFF);
  }

  //Update the link LED
  switch (radioState)
  {
    //Turn off the LED
    default:
      digitalWrite(ALT_LED_RADIO_LINK, LED_OFF);
      break;

    //Turn on the LED
    case RADIO_P2P_LINK_UP:
    case RADIO_P2P_LINK_UP_WAIT_ACK_DONE:
    case RADIO_P2P_LINK_UP_WAIT_TX_DONE:
    case RADIO_P2P_LINK_UP_WAIT_ACK:
    case RADIO_P2P_LINK_UP_HB_ACK_REXMT:
      digitalWrite(ALT_LED_RADIO_LINK, LED_ON);
      break;
  }

  //Update the RSSI LED
  if (currentMillis != previousMillis)
  {
    if (rssiCount >= 16)
    {
      rssiValue = (150 + rssi) / 10;
      rssiCount = 0;
      if (rssiValue >= rssiCount)
        digitalWrite(ALT_LED_RSSI, LED_ON);
    }

    //Turn off the RSSI LED
    else if (rssiValue < rssiCount++)
      digitalWrite(ALT_LED_RSSI, LED_OFF);
  }

  //Save the last millis value
  previousMillis = currentMillis;
}

//Update the LED values depending upon the selected display
void updateLeds()
{
  switch (settings.selectLedUse)
  {
    //Set LEDs according to RSSI level
    default:
    case 0:
      if (rssi > rssiLevelLow)
        setRSSI(0b0001);
      if (rssi > rssiLevelMed)
        setRSSI(0b0011);
      if (rssi > rssiLevelHigh)
        setRSSI(0b0111);
      if (rssi > rssiLevelMax)
        setRSSI(0b1111);
      break;

    case 1:
      radioLeds();
      break;
    }
}

//Case independent string comparison
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

//Case independent string comparison with specified maximum length
int strnicmp(const char * str1, const char * str2, int length)
{
  char char1;
  char char2;

  //Do a case insensitive comparison between the two strings
  do {
    char1 = toupper(*str1++);
    char2 = toupper(*str2++);
  } while (char1 && (char1 == char2) && --length);

  //Return the difference between the two strings
  return char1 - char2;
}

//Display the RSSI, SNR and frequency error values
void printPacketQuality()
{
  if (settings.displayPacketQuality == true)
  {
    systemPrintln();
    systemPrint("R:");
    systemPrint(rssi);
    systemPrint("\tS:");
    systemPrint(radio.getSNR());
    systemPrint("\tfE:");
    systemPrint(radio.getFrequencyError());
    systemPrintln();
  }
}

//Toggle a pin. Used for logic analyzer debugging.
void triggerFrequency(uint16_t frequency)
{
  digitalWrite(pin_trigger, LOW);
  delayMicroseconds(frequency);
  digitalWrite(pin_trigger, HIGH);
}

//The difference between when a transmitter is finished and when the receiver is finished
//is different between airSpeeds and affects things like ACK timeouts at lower airSpeeds
//Offsets were found using a logic analyzer but it looks like (1 * Tsym) or calcSymbolTime()
int16_t getReceiveCompletionOffset()
{
  switch (settings.airSpeed)
  {
    default:
      break;
    case (40):
      return (26); //Tsym: 32. Measured: 26 ms between a TX complete and the RX complete
      break;
    case (150):
      return (12); //Tsym: 16
      break;
    case (400):
      return (6); //Tsym: 8
      break;
    case (1200):
      return (3); //Tsym: 4
      break;
    case (2400):
      return (1); //Tsym: 2
      break;
    case (4800):
      return (0); //Tsym: 1
      break;
  }
}

//Verify the VC_STATE_TYPE enums against the vcStateNames table
bool verifyVcStateNames()
{
  bool valid;

  //Verify the length of the vcStateNames table
  valid = (VC_STATE_MAX == (sizeof(vcStateNames) / sizeof(vcStateNames[0])));
  if (!valid)
    systemPrintln("ERROR: Fix difference between VC_STATE_TYPE and vcStateNames");
  return valid;
}

//Verify the enums .vs. name tables, stop on failure to force software fix
void verifyTables()
{
  bool valid;

  valid = true;

  //Verify that the state table contains all of the states in increasing order
  valid &= verifyRadioStateTable();

  //Verify that the datagram type table contains all of the datagram types
  valid &= verifyRadioDatagramType();

  //Verify the VC state name table
  valid &= verifyVcStateNames();

  //Verify the RADIO_CALLS enum against the radioCallName
  valid &= verifyRadioCallNames();

  if (!valid)
  {
    outputSerialData(true);
    waitForever();
  }
}
