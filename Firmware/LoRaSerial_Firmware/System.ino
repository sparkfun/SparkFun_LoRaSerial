void systemPrint(const char* value)
{
  if (printerEndpoint == PRINT_TO_SERIAL)
  {
    Serial.print(value);

#if defined(ARDUINO_ARCH_SAMD)
    Serial1.print(value);
#endif
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

void systemPrintln()
{
  systemPrint("\r\n");
}

void systemWrite(uint8_t value)
{
  Serial.write(value);

#if defined(ARDUINO_ARCH_SAMD)
  Serial1.write(value);
#endif
}

void systemFlush()
{
  Serial.flush();

#if defined(ARDUINO_ARCH_SAMD)
  Serial1.flush();
#endif
}

uint8_t systemRead()
{
  byte incoming = 0;
#if defined(ARDUINO_ARCH_SAMD)
  if (Serial.available())
    incoming = Serial.read();
  else if (Serial1.available())
    incoming = Serial1.read();
#else
  incoming = Serial.read();
#endif
  return (incoming);
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
#if defined(ARDUINO_ARCH_SAMD)
  NVIC_SystemReset();
#elif defined(ARDUINO_ARCH_ESP32)
  ESP.restart();
#endif
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

//Given two letters, convert to base 10
uint8_t charToHex(char a, char b)
{
  a = toupper(a);
  b = toupper(b);

  if ('0' <= a && a <= '9') a -= '0';
  else if ('A' <= a && a <= 'F') a = a - 'A' + 10;
  else return 0;

  if ('0' <= b && b <= '9') b -= '0';
  else if ('A' <= b && b <= 'F') b = b - 'A' + 10;
  else return 0;

  return ((a << 4) | b);
}

void updateRSSI()
{
  //RSSI must be above these negative numbers for LED to illuminate
  const int rssiLevelLow = -150;
  const int rssiLevelMed = -70;
  const int rssiLevelHigh = -50;
  const int rssiLevelMax = -20;

  int rssi = radio.getRSSI();

  LRS_DEBUG_PRINT(F("RSSI: "));
  LRS_DEBUG_PRINTLN(rssi);

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
  if (pin_txLED != 255)
  {
    if (illuminate == true)
      digitalWrite(pin_txLED, HIGH);
    else
      digitalWrite(pin_txLED, LOW);
  }
}

void rxLED(bool illuminate)
{
  if (pin_rxLED != 255)
  {
    if (illuminate == true)
      digitalWrite(pin_rxLED, HIGH);
    else
      digitalWrite(pin_rxLED, LOW);
  }
}
