//Helper functions to print to all available ports
void systemPrint(const char* value)
{
  Serial.print(value);

#if defined(ARDUINO_ARCH_SAMD)
  Serial1.print(value);
#endif
}

void systemPrintln(const char* value)
{
  systemPrint(value);

  Serial.println();
#if defined(ARDUINO_ARCH_SAMD)
  Serial1.println();
#endif
}

void systemPrint(const __FlashStringHelper* value)
{
  //Communicate an AT response over serial or the RF link (if available)
  if (serialState == SERIAL_COMMAND)
  {
    Serial.print(value);

#if defined(ARDUINO_ARCH_SAMD)
    Serial1.print(value);
#endif
  }
  else if(serialState == SERIAL_REMOTE_COMMAND_TX || serialState == SERIAL_REMOTE_COMMAND_RX)
  {
    if (isLinked() == true && radioState == RADIO_LINKED_RECEIVING_STANDBY)
    {
      //Move this response to the outgoing packet and send
//      for (int x = 0 ; x < strlen(value) ; x++)
//        outgoingPacket[x] = value[x];

      sendCommandDataPacket();
      changeState(RADIO_LINKED_TRANSMITTING);
    }
  }
}

void systemPrintln(const __FlashStringHelper * value)
{
  systemPrint(value);

  Serial.println();
#if defined(ARDUINO_ARCH_SAMD)
  Serial1.println();
#endif
}

void systemPrint(int value)
{
  Serial.print(value);

#if defined(ARDUINO_ARCH_SAMD)
  Serial1.print(value);
#endif
}

void systemPrintln(int value)
{
  systemPrint(value);

  Serial.println();
#if defined(ARDUINO_ARCH_SAMD)
  Serial1.println();
#endif
}

void systemPrint(uint8_t value, uint8_t printType)
{
  Serial.print(value, printType);

#if defined(ARDUINO_ARCH_SAMD)
  Serial1.print(value, printType);
#endif
}

void systemPrintln(uint8_t value, uint8_t printType)
{
  systemPrint(value, printType);

  Serial.println();
#if defined(ARDUINO_ARCH_SAMD)
  Serial1.println();
#endif
}

void systemPrint(float value, uint8_t decimals)
{
  Serial.print(value, decimals);

#if defined(ARDUINO_ARCH_SAMD)
  Serial1.print(value, decimals);
#endif
}

void systemPrintln(float value, uint8_t decimals)
{
  systemPrint(value, decimals);

  Serial.println();
#if defined(ARDUINO_ARCH_SAMD)
  Serial1.println();
#endif
}

void systemPrintln()
{
  Serial.println();

#if defined(ARDUINO_ARCH_SAMD)
  Serial1.println();
#endif
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
  byte incoming;
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
      trainState = TRAIN_PRESSED_4S;
      lastTrainBlink = millis();
    }
    else if (trainState == TRAIN_PRESSED_4S && trainBtn->wasReleased())
    {
      digitalWrite(pin_linkLED, LOW);

      beginTraining();

      trainState = TRAIN_NO_PRESS;
    }
    else if (trainState == TRAIN_PRESSED_4S && trainBtn->pressedFor(trainWithDefaultsButtonTime))
    {
      trainState = TRAIN_PRESSED_10S;
    }
    else if (trainState == TRAIN_PRESSED_10S && trainBtn->wasReleased())
    {
      digitalWrite(pin_linkLED, LOW);

      beginDefaultTraining();

      trainState = TRAIN_NO_PRESS;
    }

    //Blink LEDs according to our state while we wait for user to release button
    if (trainState == TRAIN_PRESSED_4S)
    {
      if (millis() - lastTrainBlink > 500) //Slow blink
      {
        lastTrainBlink = millis();
        digitalWrite(pin_linkLED, !digitalRead(pin_linkLED));
      }
    }
    else if (trainState == TRAIN_PRESSED_10S)
    {
      if (millis() - lastTrainBlink > 100) //Fast blink
      {
        lastTrainBlink = millis();
        digitalWrite(pin_linkLED, !digitalRead(pin_linkLED));
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
