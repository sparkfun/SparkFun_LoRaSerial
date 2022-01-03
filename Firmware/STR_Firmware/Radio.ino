//Apply settings to radio
//Called after begin() and once user exits from command interface
void configureRadio()
{
  bool success = true;

  //Determine if we are using AirSpeed or custom settings
  if (settings.airSpeed != 0)
  {
    switch (settings.airSpeed)
    {
      case (0):
        //Custom settings - use settings without modification
        break;
      case (90):
        settings.radioSpreadFactor = 12;
        settings.radioBandwidth = 62.5;
        settings.radioCodingRate = 8;
        break;
      case (150):
        settings.radioSpreadFactor = 11;
        settings.radioBandwidth = 62.5;
        settings.radioCodingRate = 8;
        break;
      case (300):
        settings.radioSpreadFactor = 10;
        settings.radioBandwidth = 62.5;
        settings.radioCodingRate = 7;
        break;
      case (1200):
        settings.radioSpreadFactor = 9;
        settings.radioBandwidth = 125;
        settings.radioCodingRate = 8;
        break;
      case (2400):
        settings.radioSpreadFactor = 10;
        settings.radioBandwidth = 500;
        settings.radioCodingRate = 8;
        break;
      case (4800):
        settings.radioSpreadFactor = 9;
        settings.radioBandwidth = 500;
        settings.radioCodingRate = 8;
        break;
      case (9600):
        settings.radioSpreadFactor = 8;
        settings.radioBandwidth = 500;
        settings.radioCodingRate = 7;
        break;
      case (19200):
        settings.radioSpreadFactor = 7;
        settings.radioBandwidth = 500;
        settings.radioCodingRate = 7;
        break;
      case (28800):
        settings.radioSpreadFactor = 6;
        settings.radioBandwidth = 500;
        settings.radioCodingRate = 6;
        break;
      case (38400):
        settings.radioSpreadFactor = 6;
        settings.radioBandwidth = 500;
        settings.radioCodingRate = 5;
        break;
      default:
        if (settings.debug == true)
        {
          Serial.print(F("Unknown airSpeed: "));
          Serial.println(settings.airSpeed);
        }
        break;
    }
  }

  if (radio.setFrequency(settings.radioFrequency) == RADIOLIB_ERR_INVALID_FREQUENCY)
    success = false;

  // Set output power (accepted range is -3 - 17 dBm)
  // NOTE: 20 dBm value allows high power operation, but transmission duty cycle MUST NOT exceed 1%
  if (radio.setOutputPower(settings.radioBroadcastPower_dbm) == RADIOLIB_ERR_INVALID_OUTPUT_POWER)
    success = false;

  if (radio.setBandwidth(settings.radioBandwidth) == RADIOLIB_ERR_INVALID_BANDWIDTH)
    success = false;

  if (radio.setSpreadingFactor(settings.radioSpreadFactor) == RADIOLIB_ERR_INVALID_SPREADING_FACTOR)
    success = false;

  if (radio.setCodingRate(settings.radioCodingRate) == RADIOLIB_ERR_INVALID_CODING_RATE)
    success = false;

  if (radio.setSyncWord(settings.radioSyncWord) != RADIOLIB_ERR_NONE)
    success = false;

  if (radio.setPreambleLength(settings.radioPreambleLength) == RADIOLIB_ERR_INVALID_PREAMBLE_LENGTH)
    success = false;

  radio.setDio0Action(dioISR); //Called when packet transmission is complete

  if (pin_rxen != 255)
    radio.setRfSwitchPins(pin_rxen, pin_txen);

  if (success == false)
  {
    reportERROR();
    if (settings.debug == true)
    {
      Serial.println(F("Radio init failed. Check settings."));
    }
  }
}

//Send a given buffer to LoRa radio
//Caller must limit any transmission to 255 bytes
bool sendToRadio()
{
  //Check to see if radio link is available
  if (radioState == RADIO_RECEIVING)
  {
    digitalWrite(pin_act, HIGH);

    uint16_t bytesToSend = availableBytes();
    if (bytesToSend > settings.frameSize) bytesToSend = settings.frameSize;

    if (settings.debug == true)
    {
      Serial.print(F("Sending: "));
      Serial.println(bytesToSend);
    }

    //Because we cannot move the circular buffer directly to the SX1276,
    //we create a portion copy of the buffer here
    uint16_t tempTail = tail;
    uint8_t tempBuffer[settings.frameSize];
    for (uint8_t x = 0 ; x < bytesToSend ; x++)
    {
      tempBuffer[x] = serialReceiveBuffer[tempTail++];
      tempTail %= sizeof(serialReceiveBuffer);
    }

    startTime = millis();
    int state = radio.startTransmit(tempBuffer, bytesToSend); //Triggers interrupt when complete
    if (state == RADIOLIB_ERR_NONE)
    {
      if (settings.debug == true)
      {
        Serial.println(F("LoRa TX success!"));
        //Serial.printf("RTCM pushed to LoRa: %d", bytesToSend);
        //Serial.print(F(" Datarate: "));
        //Serial.print(radio.getDataRate());
        //Serial.println(F(" bps"));
      }
      tail = tempTail;
      radioState = RADIO_TRANSMITTING;
      digitalWrite(pin_act, LOW);
      return (true); //Data was sent
    }
    else if (state == RADIOLIB_ERR_TX_TIMEOUT)
    {
      if (settings.debug == true)
        Serial.println(F("LoRa timeout!"));
    }
    digitalWrite(pin_act, LOW);
    return (false); //Radio error
  }

  return (false); //Radio was not available
}

//ISR when DIO0 goes low
//Is called when transmission is complete or when RX is received
void dioISR(void)
{
  if (radioState == RADIO_TRANSMITTING)
  {
    //Transmission is complete
    stopTime = millis();

    //Return to receiving
    radio.startReceive();
    radioState = RADIO_RECEIVING;
  }
  else if (radioState == RADIO_RECEIVING)
  {
    //Received new frame
    radioState = RADIO_RECEIVING_AVAILABLE;
  }
}
