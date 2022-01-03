void updateSystemState()
{
  if (radioState == RADIO_RECEIVING_AVAILABLE)
  {
    if (isCTS())
    {
      //Host says it's ok to print so let's read the radio's data
      
      uint8_t incomingBuffer[settings.frameSize];

      int state = radio.readData(incomingBuffer, settings.frameSize);
      uint8_t receivedBytes = radio.getPacketLength();

      //Return to receiving
      radio.startReceive();
      radioState = RADIO_RECEIVING;

      if (state == RADIOLIB_ERR_NONE)
      {
        if (settings.debug == true)
        {
          Serial.print(F("\r\nRSSI: "));
          Serial.print(radio.getRSSI());
          Serial.print(F("dBm"));

          Serial.print(F(" SNR: "));
          Serial.print(radio.getSNR());
          Serial.print(F("dB"));

          Serial.print(F(" FreqError:\t"));
          Serial.print(radio.getFrequencyError());
          Serial.print(F("Hz: "));
        }

        //Printing 200 bytes at 57600 will take 34.7ms
        Serial.write(incomingBuffer, receivedBytes);

      } //End read radio check
    } //End CTS check
    else
    {
      if(settings.debug == true)
      {
        Serial.println(F("CTS Block"));
      }
    }
  }
  else if (radioState == RADIO_RECEIVING)
  {
    if (settings.debug == true)
    {
      //Print transmission time if detected
      if (stopTime > startTime)
      {
        Serial.print(F("LoRa Transmission time: "));
        Serial.print(stopTime - startTime);
        Serial.println(F("ms"));
        stopTime = 0; //Reset
      }
    }

    if (tail != head)
    {
      //Push any available data out
      if (availableBytes() >= settings.frameSize)
      {
        if (settings.debug == true)
          Serial.println(F("Sending max frame"));

        sendToRadio();
      }

      //Check if we should send out a partial frame
      else if ((millis() - lastByteReceived_ms) >= settings.serialTimeoutBeforeSendingFrame_ms)
      {
        if (settings.debug == true)
          Serial.println(F("Sending partial frame"));

        sendToRadio();
      }
    }
  }
}

//Returns true if CTS is asserted (high = host says it's ok to send data)
bool isCTS()
{
  if(pin_cts == 255) return(true); //CTS not implmented on this board
  if(digitalRead(pin_cts) == HIGH) return(true);
  return(false);
}
