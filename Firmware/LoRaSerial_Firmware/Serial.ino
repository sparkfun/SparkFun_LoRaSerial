//Return number of bytes sitting in the serial receive buffer
uint16_t availableRXBytes()
{
  if (rxHead >= rxTail) return (rxHead - rxTail);
  return (sizeof(serialReceiveBuffer) - rxTail + rxHead);
}

//Return number of bytes sitting in the serial transmit buffer
uint16_t availableTXBytes()
{
  if (txHead >= txTail) return (txHead - txTail);
  return (sizeof(serialTransmitBuffer) - txTail + txHead);
}

//See if there's any serial from the remote radio that needs printing
//Record any characters to the receive buffer
//Scan for escape characters
void updateSerial()
{
  //Forget printing if there are ISRs to attend to
  if (transactionComplete == false && timeToHop == false)
  {
    if (availableTXBytes())
    {
      if (isCTS())
      {
        //Print data to both ports
        for (int x = 0 ; x < availableTXBytes() ; x++)
        {
          //Take a break if there are ISRs to attend to
          petWDT();
          if (transactionComplete == true) break;
          if (timeToHop == true) hopChannel();
          if (isCTS() == false) break;

          //          int bytesToSend = availableTXBytes();
          //
          //          if (txTail + bytesToSend > sizeof(serialTransmitBuffer))
          //            bytesToSend = sizeof(serialTransmitBuffer) - txTail;
          //
          //          //TODO this may introduce delays when we should be checking ISRs
          //          Serial.write(&serialTransmitBuffer[txTail], bytesToSend);
          //          txTail += bytesToSend;
          //          txTail %= sizeof(serialTransmitBuffer);

          Serial.write(serialTransmitBuffer[txTail]);
          Serial.flush(); //Prevent serial hardware from blocking more than this one write

          //#if defined(ARDUINO_ARCH_SAMD)
          //          Serial1.write(serialTransmitBuffer[txTail]);
          //          Serial1.flush(); //Prevent serial hardware from blocking more than this one write
          //#endif

          txTail++;
          txTail %= sizeof(serialTransmitBuffer);
        }
      }
    }
  }

  //Look for local incoming serial
  while (Serial.available() && transactionComplete == false)
  {
    //Take a break if there are ISRs to attend to
    petWDT();
    if (timeToHop == true) hopChannel();

    //Handle RTS
    if (availableRXBytes() == sizeof(serialReceiveBuffer) - 1)
    {
      //Buffer full!
      if (pin_rts != 255 && settings.flowControl == true)
        digitalWrite(pin_rts, LOW); //Don't give me more
    }
    else
    {
      if (pin_rts != 255 && settings.flowControl == true)
        digitalWrite(pin_rts, HIGH); //Ok to send more
    }

    byte incoming = Serial.read();

    if (incoming == settings.escapeCharacter)
    {
      //Ignore escape characters received within 2 seconds of serial traffic
      //Allow escape characters received within first 2 seconds of power on
      if (millis() - lastByteReceived_ms > minEscapeTime_ms || millis() < minEscapeTime_ms)
      {
        escapeCharsReceived++;
        if (escapeCharsReceived == settings.maxEscapeCharacters)
        {
          if (settings.echo == true)
            Serial.write(incoming);

          commandMode();

          escapeCharsReceived = 0;
          lastByteReceived_ms = millis();
          return; //Avoid recording this incoming command char
        }
      }
      else //This is just a character in the stream, ignore
      {
        lastByteReceived_ms = millis();
        escapeCharsReceived = 0; //Update timeout check for escape char and partial frame
      }
    }
    else
    {
      lastByteReceived_ms = millis();
      escapeCharsReceived = 0; //Update timeout check for escape char and partial frame
    }

    if (settings.echo == true)
      Serial.write(incoming);

    //We must always read in characters to avoid causing the host computer blocking USB from sending more
    //If the buffer is full, we will overwrite oldest data first
    serialReceiveBuffer[rxHead++] = incoming; //Push char to holding buffer
    rxHead %= sizeof(serialReceiveBuffer);
  } //End Serial.available()
}

//Returns true if CTS is asserted (high = host says it's ok to send data)
bool isCTS()
{
  if (pin_cts == 255) return (true); //CTS not implmented on this board
  if (settings.flowControl == false) return (true); //CTS turned off
  if (digitalRead(pin_cts) == HIGH) return (true);
  return (false);
}

//If we have data to send, get the packet ready
//Return true if new data is ready to be sent
bool processWaitingSerial()
{
  //Push any available data out
  if (availableRXBytes() >= settings.frameSize)
  {
    LRS_DEBUG_PRINTLN(F("Sending max frame"));
    readyOutgoingPacket();
    return (true);
  }

  //Check if we should send out a partial frame
  else if (availableRXBytes() > 0 && (millis() - lastByteReceived_ms) >= settings.serialTimeoutBeforeSendingFrame_ms)
  {
    LRS_DEBUG_PRINTLN(F("Sending partial frame"));
    readyOutgoingPacket();
    return (true);
  }
  return (false);
}

//Send a portion of the serialReceiveBuffer to outgoingPacket
void readyOutgoingPacket()
{
  uint16_t bytesToSend = availableRXBytes();
  if (bytesToSend > settings.frameSize) bytesToSend = settings.frameSize;

  //SF6 requires an implicit header which means there is no dataLength in the header
  if (settings.radioSpreadFactor == 6)
  {
    if (bytesToSend > 255 - 3) bytesToSend = 255 - 3; //We are going to transmit 255 bytes no matter what
  }

  packetSize = bytesToSend;

  //Move this portion of the circular buffer into the outgoingPacket
  uint16_t tempTail = rxTail; //Don't move the tail until we sucessfully send packet
  for (uint8_t x = 0 ; x < packetSize ; x++)
  {
    outgoingPacket[x] = serialReceiveBuffer[tempTail++];
    tempTail %= sizeof(serialReceiveBuffer);
  }

  rxTail = tempTail; //TODO - We move the tail no matter if sendDataPacket was successful or errored out
}
