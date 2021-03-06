//Return number of bytes sitting in the serial receive buffer
uint16_t availableRXBytes()
{
  if (rxHead >= rxTail) return (rxHead - rxTail);
  return (sizeof(serialReceiveBuffer) - rxTail + rxHead);
}

//Return number of bytes sitting in the serial transmit buffer
uint16_t availableTXBytes()
{
  if (inCommandMode == true) return (0); //If we are in command mode, block printing of data from the TXBuffer

  if (txHead >= txTail) return (txHead - txTail);
  return (sizeof(serialTransmitBuffer) - txTail + txHead);
}

//Return number of bytes sitting in the serial receive buffer
uint16_t availableRXCommandBytes()
{
  if (commandRXHead >= commandRXTail) return (commandRXHead - commandRXTail);
  return (sizeof(commandRXBuffer) - commandRXTail + commandRXHead);
}

//Return number of bytes sitting in the serial transmit buffer
uint16_t availableTXCommandBytes()
{
  if (commandTXHead >= commandTXTail) return (commandTXHead - commandTXTail);
  return (sizeof(commandTXBuffer) - commandTXTail + commandTXHead);
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
        txLED(true); //Turn on LED during serial transmissions

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
          //          Serial1.write(&serialTransmitBuffer[txTail], bytesToSend);
          //          txTail += bytesToSend;
          //          txTail %= sizeof(serialTransmitBuffer);

          systemWrite(serialTransmitBuffer[txTail]);
          systemFlush(); //Prevent serial hardware from blocking more than this one write

          txTail++;
          txTail %= sizeof(serialTransmitBuffer);
        }

        txLED(false); //Turn off LED
      }
    }
  }

  //Look for local incoming serial
#if defined(ARDUINO_ARCH_SAMD)
  while ((Serial.available() || Serial1.available()) && transactionComplete == false)
#else
  while (Serial.available() && transactionComplete == false)
#endif
  {
    rxLED(true); //Turn on LED during serial reception

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

    byte incoming = systemRead();

    //Process serial into either rx buffer or command buffer
    if (inCommandMode == true)
    {
      if (incoming == '\r' && commandLength > 0)
      {
        printerEndpoint = PRINT_TO_SERIAL;
        checkCommand(); //Process command buffer
      }
      else if (incoming == '\n')
        ; //Do nothing
      else
      {
        systemWrite(incoming); //Always echo during command mode

        //Move this character into the command buffer
        commandBuffer[commandLength++] = toupper(incoming);
        commandLength %= sizeof(commandBuffer);
      }
    }
    else
    {
      //Check general serial stream for command characters
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
              systemWrite(incoming);

            systemPrintln("\r\nOK");

            inCommandMode = true; //Allow AT parsing. Prevent received RF data from being printed.

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
        systemWrite(incoming);

      //We must always read in characters to avoid causing the host computer blocking USB from sending more
      //If the buffer is full, we will overwrite oldest data first
      serialReceiveBuffer[rxHead++] = incoming; //Push char to holding buffer
      rxHead %= sizeof(serialReceiveBuffer);
    } //End process rx buffer

    rxLED(false); //Turn off LED

  } //End Serial.available()

  //Process any remote commands sitting in buffer
  if (availableRXCommandBytes() && inCommandMode == false)
  {
    commandLength = availableRXCommandBytes();

    for (int x = 0 ; x < commandLength ; x++)
    {
      commandBuffer[x] = commandRXBuffer[commandRXTail++];
      commandRXTail %= sizeof(commandRXBuffer);
    }

    if (commandBuffer[0] == 'R') //Error check
    {
      commandBuffer[0] = 'A'; //Convert this RT command to an AT command for local consumption
      printerEndpoint = PRINT_TO_RF; //Send prints to RF link
      checkCommand(); //Parse the command buffer

      //We now have the commandTXBuffer loaded. But we need to send an remoteCommandResponse. Use the printerEndpoint within the State machine.
    }
    else
    {
      LRS_DEBUG_PRINT(F("Corrupt remote command received"));
    }
  }
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
  for (uint8_t x = 0 ; x < packetSize ; x++)
  {
    outgoingPacket[x] = serialReceiveBuffer[rxTail++];
    rxTail %= sizeof(serialReceiveBuffer);
  }
}

//Send a portion of the commandTXBuffer to outgoingPacket
void readyOutgoingCommandPacket()
{
  uint16_t bytesToSend = availableTXCommandBytes();
  if (bytesToSend > settings.frameSize) bytesToSend = settings.frameSize;

  //SF6 requires an implicit header which means there is no dataLength in the header
  if (settings.radioSpreadFactor == 6)
  {
    if (bytesToSend > 255 - 3) bytesToSend = 255 - 3; //We are going to transmit 255 bytes no matter what
  }

  packetSize = bytesToSend;

  //Move this portion of the circular buffer into the outgoingPacket
  for (uint8_t x = 0 ; x < packetSize ; x++)
  {
    outgoingPacket[x] = commandTXBuffer[commandTXTail++];
    commandTXTail %= sizeof(commandTXBuffer);
  }
}
