//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//Serial RX - Data arriving at the USB or serial port
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Return number of bytes sitting in the serial receive buffer
uint16_t availableRXBytes()
{
  if (rxHead >= rxTail) return (rxHead - rxTail);
  return (sizeof(serialReceiveBuffer) - rxTail + rxHead);
}

//Returns true if CTS is asserted (high = host says it's ok to send data)
bool isCTS()
{
  if (pin_cts == PIN_UNDEFINED) return (true); //CTS not implmented on this board
  if (settings.flowControl == false) return (true); //CTS turned off
  return (digitalRead(pin_cts) == HIGH) ^ settings.invertCts;
}

//If we have data to send, get the packet ready
//Return true if new data is ready to be sent
bool processWaitingSerial(bool sendNow)
{
  //Push any available data out
  if (availableRXBytes() >= maxDatagramSize)
  {
    if (settings.debugRadio)
      systemPrintln("Sending max frame");
    readyOutgoingPacket(availableRXBytes());
    return (true);
  }

  //Check if we should send out a partial frame
  else if (sendNow || (availableRXBytes() > 0 && (millis() - lastByteReceived_ms) >= settings.serialTimeoutBeforeSendingFrame_ms))
  {
    if (settings.debugRadio)
      systemPrintln("Sending partial frame");
    readyOutgoingPacket(availableRXBytes());
    return (true);
  }
  return (false);
}

//Send a portion of the serialReceiveBuffer to outgoingPacket
void readyOutgoingPacket(uint16_t bytesToSend)
{
  uint16_t length;
  if (bytesToSend > maxDatagramSize) bytesToSend = maxDatagramSize;

  //Determine the number of bytes to send
  length = 0;
  if ((rxTail + bytesToSend) > sizeof(serialReceiveBuffer))
  {
    //Copy the first portion of the buffer
    length = sizeof(serialReceiveBuffer) - rxTail;
    memcpy(&outgoingPacket[headerBytes], &serialReceiveBuffer[rxTail], length);
    rxTail = 0;
  }

  //Copy the remaining portion of the buffer
  memcpy(&outgoingPacket[headerBytes + length], &serialReceiveBuffer[rxTail], bytesToSend - length);
  rxTail += bytesToSend - length;
  rxTail %= sizeof(serialReceiveBuffer);
  endOfTxData += bytesToSend;
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//Serial TX - Data being sent to the USB or serial port
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Return number of bytes sitting in the serial transmit buffer
uint16_t availableTXBytes()
{
  if (inCommandMode == true) return (0); //If we are in command mode, block printing of data from the TXBuffer

  if (txHead >= txTail) return (txHead - txTail);
  return (sizeof(serialTransmitBuffer) - txTail + txHead);
}

//Add serial data to the output buffer
void serialBufferOutput(uint8_t * data, uint16_t dataLength)
{
  int length;

  length = 0;
  if ((txHead + dataLength) > sizeof(serialTransmitBuffer))
  {
    //Copy the first portion of the received datagram into the buffer
    length = sizeof(serialTransmitBuffer) - txHead;
    memcpy(&serialTransmitBuffer[txHead], data, length);
    txHead = 0;
  }

  //Copy the remaining portion of the received datagram into the buffer
  memcpy(&serialTransmitBuffer[txHead], &data[length], dataLength - length);
  txHead += dataLength - length;
  txHead %= sizeof(serialTransmitBuffer);
}

//Update the output of the RTS pin (host says it's ok to send data when assertRTS = true)
void updateRTS(bool assertRTS)
{
    rtsAsserted = assertRTS;
    if (settings.flowControl && (pin_rts != PIN_UNDEFINED))
      digitalWrite(pin_rts, assertRTS ^ settings.invertRts);
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//Command RX - Remote command data received from a remote system
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Return number of bytes sitting in the serial receive buffer
uint16_t availableRXCommandBytes()
{
  if (commandRXHead >= commandRXTail) return (commandRXHead - commandRXTail);
  return (sizeof(commandRXBuffer) - commandRXTail + commandRXHead);
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//Command TX - Remote command data or command response data to be sent to the remote system
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Return number of bytes sitting in the serial transmit buffer
uint16_t availableTXCommandBytes()
{
  if (commandTXHead >= commandTXTail) return (commandTXHead - commandTXTail);
  return (sizeof(commandTXBuffer) - commandTXTail + commandTXHead);
}

//Send a portion of the commandTXBuffer to outgoingPacket
void readyOutgoingCommandPacket()
{
  uint16_t length;
  uint16_t bytesToSend = availableTXCommandBytes();
  if (bytesToSend > maxDatagramSize) bytesToSend = maxDatagramSize;

  //SF6 requires an implicit header which means there is no dataLength in the header
  if (settings.radioSpreadFactor == 6)
  {
    if (bytesToSend > maxDatagramSize) bytesToSend = maxDatagramSize; //We are going to transmit 255 bytes no matter what
  }

  packetSize = bytesToSend;

  //Determine the number of bytes to send
  length = 0;
  if ((commandTXTail + packetSize) > sizeof(commandTXBuffer))
  {
    //Copy the first portion of the buffer
    length = sizeof(commandTXBuffer) - commandTXTail;
    memcpy(&outgoingPacket[headerBytes], &commandTXBuffer[commandTXTail], length);
    commandTXTail = 0;
  }

  //Copy the remaining portion of the buffer
  memcpy(&outgoingPacket[headerBytes + length], &commandTXBuffer[commandTXTail], packetSize - length);
  commandTXTail += packetSize - length;
  commandTXTail %= sizeof(commandTXBuffer);
  endOfTxData += packetSize;
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//Move serial data through the system
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//See if there's any serial from the remote radio that needs printing
//Record any characters to the receive buffer
//Scan for escape characters
void updateSerial()
{
  int x;

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
  while (arch.serialAvailable() && transactionComplete == false)
  {
    rxLED(true); //Turn on LED during serial reception

    //Take a break if there are ISRs to attend to
    petWDT();
    if (timeToHop == true) hopChannel();

    //Handle RTS
    if (availableRXBytes() == sizeof(serialReceiveBuffer) - 1)
      updateRTS(false); //Buffer full!
    else
      updateRTS(true); //Ok to send more

    byte incoming = systemRead();

    //Process serial into either rx buffer or command buffer
    if (inCommandMode == true)
    {
      if (incoming == '\r' && commandLength > 0)
      {
        printerEndpoint = PRINT_TO_SERIAL;
        systemPrintln();
        checkCommand(); //Process command buffer
      }
      else if (incoming == '\n')
        ; //Do nothing
      else
      {
        if (incoming == 8)
        {
          if (commandLength > 0)
          {
            //Remove this character from the command buffer
            commandLength--;

            //Erase the previous character
            systemWrite(incoming);
            systemWrite(' ');
            systemWrite(incoming);
          }
          else
            systemWrite(7);
        }
        else
        {
          systemWrite(incoming); //Always echo during command mode

          //Move this character into the command buffer
          commandBuffer[commandLength++] = toupper(incoming);
          commandLength %= sizeof(commandBuffer);
        }
      }
    }
    else
    {
      //Check general serial stream for command characters
      if (incoming == escapeCharacter)
      {
        //Ignore escape characters received within 2 seconds of serial traffic
        //Allow escape characters received within first 2 seconds of power on
        if (millis() - lastByteReceived_ms > minEscapeTime_ms || millis() < minEscapeTime_ms)
        {
          escapeCharsReceived++;
          if (escapeCharsReceived == maxEscapeCharacters)
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

    for (x = 0 ; x < commandLength ; x++)
    {
      commandBuffer[x] = commandRXBuffer[commandRXTail++];
      commandRXTail %= sizeof(commandRXBuffer);
    }

    if (commandBuffer[0] == 'R') //Error check
    {
      commandBuffer[0] = 'A'; //Convert this RT command to an AT command for local consumption
      printerEndpoint = PRINT_TO_RF; //Send prints to RF link
      checkCommand(); //Parse the command buffer
      printerEndpoint = PRINT_TO_SERIAL;
      remoteCommandResponse = true;
    }
    else
    {
      if (settings.debugRadio)
        systemPrintln("Corrupt remote command received");
    }
  }
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//Virtual-Circuit support
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Get the message length byte from the serial buffer
uint8_t vcSerialMsgGetLengthByte()
{
  //Get the length byte for the received serial message
  return serialReceiveBuffer[rxTail];
}

//Get the destination virtual circuit byte from the serial buffer
uint8_t vcSerialMsgGetVcDest()
{
  uint16_t index;

  //Get the destination address byte
  index = rxTail + 1;
  if (index >= sizeof(serialReceiveBuffer))
    index -= sizeof(serialReceiveBuffer);
  return serialReceiveBuffer[index];
}

//Determine if received serial data may be sent to the remote system
bool vcSerialMessageReceived()
{
  int8_t vcDest;
  uint8_t msgLength;

  do
  {
    //Determine if the radio is idle
    if (receiveInProcess())
      //The radio is busy, wait until it is idle
      break;

    //Wait until at least one byte is available
    if (!availableRXBytes())
      //No data available
      break;

    //Verify that the entire message is in the serial buffer
    msgLength = vcSerialMsgGetLengthByte();
    if (availableRXBytes() < msgLength)
      //The entire message is not in the buffer
      break;

    //Determine if the message is too large
    vcDest = vcSerialMsgGetVcDest();
    if (msgLength > maxDatagramSize)
    {
      //Discard this message, it is too long to transmit over the radio link
      rxTail += msgLength;
      if (rxTail >= sizeof(serialReceiveBuffer))
        rxTail -= sizeof(serialReceiveBuffer);

      //Nothing to do for invalid addresses or the broadcast address
      if ((vcDest >= MAX_VC) || (vcDest == VC_BROADCAST))
        break;

      //Break the link to this host
      vcBreakLink(vcDest);
      break;
    }

    //Validate the destination VC
    if ((vcDest < VC_BROADCAST) || (vcDest >= MAX_VC))
    {
      if (settings.debugTransmit)
      {
        systemPrint("ERROR: Invalid vcDest ");
        systemPrint(vcDest);
        systemPrintln(", discarding message!");
      }

      //Discard this message
      rxTail += msgLength;
      if (rxTail >= sizeof(serialReceiveBuffer))
        rxTail -= sizeof(serialReceiveBuffer);
      break;
    }

    //If sending to ourself, just place the data in the serial output buffer
    readyOutgoingPacket(msgLength);
    if (vcDest == myVc)
    {
      serialBufferOutput(outgoingPacket, msgLength);
      endOfTxData -= msgLength;
      break;
    }

    //Send this message
    return true;
  } while (0);

  //Nothing to send at this time
  return false;
}

void resetSerial()
{
  uint32_t delayTime;
  uint32_t lastCharacterReceived;

  //Determine the amount of time needed to receive a character
  delayTime = 200;
  if (settings.airSpeed)
  {
    delayTime = (1000 * 8 * 2) / settings.airSpeed;
    if (delayTime < 200)
      delayTime = 200;
  }

  //Enable RTS
  updateRTS(true);

  //Flush the incoming serial
  lastCharacterReceived = millis();
  do
  {
    //Discard any incoming serial data
    while (arch.serialAvailable())
    {
      petWDT();
      systemRead();
      lastCharacterReceived = millis();
    }
    petWDT();

    //Wait enough time to receive any remaining data from the host
  } while ((millis() - lastCharacterReceived) < delayTime);

  //Empty the buffers
  rxHead = rxTail;
  txHead = txTail;
  commandRXHead = commandRXTail;
  commandTXHead = commandTXTail;
  endOfTxData = &outgoingPacket[headerBytes];
}
