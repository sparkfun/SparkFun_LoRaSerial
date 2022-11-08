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

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//Serial TX - Data being sent to the USB or serial port
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Return number of bytes sitting in the serial transmit buffer
uint16_t availableTXBytes()
{
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

//Add a single byte to the output buffer
void serialOutputByte(uint8_t data)
{
  serialTransmitBuffer[txHead++] = data;
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
//Radio TX - Data to provide to the long range radio
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Return number of bytes sitting in the radio TX buffer
uint16_t availableRadioTXBytes()
{
  if (radioTxHead >= radioTxTail) return (radioTxHead - radioTxTail);
  return (sizeof(radioTxBuffer) - radioTxTail + radioTxHead);
}

//If we have data to send, get the packet ready
//Return true if new data is ready to be sent
bool processWaitingSerial(bool sendNow)
{
  uint16_t dataBytes;

  //Push any available data out
  dataBytes = availableRadioTXBytes();
  if (dataBytes >= maxDatagramSize)
  {
    if (settings.debugRadio)
      systemPrintln("Sending max frame");
    readyOutgoingPacket(dataBytes);
    return (true);
  }

  //Check if we should send out a partial frame
  else if (sendNow || (dataBytes > 0 && (millis() - lastByteReceived_ms) >= settings.serialTimeoutBeforeSendingFrame_ms))
  {
    if (settings.debugRadio)
      systemPrintln("Sending partial frame");
    readyOutgoingPacket(dataBytes);
    return (true);
  }
  return (false);
}

//Send a portion of the radioTxBuffer to outgoingPacket
void readyOutgoingPacket(uint16_t bytesToSend)
{
  uint16_t length;
  if (bytesToSend > maxDatagramSize) bytesToSend = maxDatagramSize;

  //Determine the number of bytes to send
  length = 0;
  if ((radioTxTail + bytesToSend) > sizeof(radioTxBuffer))
  {
    //Copy the first portion of the buffer
    length = sizeof(radioTxBuffer) - radioTxTail;
    memcpy(&outgoingPacket[headerBytes], &radioTxBuffer[radioTxTail], length);
    radioTxTail = 0;
  }

  //Copy the remaining portion of the buffer
  memcpy(&outgoingPacket[headerBytes + length], &radioTxBuffer[radioTxTail], bytesToSend - length);
  radioTxTail += bytesToSend - length;
  radioTxTail %= sizeof(radioTxBuffer);
  endOfTxData += bytesToSend;
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

  //Assert RTS when there is enough space in the receive buffer
  if ((!rtsAsserted) && (availableRXBytes() < (sizeof(serialReceiveBuffer) / 2))
    && (availableTXBytes() < (sizeof(serialTransmitBuffer) / 4)))
    updateRTS(true);

  //Attempt to empty the serialTransmitBuffer
  outputSerialData(false);

  //Look for local incoming serial
  while (rtsAsserted && arch.serialAvailable() && (transactionComplete == false))
  {
    rxLED(true); //Turn on LED during serial reception

    //Take a break if there are ISRs to attend to
    petWDT();
    if (timeToHop == true) hopChannel();

    //Deassert RTS when the buffer gets full
    if (rtsAsserted && (sizeof(serialReceiveBuffer) - availableRXBytes()) < 32)
      updateRTS(false);

    byte incoming = systemRead();

    serialReceiveBuffer[rxHead++] = incoming; //Push char to holding buffer
    rxHead %= sizeof(serialReceiveBuffer);
  } //End Serial.available()
  rxLED(false); //Turn off LED

  //Process the serial data
  while (availableRXBytes() && (availableRadioTXBytes() < (sizeof(radioTxBuffer) - 1))
    && (transactionComplete == false))
  {
    //Take a break if there are ISRs to attend to
    petWDT();
    if (timeToHop == true) hopChannel();

    byte incoming = serialReceiveBuffer[rxTail++];
    rxTail %= sizeof(serialReceiveBuffer);

    if ((settings.echo == true) || (inCommandMode == true))
    {
      systemWrite(incoming);
      outputSerialData(true);
    }

    //Process serial into either rx buffer or command buffer
    if (inCommandMode == true)
    {
      if (incoming == '\r' && commandLength > 0)
      {
        printerEndpoint = PRINT_TO_SERIAL;
        systemPrintln();
        checkCommand(); //Process command buffer
        outputSerialData(true);
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
          outputSerialData(true);
        }
        else
        {
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
            systemPrintln("\r\nOK");
            outputSerialData(true);

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

      //This data byte will be sent over the long range radio
      radioTxBuffer[radioTxHead++] = incoming;
      radioTxHead %= sizeof(radioTxBuffer);
    } //End process rx buffer
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

void outputSerialData(bool ignoreISR)
{
  int dataBytes;

  //Forget printing if there are ISRs to attend to
  dataBytes = availableTXBytes();
  while (dataBytes-- && isCTS() && (ignoreISR || (!transactionComplete)))
  {
    txLED(true); //Turn on LED during serial transmissions

    //Take a break if there are ISRs to attend to
    petWDT();
    if (timeToHop == true) hopChannel();

    arch.serialWrite(serialTransmitBuffer[txTail]);
    systemFlush(); //Prevent serial hardware from blocking more than this one write

    txTail++;
    txTail %= sizeof(serialTransmitBuffer);
  }
  txLED(false); //Turn off LED
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//Virtual-Circuit support
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Get the message length byte from the serial buffer
uint8_t vcSerialMsgGetLengthByte()
{
  //Get the length byte for the received serial message
  return radioTxBuffer[radioTxTail];
}

//Get the destination virtual circuit byte from the serial buffer
uint8_t vcSerialMsgGetVcDest()
{
  uint16_t index;

  //Get the destination address byte
  index = radioTxTail + 1;
  if (index >= sizeof(radioTxBuffer))
    index -= sizeof(radioTxBuffer);
  return radioTxBuffer[index];
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
    if (!availableRadioTXBytes())
      //No data available
      break;

    //Verify that the entire message is in the serial buffer
    msgLength = vcSerialMsgGetLengthByte();
    if (availableRadioTXBytes() < msgLength)
      //The entire message is not in the buffer
      break;

    //Determine if the message is too large
    vcDest = vcSerialMsgGetVcDest();
    if (msgLength > maxDatagramSize)
    {
      //Discard this message, it is too long to transmit over the radio link
      radioTxTail += msgLength;
      if (radioTxTail >= sizeof(radioTxBuffer))
        radioTxTail -= sizeof(radioTxBuffer);

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
      radioTxTail += msgLength;
      if (radioTxTail >= sizeof(radioTxBuffer))
        radioTxTail -= sizeof(radioTxBuffer);
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
  radioTxHead = radioTxTail;
  txHead = txTail;
  commandRXHead = commandRXTail;
  commandTXHead = commandTXTail;
  endOfTxData = &outgoingPacket[headerBytes];
}
