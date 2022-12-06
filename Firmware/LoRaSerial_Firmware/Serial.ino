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

//Send a portion of the commandTXBuffer to serialTransmitBuffer
void readyLocalCommandPacket()
{
  uint16_t bytesToSend;
  uint16_t length;
  uint16_t maxLength;

  bytesToSend = availableTXCommandBytes();
  maxLength = maxDatagramSize;
  if (settings.operatingMode == MODE_VIRTUAL_CIRCUIT)
    maxLength -= VC_RADIO_HEADER_BYTES;
  if (bytesToSend > maxLength)
    bytesToSend = maxLength;
  if (settings.operatingMode == MODE_VIRTUAL_CIRCUIT)
  {
    //Build the VC header
    serialOutputByte(START_OF_VC_SERIAL);
    serialOutputByte(bytesToSend + VC_RADIO_HEADER_BYTES);
    serialOutputByte(PC_REMOTE_RESPONSE | myVc);
    serialOutputByte(myVc);
    if (settings.debugSerial)
      systemPrintln("Built VC header in serialTransmitBuffer");
  }

  //Place the command response bytes into serialTransmitBuffer
  if (settings.debugSerial)
  {
    systemPrint("Moving ");
    systemPrint(bytesToSend);
    systemPrintln(" bytes from commandTXBuffer into serialTransmitBuffer");
    outputSerialData(true);
  }
  for (length = 0; length < bytesToSend; length++)
  {
    serialOutputByte(commandTXBuffer[commandTXTail++]);
    commandTXTail %= sizeof(commandTXBuffer);
  }
}

//Send a portion of the commandTXBuffer to outgoingPacket
uint8_t readyOutgoingCommandPacket(uint16_t offset)
{
  uint16_t bytesToSend = availableTXCommandBytes();
  uint16_t length;
  uint16_t maxLength;

  maxLength = maxDatagramSize - offset;
  bytesToSend = availableTXCommandBytes();
  if (bytesToSend > maxLength)
    bytesToSend = maxLength;

  //Determine the number of bytes to send
  length = 0;
  if ((commandTXTail + bytesToSend) > sizeof(commandTXBuffer))
  {
    //Copy the first portion of the buffer
    length = sizeof(commandTXBuffer) - commandTXTail;
    memcpy(&outgoingPacket[headerBytes + offset], &commandTXBuffer[commandTXTail], length);
    commandTXTail = 0;
  }

  //Copy the remaining portion of the buffer
  memcpy(&outgoingPacket[headerBytes + offset + length], &commandTXBuffer[commandTXTail], bytesToSend - length);
  commandTXTail += bytesToSend - length;
  commandTXTail %= sizeof(commandTXBuffer);
  endOfTxData += bytesToSend;
  return (uint8_t)bytesToSend;
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//Move serial data through the system
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//See if there's any serial from the remote radio that needs printing
//Record any characters to the receive buffer
//Scan for escape characters
void updateSerial()
{
  uint16_t previousHead;
  int x;

  //Assert RTS when there is enough space in the receive buffer
  if ((!rtsAsserted) && (availableRXBytes() < (sizeof(serialReceiveBuffer) / 2))
    && (availableTXBytes() < (sizeof(serialTransmitBuffer) / 4)))
    updateRTS(true);

  //Attempt to empty the serialTransmitBuffer
  outputSerialData(false);

  //Look for local incoming serial
  previousHead = rxHead;
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

  //Print the number of bytes received via serial
  if (settings.debugSerial && (rxHead - previousHead))
  {
    systemPrint("updateSerial moved ");
    systemPrint(rxHead - previousHead);
    systemPrintln(" bytes into serialReceiveBuffer");
    outputSerialData(true);
    petWDT();
  }

  //Process the serial data
  if (serialOperatingMode == MODE_VIRTUAL_CIRCUIT)
    vcProcessSerialInput();
  else
    processSerialInput();

  //Process any remote commands sitting in buffer
  if (availableRXCommandBytes() && inCommandMode == false)
  {
    commandLength = availableRXCommandBytes();

    for (x = 0 ; x < commandLength ; x++)
    {
      commandBuffer[x] = commandRXBuffer[commandRXTail++];
      commandRXTail %= sizeof(commandRXBuffer);
    }

    //Print the number of bytes moved into the command buffer
    if (settings.debugSerial && commandLength)
    {
      systemPrint("updateSerial moved ");
      systemPrint(commandLength);
      systemPrintln(" bytes from commandRXBuffer into commandBuffer");
      outputSerialData(true);
      petWDT();
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

//Process serial input for point-to-point and multi-point modes
void processSerialInput()
{
  uint16_t radioHead;

  //Process the serial data
  radioHead = radioTxHead;
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
          commandBuffer[commandLength++] = incoming;
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
  }

  //Print the number of bytes placed into the rxTxBuffer
  if (settings.debugSerial && (radioTxHead != radioHead))
  {
    systemPrint("processSerialInput moved ");
    systemPrint((radioTxHead - radioHead) % sizeof(radioTxBuffer));
    systemPrintln(" bytes from serialReceiveBuffer into radioTxBuffer");
    outputSerialData(true);
  }
}

//Move the serial data from serialTransmitBuffer to the USB or serial port
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
bool vcSerialMsgGetLengthByte(uint8_t * msgLength)
{
  //Get the length byte for the received serial message, account for the start byte
  *msgLength = radioTxBuffer[radioTxTail];
  return true;
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
  int8_t channel;
  uint16_t dataBytes;
  uint8_t msgLength;
  int8_t vcDest;

  do
  {
    //Determine if the radio is idle
    if (receiveInProcess())
      //The radio is busy, wait until it is idle
      break;

    //Check for some data bytes
    dataBytes = availableRadioTXBytes();
    if (!dataBytes)
      break;

    //Wait until the length byte is available
    msgLength = radioTxBuffer[radioTxTail];

    //Print the message length and availableRadioTXBytes
    if (settings.debugSerial)
    {
      systemPrint("msgLength: ");
      systemPrintln(msgLength);
      systemPrint("availableRadioTXBytes(): ");
      systemPrintln(availableRadioTXBytes());
      outputSerialData(true);
    }

    //Verify that the entire message is in the serial buffer
    if (availableRadioTXBytes() < msgLength)
    {
      //The entire message is not in the buffer
      if (settings.debugSerial)
      {
        systemPrintln("VC serial RX: Waiting for entire buffer");
        outputSerialData(true);
      }
      break;
    }

    //vcProcessSerialInput validates the vcDest value, this check validates
    //that internally generated traffic uses valid vcDest values.  Only messages
    //enabled for receive on a remote radio may be transmitted.
    vcDest = vcSerialMsgGetVcDest();
    if (((uint8_t)vcDest >= (uint8_t)MIN_RX_NOT_ALLOWED) && (vcDest != VC_BROADCAST))
    {
      if (settings.debugSerial || settings.debugTransmit)
      {
        systemPrint("ERROR: Invalid internally generated vcDest ");
        systemPrint(vcDest);
        systemPrintln(", discarding message!");
        outputSerialData(true);
      }

      //Discard this message
      radioTxTail += msgLength;
      if (radioTxTail >= sizeof(radioTxBuffer))
        radioTxTail -= sizeof(radioTxBuffer);
      break;
    }

    //Determine if the message is too large
    if (msgLength > maxDatagramSize)
    {
      if (settings.debugSerial || settings.debugTransmit)
      {
        systemPrintln("VC serial RX: Message too long, discarded");
        outputSerialData(true);
      }

      //Discard this message, it is too long to transmit over the radio link
      radioTxTail += msgLength;
      if (radioTxTail >= sizeof(radioTxBuffer))
        radioTxTail -= sizeof(radioTxBuffer);

      //Nothing to do for invalid addresses or the broadcast address
      if (((uint8_t)vcDest >= (uint8_t)MIN_TX_NOT_ALLOWED) && (vcDest != VC_BROADCAST))
        break;

      //Break the link to this host since message delivery is guarranteed and
      //this message is being discarded
      vcBreakLink(vcDest & VCAB_NUMBER_MASK);
      break;
    }

    //Verify that the destination link is up
    if ((vcDest != VC_BROADCAST)
      && (virtualCircuitList[vcDest & VCAB_NUMBER_MASK].vcState == VC_STATE_LINK_DOWN))
    {
      if (settings.debugSerial || settings.debugTransmit)
      {
        systemPrint("Link down ");
        systemPrint((vcDest == VC_BROADCAST) ? vcDest : vcDest & VCAB_NUMBER_MASK);
        systemPrintln(", discarding message!");
        outputSerialData(true);
      }

      //Discard this message
      radioTxTail += msgLength;
      if (radioTxTail >= sizeof(radioTxBuffer))
        radioTxTail -= sizeof(radioTxBuffer);
      break;
    }

    //Print the data ready for transmission
    if (settings.debugSerial)
    {
      systemPrint("Readying ");
      systemPrint(msgLength);
      systemPrintln(" byte for transmission");
      outputSerialData(true);
    }

    //If sending to ourself, just place the data in the serial output buffer
    readyOutgoingPacket(msgLength);
    channel = GET_CHANNEL_NUMBER(vcDest);
    if ((vcDest != VC_BROADCAST) && ((vcDest & VCAB_NUMBER_MASK) == myVc)
      && (channel == 0))
    {
      if (settings.debugSerial)
        systemPrintln("VC: Sending data to ourself");
      systemWrite(START_OF_VC_SERIAL);
      systemWrite(outgoingPacket, msgLength);
      endOfTxData -= msgLength;
      break;
    }

    //Send this message
    return true;
  } while (0);

  //Nothing to send at this time
  return false;
}

//Process serial input when running in MODE_VIRTUAL_CIRCUIT
void vcProcessSerialInput()
{
  char * cmd;
  uint8_t data;
  uint16_t dataBytes;
  uint16_t index;
  int8_t vcDest;
  int8_t vcSrc;
  uint8_t length;

  //Process the serial data while there is space in radioTxBuffer
  dataBytes = availableRXBytes();
  while (dataBytes && (availableRadioTXBytes() < (sizeof(radioTxBuffer) - 256))
    && (transactionComplete == false))
  {
    //Take a break if there are ISRs to attend to
    petWDT();
    if (timeToHop == true) hopChannel();

    //Skip any garbage in the input stream
    data = serialReceiveBuffer[rxTail++];
    rxTail %= sizeof(radioTxBuffer);
    if (data != START_OF_VC_SERIAL)
    {
      if (settings.debugSerial)
      {
        systemPrint("vcProcessSerialInput discarding 0x");
        systemPrint(data, HEX);
        systemPrintln();
        outputSerialData(true);
      }

      //Discard this data byte
      dataBytes = availableRXBytes();
      continue;
    }

    //Get the virtual circuit header
    //The start byte has already been removed
    length = serialReceiveBuffer[rxTail];
    if (length <= VC_SERIAL_HEADER_BYTES)
    {
      if (settings.debugSerial)
      {
        systemPrint("ERROR - Invalid length ");
        systemPrint(length);
        systemPrint(", discarding 0x");
        systemPrint(data, HEX);
        systemPrintln();
        outputSerialData(true);
      }

      //Invalid message length, discard the START_OF_VC_SERIAL
      dataBytes = availableRXBytes();
      continue;
    }

    //Skip if there is not enough data
    if (dataBytes < length)
    {
      if (settings.debugSerial)
      {
        systemPrint("ERROR - Invalid length ");
        systemPrint(length);
        systemPrint(", discarding 0x");
        systemPrint(data, HEX);
        systemPrintln();
        outputSerialData(true);
      }
      dataBytes = availableRXBytes();
      continue;
    }

    //Get the source and destination virtual circuits
    index = (rxTail + 1) % sizeof(serialReceiveBuffer);
    vcDest = serialReceiveBuffer[index];
    index = (rxTail + 2) % sizeof(serialReceiveBuffer);
    vcSrc = serialReceiveBuffer[index];

    //Process this message
    switch ((uint8_t)vcDest)
    {
      //Send data over the radio link
      default:
        //Validate the source virtual circuit
        //Data that is being transmitted should always use myVC
        if ((vcSrc != myVc) || (myVc == VC_UNASSIGNED))
        {
          if (settings.debugSerial)
          {
            systemPrint("ERROR: Invalid myVc ");
            systemPrint(myVc);
            systemPrintln(" for data message, discarding message!");
            outputSerialData(true);
          }

          //Discard this message
          rxTail = (rxTail + length) % sizeof(radioTxBuffer);
          break;
        }

        //Verify the destination virtual circuit
        if (((uint8_t)vcDest >= (uint8_t)MIN_TX_NOT_ALLOWED) && (vcDest < VC_BROADCAST))
        {
          if (settings.debugSerial)
          {
            systemPrint("ERROR: Invalid vcDest ");
            systemPrint(vcDest);
            systemPrintln(" for data message, discarding message!");
            outputSerialData(true);
          }

          //Discard this message
          rxTail = (rxTail + length) % sizeof(radioTxBuffer);
          break;
        }

        if (settings.debugSerial)
        {
          systemPrint("vcProcessSerialInput moving ");
          systemPrint(length);
          systemPrintln(" bytes into radioTxBuffer");
          dumpCircularBuffer(serialReceiveBuffer, rxTail, sizeof(serialReceiveBuffer), length);
          outputSerialData(true);
        }

        //Place the data in radioTxBuffer
        for (; length > 0; length--)
        {
          radioTxBuffer[radioTxHead++] = serialReceiveBuffer[rxTail++];
          radioTxHead %= sizeof(radioTxBuffer);
          rxTail %= sizeof(serialReceiveBuffer);
        }
        break;

      //Process this command
      case (uint8_t)VC_COMMAND:
        //Validate the source virtual circuit
        if ((vcSrc != PC_COMMAND) || (length > (sizeof(commandBuffer) - 1)))
        {
          if (settings.debugSerial)
          {
            systemPrint("ERROR: Invalid vcSrc ");
            systemPrint(myVc);
            systemPrintln(" for command message, discarding message!");
            outputSerialData(true);
          }

          //Discard this message
          rxTail = (rxTail + length) % sizeof(radioTxBuffer);
          break;
        }

        //Discard the VC header
        length -= VC_RADIO_HEADER_BYTES;
        rxTail = (rxTail + VC_RADIO_HEADER_BYTES) % sizeof(radioTxBuffer);

        //Move this message into the command buffer
        for (cmd = commandBuffer; length > 0; length--)
        {
          *cmd++ = toupper(serialReceiveBuffer[rxTail++]);
          rxTail %= sizeof(serialReceiveBuffer);
        }
        commandLength = cmd - commandBuffer;

        if (settings.debugSerial)
        {
          systemPrint("vcProcessSerialInput moving ");
          systemPrint(commandLength);
          systemPrintln(" bytes into commandBuffer");
          outputSerialData(true);
          dumpBuffer((uint8_t *)commandBuffer, commandLength);
        }

        //Process this command
        checkCommand();
        break;
    }

    //Determine how much data is left in the buffer
    dataBytes = availableRXBytes();
  }

  //Take a break if there are ISRs to attend to
  petWDT();
  if (timeToHop == true) hopChannel();
}

//Display any serial data for output, discard:
// * Serial input data
// * Radio transmit data
// * Received remote command data
// * Remote command response data
void resetSerial()
{
  uint32_t delayTime;
  uint32_t lastCharacterReceived;

  //Display any debug output
  outputSerialData(true);

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
  commandLength = 0;
}
