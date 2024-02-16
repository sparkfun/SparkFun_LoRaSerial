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
  //The SAMD21 specification (page 448) indicates that CTS is low when data is flowing
  return (digitalRead(pin_cts) == LOW) ^ settings.invertCts;
}

#define NEXT_RX_TAIL(n)   ((rxTail + n) % sizeof(serialReceiveBuffer))

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

  //Make sure there is enough room in the buffer
  if ((sizeof(serialTransmitBuffer) - availableTXBytes()) < (dataLength + 32))
    outputSerialData(true);

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
  if (printerEndpoint == PRINT_TO_SERIAL)
  {
    //Make sure there is enough room in the buffer
    if ((sizeof(serialTransmitBuffer) - availableTXBytes()) < 32)
      outputSerialData(true);

    //Add this byte to the serial output buffer
    serialTransmitBuffer[txHead++] = data;
    txHead %= sizeof(serialTransmitBuffer);
  }
  else
  {
    //Add this byte to the command response buffer
    commandTXBuffer[commandTXHead++] = data;
    commandTXHead %= sizeof(commandTXBuffer);
  }
}

//Update the output of the RTS pin
//LoRaSerial will drive RTS low when it is ready for data
//Given false, we tell the host we *do not* need more data
//Given true, we tell the host we are ready for more data
void updateRTS(bool assertRTS)
{
  rtsAsserted = assertRTS;
  if (settings.flowControl && (pin_rts != PIN_UNDEFINED))
    //The SAMD21 specification (page 448) indicates that RTS is low to enable data flow
    digitalWrite(pin_rts, (assertRTS ? 0 : 1) ^ settings.invertRts);
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

#define NEXT_RADIO_TX_HEAD(n)   ((radioTxHead + n) % sizeof(radioTxBuffer))
#define NEXT_RADIO_TX_TAIL(n)   ((radioTxTail + n) % sizeof(radioTxBuffer))

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
  radioTxTail = NEXT_RADIO_TX_TAIL(bytesToSend - length);
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

#define NEXT_COMMAND_TX_TAIL(n)     ((commandTXTail + n) % sizeof(commandTXBuffer))

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
    serialOutputByte(commandTXBuffer[commandTXTail]);
    commandTXTail = NEXT_COMMAND_TX_TAIL(1);
  }
}

//Send a portion of the commandTXBuffer to outgoingPacket
uint8_t readyOutgoingCommandPacket(uint16_t offset)
{
  uint16_t bytesToSend;
  uint16_t length;
  uint16_t maxLength;

  //Limit the length to the frame size
  maxLength = maxDatagramSize - offset;
  bytesToSend = availableTXCommandBytes();
  if (bytesToSend > maxLength)
  {
    bytesToSend = maxLength;

    //checkCommand delivers the entire command response to the commandTXBuffer.
    //The response to be broken into multiple frames for transmission to the remote
    //radio and host.  The code below separates the commnad response from the
    //VC_COMMAND_COMPLETE_MESSAGE which follows the response.  This separation
    //ensures that the entire VC_COMMAND_COMPLETE_MESSAGE is delivered within a
    //single frame.  The result enables easy detection by the remote radio.
    //
    //Determine the number of command response bytes to send
    if (settings.operatingMode == MODE_VIRTUAL_CIRCUIT)
    {
      //Reserve the bytes for the VC heeader
      bytesToSend -= VC_SERIAL_HEADER_BYTES;

      //Determine if the VC_COMMAND_COMPLETE_MESSAGE is split across two buffers

      if (commandTXBuffer[commandTXTail] != START_OF_VC_SERIAL)
      {
        //OK if the entire VC_COMMAND_COMPLETE_MESSAGE is in the buffer.  Start
        //the search one byte into the VC_COMMAND_COMPLETE_MESSAGE position.
        for (length = bytesToSend - VC_SERIAL_HEADER_BYTES
                      - sizeof(VC_COMMAND_COMPLETE_MESSAGE) + 1;
             length < bytesToSend; length++)
          if (commandTXBuffer[NEXT_COMMAND_TX_TAIL(length)] == START_OF_VC_SERIAL)
          {
            //Exclude the partial piece of the VC_COMMAND_COMPLETE_MESSAGE from
            //this command response frame.
            bytesToSend = length;
            break;
          }
      }
    }
  }

  //Display the amount of data being sent
  if (settings.debugSerial)
  {
    systemPrint("Moving ");
    systemPrint(bytesToSend);
    systemPrintln(" bytes from commandTXBuffer into outgoingPacket");
    dumpCircularBuffer(commandTXBuffer, commandTXTail, sizeof(commandTXBuffer), bytesToSend);
  }

  //Determine if the data wraps around to the beginning of the buffer
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
  commandTXTail = NEXT_COMMAND_TX_TAIL(bytesToSend - length);
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
  int bufferSpace;
  uint16_t previousHead;
  int x;

  //Assert RTS when there is enough space in the receive buffer
  if ((!rtsAsserted) && (availableRXBytes() < (sizeof(serialReceiveBuffer) / 2))
      && (availableTXBytes() <= settings.rtsOnBytes))
    updateRTS(true); //We're ready for more data

  //Attempt to empty the serialTransmitBuffer
  outputSerialData(false);

  //Look for local incoming serial
  previousHead = rxHead;
  bufferSpace = sizeof(serialReceiveBuffer) - 1 - availableRXBytes();
  while (bufferSpace-- && arch.serialAvailable() && (transactionComplete == false))
  {
    blinkSerialRxLed(true); //Turn on LED during serial reception

    //Take a break if there are ISRs to attend to
    CheckChannelHopAndKickWatchdog();

    byte incoming = systemRead();

    serialReceiveBuffer[rxHead++] = incoming; //Push char to holding buffer
    rxHead %= sizeof(serialReceiveBuffer);

    //Deassert RTS when the buffer gets full
    if (rtsAsserted && (sizeof(serialReceiveBuffer) - availableRXBytes()) <= settings.rtsOffBytes)
      updateRTS(false);
  } //End Serial.available()
  blinkSerialRxLed(false); //Turn off LED

  //Print the number of bytes received via serial
  if (settings.debugSerial && (rxHead - previousHead))
  {
    systemPrint("updateSerial moved ");
    if (rxHead > previousHead)
      systemPrint(rxHead - previousHead);
    else
      systemPrint(sizeof(serialReceiveBuffer) - previousHead + rxHead);
    systemPrintln(" bytes into serialReceiveBuffer");
    outputSerialData(true);
    CheckChannelHopAndKickWatchdog();
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

    //Don't overflow the command buffer, save space for the zero termination
    if (commandLength >= sizeof(commandBuffer))
      commandLength = sizeof(commandBuffer) - 1;

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
      CheckChannelHopAndKickWatchdog();
    }

    if (commandBuffer[0] == 'R') //Error check
    {
      commandBuffer[0] = 'A'; //Convert this RT command to an AT command for local consumption
      printerEndpoint = PRINT_TO_RF; //Send prints to RF link
      checkCommand(); //Parse the command buffer
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
  static uint32_t lastEscapeCharEnteredMillis;

  //Process the serial data
  radioHead = radioTxHead;
  while (availableRXBytes()
         && (availableRadioTXBytes() < (sizeof(radioTxBuffer) - maxEscapeCharacters))
         && ((!inCommandMode) || (!waitRemoteCommandResponse))
         && (transactionComplete == false))
  {
    //Take a break if there are ISRs to attend to
    CheckChannelHopAndKickWatchdog();

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
      //Check for end of command
      if ((incoming == '\r') || (incoming == ';'))
      {
        //Ignore end of command if no command in the buffer
        if (commandLength > 0)
        {
          systemPrintln();
          if (settings.debugSerial)
          {
            systemPrint("processSerialInput moved ");
            systemPrint(commandLength);
            systemPrintln(" from serialReceiveBuffer into commandBuffer");
          }
          checkCommand(); //Process command buffer
          break;
        }
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

          //Don't allow the command to overflow the command buffer
          //Process the long command instead
          //Save room for the zero termination
          if (commandLength >= (sizeof(commandBuffer) - 1))
          {
            printerEndpoint = PRINT_TO_SERIAL;
            systemPrintln();
            if (settings.debugSerial)
            {
              systemPrint("processSerialInput moved ");
              systemPrint(commandLength);
              systemPrintln(" from serialReceiveBuffer into commandBuffer");
            }
            checkCommand(); //Process command buffer
            break;
          }
        }
      }
    }
    else
    {
      //Check general serial stream for command characters
      //Ignore escape characters received within 2 seconds of serial traffic
      //Allow escape characters received within first 2 seconds of power on
      if ((incoming == escapeCharacter)
          && ((millis() - lastByteReceived_ms > minEscapeTime_ms)
              || (millis() < minEscapeTime_ms)))
      {
        escapeCharsReceived++;
        lastEscapeCharEnteredMillis = millis();
        if (escapeCharsReceived == maxEscapeCharacters)
        {
          systemPrintln("\r\nOK");
          outputSerialData(true);

          inCommandMode = true; //Allow AT parsing. Prevent received RF data from being printed.
          forceRadioReset = false; //Don't reset the radio link unless a setting requires it
          writeOnCommandExit = false; //Don't record settings changes unless user commands it
          airSpeed = convertSettingsToAirSpeed(&settings); //Find current airSpeed

          tempSettings = settings;

          escapeCharsReceived = 0;
          lastByteReceived_ms = millis();
          return; //Avoid recording this incoming command char
        }
      }

      //This is just a character in the stream, ignore
      else
      {
        //Update timeout check for escape char and partial frame
        lastByteReceived_ms = millis();

        //Replay any escape characters if the sequence was not entered in the
        //necessary time
        while (escapeCharsReceived)
        {
          escapeCharsReceived--;
          radioTxBuffer[radioTxHead++] = escapeCharacter;
          radioTxHead %= sizeof(radioTxBuffer);
        }

        //The input data byte will be sent over the long range radio
        radioTxBuffer[radioTxHead++] = incoming;
        radioTxHead %= sizeof(radioTxBuffer);
      }
    } //End process rx buffer
  }

  //Check for escape timeout, more than two seconds have elapsed without entering
  //command mode.  The escape characters must be part of the input stream but were
  //the last characters entered.  Send these characters over the long range radio.
  if (escapeCharsReceived && (millis() - lastEscapeCharEnteredMillis > minEscapeTime_ms)
      && (availableRXBytes() < (sizeof(radioTxBuffer) - maxEscapeCharacters)))
  {
    //Replay the escape characters
    while (escapeCharsReceived)
    {
      escapeCharsReceived--;

      //Transmit the escape character over the long range radio
      radioTxBuffer[radioTxHead++] = escapeCharacter;
      radioTxHead %= sizeof(radioTxBuffer);
    }
  }

  //Print the number of bytes placed into the rxTxBuffer
  if (settings.debugSerial && (radioTxHead != radioHead))
  {
    systemPrint("processSerialInput moved ");
    if (radioTxHead > radioHead)
      systemPrint(radioTxHead - radioHead);
    else
      systemPrint(sizeof(radioTxBuffer) - radioHead + radioTxHead);
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
    blinkSerialTxLed(true); //Turn on LED during serial transmissions

    //Take a break if there are ISRs to attend to
    CheckChannelHopAndKickWatchdog();

    arch.serialWrite(serialTransmitBuffer[txTail]);
    systemFlush(); //Prevent serial hardware from blocking more than this one write

    txTail++;
    txTail %= sizeof(serialTransmitBuffer);
  }
  blinkSerialTxLed(false); //Turn off LED
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

//Determine if received serial data may be sent to the remote system
bool vcSerialMessageReceived()
{
  int8_t channel;
  uint16_t dataBytes;
  uint8_t msgLength;
  int8_t vcDest; //Byte from VC_RADIO_MESSAGE_HEADER
  int8_t vcIndex; //Index into virtualCircuitList

  do
  {
    //Determine if the radio is idle
    if (receiveInProcess(false))
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
    vcDest = radioTxBuffer[NEXT_RADIO_TX_TAIL(1)];
    vcIndex = -1;
    if ((uint8_t)vcDest < (uint8_t)MIN_RX_NOT_ALLOWED)
      vcIndex = vcDest & VCAB_NUMBER_MASK;
    if ((vcIndex < 0) && (vcDest != VC_BROADCAST))
    {
      if (settings.debugSerial || settings.debugTransmit)
      {
        systemPrint("ERROR: Invalid internally generated vcDest ");
        systemPrint(vcDest);
        systemPrintln(", discarding message!");
        outputSerialData(true);
      }

      //Discard this message
      radioTxTail = NEXT_RADIO_TX_TAIL(msgLength);
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
      radioTxTail = NEXT_RADIO_TX_TAIL(msgLength);

      //Nothing to do for invalid addresses or the broadcast address
      if (((uint8_t)vcDest >= (uint8_t)MIN_TX_NOT_ALLOWED) && (vcDest != VC_BROADCAST))
        break;

      //Break the link to this host since message delivery is guarranteed and
      //this message is being discarded
      vcBreakLink(vcDest & VCAB_NUMBER_MASK);
      break;
    }

    //Verify that the destination link is connected
    if ((vcDest != VC_BROADCAST)
        && (virtualCircuitList[vcIndex].vcState < VC_STATE_CONNECTED))
    {
      if (settings.debugSerial || settings.debugTransmit)
      {
        systemPrint("Link not connected ");
        systemPrint(vcIndex);
        systemPrintln(", discarding message!");
        outputSerialData(true);
      }

      //Discard this message
      radioTxTail = NEXT_RADIO_TX_TAIL(msgLength);

      //If the PC is trying to send this message then notify the PC of the delivery failure
      if ((uint8_t)vcDest < (uint8_t)MIN_TX_NOT_ALLOWED)
        vcSendPcAckNack(vcIndex, false);
      break;
    }

    //Print the data ready for transmission
    if (settings.debugSerial)
    {
      systemPrint("Readying ");
      systemPrint(msgLength);
      systemPrintln(" byte for transmission");
      dumpCircularBuffer(radioTxBuffer, radioTxTail, sizeof(radioTxBuffer), msgLength);
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

//Notify the PC of the message delivery failure
void vcSendPcAckNack(int8_t vcIndex, bool ackMsg)
{
  //Build the VC state message
  VC_DATA_ACK_NACK_MESSAGE message;
  message.msgDestVc = vcIndex;

  //Build the message header
  VC_SERIAL_MESSAGE_HEADER header;
  header.start = START_OF_VC_SERIAL;
  header.radio.length = VC_RADIO_HEADER_BYTES + sizeof(message);
  header.radio.destVc = ackMsg ? PC_DATA_ACK : PC_DATA_NACK;
  header.radio.srcVc = myVc;

  //Send the VC state message
  systemWrite((uint8_t *)&header, sizeof(header));
  systemWrite((uint8_t *)&message, sizeof(message));
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
    CheckChannelHopAndKickWatchdog();

    //Skip any garbage in the input stream
    data = serialReceiveBuffer[rxTail];
    if (data != START_OF_VC_SERIAL)
    {
      rxTail = NEXT_RX_TAIL(1);
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
    length = serialReceiveBuffer[NEXT_RX_TAIL(1)];
    if (length < VC_SERIAL_HEADER_BYTES)
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

    //Skip if there is not enough data in the buffer
    if (dataBytes < (length + 1))
      break;

    //Remove the START_OF_VC_SERIAL byte
    rxTail = NEXT_RX_TAIL(1);

    //Get the source and destination virtual circuits
    index = NEXT_RX_TAIL(1);
    vcDest = serialReceiveBuffer[index];
    index = NEXT_RX_TAIL(2);
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
          rxTail = NEXT_RX_TAIL(length);
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
          rxTail = NEXT_RX_TAIL(length);
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
          radioTxBuffer[radioTxHead] = serialReceiveBuffer[rxTail];
          rxTail = NEXT_RX_TAIL(1);
          radioTxHead = NEXT_RADIO_TX_HEAD(1);
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
          rxTail = NEXT_RX_TAIL(length);
          break;
        }

        //Discard the VC header
        length -= VC_RADIO_HEADER_BYTES;
        rxTail = NEXT_RX_TAIL(VC_RADIO_HEADER_BYTES);

        //Move this message into the command buffer
        for (cmd = commandBuffer; length > 0; length--)
        {
          *cmd++ = toupper(serialReceiveBuffer[rxTail]);
          rxTail = NEXT_RX_TAIL(1);
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
        tempSettings = settings;
        checkCommand();
        settings = tempSettings;
        break;
    }

    //Determine how much data is left in the buffer
    dataBytes = availableRXBytes();
  }

  //Take a break if there are ISRs to attend to
  CheckChannelHopAndKickWatchdog();
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
  delayTime = (1000 * 8 * 2) / settings.serialSpeed;
  if (delayTime < 200)
    delayTime = 200;

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
  commandLength = 0;
}
