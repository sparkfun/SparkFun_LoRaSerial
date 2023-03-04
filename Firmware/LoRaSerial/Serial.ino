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
  if (bytesToSend > maxLength)
    bytesToSend = maxLength;

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
    bytesToSend = maxLength;

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
    blinkSerialRxLed(true); //Turn on LED during serial reception

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
    petWDT();
  }

  //Process the serial data
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
  static uint32_t lastEscapeCharEnteredMillis;

  //Process the serial data
  radioHead = radioTxHead;
  while (availableRXBytes()
         && (availableRadioTXBytes() < (sizeof(radioTxBuffer) - maxEscapeCharacters))
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
    petWDT();
    if (timeToHop == true) hopChannel();

    arch.serialWrite(serialTransmitBuffer[txTail]);
    systemFlush(); //Prevent serial hardware from blocking more than this one write

    txTail++;
    txTail %= sizeof(serialTransmitBuffer);
  }
  blinkSerialTxLed(false); //Turn off LED
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
  radioTxHead = radioTxTail;
  txHead = txTail;
  commandRXHead = commandRXTail;
  commandTXHead = commandTXTail;
  endOfTxData = &outgoingPacket[headerBytes];
  commandLength = 0;
}
