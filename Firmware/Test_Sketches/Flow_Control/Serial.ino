//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//Serial RX - Data arriving at the USB or serial port
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Returns true if CTS is asserted (host says it's ok to send data)
bool isCTS()
{
  return (digitalRead(pin_cts) == (INVERT_RTS_CTS ? LOW : HIGH));
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//Serial TX - Data being sent to the USB or serial port
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Return number of bytes sitting in the serial transmit buffer
uint16_t availableTXBytes()
{
  if (txHead >= txTail)
    return (txHead - txTail);
  return (sizeof(serialTransmitBuffer) - txTail + txHead);
}

//Returns true if CTS is asserted (host says it's ok to send data)
void updateRTS(bool assertRTS)
{
  rtsAsserted = assertRTS;
  digitalWrite(pin_rts, assertRTS ^ INVERT_RTS_CTS);
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//Move serial data through the system
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//See if there's any serial from the remote radio that needs printing
//Record any characters to the receive buffer
//Scan for escape characters
void updateSerial()
{
  bool bufferFull;
  static uint32_t bufferFullCount;
  static bool copyStarted;
  static uint32_t delayMillis;
  int x;
  static uint32_t lastTime;
  static uint32_t totalDelayMillis;

  //Determine if bytea are available for transmission
  while (availableTXBytes() && isCTS() && ((millis() - lastTime) >= delayMillis))
  {
    //Account for the delay time.
    lastTime = millis();
    totalDelayMillis += delayMillis;
    delayMillis = random(1, 15);
    copyStarted = true;

    //Turn on LED during serial transmissions
    txLED(true);

    //Send a byte to the host
    samdSerialWrite(serialTransmitBuffer[txTail++]);
    txTail %= sizeof(serialTransmitBuffer);

    //Assert RTS when enough space is available
    if (availableTXBytes() <= (sizeof(serialTransmitBuffer) / 2))
      updateRTS(true); //Ok to send more

    //Since the UART does not have RTS and CTS, prevent loss of serial data
    //by sending one character at a time
    samdSerialFlush();

    //Turn off LED
    txLED(false);
  }

  //Look for local incoming serial
  if (samdSerialAvailable() && rtsAsserted)
  {
    do
    {
      rxLED(true); //Turn on LED during serial reception

      //Deassert RTS when the buffer becomes full
      if (availableTXBytes() >= (sizeof(serialTransmitBuffer) - 32))
      {
        updateRTS(false); //Don't give me more
        bufferFullCount += 1;
      }

      //Get the next character
      serialTransmitBuffer[txHead++] = samdSerialRead();
      txHead %= sizeof(serialTransmitBuffer);

      rxLED(false); //Turn off LED
    } while (samdSerialAvailable() && rtsAsserted);
  }

  if (copyStarted && ((millis() - lastTime) >= (5 * 1000)))
  {
    copyStarted = false;
    systemPrint("Delay time: ");
    systemPrint(totalDelayMillis);
    systemPrint(" mSec");
    systemPrintln();
    systemPrint("RX Buffer Full Count: ");
    systemPrint(bufferFullCount);
    systemPrintln();
    bufferFullCount = 0;
    totalDelayMillis = 0;
  }
}

void txLED(bool illuminate)
{
  if (pin_txLED != PIN_UNDEFINED)
  {
    if (illuminate == true)
      digitalWrite(pin_txLED, HIGH);
    else
      digitalWrite(pin_txLED, LOW);
  }
}

void rxLED(bool illuminate)
{
  if (pin_rxLED != PIN_UNDEFINED)
  {
    if (illuminate == true)
      digitalWrite(pin_rxLED, HIGH);
    else
      digitalWrite(pin_rxLED, LOW);
  }
}
