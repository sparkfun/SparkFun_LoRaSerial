//Return number of bytes sitting in the serial receive buffer
uint16_t availableBytes()
{
  if (head > tail) return (head - tail);
  return (sizeof(serialReceiveBuffer) - tail + head);
}

//Record any characters to the receive buffer
//Scan for escape characters
void updateSerial()
{
  if (Serial.available())
  {
    if (availableBytes() == sizeof(serialReceiveBuffer) - 1)
    {
      //Buffer full! Don't read bytes.
      if (pin_rts != 255)
        digitalWrite(pin_rts, LOW); //Don't give me more
    }
    else
    {
      if (pin_rts != 255)
        digitalWrite(pin_rts, HIGH); //Ok to send more

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

      serialReceiveBuffer[head++] = incoming; //Push char to holding buffer
      head %= sizeof(serialReceiveBuffer);
    } //End buffer available
  } //End Serial.available()
}
