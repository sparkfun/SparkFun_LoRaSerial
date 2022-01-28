void updateRadioState()
{
  switch (radioState)
  {
    case RADIO_NO_LINK_RECEIVING_STANDBY:
      {
        if (transactionComplete == true) //If dio0ISR has fired, a packet has arrived
        {
          transactionComplete = false; //Reset ISR flag
          changeState(RADIO_NO_LINK_RECEIVED_PACKET);
        }

        else if (timeToHop == true) //If the dio1ISR has fired, move to next frequency
          hopChannel();

        //If we have started hopping, and the radio no longer is receiving a packet, then the packet was corrupt.
        //Return to receiving
        else if (hopsCompleted > 0)
        {
          if (receiveInProcess() == false)
          {
            LRS_DEBUG_PRINTLN("Noise triggered hop");
            returnToReceiving(); //Reset our channel to 0
            changeState(RADIO_NO_LINK_RECEIVING_STANDBY);
          }
        }

        //Check to see if we need to send a ping
        else if ( (millis() - packetTimestamp) > (settings.heartbeatTimeout + random(0, 1000)) //Avoid pinging each other at same time
                  || sentFirstPing == false) //Immediately send pings at POR
        {
          if (receiveInProcess() == false)
          {
            if (settings.debug == true)
            {
              digitalWrite(pin_trigger, LOW);
              delayMicroseconds(500);
              digitalWrite(pin_trigger, HIGH);
            }
            sentFirstPing = true;
            sendPingPacket();
            transactionComplete = false; //Reset ISR flag
            changeState(RADIO_NO_LINK_TRANSMITTING);
          }
          else
            LRS_DEBUG_PRINTLN("NO_LINK_RECEIVING_STANDBY: RX In Progress");
        }
      }
      break;

    case RADIO_NO_LINK_TRANSMITTING:
      {
        if (transactionComplete == true) //If dio0ISR has fired, we are done transmitting
        {
          transactionComplete = false; //Reset ISR flag

          if (expectingAck == false)
          {
            if (settings.debug == true)
            {
              digitalWrite(pin_trigger, LOW);
              delayMicroseconds(1000);
              digitalWrite(pin_trigger, HIGH);
            }
            //We're done transmitting our ack packet
            //Yay! Return to normal communication
            packetsLost = 0; //Reset, used for linkLost testing
            digitalWrite(pin_linkLED, HIGH);
            returnToReceiving();
            changeState(RADIO_RECEIVING_STANDBY);
          }
          else
          {
            returnToReceiving();
            changeState(RADIO_NO_LINK_ACK_WAIT);
          }
        }
        else if (timeToHop == true) //If the dio1ISR has fired, move to next frequency
          hopChannel();
      }
      break;

    case RADIO_NO_LINK_ACK_WAIT:
      {
        if (transactionComplete == true) //If dio0ISR has fired, a packet has arrived
        {
          randomSeed(radio.randomByte()); //Reseed the random delay between heartbeats

          transactionComplete = false; //Reset ISR flag
          changeState(RADIO_NO_LINK_RECEIVED_PACKET);
        }

        else if (timeToHop == true) //If the dio1ISR has fired, move to next frequency
          hopChannel();

        //If we have started hopping, and the radio no longer is receiving a packet, then the packet was corrupt.
        //Return to receiving
        else if (hopsCompleted > 0)
        {
          if (receiveInProcess() == false)
          {
            if (settings.debug == true)
            {

              digitalWrite(pin_trigger, LOW);
              delayMicroseconds(1500);
              digitalWrite(pin_trigger, HIGH);
            }
            LRS_DEBUG_PRINTLN("Noise triggered hop");
            returnToReceiving(); //Reset our channel to 0
            changeState(RADIO_NO_LINK_RECEIVING_STANDBY);
          }
        }

        else if ((millis() - packetTimestamp) > (packetAirTime + controlPacketAirTime)) //Wait for xmit of packet and ACK response
        {
          //Give up. No ACK recevied.
          if (settings.debug == true)
          {

            digitalWrite(pin_trigger, LOW);
            delayMicroseconds(2000);
            digitalWrite(pin_trigger, HIGH);
          }
          returnToReceiving();
          changeState(RADIO_NO_LINK_RECEIVING_STANDBY);
        }
      }
      break;

    case RADIO_NO_LINK_RECEIVED_PACKET:
      {
        PacketType packetType = identifyPacketType(); //Look at the packet we just received

        if (packetType == PROCESS_BAD_PACKET || packetType == PROCESS_NETID_MISMATCH)
        {
          returnToReceiving(); //Ignore
          changeState(RADIO_NO_LINK_RECEIVING_STANDBY);
        }
        else if (packetType == PROCESS_ACK_PACKET)
        {
          //Yay! Return to normal communication
          packetsLost = 0; //Reset, used for linkLost testing
          digitalWrite(pin_linkLED, HIGH);
          returnToReceiving();
          changeState(RADIO_RECEIVING_STANDBY);
        }
        else if (packetType == PROCESS_DUPLICATE_PACKET)
        {
          sendAckPacket(); //It's a duplicate. Ack then ignore.
          changeState(RADIO_NO_LINK_TRANSMITTING);
        }
        else if (packetType == PROCESS_CONTROL_PACKET)
        {
          if (settings.debug == true)
          {

            digitalWrite(pin_trigger, LOW);
            delayMicroseconds(2500);
            digitalWrite(pin_trigger, HIGH);
          }
          sendAckPacket(); //Someone is pinging us
          changeState(RADIO_NO_LINK_TRANSMITTING);
        }
        else if (packetType == PROCESS_DATA_PACKET)
          ; //No data packets allowed when not linked
      }
      break;

    case RADIO_RECEIVING_STANDBY:
      {
        if (linkLost())
        {
          digitalWrite(pin_linkLED, LOW);

          //Return to home channel and begin linking process
          currentChannel = 0;
          radio.setFrequency(channels[currentChannel]);
          radio.clearFHSSInt();
          timeToHop = false;

          returnToReceiving();
          changeState(RADIO_NO_LINK_RECEIVING_STANDBY);
        }

        else if (transactionComplete == true) //If dio0ISR has fired, a packet has arrived
        {
          transactionComplete = false; //Reset ISR flag
          changeState(RADIO_RECEIVED_PACKET);
        }

        else if (timeToHop == true) //If the dio1ISR has fired, move to next frequency
          hopChannel();

        //If we have started hopping, and the radio no longer is receiving a packet, then the packet was corrupt.
        //Return to receiving
        else if (hopsCompleted > 0)
        {
          if (receiveInProcess() == false)
          {
            LRS_DEBUG_PRINTLN("Noise triggered hop");
            returnToReceiving(); //Reset our channel to 0
            changeState(RADIO_RECEIVING_STANDBY);
          }
        }

        else if ((millis() - packetTimestamp) > (settings.heartbeatTimeout + random(0, 1000))) //Avoid pinging each other at same time
        {
          if (receiveInProcess() == false)
          {
            if (settings.debug == true)
            {
              digitalWrite(pin_trigger, LOW);
              delayMicroseconds(4500);
              digitalWrite(pin_trigger, HIGH);
            }
            sendPingPacket();
            changeState(RADIO_TRANSMITTING);
          }
          else
            LRS_DEBUG_PRINTLN("RECEIVING_STANDBY: RX In Progress");
        }

        else //Process waiting serial
        {
          //If the radio is available, send any data in the serial buffer over the radio
          if (receiveInProcess() == false)
          {
            if (availableRXBytes()) //If we have bytes
            {
              if (processWaitingSerial() == true) //If we've hit a frame size or frame-timed-out
              {
                //Trigger for transmission timing
                digitalWrite(pin_trigger, LOW);
                delayMicroseconds(25);
                digitalWrite(pin_trigger, HIGH);
                if (settings.debug == true)
                {
                  digitalWrite(pin_trigger, LOW);
                  delayMicroseconds(3000);
                  digitalWrite(pin_trigger, HIGH);
                }
                sendDataPacket();
                changeState(RADIO_TRANSMITTING);
              }
            }
          }
        } //End processWaitingSerial
      }
      break;

    case RADIO_TRANSMITTING:
      {
        if (transactionComplete == true) //If dio0ISR has fired, we are done transmitting
        {
          transactionComplete = false; //Reset ISR flag

          if (expectingAck == true)
          {
            returnToReceiving();
            changeState(RADIO_ACK_WAIT);
          }
          else
          {
            if (settings.debug == true)
            {
              digitalWrite(pin_trigger, LOW);
              delayMicroseconds(3500);
              digitalWrite(pin_trigger, HIGH);
            }
            returnToReceiving();
            changeState(RADIO_RECEIVING_STANDBY);
          }
        }
        else if (timeToHop == true) //If the dio1ISR has fired, move to next frequency
          hopChannel();
      }
      break;

    case RADIO_ACK_WAIT:
      {
        if (transactionComplete == true) //If dio0ISR has fired, a packet has arrived
        {
          transactionComplete = false; //Reset ISR flag
          changeState(RADIO_RECEIVED_PACKET);
        }

        else if (timeToHop == true) //If the dio1ISR has fired, move to next frequency
          hopChannel();

        //If we have started hopping, and the radio no longer is receiving a packet, then the packet was corrupt.
        //Return to receiving
        else if (hopsCompleted > 0)
        {
          if (receiveInProcess() == false)
          {
            //The processor can get to a receiveInProgress = false before the radio triggers the receiveComplete ISR
            //If we are using FHSS then check channel freq. If radio has jumped back to channel 0 then we know a transaction is complete.
            if (settings.frequencyHop == true)
            {
              if (radio.getFHSSChannel() != 0)
              {
                digitalWrite(pin_trigger, LOW);
                delayMicroseconds(75);
                digitalWrite(pin_trigger, HIGH);
                LRS_DEBUG_PRINTLN("Noise triggered hop");
                returnToReceiving(); //Reset our channel to 0
                changeState(RADIO_RECEIVING_STANDBY);
              }
            }
            else
            {
              //If we're not using FHSS, use small delay.
              delay(5); //Small wait before checking RX complete ISR again
              if (transactionComplete == false)
              {
                LRS_DEBUG_PRINTLN("Noise triggered hop");
                returnToReceiving(); //Reset our channel to 0
                changeState(RADIO_RECEIVING_STANDBY);
              }
            }
          } //End receive in progress
        } //End noise triggered hop testing

        //Check to see if we need to retransmit
        if ((millis() - packetTimestamp) > (packetAirTime + controlPacketAirTime)) //Wait for xmit of packet and ACK response
        {
          if (packetSent > maxResends)
          {
            LRS_DEBUG_PRINTLN(F("Packet Lost"));
            packetsLost++;
            totalPacketsLost++;
            returnToReceiving();
            changeState(RADIO_RECEIVING_STANDBY);
          }
          else
          {
            if (receiveInProcess() == false)
            {
              LRS_DEBUG_PRINTLN(F("Packet Resend"));
              if (settings.debug == true)
              {
                digitalWrite(pin_trigger, LOW);
                delayMicroseconds(4000);
                digitalWrite(pin_trigger, HIGH);
              }
              packetsResent++;
              sendResendPacket();
              changeState(RADIO_TRANSMITTING);
            }
            else
              LRS_DEBUG_PRINTLN("ACK_WAIT: RX In Progress");
          }
        }
      }
      break;

    case RADIO_RECEIVED_PACKET:
      {
        PacketType packetType = identifyPacketType(); //Look at the packet we just received

        if (packetType == PROCESS_BAD_PACKET || packetType == PROCESS_NETID_MISMATCH)
        {
          returnToReceiving();
          changeState(RADIO_RECEIVING_STANDBY);
        }
        //This packet is an ack. Are we expecting one?
        else if (packetType == PROCESS_ACK_PACKET)
        {
          //Trigger for transmission timing
          digitalWrite(pin_trigger, LOW);
          delayMicroseconds(5);
          digitalWrite(pin_trigger, HIGH);

          if (settings.displayPacketQuality == true)
          {
            Serial.println();
            Serial.print(F("R:"));
            Serial.print(radio.getRSSI());
            Serial.print(F("\tS:"));
            Serial.print(radio.getSNR());
            Serial.print(F("\tfE:"));
            Serial.print(radio.getFrequencyError());
            Serial.println();
          }
          packetsLost = 0; //Reset, used for linkLost testing
          frequencyCorrection += radio.getFrequencyError() / 1000000.0;
          returnToReceiving();
          changeState(RADIO_RECEIVING_STANDBY);
        }
        else if (packetType == PROCESS_DUPLICATE_PACKET)
        {
          packetsLost = 0; //Reset, used for linkLost testing
          frequencyCorrection += radio.getFrequencyError() / 1000000.0;
          sendAckPacket(); //It's a duplicate. Ack then ignore
          changeState(RADIO_TRANSMITTING);
        }
        else if (packetType == PROCESS_CONTROL_PACKET)
        {
          packetsLost = 0; //Reset, used for linkLost testing
          frequencyCorrection += radio.getFrequencyError() / 1000000.0;
          sendAckPacket(); //Someone is pinging us
          changeState(RADIO_TRANSMITTING);
        }
        else if (packetType == PROCESS_DATA_PACKET)
        {
          if (settings.displayPacketQuality == true)
          {
            Serial.println();
            Serial.print(F("R:"));
            Serial.print(radio.getRSSI());
            Serial.print(F("\tS:"));
            Serial.print(radio.getSNR());
            Serial.print(F("\tfE:"));
            Serial.print(radio.getFrequencyError());
            Serial.println();
          }

          //Move this packet into the tx buffer
          //We cannot directly print here because Serial.print is blocking
          for (int x = 0 ; x < lastPacketSize ; x++)
          {
            serialTransmitBuffer[txHead++] = lastPacket[x];
            txHead %= sizeof(serialTransmitBuffer);
          }

          packetsLost = 0; //Reset, used for linkLost testing
          frequencyCorrection += radio.getFrequencyError() / 1000000.0;
          sendAckPacket(); //Transmit ACK
          changeState(RADIO_TRANSMITTING);
        }
      }
      break;
    default:
      {
        Serial.println(F("Unknown state"));
      }
      break;
  }
}

//Change states and print the new state
void changeState(RadioStates newState)
{
  radioState = newState;

  if (settings.debug == false)
    return;

  //Debug print
  switch (radioState)
  {
    case (RADIO_NO_LINK_RECEIVING_STANDBY):
      Serial.print(F("State: [No Link] Receiving Standby"));
      break;
    case (RADIO_NO_LINK_RECEIVED_PACKET):
      Serial.print(F("State: [No Link] Received Packet"));
      break;
    case (RADIO_NO_LINK_TRANSMITTING):
      Serial.print(F("State: [No Link] Transmitting"));
      break;
    case (RADIO_NO_LINK_ACK_WAIT):
      Serial.print(F("State: [No Link] Ack Wait"));
      break;

    case (RADIO_RECEIVING_STANDBY):
      Serial.print(F("State: Receiving Standby "));
      Serial.print(channels[currentChannel]);
      break;
    case (RADIO_RECEIVED_PACKET):
      Serial.print(F("State: Received Packet "));
      Serial.print(channels[currentChannel]);
      break;
    case (RADIO_TRANSMITTING):
      Serial.print(F("State: Transmitting "));
      Serial.print(channels[currentChannel]);
      break;
    case (RADIO_ACK_WAIT):
      Serial.print(F("State: Ack Wait "));
      Serial.print(channels[currentChannel]);
      break;

    default:
      Serial.print(F("Change State Unknown: "));
      Serial.print(radioState);
      break;
  }
  Serial.println();
}
