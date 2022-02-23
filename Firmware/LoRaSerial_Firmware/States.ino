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

        //Check to see if we need to send a ping
        else if ( (millis() - packetTimestamp) > (settings.heartbeatTimeout + random(0, 1000)) //Avoid pinging each other at same time
                  || sentFirstPing == false) //Immediately send pings at POR
        {
          if (receiveInProcess() == false)
          {
            triggerEvent(TRIGGER_NOLINK_SEND_PING);
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
            triggerEvent(TRIGGER_NOLINK_SEND_ACK_PACKET);
            //We're done transmitting our ack packet
            //Yay! Return to normal communication
            packetsLost = 0; //Reset, used for linkLost testing
            digitalWrite(pin_linkLED, HIGH);
            returnToReceiving();
            changeState(RADIO_LINKED_RECEIVING_STANDBY);
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
          transactionComplete = false; //Reset ISR flag
          changeState(RADIO_NO_LINK_RECEIVED_PACKET);
        }

        else if (timeToHop == true) //If the dio1ISR has fired, move to next frequency
          hopChannel();

        else if ((millis() - packetTimestamp) > (packetAirTime + controlPacketAirTime)) //Wait for xmit of packet and ACK response
        {
          //Give up. No ACK recevied.
          randomSeed(radio.randomByte()); //Reseed the random delay between heartbeats
          triggerEvent(TRIGGER_NOLINK_NO_ACK_GIVEUP);
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
          changeState(RADIO_LINKED_RECEIVING_STANDBY);
        }
        else if (packetType == PROCESS_DUPLICATE_PACKET)
        {
          sendAckPacket(); //It's a duplicate. Ack then ignore.
          changeState(RADIO_NO_LINK_TRANSMITTING);
        }
        else if (packetType == PROCESS_CONTROL_PACKET)
        {
          triggerEvent(TRIGGER_NOLINK_IDENT_PACKET);
          sendAckPacket(); //Someone is pinging us
          changeState(RADIO_NO_LINK_TRANSMITTING);
        }
        else if (packetType == PROCESS_DATA_PACKET)
          ; //No data packets allowed when not linked
      }
      break;

    case RADIO_LINKED_RECEIVING_STANDBY:
      {
        if (linkLost())
        {
          digitalWrite(pin_linkLED, LOW);

          //Return to home channel and begin linking process
          returnToReceiving();
          changeState(RADIO_NO_LINK_RECEIVING_STANDBY);
        }

        else if (transactionComplete == true) //If dio0ISR has fired, a packet has arrived
        {
          triggerEvent(TRIGGER_LINK_PACKET_RECEIVED);
          transactionComplete = false; //Reset ISR flag
          changeState(RADIO_LINKED_RECEIVED_PACKET);
        }

        else if (timeToHop == true) //If the dio1ISR has fired, move to next frequency
          hopChannel();

        else if ((millis() - packetTimestamp) > (settings.heartbeatTimeout + random(0, 1000))) //Avoid pinging each other at same time
        {
          if (receiveInProcess() == false)
          {
            randomSeed(radio.randomByte()); //Takes 10ms. Reseed the random delay between heartbeats
            triggerEvent(TRIGGER_LINK_SEND_PING);
            sendPingPacket();
            changeState(RADIO_LINKED_TRANSMITTING);
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
                triggerEvent(TRIGGER_LINK_DATA_PACKET);
                sendDataPacket();
                changeState(RADIO_LINKED_TRANSMITTING);
              }
            }
          }
        } //End processWaitingSerial
      }
      break;

    case RADIO_LINKED_TRANSMITTING:
      {
        if (transactionComplete == true) //If dio0ISR has fired, we are done transmitting
        {
          transactionComplete = false; //Reset ISR flag

          if (expectingAck == true)
          {
            returnToReceiving();
            changeState(RADIO_LINKED_ACK_WAIT);
          }
          else
          {
            triggerEvent(TRIGGER_LINK_SENT_ACK_PACKET);
            returnToReceiving();
            changeState(RADIO_LINKED_RECEIVING_STANDBY);
          }
        }
        else if (timeToHop == true) //If the dio1ISR has fired, move to next frequency
          hopChannel();
      }
      break;

    case RADIO_LINKED_ACK_WAIT:
      {
        if (transactionComplete == true) //If dio0ISR has fired, a packet has arrived
        {
          transactionComplete = false; //Reset ISR flag
          changeState(RADIO_LINKED_RECEIVED_PACKET);
        }

        else if (timeToHop == true) //If the dio1ISR has fired, move to next frequency
          hopChannel();

        //Check to see if we need to retransmit
        if ((millis() - packetTimestamp) > (packetAirTime + controlPacketAirTime)) //Wait for xmit of packet and ACK response
        {
          if (packetSent > settings.maxResends)
          {
            LRS_DEBUG_PRINTLN(F("Packet Lost"));
            packetsLost++;
            totalPacketsLost++;
            returnToReceiving();
            changeState(RADIO_LINKED_RECEIVING_STANDBY);
          }
          else
          {
            if (receiveInProcess() == false)
            {
              triggerEvent(TRIGGER_LINK_PACKET_RESEND);
              packetsResent++;
              sendResendPacket();
              changeState(RADIO_LINKED_TRANSMITTING);
            }
            else
            {
              LRS_DEBUG_PRINTLN("ACK_WAIT: RX In Progress");
              triggerEvent(TRIGGER_RX_IN_PROGRESS);
            }
          }
        }
      }
      break;

    case RADIO_LINKED_RECEIVED_PACKET:
      {
        PacketType packetType = identifyPacketType(); //Look at the packet we just received

        if (packetType == PROCESS_BAD_PACKET || packetType == PROCESS_NETID_MISMATCH)
        {
          triggerEvent(TRIGGER_LINK_BAD_PACKET);
          returnToReceiving();
          changeState(RADIO_LINKED_RECEIVING_STANDBY);
        }
        //This packet is an ack. Are we expecting one?
        else if (packetType == PROCESS_ACK_PACKET)
        {
          triggerEvent(TRIGGER_ACK_PROCESSED); //Trigger for transmission timing

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
          changeState(RADIO_LINKED_RECEIVING_STANDBY);
        }
        else if (packetType == PROCESS_DUPLICATE_PACKET)
        {
          triggerEvent(TRIGGER_LINK_DUPLICATE_PACKET);
          packetsLost = 0; //Reset, used for linkLost testing
          frequencyCorrection += radio.getFrequencyError() / 1000000.0;
          sendAckPacket(); //It's a duplicate. Ack then ignore
          changeState(RADIO_LINKED_TRANSMITTING);
        }
        else if (packetType == PROCESS_CONTROL_PACKET)
        {
          triggerEvent(TRIGGER_LINK_CONTROL_PACKET);
          packetsLost = 0; //Reset, used for linkLost testing
          frequencyCorrection += radio.getFrequencyError() / 1000000.0;
          sendAckPacket(); //Someone is pinging us
          changeState(RADIO_LINKED_TRANSMITTING);
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
          changeState(RADIO_LINKED_TRANSMITTING);
        }
      }
      break;

    case RADIO_BROADCASTING_RECEIVING_STANDBY:
      {
        if (transactionComplete == true) //If dio0ISR has fired, a packet has arrived
        {
          triggerEvent(TRIGGER_BROADCAST_PACKET_RECEIVED);
          transactionComplete = false; //Reset ISR flag
          changeState(RADIO_BROADCASTING_RECEIVED_PACKET);
        }

        else if (timeToHop == true) //If the dio1ISR has fired, move to next frequency
          hopChannel();

        else //Process waiting serial
        {
          //If the radio is available, send any data in the serial buffer over the radio
          if (receiveInProcess() == false)
          {
            if (availableRXBytes()) //If we have bytes
            {
              if (processWaitingSerial() == true) //If we've hit a frame size or frame-timed-out
              {
                triggerEvent(TRIGGER_BROADCAST_DATA_PACKET);
                sendDataPacket();
                changeState(RADIO_BROADCASTING_TRANSMITTING);
              }
            }
          }
        } //End processWaitingSerial

        //Blink Link LED if we've had successful data received or transmitted
        if (millis() - lastPacketReceived < 5000 || millis() - packetTimestamp < 5000)
        {
          if (millis() - lastLinkBlink > 500) //Blink at 2Hz
          {
            lastLinkBlink = millis();
            digitalWrite(pin_linkLED, !digitalRead(pin_linkLED)); //Toggle LED
          }
        }
        else
          digitalWrite(pin_linkLED, LOW); //No link
      }
      break;

    case RADIO_BROADCASTING_TRANSMITTING:
      {
        if (transactionComplete == true) //If dio0ISR has fired, we are done transmitting
        {
          transactionComplete = false; //Reset ISR flag
          returnToReceiving();
          changeState(RADIO_BROADCASTING_RECEIVING_STANDBY); //No ack response when in broadcasting mode
          digitalWrite(pin_activityLED, LOW);
        }

        else if (timeToHop == true) //If the dio1ISR has fired, move to next frequency
          hopChannel();
      }
      break;

    case RADIO_BROADCASTING_RECEIVED_PACKET:
      {
        PacketType packetType = identifyPacketType(); //Look at the packet we just received

        if (packetType == PROCESS_BAD_PACKET || packetType == PROCESS_NETID_MISMATCH)
        {
          returnToReceiving();
          changeState(RADIO_BROADCASTING_RECEIVING_STANDBY);
        }
        else if (packetType == PROCESS_ACK_PACKET)
        {
          frequencyCorrection += radio.getFrequencyError() / 1000000.0;
          returnToReceiving();
          changeState(RADIO_BROADCASTING_RECEIVING_STANDBY);
        }
        else if (packetType == PROCESS_DUPLICATE_PACKET || packetType == PROCESS_CONTROL_PACKET)
        {
          //We should not be receiving control packets, but if we do, just ignore
          frequencyCorrection += radio.getFrequencyError() / 1000000.0;
          returnToReceiving(); //No response when in broadcasting mode
          changeState(RADIO_BROADCASTING_RECEIVING_STANDBY);
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

          frequencyCorrection += radio.getFrequencyError() / 1000000.0;
          returnToReceiving(); //No response when in broadcasting mode
          changeState(RADIO_BROADCASTING_RECEIVING_STANDBY);

          lastPacketReceived = millis(); //Update timestamp for Link LED
        }
      }
      break;

    case RADIO_TRAINING_TRANSMITTING:
      {
        //Transmit with .train = 1 to see if anyone else is sitting on the training channel

      }
      break;
    case RADIO_TRAINING_ACK_WAIT:
      {
        //If we receive an ACK, absorb training data

        //If timeout, create new link data

        //Seed random number based on RF noise. We use Arduino random() because platform specific generation does not matter
        randomSeed(radio.randomByte());

        //Generate new NetID
        uint8_t newNetID = random(0, 256); //Inclusive, exclusive

        //Generate new AES Key. User may not be using AES but we still need both radios to have the same key in case they do enable AES.
        uint8_t newEncryptionKey[16];
        for (int x = 0 ; x < 16 ; x++)
          newEncryptionKey[x] = random(0, 256); //Inclusive, exclusive

        //Avoid NetID checking
        settings.pointToPoint = false;

        changeState(RADIO_TRAINING_RECIEVE_HERE_FIRST);
      }
      break;
    case RADIO_TRAINING_RECIEVE_HERE_FIRST:
      {
        //Wait for ping. Once received, transmit training data
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

  //if (settings.debug == false)
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

    case (RADIO_LINKED_RECEIVING_STANDBY):
      Serial.print(F("State: Receiving Standby "));
      Serial.print(channels[radio.getFHSSChannel()]);
      break;
    case (RADIO_LINKED_RECEIVED_PACKET):
      Serial.print(F("State: Received Packet "));
      Serial.print(channels[radio.getFHSSChannel()]);
      break;
    case (RADIO_LINKED_TRANSMITTING):
      Serial.print(F("State: Transmitting "));
      Serial.print(channels[radio.getFHSSChannel()]);
      break;
    case (RADIO_LINKED_ACK_WAIT):
      Serial.print(F("State: Ack Wait "));
      Serial.print(channels[radio.getFHSSChannel()]);
      break;

    case (RADIO_BROADCASTING_RECEIVING_STANDBY):
      Serial.print(F("State: B-Receiving Standby "));
      Serial.print(channels[radio.getFHSSChannel()]);
      break;
    case (RADIO_BROADCASTING_RECEIVED_PACKET):
      Serial.print(F("State: B-Received Packet "));
      Serial.print(channels[radio.getFHSSChannel()]);
      break;
    case (RADIO_BROADCASTING_TRANSMITTING):
      Serial.print(F("State: B-Transmitting "));
      Serial.print(channels[radio.getFHSSChannel()]);
      break;

    case (RADIO_TRAINING_TRANSMITTING):
      Serial.print(F("State: [Training] TX"));
      break;
    case (RADIO_TRAINING_ACK_WAIT):
      Serial.print(F("State: [Training] Ack Wait"));
      break;
    case (RADIO_TRAINING_RECIEVE_HERE_FIRST):
      Serial.print(F("State: [Training] RX Here First"));
      break;

    default:
      Serial.print(F("Change State Unknown: "));
      Serial.print(radioState);
      break;
  }
  Serial.println();
}
