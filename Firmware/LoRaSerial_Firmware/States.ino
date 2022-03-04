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

        if (packetType == PROCESS_ACK_PACKET)
        {
          //This packet is an ack. Return to receiving.
          triggerEvent(TRIGGER_ACK_PROCESSED); //Trigger for transmission timing

          if (settings.displayPacketQuality == true)
          {
            systemPrintln();
            systemPrint(F("R:"));
            systemPrint(radio.getRSSI());
            systemPrint(F("\tS:"));
            systemPrint(radio.getSNR());
            systemPrint(F("\tfE:"));
            systemPrint(radio.getFrequencyError());
            systemPrintln();
          }
          packetsLost = 0; //Reset, used for linkLost testing
          frequencyCorrection += radio.getFrequencyError() / 1000000.0;
          returnToReceiving();
          changeState(RADIO_LINKED_RECEIVING_STANDBY);
        }
        else if (packetType == PROCESS_DUPLICATE_PACKET)
        {
          //It's a duplicate. Ack then throw data away.
          triggerEvent(TRIGGER_LINK_DUPLICATE_PACKET);
          packetsLost = 0; //Reset, used for linkLost testing
          frequencyCorrection += radio.getFrequencyError() / 1000000.0;
          sendAckPacket();
          changeState(RADIO_LINKED_TRANSMITTING);
        }
        else if (packetType == PROCESS_CONTROL_PACKET)
        {
          //Someone is pinging us. Ack back.
          triggerEvent(TRIGGER_LINK_CONTROL_PACKET);
          packetsLost = 0; //Reset, used for linkLost testing
          frequencyCorrection += radio.getFrequencyError() / 1000000.0;
          sendAckPacket();
          changeState(RADIO_LINKED_TRANSMITTING);
        }
        else if (packetType == PROCESS_DATA_PACKET)
        {
          //Pull data from packet and move into outbound serial buffer
          if (settings.displayPacketQuality == true)
          {
            systemPrintln();
            systemPrint(F("R:"));
            systemPrint(radio.getRSSI());
            systemPrint(F("\tS:"));
            systemPrint(radio.getSNR());
            systemPrint(F("\tfE:"));
            systemPrint(radio.getFrequencyError());
            systemPrintln();
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
        else if (packetType == PROCESS_COMMAND_CONTROL_PACKET)
        {
          //Someone is pinging us with a empty command packet, send settings back
          triggerEvent(TRIGGER_COMMAND_CONTROL_PACKET);
          packetsLost = 0; //Reset, used for linkLost testing
          sendCommandPacketAck();
          changeState(RADIO_LINKED_COMMAND_RECEIVING_STANDBY); //Move to command receive, standby for new settings
        }
        else //packetType == PROCESS_BAD_PACKET, packetType == PROCESS_NETID_MISMATCH
        {
          //Packet type not supported in this state
          triggerEvent(TRIGGER_LINK_BAD_PACKET);
          returnToReceiving();
          changeState(RADIO_LINKED_RECEIVING_STANDBY);
        }
      }
      break;



    case RADIO_LINKED_COMMAND_RECEIVING_STANDBY:
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
          triggerEvent(TRIGGER_COMMAND_PACKET_RECEIVED);
          transactionComplete = false; //Reset ISR flag
          changeState(RADIO_LINKED_COMMAND_RECEIVED_PACKET);
        }

        else if (timeToHop == true) //If the dio1ISR has fired, move to next frequency
          hopChannel();

        else if ((millis() - packetTimestamp) > (settings.heartbeatTimeout + random(0, 1000))) //Avoid pinging each other at same time
        {
          if (receiveInProcess() == false)
          {
            randomSeed(radio.randomByte()); //Takes 10ms. Reseed the random delay between heartbeats
            triggerEvent(TRIGGER_COMMAND_SEND_PING);
            sendPingPacket();
            changeState(RADIO_LINKED_COMMAND_TRANSMITTING);
          }
          else
            LRS_DEBUG_PRINTLN("RECEIVING_STANDBY: RX In Progress");
        }
      }
      break;

    case RADIO_LINKED_COMMAND_TRANSMITTING:
      {
        if (transactionComplete == true) //If dio0ISR has fired, we are done transmitting
        {
          transactionComplete = false; //Reset ISR flag

          if (expectingAck == true)
          {
            returnToReceiving();
            changeState(RADIO_LINKED_COMMAND_ACK_WAIT);
          }
          else
          {
            triggerEvent(TRIGGER_COMMAND_SENT_ACK_PACKET);
            returnToReceiving();
            changeState(RADIO_LINKED_COMMAND_RECEIVING_STANDBY);
          }
        }
        else if (timeToHop == true) //If the dio1ISR has fired, move to next frequency
          hopChannel();
      }
      break;

    case RADIO_LINKED_COMMAND_ACK_WAIT:
      {
        if (transactionComplete == true) //If dio0ISR has fired, a packet has arrived
        {
          transactionComplete = false; //Reset ISR flag
          changeState(RADIO_LINKED_COMMAND_RECEIVED_PACKET);
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
            changeState(RADIO_LINKED_COMMAND_RECEIVING_STANDBY);
          }
          else
          {
            if (receiveInProcess() == false)
            {
              triggerEvent(TRIGGER_COMMAND_PACKET_RESEND);
              packetsResent++;
              sendResendPacket();
              changeState(RADIO_LINKED_COMMAND_TRANSMITTING);
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

    case RADIO_LINKED_COMMAND_RECEIVED_PACKET:
      {
        PacketType packetType = identifyPacketType(); //Look at the packet we just received

        if (packetType == PROCESS_ACK_PACKET)
        {
          //If the remote unit has pinged us with a normal packet, we are not exchanging new settings
          //Return to normal, unlinked operation
          packetsLost = 0; //Reset, used for linkLost testing
          frequencyCorrection += radio.getFrequencyError() / 1000000.0;
          returnToReceiving();
          changeState(RADIO_NO_LINK_RECEIVING_STANDBY);
        }
        else if (packetType == PROCESS_DUPLICATE_PACKET)
        {
          //It's a duplicate. Ack then throw data away.
          packetsLost = 0; //Reset, used for linkLost testing
          frequencyCorrection += radio.getFrequencyError() / 1000000.0;
          sendAckPacket();
          changeState(RADIO_LINKED_COMMAND_TRANSMITTING);
        }

        else if (packetType == PROCESS_COMMAND_CONTROL_PACKET)
        {
          //Someone is pinging us with a empty command packet, send settings back
          triggerEvent(TRIGGER_COMMAND_CONTROL_PACKET);
          packetsLost = 0; //Reset, used for linkLost testing
          sendCommandPacketAck(); //Respond with setting data, ACK = 1
          changeState(RADIO_LINKED_COMMAND_TRANSMITTING);
        }
        else if (packetType == PROCESS_COMMAND_CONTROL_PACKET_ACK)
        {
          Serial.println("Settings received!");
          
          //Remote has responded with its settings
          triggerEvent(TRIGGER_COMMAND_CONTROL_PACKET_ACK);
          packetsLost = 0; //Reset, used for linkLost testing

          //Error check
          if (lastPacketSize == sizeof(settings))
          {
            //Record these settings locally. Will be modified by commands.
            movePacketToSettings(remoteSettings, lastPacket);
          }

          returnToReceiving();
          changeState(RADIO_LINKED_COMMAND_RECEIVING_STANDBY);
        }
        else if (packetType == PROCESS_COMMAND_DATA_PACKET)
        {
          //Remote unit is giving us new settings to go to, now
          //Absorb settings data, reconfigure radio with new settings, go to standby
          triggerEvent(TRIGGER_TRAINING_DATA_PACKET);
          packetsLost = 0; //Reset, used for linkLost testing

          //Error check
          if (lastPacketSize == sizeof(settings))
          {
            //Fire off ack packet
            sendCommandDataPacketAck();

            //Wait for transmit completion
            while (transactionComplete == false) //If dio0ISR has fired, a packet has arrived
            {
              if ((millis() - packetTimestamp) > (packetAirTime + controlPacketAirTime)) //Wait for xmit of packet and ACK response
              {
                LRS_DEBUG_PRINTLN("Timeout");
                break;
              }
            }

            //Apply new settings
            movePacketToSettings(settings, lastPacket);

            generateHopTable(); //Generate frequency table based on randomByte

            configureRadio(); //Generate freq table, setup radio, go to receiving, change state to standby

            returnToReceiving();

            if (settings.pointToPoint == true)
              changeState(RADIO_NO_LINK_RECEIVING_STANDBY);
            else
              changeState(RADIO_BROADCASTING_RECEIVING_STANDBY);

            LRS_DEBUG_PRINTLN("Ending Remote settings");
          }
          else
          {
            //This is very bad indeed. Give up and go home.
            returnToReceiving();

            if (settings.pointToPoint == true)
              changeState(RADIO_NO_LINK_RECEIVING_STANDBY);
            else
              changeState(RADIO_BROADCASTING_RECEIVING_STANDBY);
          }

        }

        else if (packetType == PROCESS_COMMAND_DATA_PACKET_ACK)
        {
          //A remote unit has ack'd the delivery of a settings packet
          settingsDelivered++;
          triggerEvent(TRIGGER_COMMAND_DATA_PACKET_ACK);
          packetsLost = 0; //Reset, used for linkLost testing
          //We've successfully delivered the settings to the remote unit
          //Apply current settings to radio and return to normal operation

          generateHopTable(); //Generate frequency table based on randomByte

          configureRadio(); //Generate freq table, setup radio, go to receiving, change state to standby

          returnToReceiving();

          serialState = RADIO_SERIAL_PASSTHROUGH;

          if (settings.pointToPoint == true)
            changeState(RADIO_NO_LINK_RECEIVING_STANDBY);
          else
            changeState(RADIO_BROADCASTING_RECEIVING_STANDBY);

          changeState(RADIO_LINKED_RECEIVING_STANDBY);

          //          if(settingsDelivered == 3)
          //          {
          //            //We've successfully delivered the settings to the remote unit
          //            //Apply new settings and return to normal operation
          //            changeState(RADIO_LINKED_RECEIVING_STANDBY);
          //          }
          //          else
          //          {
          //            //Continue sending settings
          //            sendSettingsDataPacket(false); //This new settings to be used, send data with ack = 0
          //          }
        }

        else //PROCESS_BAD_PACKET, PROCESS_NETID_MISMATCH
        {
          Serial.print("Unknown packet: ");
          if(packetType == PROCESS_BAD_PACKET) Serial.print("Bad packet");
          if(packetType == PROCESS_NETID_MISMATCH) Serial.print("Mismatch ID");
          Serial.println();
          
          //This packet type is not supported in this state
          triggerEvent(TRIGGER_LINK_BAD_PACKET);
          returnToReceiving();
          changeState(RADIO_LINKED_COMMAND_RECEIVING_STANDBY);
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

        if (packetType == PROCESS_ACK_PACKET)
        {
          //We should not be receiving ack packets, but if we do, just ignore
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
            systemPrintln();
            systemPrint(F("R:"));
            systemPrint(radio.getRSSI());
            systemPrint(F("\tS:"));
            systemPrint(radio.getSNR());
            systemPrint(F("\tfE:"));
            systemPrint(radio.getFrequencyError());
            systemPrintln();
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
        else //PROCESS_BAD_PACKET, PROCESS_NETID_MISMATCH
        {
          //This packet type is not supported in this state
          returnToReceiving();
          changeState(RADIO_BROADCASTING_RECEIVING_STANDBY);
        }

      }
      break;

    case RADIO_TRAINING_TRANSMITTING:
      {
        if (transactionComplete == true) //If dio0ISR has fired, we are done transmitting
        {
          transactionComplete = false; //Reset ISR flag
          returnToReceiving();
          changeState(RADIO_TRAINING_ACK_WAIT);
          digitalWrite(pin_activityLED, LOW);
        }
      }
      break;
    case RADIO_TRAINING_ACK_WAIT:
      {
        //If we receive an ACK, absorb training data
        if (transactionComplete == true) //If dio0ISR has fired, a packet has arrived
        {
          transactionComplete = false; //Reset ISR flag
          changeState(RADIO_TRAINING_RECEIVED_PACKET);
        }

        //If timeout, create new link data and return to receive, and wait for training ping from remote
        if ((millis() - packetTimestamp) > (packetAirTime + controlPacketAirTime)) //Wait for xmit of packet and ACK response
        {
          triggerEvent(TRIGGER_TRAINING_NO_ACK);
          generateTrainingSettings();
          returnToReceiving();
          changeState(RADIO_TRAINING_RECEIVING_HERE_FIRST);
        }
      }
      break;
    case RADIO_TRAINING_RECEIVING_HERE_FIRST:
      {
        //Wait for ping. Once received, transmit training data
        if (transactionComplete == true) //If dio0ISR has fired, a packet has arrived
        {
          transactionComplete = false; //Reset ISR flag
          changeState(RADIO_TRAINING_RECEIVED_PACKET);
        }
      }
      break;

    case RADIO_TRAINING_RECEIVED_PACKET:
      {
        PacketType packetType = identifyPacketType(); //Look at the packet we just received

        if (packetType == PROCESS_TRAINING_CONTROL_PACKET)
        {
          triggerEvent(TRIGGER_TRAINING_CONTROL_PACKET);
          packetsLost = 0; //Reset, used for linkLost testing
          sendTrainingDataPacket(); //Someone is pinging us, send training data back

          //Wait for transmission to complete before ending training
          while (transactionComplete == false) //If dio0ISR has fired, a packet has arrived
          {
            if ((millis() - packetTimestamp) > (packetAirTime + controlPacketAirTime)) //Wait for xmit of packet and ACK response
            {
              LRS_DEBUG_PRINTLN("Timeout");
              break;
            }
          }
          LRS_DEBUG_PRINTLN("Ending Training");

          endTraining(false); //We do not have data to apply to settings
        }

        else if (packetType == PROCESS_TRAINING_DATA_PACKET)
        {
          //The waiting node has responded with a data packet
          //Absorb training data and then return to normal operation
          triggerEvent(TRIGGER_TRAINING_DATA_PACKET);
          packetsLost = 0; //Reset, used for linkLost testing
          endTraining(true); //Apply data from packet to settings
        }

        //During training, only training packets are valid
        else //PROCESS_BAD_PACKET, PROCESS_NETID_MISMATCH, PROCESS_ACK_PACKET, PROCESS_CONTROL_PACKET, PROCESS_DATA_PACKET
        {
          triggerEvent(TRIGGER_TRAINING_BAD_PACKET);
          returnToReceiving();
          changeState(RADIO_TRAINING_RECEIVING_HERE_FIRST);
        }
      }
      break;

    default:
      {
        systemPrintln(F("Unknown state"));
      }
      break;
  }
}

//Return true if the radio is in a linked state
//This is used for determining if we can do remote AT commands or not
bool isLinked()
{
  if (radioState == RADIO_LINKED_RECEIVING_STANDBY
      || radioState == RADIO_LINKED_TRANSMITTING
      || radioState == RADIO_LINKED_ACK_WAIT
      || radioState == RADIO_LINKED_RECEIVED_PACKET

      || radioState == RADIO_LINKED_COMMAND_RECEIVING_STANDBY
      || radioState == RADIO_LINKED_COMMAND_TRANSMITTING
      || radioState == RADIO_LINKED_COMMAND_ACK_WAIT
      || radioState == RADIO_LINKED_COMMAND_RECEIVED_PACKET
      )
    return (true);

  return (false);
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
      systemPrint(F("State: [No Link] Receiving Standby"));
      break;
    case (RADIO_NO_LINK_RECEIVED_PACKET):
      systemPrint(F("State: [No Link] Received Packet"));
      break;
    case (RADIO_NO_LINK_TRANSMITTING):
      systemPrint(F("State: [No Link] Transmitting"));
      break;
    case (RADIO_NO_LINK_ACK_WAIT):
      systemPrint(F("State: [No Link] Ack Wait"));
      break;

    case (RADIO_LINKED_RECEIVING_STANDBY):
      systemPrint(F("State: Receiving Standby "));
      systemPrint(channels[radio.getFHSSChannel()], 3);
      break;
    case (RADIO_LINKED_RECEIVED_PACKET):
      systemPrint(F("State: Received Packet "));
      systemPrint(channels[radio.getFHSSChannel()], 3);
      break;
    case (RADIO_LINKED_TRANSMITTING):
      systemPrint(F("State: Transmitting "));
      systemPrint(channels[radio.getFHSSChannel()], 3);
      break;
    case (RADIO_LINKED_ACK_WAIT):
      systemPrint(F("State: Ack Wait "));
      systemPrint(channels[radio.getFHSSChannel()], 3);
      break;

    case (RADIO_BROADCASTING_RECEIVING_STANDBY):
      systemPrint(F("State: B-Receiving Standby "));
      systemPrint(channels[radio.getFHSSChannel()], 3);
      break;
    case (RADIO_BROADCASTING_RECEIVED_PACKET):
      systemPrint(F("State: B-Received Packet "));
      systemPrint(channels[radio.getFHSSChannel()], 3);
      break;
    case (RADIO_BROADCASTING_TRANSMITTING):
      systemPrint(F("State: B-Transmitting "));
      systemPrint(channels[radio.getFHSSChannel()], 3);
      break;

    case (RADIO_LINKED_COMMAND_RECEIVING_STANDBY):
      systemPrint(F("State: [Command] RX Standby"));
      break;
    case (RADIO_LINKED_COMMAND_TRANSMITTING):
      systemPrint(F("State: [Command] TXing"));
      break;
    case (RADIO_LINKED_COMMAND_ACK_WAIT):
      systemPrint(F("State: [Command] Ack Wait"));
      break;
    case (RADIO_LINKED_COMMAND_RECEIVED_PACKET):
      systemPrint(F("State: [Command] RX Packet Received"));
      break;

    case (RADIO_TRAINING_TRANSMITTING):
      systemPrint(F("State: [Training] TX"));
      break;
    case (RADIO_TRAINING_ACK_WAIT):
      systemPrint(F("State: [Training] Ack Wait"));
      break;
    case (RADIO_TRAINING_RECEIVING_HERE_FIRST):
      systemPrint(F("State: [Training] RX Here First"));
      break;
    case (RADIO_TRAINING_RECEIVED_PACKET):
      systemPrint(F("State: [Training] RX Packet"));
      break;      

    default:
      systemPrint(F("Change State Unknown: "));
      systemPrint(radioState);
      break;
  }
  systemPrintln();
}
