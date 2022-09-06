void updateRadioState()
{
  switch (radioState)
  {

    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

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
        else if ( (millis() - packetTimestamp) > (unsigned int)(settings.heartbeatTimeout + random(0, 1000)) //Avoid pinging each other at same time
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
            updateRSSI(); //Adjust LEDs to RSSI level. We will soon be linked.
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

        if (packetType == PACKET_BAD || packetType == PACKET_NETID_MISMATCH)
        {
          returnToReceiving(); //Ignore
          changeState(RADIO_NO_LINK_RECEIVING_STANDBY);
        }
        else if (packetType == PACKET_ACK)
        {
          //Yay! Return to normal communication
          packetsLost = 0; //Reset, used for linkLost testing
          updateRSSI(); //Adjust LEDs to RSSI level
          returnToReceiving();
          changeState(RADIO_LINKED_RECEIVING_STANDBY);
        }
        else if (packetType == PACKET_DUPLICATE)
        {
          sendAckPacket(); //It's a duplicate. Ack then ignore.
          changeState(RADIO_NO_LINK_TRANSMITTING);
        }
        else if (packetType == PACKET_PING)
        {
          triggerEvent(TRIGGER_NOLINK_IDENT_PACKET);
          updateRSSI(); //Adjust LEDs to RSSI level. We will soon be linked.
          sendAckPacket(); //Someone is pinging us
          changeState(RADIO_NO_LINK_TRANSMITTING);
        }
        else if (packetType == PACKET_DATA)
        {
          ; //No data packets allowed when not linked
        }
      }
      break;

    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

    case RADIO_LINKED_RECEIVING_STANDBY:
      {
        if (linkLost())
        {
          setRSSI(0);

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

        else if ((millis() - packetTimestamp) > (unsigned int)(settings.heartbeatTimeout + random(0, 1000))) //Avoid pinging each other at same time
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

        else //Process any waiting serial or commands
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
            else if (availableTXCommandBytes()) //If we have command bytes to send out
            {
              //Load command bytes into outgoing packet
              readyOutgoingCommandPacket();

              triggerEvent(TRIGGER_LINK_DATA_PACKET);

              //Serial.print("Sending Command/Response: ");
              //for (int x = 0 ; x < packetSize ; x++)
              //  Serial.write(outgoingPacket[x]);
              //Serial.println();

              //We now have the commandTXBuffer loaded. But we need to send an remoteCommandResponse if we are pointed at PRINT_TO_RF.
              if (printerEndpoint == PRINT_TO_RF)
              {
                //If printerEndpoint is PRINT_TO_RF we know the last commandTXBuffer was filled with a response to a command
                sendCommandResponseDataPacket();
              }
              else
              {
                //This packet was generated with sendRemoteCommand() and is a command packet
                sendCommandDataPacket();
              }

              if (availableTXCommandBytes() == 0)
                printerEndpoint = PRINT_TO_SERIAL; //Once the response is received, we need to print it to serial

              changeState(RADIO_LINKED_TRANSMITTING);
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

        if (packetType == PACKET_ACK || packetType == PACKET_COMMAND_ACK || packetType == PACKET_COMMAND_RESPONSE_ACK)
        {
          //This packet is an ack. Return to receiving.
          triggerEvent(TRIGGER_ACK_PROCESSED); //Trigger for transmission timing

          if (settings.displayPacketQuality == true)
          {
            systemPrintln();
            systemPrint("R:");
            systemPrint(radio.getRSSI());
            systemPrint("\tS:");
            systemPrint(radio.getSNR());
            systemPrint("\tfE:");
            systemPrint(radio.getFrequencyError());
            systemPrintln();
          }
          packetsLost = 0; //Reset, used for linkLost testing
          updateRSSI(); //Adjust LEDs to RSSI level
          frequencyCorrection += radio.getFrequencyError() / 1000000.0;
          returnToReceiving();
          changeState(RADIO_LINKED_RECEIVING_STANDBY);
        }
        else if (packetType == PACKET_DUPLICATE)
        {
          //It's a duplicate. Ack then throw data away.
          triggerEvent(TRIGGER_LINK_DUPLICATE_PACKET);
          packetsLost = 0; //Reset, used for linkLost testing
          updateRSSI(); //Adjust LEDs to RSSI level
          frequencyCorrection += radio.getFrequencyError() / 1000000.0;
          sendAckPacket();
          changeState(RADIO_LINKED_TRANSMITTING);
        }
        else if (packetType == PACKET_PING)
        {
          //Someone is pinging us. Ack back.
          triggerEvent(TRIGGER_LINK_CONTROL_PACKET);
          packetsLost = 0; //Reset, used for linkLost testing
          updateRSSI(); //Adjust LEDs to RSSI level
          frequencyCorrection += radio.getFrequencyError() / 1000000.0;
          sendAckPacket();
          changeState(RADIO_LINKED_TRANSMITTING);
        }
        else if (packetType == PACKET_DATA)
        {
          //Pull data from packet and move into outbound serial buffer
          if (settings.displayPacketQuality == true)
          {
            systemPrintln();
            systemPrint("R:");
            systemPrint(radio.getRSSI());
            systemPrint("\tS:");
            systemPrint(radio.getSNR());
            systemPrint("\tfE:");
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
          updateRSSI(); //Adjust LEDs to RSSI level
          frequencyCorrection += radio.getFrequencyError() / 1000000.0;
          sendAckPacket(); //Transmit ACK
          changeState(RADIO_LINKED_TRANSMITTING);
        }

        else if (packetType == PACKET_COMMAND_DATA)
        {
          //Serial.print("Received Command Data: ");
          //for (int x = 0 ; x < lastPacketSize ; x++)
          //  Serial.write(lastPacket[x]);
          //Serial.println();

          //Move this packet into the command RX buffer
          for (int x = 0 ; x < lastPacketSize ; x++)
          {
            commandRXBuffer[commandRXHead++] = lastPacket[x];
            commandRXHead %= sizeof(commandRXBuffer);
          }

          packetsLost = 0; //Reset, used for linkLost testing
          updateRSSI(); //Adjust LEDs to RSSI level
          frequencyCorrection += radio.getFrequencyError() / 1000000.0;
          sendCommandAckPacket(); //Transmit ACK
          changeState(RADIO_LINKED_TRANSMITTING);
        }

        else if (packetType == PACKET_COMMAND_RESPONSE_DATA)
        {
          //Print received data. This is blocking but we do not use the serialTransmitBuffer because we're in command mode (and it's not much data to print).
          for (int x = 0 ; x < lastPacketSize ; x++)
            Serial.write(lastPacket[x]);

          packetsLost = 0; //Reset, used for linkLost testing
          updateRSSI(); //Adjust LEDs to RSSI level
          frequencyCorrection += radio.getFrequencyError() / 1000000.0;
          sendCommandResponseAckPacket(); //Transmit ACK
          changeState(RADIO_LINKED_TRANSMITTING);
        }
        else if (packetType == PACKET_COMMAND_RESPONSE_ACK)
        {
          //If we are waiting for ack before radio config, apply settings
          if (confirmDeliveryBeforeRadioConfig == true)
          {
            confirmDeliveryBeforeRadioConfig = false;

            //Apply settings
            generateHopTable(); //Generate freq with new settings
            configureRadio(); //Apply any new settings

            setRSSI(0); //Turn off RSSI LEDs
            if (settings.pointToPoint == true)
              changeState(RADIO_NO_LINK_RECEIVING_STANDBY);
            else
              changeState(RADIO_BROADCASTING_RECEIVING_STANDBY);
          }
          else //It was just an ACK
          {
            packetsLost = 0; //Reset, used for linkLost testing
            updateRSSI(); //Adjust LEDs to RSSI level
            frequencyCorrection += radio.getFrequencyError() / 1000000.0;
            returnToReceiving();
            changeState(RADIO_LINKED_RECEIVING_STANDBY);
          }
        }
        else //packetType == PACKET_BAD, packetType == PACKET_NETID_MISMATCH
        {
          //Packet type not supported in this state
          triggerEvent(TRIGGER_LINK_BAD_PACKET);
          returnToReceiving();
          changeState(RADIO_LINKED_RECEIVING_STANDBY);
        }
      }
      break;

    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

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

        //Toggle 2 LEDs if we have recently transmitted
        if (millis() - packetTimestamp < 5000)
        {
          if (millis() - lastLinkBlink > 250) //Blink at 4Hz
          {
            lastLinkBlink = millis();
            if (digitalRead(pin_rssi2LED) == HIGH)
              setRSSI(0b0001);
            else
              setRSSI(0b0010);
          }
        }
        else if (millis() - lastPacketReceived < 5000)
        {
          updateRSSI(); //Adjust LEDs to RSSI level
        }

        //Turn off RSSI after 5 seconds of no activity
        else
          setRSSI(0);

      }
      break;

    case RADIO_BROADCASTING_TRANSMITTING:
      {
        if (transactionComplete == true) //If dio0ISR has fired, we are done transmitting
        {
          transactionComplete = false; //Reset ISR flag
          returnToReceiving();
          changeState(RADIO_BROADCASTING_RECEIVING_STANDBY); //No ack response when in broadcasting mode
          setRSSI(0b0001);
        }

        else if (timeToHop == true) //If the dio1ISR has fired, move to next frequency
          hopChannel();
      }
      break;
    case RADIO_BROADCASTING_RECEIVED_PACKET:
      {
        PacketType packetType = identifyPacketType(); //Look at the packet we just received

        if (packetType == PACKET_ACK)
        {
          //We should not be receiving ack packets, but if we do, just ignore
          frequencyCorrection += radio.getFrequencyError() / 1000000.0;
          returnToReceiving();
          changeState(RADIO_BROADCASTING_RECEIVING_STANDBY);
        }
        else if (packetType == PACKET_DUPLICATE || packetType == PACKET_PING)
        {
          //We should not be receiving control packets, but if we do, just ignore
          frequencyCorrection += radio.getFrequencyError() / 1000000.0;
          returnToReceiving(); //No response when in broadcasting mode
          changeState(RADIO_BROADCASTING_RECEIVING_STANDBY);
        }
        else if (packetType == PACKET_DATA)
        {
          if (settings.displayPacketQuality == true)
          {
            systemPrintln();
            systemPrint("R:");
            systemPrint(radio.getRSSI());
            systemPrint("\tS:");
            systemPrint(radio.getSNR());
            systemPrint("\tfE:");
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

          updateRSSI(); //Adjust LEDs to RSSI level
          frequencyCorrection += radio.getFrequencyError() / 1000000.0;
          returnToReceiving(); //No response when in broadcasting mode
          changeState(RADIO_BROADCASTING_RECEIVING_STANDBY);

          lastPacketReceived = millis(); //Update timestamp for Link LED
        }
        else //PACKET_BAD, PACKET_NETID_MISMATCH
        {
          //This packet type is not supported in this state
          returnToReceiving();
          changeState(RADIO_BROADCASTING_RECEIVING_STANDBY);
        }

      }
      break;

    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

    /*
          beginTraining                beginDefaultTraining
                | Save current settings          | Save default settings
                V                                |
                +<-------------------------------’
                |
                V
        moveToTrainingFreq
                |
                V
        RADIO_TRAINING_TRANSMITTING
                |
                V
        RADIO_TRAINING_ACK_WAIT --------------.
                |                             |
                V                             |
        RADIO_TRAINING_RECEIVING_HERE_FIRST   |
                |                             |
                +<----------------------------’
                |
                V
        RADIO_TRAINING_RECEIVED_PACKET
                |
                V
           endTraining
    */

    case RADIO_TRAINING_TRANSMITTING:
      {
        if (transactionComplete == true) //If dio0ISR has fired, we are done transmitting
        {
          transactionComplete = false; //Reset ISR flag
          returnToReceiving();
          changeState(RADIO_TRAINING_ACK_WAIT);
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
        else if ( (millis() - lastTrainBlink) > 75) //Blink while unit waits in training state
        {
          lastTrainBlink = millis();

          //Cylon the RSSI LEDs
          setRSSI(trainCylonNumber);

          if (trainCylonNumber == 0b1000 || trainCylonNumber == 0b0001)
            trainCylonDirection *= -1; //Change direction

          if (trainCylonDirection > 0)
            trainCylonNumber <<= 1;
          else
            trainCylonNumber >>= 1;
        }
      }
      break;

    case RADIO_TRAINING_RECEIVED_PACKET:
      {
        PacketType packetType = identifyPacketType(); //Look at the packet we just received

        if (packetType == PACKET_TRAINING_PING)
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

        else if (packetType == PACKET_TRAINING_DATA)
        {
          //The waiting node has responded with a data packet
          //Absorb training data and then return to normal operation
          triggerEvent(TRIGGER_TRAINING_DATA_PACKET);
          packetsLost = 0; //Reset, used for linkLost testing
          endTraining(true); //Apply data from packet to settings
        }

        //During training, only training packets are valid
        else //PACKET_BAD, PACKET_NETID_MISMATCH, PACKET_ACK, PACKET_PING, PACKET_DATA
        {
          triggerEvent(TRIGGER_TRAINING_BAD_PACKET);
          returnToReceiving();
          changeState(RADIO_TRAINING_RECEIVING_HERE_FIRST);
        }
      }
      break;

    default:
      {
        systemPrintln("Unknown state");
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
      systemPrint("State: [No Link] Receiving Standby");
      break;
    case (RADIO_NO_LINK_RECEIVED_PACKET):
      systemPrint("State: [No Link] Received Packet");
      break;
    case (RADIO_NO_LINK_TRANSMITTING):
      systemPrint("State: [No Link] Transmitting");
      break;
    case (RADIO_NO_LINK_ACK_WAIT):
      systemPrint("State: [No Link] Ack Wait");
      break;

    case (RADIO_LINKED_RECEIVING_STANDBY):
      systemPrint("State: Receiving Standby ");
      systemPrint(channels[radio.getFHSSChannel()], 3);
      break;
    case (RADIO_LINKED_RECEIVED_PACKET):
      systemPrint("State: Received Packet ");
      systemPrint(channels[radio.getFHSSChannel()], 3);
      break;
    case (RADIO_LINKED_TRANSMITTING):
      systemPrint("State: Transmitting ");
      systemPrint(channels[radio.getFHSSChannel()], 3);
      break;
    case (RADIO_LINKED_ACK_WAIT):
      systemPrint("State: Ack Wait ");
      systemPrint(channels[radio.getFHSSChannel()], 3);
      break;

    case (RADIO_BROADCASTING_RECEIVING_STANDBY):
      systemPrint("State: B-Receiving Standby ");
      systemPrint(channels[radio.getFHSSChannel()], 3);
      break;
    case (RADIO_BROADCASTING_RECEIVED_PACKET):
      systemPrint("State: B-Received Packet ");
      systemPrint(channels[radio.getFHSSChannel()], 3);
      break;
    case (RADIO_BROADCASTING_TRANSMITTING):
      systemPrint("State: B-Transmitting ");
      systemPrint(channels[radio.getFHSSChannel()], 3);
      break;

    case (RADIO_TRAINING_TRANSMITTING):
      systemPrint("State: [Training] TX");
      break;
    case (RADIO_TRAINING_ACK_WAIT):
      systemPrint("State: [Training] Ack Wait");
      break;
    case (RADIO_TRAINING_RECEIVING_HERE_FIRST):
      systemPrint("State: [Training] RX Here First");
      break;
    case (RADIO_TRAINING_RECEIVED_PACKET):
      systemPrint("State: [Training] RX Packet");
      break;

    default:
      systemPrint("Change State Unknown: ");
      systemPrint(radioState);
      break;
  }
  systemPrintln();
}
