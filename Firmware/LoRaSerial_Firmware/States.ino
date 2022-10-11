void updateRadioState()
{
  uint8_t * header = outgoingPacket;
  bool heartbeatTimeout;
  uint16_t length;
  uint8_t radioSeed;
  static uint8_t rexmtBuffer[MAX_PACKET_SIZE];
  static CONTROL_U8 rexmtControl;
  static uint8_t rexmtLength;

  switch (radioState)
  {
    default:
      {
        systemPrint("Unknown state: ");
        systemPrintln(radioState);
      }
      break;

    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    // Reset
    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

    case RADIO_RESET:
      petWDT();

      //Start the TX timer: time to delay before transmitting the PING
      heartbeatTimer = millis();
      pingRandomTime = random(settings.maxDwellTime / 10, settings.maxDwellTime / 2); //Fast ping

      //Set all of the ACK numbers to zero
      *(uint8_t *)(&txControl) = 0;

      //Determine the components of the frame header
      headerBytes = 0;

      //Add the netID to the header
      if (settings.pointToPoint || settings.verifyRxNetID)
        headerBytes += 1;

      //Add the control byte to the header
      headerBytes += 1;

      //Determine the maximum frame size
      if (settings.radioSpreadFactor == 6)
        headerBytes += 1;

      //Set the beginning of the data portion of the transmit buffer
      endOfTxData = &outgoingPacket[headerBytes];

      //Determine the size of the trailer
      trailerBytes = 0;

      radioSeed = radio.randomByte(); //Puts radio into standy-by state
      randomSeed(radioSeed);
      if ((settings.debug == true) || (settings.debugRadio == true))
      {
        systemPrint("RadioSeed: ");
        systemPrintln(radioSeed);
      }

      generateHopTable(); //Generate frequency table based on user settings

      stopChannelTimer(); //Prevent radio from frequency hopping

      petWDT();
      configureRadio(); //Setup radio, set freq to channel 0

      returnToReceiving(); //Start receiving

      //Start the link between the radios
      if (settings.useV2)
      {
        //Determine the minimum and maximum datagram sizes
        minDatagramSize = headerBytes + trailerBytes;
        maxDatagramSize = sizeof(outgoingPacket) - minDatagramSize;

        //Start the V2 protocol
        if (settings.pointToPoint == true)
          changeState(RADIO_P2P_LINK_DOWN);
        else
          changeState(RADIO_MP_STANDBY);
      }
      else
      {
        //V1 - SF6 length, netID and control are at the end of the datagram
        trailerBytes = headerBytes;
        headerBytes = 0;

        //Determine the minimum and maximum datagram sizes
        minDatagramSize = headerBytes + trailerBytes;
        maxDatagramSize = sizeof(outgoingPacket) - minDatagramSize;

        //Start the V1 protocol
        if (settings.pointToPoint == true)
          changeState(RADIO_NO_LINK_RECEIVING_STANDBY);
        else
          changeState(RADIO_BROADCASTING_RECEIVING_STANDBY);
      }
      break;

    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    //V2 - No Link
    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    //Point-To-Point: Bring up the link
    //
    //A three way handshake is used to get both systems to agree that data can flow in both
    //directions. This handshake is also used to synchronize the HOP timer.
    /*
                    System A                 System B

                     RESET                     RESET
                       |                         |
             Channel 0 |                         | Channel 0
                       V                         V
           .---> P2P_LINK_DOWN              P2P_LINK_DOWN
           |           | Tx PING                 |
           | Timeout   |                         |
           |           V                         |
           | P2P_WAIT_TX_PING_DONE               |
           |           |                         |
           |           | Tx Complete - - - - - > | Rx PING
           |           |   Start Rx              |
           |           |   MAX_PACKET_SIZE       |
           |           V                         V
           `---- P2P_WAIT_ACK_1                  +<----------------------.
                       |                         | Tx PING ACK1          |
                       |                         V                       |
                       |              P2P_WAIT_TX_ACK_1_DONE             |
                       |                         |                       |
          Rx PING ACK1 | < - - - - - - - - - - - | Tx Complete           |
                       |                         |   Start Rx            |
                       |                         |   MAX_PACKET_SIZE     |
                       |                         |                       |
                       V                         V         Timeout       |
           .---------->+                   P2P_WAIT_ACK_2 -------------->+
           |   TX PING |                         |                       ^
           |      ACK2 |                         |                       |
           |           V                         |                       |
           | P2P_WAIT_TX_ACK_2_DONE              |                       |
           |           | Tx Complete - - - - - > | Rx PING ACK2          |
           | Stop      |   Start HOP timer       |   Start HOP Timer     | Stop
           | HOP       |   Start Rx              |   Start Rx            | HOP
           | Timer     |   MAX_PACKET_SIZE       |   MAX_PACKET_SIZE     | Timer
           |           |                         |                       |
           | Rx        |                         |                       |
           | PING ACK  V                         V         Rx PING       |
           `----- P2P_LINK_UP               P2P_LINK_UP -----------------’
                       |                         |
                       | Rx Data                 | Rx Data
                       |                         |
                       V                         V

      Two timers are in use:
        datagramTimer:  Set at end of transmit, measures ACK timeout
        heartbeatTimer: Set upon entry to P2P_NO_LINK, measures time to send next PING
    */
    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

    case RADIO_P2P_LINK_DOWN:
      //If we are on the wrong channel, go home
      if (channelNumber != 0)
      {
        stopChannelTimer();
        channelNumber = 0;
        radio.setFrequency(channels[channelNumber]);
      }

      //Is it time to send the PING to the remote system
      if ((millis() - heartbeatTimer) >= pingRandomTime)
      {
        //Transmit the PING
        triggerEvent(TRIGGER_HANDSHAKE_SEND_PING);
        xmitDatagramP2PPing();
        changeState(RADIO_P2P_WAIT_TX_PING_DONE);
      }

      //Determine if a PING was received
      else if (transactionComplete)
      {
        transactionComplete = false; //Reset ISR flag

        //Decode the received packet
        PacketType packetType = rcvDatagram();
        if (packetType != DATAGRAM_PING)
          returnToReceiving();
        else
        {
          //Acknowledge the PING
          triggerEvent(TRIGGER_SEND_ACK1);
          xmitDatagramP2PAck1();
          changeState(RADIO_P2P_WAIT_TX_ACK_1_DONE);
        }
      }
      break;

    case RADIO_P2P_WAIT_TX_PING_DONE:
      //Determine if a PING has completed transmission
      if (transactionComplete)
      {
        triggerEvent(TRIGGER_HANDSHAKE_SEND_PING_COMPLETE);
        transactionComplete = false; //Reset ISR flag
        returnToReceiving();
        changeState(RADIO_P2P_WAIT_ACK_1);
      }
      break;

    case RADIO_P2P_WAIT_ACK_1:
      if (transactionComplete)
      {
        transactionComplete = false; //Reset ISR flag

        //Decode the received packet
        PacketType packetType = rcvDatagram();
        if (packetType == DATAGRAM_PING)
        {
          //Acknowledge the PING
          triggerEvent(TRIGGER_SEND_ACK1);
          xmitDatagramP2PAck1();
          changeState(RADIO_P2P_WAIT_TX_ACK_1_DONE);
        }
        else if (packetType != DATAGRAM_ACK_1)
          returnToReceiving();
        else
        {
          //Acknowledge the ACK1
          triggerEvent(TRIGGER_SEND_ACK2);
          xmitDatagramP2PAck2();
          changeState(RADIO_P2P_WAIT_TX_ACK_2_DONE);
        }
      }
      else
      {
        if ((millis() - datagramTimer) >= (datagramAirTime + ackAirTime + settings.overheadTime))
        {
          if (settings.debugDatagrams)
          {
            systemPrintTimestamp();
            systemPrintln("RX: ACK1 Timeout");
          }
          returnToReceiving();

          //Start the TX timer: time to delay before transmitting the PING
          triggerEvent(TRIGGER_HANDSHAKE_ACK1_TIMEOUT);
          heartbeatTimer = millis();
          pingRandomTime = random(settings.maxDwellTime * 2, settings.maxDwellTime * 4); //Slow ping
          changeState(RADIO_P2P_LINK_DOWN);
        }
      }
      break;

    case RADIO_P2P_WAIT_TX_ACK_1_DONE:
      //Determine if a ACK 1 has completed transmission
      if (transactionComplete)
      {
        triggerEvent(TRIGGER_HANDSHAKE_SEND_ACK1_COMPLETE);
        transactionComplete = false; //Reset ISR flag
        returnToReceiving();
        changeState(RADIO_P2P_WAIT_ACK_2);
      }
      break;

    case RADIO_P2P_WAIT_ACK_2:
      if (transactionComplete == true)
      {
        transactionComplete = false; //Reset ISR flag

        //Decode the received packet
        PacketType packetType = rcvDatagram();
        if (packetType != DATAGRAM_ACK_2)
          returnToReceiving();
        else
          //Bring up the link
          v2EnterLinkUp();
      }
      else
      {
        if ((millis() - datagramTimer) >= (datagramAirTime +  ackAirTime + settings.overheadTime))
        {
          if (settings.debugDatagrams)
          {
            systemPrintTimestamp();
            systemPrintln("RX: ACK2 Timeout");
          }

          //Start the TX timer: time to delay before transmitting the PING
          triggerEvent(TRIGGER_HANDSHAKE_ACK2_TIMEOUT);
          heartbeatTimer = millis();
          pingRandomTime = random(settings.maxDwellTime * 2, settings.maxDwellTime * 4); //Slow ping
          changeState(RADIO_P2P_LINK_DOWN);
        }
      }
      break;

    case RADIO_P2P_WAIT_TX_ACK_2_DONE:
      //Determine if a ACK 2 has completed transmission
      if (transactionComplete)
      {
        transactionComplete = false; //Reset ISR flag

        //Bring up the link
        v2EnterLinkUp();
      }
      break;

    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    //V2 - Link Up
    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    //Point-To-Point: Data Exchange

    /*
                          System A                 System B

                       P2P_LINK_DOWN                   .-----------------------.
                             ^                         |                       |
                             |                         V                       |
                     timeout |             P2P_LINK_UP_WAIT_ACK_DONE           |
                             |                         |                       |
         .--------- P2P_LINK_UP_WAIT_ACK -----.        |                       |
         |                   |                |        |                       |
         |                   |    ACK Timeout |        |                       |
         |                   |                |        |                       |
         |            Rx ACK | < - - - - - - -)- - - - | Tx Complete           |
         |                   |                |        |   Start Rx            |
         |                   |                |        |   MAX_PACKET_SIZE     |
         |                   |     Retransmit |        |                       |
         |                   V                |        V                       |
         |              P2P_LINK_UP           |   P2P_LINK_UP                  |
         |                   |                |        |                       |
         |                   | Tx DATA        |        |                       |
         |                   |                |        |                       |
         |                   V                |        |                       |
         |        P2P_LINK_UP_WAIT_TX_DONE <--'        |                       |
         |                   |                         |                       |
         |                   |                         |                       |
         |      Tx Complete  | Tx Complete - - - - - > | Rx DATA               |
         |        Start Rx   |                         | Rx Duplicate          |
         |   MAX_PACKET_SIZE |                         |                       |
         |                   |                         | Tx ACK                |
         '-------------------'                         |                       |
                                                       '-----------------------'

      Three timers are in use:
        datagramTimer:  Set at end of transmit, measures ACK timeout
        heartbeatTimer: Set upon entry to P2P_LINK_UP, reset upon HEARTBEAT transmit,
                        measures time to send next HEARTBEAT
    */

    case RADIO_P2P_LINK_UP:
      updateRSSI();

      if (timeToHop == true) //If the channelTimer has expired, move to next frequency
        hopChannel();

      //Check for a received datagram
      if (transactionComplete == true)
      {
        transactionComplete = false; //Reset ISR flag

        //Decode the received datagram
        PacketType packetType = rcvDatagram();

        //Process the received datagram
        switch (packetType)
        {
          default:
            triggerEvent(TRIGGER_UNKNOWN_PACKET);
            returnToReceiving();
            break;

          case PACKET_BAD:
            triggerEvent(TRIGGER_BAD_PACKET);
            returnToReceiving();
            break;

          case DATAGRAM_PING:
            //Acknowledge the PING
            triggerEvent(TRIGGER_SEND_ACK1);
            xmitDatagramP2PAck1();
            changeState(RADIO_P2P_WAIT_TX_ACK_1_DONE);
            break;

          case PACKET_DUPLICATE:
            //Display the signal strength
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

            triggerEvent(TRIGGER_LINK_SEND_ACK_FOR_DUP);

            if (expectedAckNumber == 0) expectedAckNumber = 3; //Reduce eAN by one to align with remote's count
            else expectedAckNumber--;
            xmitDatagramP2PAck(); //Transmit ACK

            changeState(RADIO_P2P_LINK_UP_WAIT_ACK_DONE);
            break;

          case DATAGRAM_HEARTBEAT:
            //Received heartbeat while link was idle. Send ack to sync clocks.

            //Display the signal strength
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

            resetHeartbeat();

            triggerEvent(TRIGGER_LINK_SEND_ACK_FOR_HEARTBEAT);
            xmitDatagramP2PAck(); //Transmit ACK
            changeState(RADIO_P2P_LINK_UP_WAIT_ACK_DONE);
            break;

          case DATAGRAM_DATA:
            //Display the signal strength
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

            //Determine the number of bytes received
            length = 0;
            if ((txHead + rxDataBytes) > sizeof(serialTransmitBuffer))
            {
              //Copy the first portion of the received datagram into the buffer
              length = sizeof(serialTransmitBuffer) - txHead;
              memcpy(&serialTransmitBuffer[txHead], rxData, length);
              txHead = 0;
            }

            //Copy the remaining portion of the received datagram into the buffer
            memcpy(&serialTransmitBuffer[txHead], &rxData[length], rxDataBytes - length);
            txHead += rxDataBytes - length;
            txHead %= sizeof(serialTransmitBuffer);

            packetsLost = 0; //Reset, used for linkLost testing
            updateRSSI(); //Adjust LEDs to RSSI level
            frequencyCorrection += radio.getFrequencyError() / 1000000.0;

            resetHeartbeat();

            triggerEvent(TRIGGER_LINK_SEND_ACK_FOR_DATA);
            xmitDatagramP2PAck(); //Transmit ACK
            changeState(RADIO_P2P_LINK_UP_WAIT_ACK_DONE);
            break;

          case DATAGRAM_REMOTE_COMMAND:
            //Determine the number of bytes received
            length = 0;
            if ((commandRXHead + rxDataBytes) > sizeof(commandRXBuffer))
            {
              //Copy the first portion of the received datagram into the buffer
              length = sizeof(commandRXBuffer) - commandRXHead;
              memcpy(&commandRXBuffer[commandRXHead], rxData, length);
              commandRXHead = 0;
            }

            //Copy the remaining portion of the received datagram into the buffer
            memcpy(&commandRXBuffer[commandRXHead], &rxData[length], rxDataBytes - length);
            commandRXHead += rxDataBytes - length;
            commandRXHead %= sizeof(commandRXBuffer);

            packetsLost = 0; //Reset, used for linkLost testing
            updateRSSI(); //Adjust LEDs to RSSI level
            frequencyCorrection += radio.getFrequencyError() / 1000000.0;
            triggerEvent(TRIGGER_LINK_SEND_ACK_FOR_REMOTE_COMMAND);
            xmitDatagramP2PAck(); //Transmit ACK
            changeState(RADIO_P2P_LINK_UP_WAIT_ACK_DONE);
            break;

          case DATAGRAM_REMOTE_COMMAND_RESPONSE:
            //Print received data. This is blocking but we do not use the serialTransmitBuffer because we're in command mode (and it's not much data to print).
            for (int x = 0 ; x < rxDataBytes ; x++)
              Serial.write(rxData[x]);

            packetsLost = 0; //Reset, used for linkLost testing
            updateRSSI(); //Adjust LEDs to RSSI level
            frequencyCorrection += radio.getFrequencyError() / 1000000.0;

            triggerEvent(TRIGGER_LINK_SEND_ACK_FOR_REMOTE_COMMAND_RESPONSE);
            xmitDatagramP2PAck(); //Transmit ACK
            changeState(RADIO_P2P_LINK_UP_WAIT_ACK_DONE);
            break;
        }
      }

      //If the radio is available, send any data in the serial buffer over the radio
      else if (receiveInProcess() == false)
      {
        heartbeatTimeout = ((millis() - heartbeatTimer) > heartbeatRandomTime);

        //Check for time to send serial data
        if (availableRXBytes() && (processWaitingSerial(heartbeatTimeout) == true))
        {
          triggerEvent(TRIGGER_LINK_DATA_XMIT);
          xmitDatagramP2PData();
          changeState(RADIO_P2P_LINK_UP_WAIT_TX_DONE);
        }
        else if (availableTXCommandBytes()) //If we have command bytes to send out
        {
          //Load command bytes into outgoing packet
          readyOutgoingCommandPacket();

          triggerEvent(TRIGGER_LINK_DATA_XMIT);

          //We now have the commandTXBuffer loaded
          if (remoteCommandResponse)
            xmitDatagramP2PCommandResponse();
          else
            xmitDatagramP2PCommand();

          changeState(RADIO_P2P_LINK_UP_WAIT_TX_DONE);
        }
        else if (heartbeatTimeout)
        {
          triggerEvent(TRIGGER_HEARTBEAT);
          if (receiveInProcess() == false && transactionComplete == false) //Avoid race condition
          {
            xmitDatagramP2PHeartbeat();

            resetHeartbeat();

            //Wait for heartbeat to transmit
            changeState(RADIO_P2P_LINK_UP_WAIT_TX_DONE);
          }
        }
        else if ((millis() - linkDownTimer) >= (3 * settings.heartbeatTimeout))
          //Break the link
          v2BreakLink();
      }
      break;

    //Wait for the ACK or HEARTBEAT to finish transmission
    case RADIO_P2P_LINK_UP_WAIT_ACK_DONE:
      if (timeToHop == true) //If the channelTimer has expired, move to next frequency
        hopChannel();

      if (transactionComplete)
      {
        triggerEvent(TRIGGER_LINK_ACK_SENT);
        transactionComplete = false; //Reset ISR flag
        returnToReceiving();
        changeState(RADIO_P2P_LINK_UP);
      }
      break;

    //Wait for the data transmission to complete
    case RADIO_P2P_LINK_UP_WAIT_TX_DONE:
      if (timeToHop == true) //If the channelTimer has expired, move to next frequency
        hopChannel();

      if (transactionComplete)
      {
        triggerEvent(TRIGGER_LINK_WAIT_FOR_ACK);
        transactionComplete = false; //Reset ISR flag
        returnToReceiving();
        changeState(RADIO_P2P_LINK_UP_WAIT_ACK);
      }
      break;

    //Wait for the ACK to be received
    case RADIO_P2P_LINK_UP_WAIT_ACK:
      if (timeToHop == true) //If the channelTimer has expired, move to next frequency
        hopChannel();

      if (transactionComplete)
      {
        transactionComplete = false; //Reset ISR flag

        //Decode the received datagram
        PacketType packetType = rcvDatagram();

        //Process the received datagram
        switch (packetType)
        {
          default:
            triggerEvent(TRIGGER_UNKNOWN_PACKET);
            returnToReceiving();
            break;

          case PACKET_BAD:
            triggerEvent(TRIGGER_BAD_PACKET);
            returnToReceiving();
            break;

          case DATAGRAM_PING:
            //Break the link
            v2BreakLink();
            break;

          case DATAGRAM_DATA_ACK:
            syncChannelTimer(); //Adjust freq hop ISR based on remote's remaining clock

            resetHeartbeat(); //Extend time before next heartbeat

            //Display the signal strength
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

            triggerEvent(TRIGGER_LINK_ACK_RECEIVED);
            returnToReceiving();
            changeState(RADIO_P2P_LINK_UP);
            break;

          case DATAGRAM_HEARTBEAT:
            //Received heartbeat while waiting for ack.

            //Display the signal strength
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

            //An ACK was expected for a previous transmission that must have been
            //lost.  Save the current transmit buffer for later retransmission
            //and ACK the heartbeat.  Later perform the retransmission for the
            //datagram that was lost.
            petWDT();
            memcpy(rexmtBuffer, outgoingPacket, MAX_PACKET_SIZE);
            rexmtControl = txControl;
            rexmtLength = txDatagramSize;

            triggerEvent(TRIGGER_LINK_SEND_ACK_FOR_HEARTBEAT);
            xmitDatagramP2PAck(); //Transmit ACK
            changeState(RADIO_P2P_LINK_UP_HB_ACK_REXMT);

            break;
        }
      }

      //Check for ACK timeout
      else if ((millis() - datagramTimer) >= (datagramAirTime + ackAirTime + settings.overheadTime))
        //Set at end of transmit, measures ACK timeout
      {
        if (settings.debugDatagrams)
        {
          systemPrintTimestamp();
          systemPrintln("RX: ACK Timeout");
        }

        //Retransmit the packet
        if (packetSent < settings.maxResends)
        {
          triggerEvent(TRIGGER_LINK_RETRANSMIT);
          if (settings.debugDatagrams)
          {
            systemPrintTimestamp();
            systemPrint("TX: Retransmit ");
            systemPrint(packetSent);
            systemPrint(", ");
            systemPrint(v2DatagramType[txControl.datagramType]);
            switch (txControl.datagramType)
            {
              default:
                systemPrintln();
                break;

              case DATAGRAM_DATA:
              case DATAGRAM_DATA_ACK:
              case DATAGRAM_SF6_DATA:
              case DATAGRAM_REMOTE_COMMAND:
              case DATAGRAM_REMOTE_COMMAND_RESPONSE:
              case DATAGRAM_HEARTBEAT:
                systemPrint(" (ACK #");
                systemPrint(txControl.ackNumber);
                systemPrint(")");
                systemPrintln();
                break;
            }
          }
          if (receiveInProcess() == false)
          {
            retransmitDatagram();
            packetSent++;
            changeState(RADIO_P2P_LINK_UP_WAIT_TX_DONE);
          }
        }
        else
        {
          //Failed to reach the other system, break the link
          triggerEvent(TRIGGER_LINK_RETRANSMIT_FAIL);

          //Break the link
          v2BreakLink();
        }
      }
      break;

    case RADIO_P2P_LINK_UP_HB_ACK_REXMT:
      if (timeToHop == true) //If the channelTimer has expired, move to next frequency
        hopChannel();

      //An ACK was expected for a previous transmission that must have been
      //lost.  A heartbeat was received instead which was ACKed.  Once the ACK
      //completes transmission, retransmit the previously lost datagram.
      if (transactionComplete)
      {
        transactionComplete = false; //Reset ISR flag

        //Retransmit the packet
        if (packetSent++ < settings.maxResends)
        {
          memcpy(outgoingPacket, rexmtBuffer, MAX_PACKET_SIZE);
          txControl = rexmtControl;
          txDatagramSize = rexmtLength;
          if (settings.debugDatagrams)
          {
            systemPrintTimestamp();
            systemPrint("TX: Retransmit ");
            systemPrint(packetSent);
            systemPrint(", ");
            systemPrint(v2DatagramType[txControl.datagramType]);
            switch (txControl.datagramType)
            {
              default:
                systemPrintln();
                break;

              case DATAGRAM_DATA:
              case DATAGRAM_DATA_ACK:
              case DATAGRAM_SF6_DATA:
              case DATAGRAM_REMOTE_COMMAND:
              case DATAGRAM_REMOTE_COMMAND_RESPONSE:
              case DATAGRAM_HEARTBEAT:
                systemPrint(" (ACK #");
                systemPrint(txControl.ackNumber);
                systemPrint(")");
                systemPrintln();
                break;
            }
          }
          retransmitDatagram();
          changeState(RADIO_P2P_LINK_UP_WAIT_TX_DONE);
        }
        else
        {
          //Failed to reach the other system, break the link
          if (settings.debugDatagrams)
            systemPrintln("---------- Link DOWN ----------");
          heartbeatTimer = millis();
          pingRandomTime = random(settings.maxDwellTime / 10, settings.maxDwellTime / 2); //Fast ping
          changeState(RADIO_P2P_LINK_DOWN);
        }
      }
      break;

    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    //V2 - Multi-Point Data Exchange
    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

    case RADIO_MP_STANDBY:
      //Hop channels when required
      if (timeToHop == true)
        hopChannel();

      //Process the receive packet
      if (transactionComplete == true)
      {
        triggerEvent(TRIGGER_BROADCAST_PACKET_RECEIVED);
        transactionComplete = false; //Reset ISR flag

        //Decode the received datagram
        PacketType packetType = rcvDatagram();

        //Process the received datagram
        switch (packetType)
        {
          default:
            returnToReceiving();
            changeState(RADIO_MP_STANDBY);
            break;

          case DATAGRAM_ACK_1:
          case DATAGRAM_ACK_2:
          case DATAGRAM_DATA:
          case DATAGRAM_DATA_ACK:
          case DATAGRAM_HEARTBEAT:
          case DATAGRAM_PING:
          case DATAGRAM_REMOTE_COMMAND:
          case DATAGRAM_REMOTE_COMMAND_RESPONSE:
            //We should not be receiving these datagrams, but if we do, just ignore
            frequencyCorrection += radio.getFrequencyError() / 1000000.0;
            returnToReceiving();
            break;

          case DATAGRAM_DATAGRAM:
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

            //Determine the number of bytes received
            length = 0;
            if ((txHead + rxDataBytes) > sizeof(serialTransmitBuffer))
            {
              //Copy the first portion of the received datagram into the buffer
              length = sizeof(serialTransmitBuffer) - txHead;
              memcpy(&serialTransmitBuffer[txHead], rxData, length);
              txHead = 0;
            }

            //Copy the remaining portion of the received datagram into the buffer
            memcpy(&serialTransmitBuffer[txHead], &rxData[length], rxDataBytes - length);
            txHead += rxDataBytes - length;
            txHead %= sizeof(serialTransmitBuffer);

            updateRSSI(); //Adjust LEDs to RSSI level
            frequencyCorrection += radio.getFrequencyError() / 1000000.0;
            returnToReceiving(); //No response when in broadcasting mode

            lastPacketReceived = millis(); //Update timestamp for Link LED
            break;
        }
      }

      else //Process waiting serial
      {
        //If the radio is available, send any data in the serial buffer over the radio
        if (receiveInProcess() == false)
        {
          if (availableRXBytes()) //If we have bytes
          {
            if (processWaitingSerial(false) == true) //If we've hit a frame size or frame-timed-out
            {
              triggerEvent(TRIGGER_BROADCAST_DATA_PACKET);
              xmitDatagramMpDatagram();
              sendDataPacket();
              changeState(RADIO_MP_WAIT_TX_DONE);
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
        updateRSSI(); //Adjust LEDs to RSSI level

      //Turn off RSSI after 5 seconds of no activity
      else
        setRSSI(0);
      break;

    case RADIO_MP_WAIT_TX_DONE:
      //Hop channels when required
      if (timeToHop == true)
        hopChannel();

      //If transmit is complete then start receiving
      if (transactionComplete == true)
      {
        transactionComplete = false; //Reset ISR flag
        returnToReceiving();
        changeState(RADIO_MP_STANDBY);
        setRSSI(0b0001);
      }
      else if (timeToHop == true) //If the dio1ISR has fired, move to next frequency
        hopChannel();
      break;

    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    //V1 - No Link
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
          else if (settings.debugRadio)
            systemPrintln("NO_LINK_RECEIVING_STANDBY: RX In Progress");
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
    //V1 - Link
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
          else if (settings.debugRadio)
            systemPrintln("RECEIVING_STANDBY: RX In Progress");
        }

        else //Process any waiting serial or commands
        {
          //If the radio is available, send any data in the serial buffer over the radio
          if (receiveInProcess() == false)
          {
            if (availableRXBytes()) //If we have bytes
            {
              if (processWaitingSerial(false) == true) //If we've hit a frame size or frame-timed-out
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
              if (remoteCommandResponse)
                sendCommandResponseDataPacket();
              else
                sendCommandDataPacket();

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
            if (settings.debugRadio)
              systemPrintln("Packet Lost");
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
              if (settings.debugRadio)
                systemPrintln("ACK_WAIT: RX In Progress");
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
            changeState(RADIO_RESET);
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
              if (processWaitingSerial(false) == true) //If we've hit a frame size or frame-timed-out
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
    //V1 - Point-to-Point Training
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
        updateCylonLEDs();
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
              if (settings.debugRadio)
                systemPrintln("Timeout");
              break;
            }
          }
          if (settings.debugRadio)
            systemPrintln("Ending Training");

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
  }
}

//Return true if the radio is in a linked state
//This is used for determining if we can do remote AT commands or not
bool isLinked()
{
  if (((radioState >= RADIO_P2P_LINK_UP)
       && (radioState <= RADIO_P2P_LINK_UP_WAIT_ACK))
      || ((radioState >= RADIO_LINKED_RECEIVING_STANDBY)
          && (radioState <= RADIO_LINKED_RECEIVED_PACKET)))
    return (true);

  return (false);
}

const RADIO_STATE_ENTRY radioStateTable[] =
{
  {RADIO_RESET,                          "RESET",                          NULL},                         // 0

  //V1
  //    State                                 Name                              Description
  {RADIO_NO_LINK_RECEIVING_STANDBY,      "NO_LINK_RECEIVING_STANDBY",      "[No Link] Receiving Standby"},// 1
  {RADIO_NO_LINK_TRANSMITTING,           "NO_LINK_TRANSMITTING",           "[No Link] Transmitting"},     // 2
  {RADIO_NO_LINK_ACK_WAIT,               "NO_LINK_ACK_WAIT",               "[No Link] Ack Wait"},         // 3
  {RADIO_NO_LINK_RECEIVED_PACKET,        "NO_LINK_RECEIVED_PACKET",        "[No Link] Received Packet"},  // 4
  {RADIO_LINKED_RECEIVING_STANDBY,       "LINKED_RECEIVING_STANDBY",       "Receiving Standby "},         // 5
  {RADIO_LINKED_TRANSMITTING,            "LINKED_TRANSMITTING",            "Transmitting "},              // 6
  {RADIO_LINKED_ACK_WAIT,                "LINKED_ACK_WAIT",                "Ack Wait "},                  // 7
  {RADIO_LINKED_RECEIVED_PACKET,         "LINKED_RECEIVED_PACKET",         "Received Packet "},           // 8
  {RADIO_BROADCASTING_RECEIVING_STANDBY, "BROADCASTING_RECEIVING_STANDBY", "B-Receiving Standby "},       // 9
  {RADIO_BROADCASTING_TRANSMITTING,      "BROADCASTING_TRANSMITTING",      "B-Transmitting "},            //10
  {RADIO_BROADCASTING_RECEIVED_PACKET,   "BROADCASTING_RECEIVED_PACKET",   "B-Received Packet "},         //11
  {RADIO_TRAINING_RECEIVING_HERE_FIRST,  "TRAINING_RECEIVING_HERE_FIRST",  "[Training] RX Here First"},   //12
  {RADIO_TRAINING_TRANSMITTING,          "TRAINING_TRANSMITTING",          "[Training] TX"},              //13
  {RADIO_TRAINING_ACK_WAIT,              "TRAINING_ACK_WAIT",              "[Training] Ack Wait"},        //14
  {RADIO_TRAINING_RECEIVED_PACKET,       "TRAINING_RECEIVED_PACKET",       "[Training] RX Packet"},       //15

  //V2 - Point-to-Point link handshake
  //    State                                 Name                              Description
  {RADIO_P2P_LINK_DOWN,                  "P2P_LINK_DOWN",                  "V2 P2P: [No Link] Waiting for Ping"}, //16
  {RADIO_P2P_WAIT_TX_PING_DONE,          "P2P_WAIT_TX_PING_DONE",          "V2 P2P: [No Link] Wait Ping TX Done"},//17
  {RADIO_P2P_WAIT_ACK_1,                 "P2P_WAIT_ACK_1",                 "V2 P2P: [No Link] Waiting for ACK1"}, //18
  {RADIO_P2P_WAIT_TX_ACK_1_DONE,         "P2P_WAIT_TX_ACK_1_DONE",         "V2 P2P: [No Link] Wait ACK1 TX Done"},//19
  {RADIO_P2P_WAIT_ACK_2,                 "P2P_WAIT_ACK_2",                 "V2 P2P: [No Link] Waiting for ACK2"}, //20
  {RADIO_P2P_WAIT_TX_ACK_2_DONE,         "P2P_WAIT_TX_ACK_2_DONE",         "V2 P2P: [No Link] Wait ACK2 TX Done"},//21

  //V2 - Point-to-Point, link up, data exchange
  //    State                                 Name                              Description
  {RADIO_P2P_LINK_UP,                    "P2P_LINK_UP",                    "V2 P2P: Receiving Standby"},          //22
  {RADIO_P2P_LINK_UP_WAIT_ACK_DONE,      "P2P_LINK_UP_WAIT_ACK_DONE",      "V2 P2P: Waiting ACK TX Done"},        //23
  {RADIO_P2P_LINK_UP_WAIT_TX_DONE,       "P2P_LINK_UP_WAIT_TX_DONE",       "V2 P2P: Waiting TX done"},            //24
  {RADIO_P2P_LINK_UP_WAIT_ACK,           "P2P_LINK_UP_WAIT_ACK",           "V2 P2P: Waiting for ACK"},            //25
  {RADIO_P2P_LINK_UP_HB_ACK_REXMT,       "P2P_LINK_UP_HB_ACK_REXMT",       "V2 P2P: Heartbeat ACK ReXmt"},            //26

  //V2 - Multi-Point data exchange
  //    State                                 Name                              Description
  {RADIO_MP_STANDBY,                     "MP_STANDBY",                     "V2 MP: Wait for TX or RX"},           //27
  {RADIO_MP_WAIT_TX_DONE,                "MP_WAIT_TX_DONE",                "V2 MP: Waiting for TX done"},         //28
};

void verifyRadioStateTable()
{
  int expectedState;
  unsigned int i;
  unsigned int index;
  unsigned int maxDescriptionLength;
  unsigned int maxNameLength;
  bool missing;
  int * order;
  unsigned int tableEntries;
  int temp;

  //Verify that all the entries are in the state table
  tableEntries = sizeof(radioStateTable) / sizeof(radioStateTable[0]);
  for (index = 0; index < tableEntries; index++)
  {
    //Validate the current table entry
    if (index == radioStateTable[index].state)
      continue;

    //Wait for the USB serial port
    if (!settings.usbSerialWait)
      while (!Serial);

    //An invalid entry was found
    //Try to build a valid table
    order = (int *)malloc(tableEntries * sizeof(*order));
    if (!order)
    {
      systemPrintln("ERROR - Failed to allocate the order table from the heap!");
      while (1);
    }

    //Assume the table is in the correct order
    maxDescriptionLength = 0;
    maxNameLength = 0;
    for (index = 0; index < tableEntries; index++)
    {
      order[index] = index;
      if (radioStateTable[index].state < RADIO_MAX_STATE)
      {
        if (maxNameLength < strlen(radioStateTable[index].name))
          maxNameLength = strlen(radioStateTable[index].name);
        if (radioStateTable[index].description
            && (maxDescriptionLength < strlen(radioStateTable[index].description)))
          maxDescriptionLength = strlen(radioStateTable[index].description);
      }
    }

    //Bubble sort the entries
    for (index = 0; index < tableEntries; index++)
    {
      for (i = index + 1; i < tableEntries; i++)
      {
        if (radioStateTable[order[i]].state < radioStateTable[order[index]].state)
        {
          //Swap the two values
          temp = order[i];
          order[i] = order[index];
          order[index] = temp;
        }
      }
    }

    //Determine if there are missing states
    missing = false;
    for (index = 0; index < tableEntries; index++)
    {
      if (radioStateTable[order[i]].state != index)
      {
        missing = true;
        break;
      }
    }

    //Display the request
    systemPrintln("Please replace radioStateTable in States.ino with the following table:");
    if (missing)
      systemPrintln("Please update the table with the missing states");
    systemPrintln();

    //Display the new table
    systemPrintln("const RADIO_STATE_ENTRY radioStateTable[] =");
    systemPrintln("{");
    systemPrintln("  //    State                                 Name                              Description");
    expectedState = 0;
    for (index = 0; index < tableEntries; index++)
    {
      //Remove bad states from the table
      if (radioStateTable[order[index]].state >= RADIO_MAX_STATE)
        break;
      if (radioStateTable[order[index]].state > expectedState)
      {
        //Print the missing entries
        systemPrint("Missing state");
        if ((expectedState + 1) < radioStateTable[order[index]].state)
        {
          systemPrint("s ");
          systemPrint(expectedState);
          systemPrint(" - ");
          systemPrintln(radioStateTable[order[index]].state - 1);
        }
        else
        {
          systemPrint(" ");
          systemPrintln(expectedState);
        }
        while (radioStateTable[order[index]].state > expectedState)
        {
          systemPrint("  {");
          systemPrint(", ");
          for (i = 0; i < (6 + maxNameLength); i++)
            systemPrint(" ");
          systemPrint("\"\", ");
          for (i = 0; i < maxNameLength; i++)
            systemPrint(" ");
          systemPrint("\"\"},");
          for (i = 0; i < maxDescriptionLength; i++)
            systemPrint(" ");
          systemPrint("//");
          if (expectedState < 10)
            systemPrint(" ");
          systemPrintln(expectedState++);
        }
      }

      //Print the state
      systemPrint("  {RADIO_");
      systemPrint(radioStateTable[order[index]].name);
      systemPrint(", ");

      //Align the name column
      for (i = strlen(radioStateTable[order[index]].name); i < maxNameLength; i++)
        systemPrint(" ");

      //Print the name
      systemPrint("\"");
      systemPrint(radioStateTable[order[index]].name);
      systemPrint("\", ");

      //Align the description column
      for (i = strlen(radioStateTable[order[index]].name); i < maxNameLength; i++)
        systemPrint(" ");

      //Print the description
      if (radioStateTable[order[index]].description)
      {
        systemPrint("\"");
        systemPrint(radioStateTable[order[index]].description);
        systemPrint("\"},");
      }
      else
        systemPrint("NULL},");

      //Align the comments
      for (i = radioStateTable[order[index]].description ? strlen(radioStateTable[order[index]].description) : 2;
           i < maxDescriptionLength; i++)
        systemPrint(" ");

      //Add the state value comment
      systemPrint("//");
      if (expectedState < 10)
        systemPrint(" ");
      systemPrintln(expectedState++);
    }
    systemPrintln("};");
    systemFlush();

    //Wait forever
    while (1);
  }
}

//Change states and print the new state
void changeState(RadioStates newState)
{
  radioState = newState;

  if ((settings.debug == false) && (settings.debugStates == false))
    return;

  //Debug print
  if (settings.printTimestamp)
    systemPrintTimestamp();
  systemPrint("State: ");
  if (newState >= RADIO_MAX_STATE)
  {
    systemPrint(radioState);
    systemPrintln(" is Unknown");
  }
  else
  {
    if (radioStateTable[radioState].description)
      systemPrintln(radioStateTable[radioState].description);
    else
      systemPrintln(radioStateTable[radioState].name);
  }
}

void v2BreakLink()
{
  //Break the link
  systemPrintln("--------- Link DOWN ---------");
  triggerEvent(TRIGGER_RADIO_RESET);
  changeState(RADIO_RESET);
}

void v2EnterLinkUp()
{
  //Bring up the link
  triggerEvent(TRIGGER_HANDSHAKE_COMPLETE);
  startChannelTimer();
  hopChannel(); //Leave home
  resetHeartbeat();

  //Synchronize the ACK numbers
  txControl.ackNumber = 0;
  expectedAckNumber = 0;
  expectedDatagramNumber = 0;

  //Discard any previous data
  rxTail = rxHead;
  txTail = txHead;
  commandRXTail = commandRXHead;
  commandTXTail = commandTXHead;

  //Start the receiver
  returnToReceiving();
  changeState(RADIO_P2P_LINK_UP);
  systemPrintln("========== Link UP ==========");
}
