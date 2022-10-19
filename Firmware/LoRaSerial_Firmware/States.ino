#define SAVE_TX_BUFFER()                                \
{                                                       \
  memcpy(rexmtBuffer, outgoingPacket, MAX_PACKET_SIZE); \
  rexmtControl = txControl;                             \
  rexmtLength = txDatagramSize;                         \
  rexmtFrameSentCount = frameSentCount;                 \
}

#define RESTORE_TX_BUFFER()                             \
{                                                       \
  memcpy(outgoingPacket, rexmtBuffer, MAX_PACKET_SIZE); \
  txControl = rexmtControl;                             \
  txDatagramSize = rexmtLength;                         \
  frameSentCount = rexmtFrameSentCount;                 \
}

void updateRadioState()
{
  unsigned long clockOffset;
  unsigned long currentMillis;
  unsigned long deltaMillis;
  uint8_t * header = outgoingPacket;
  bool heartbeatTimeout;
  uint16_t length;
  uint8_t radioSeed;
  static uint8_t rexmtBuffer[MAX_PACKET_SIZE];
  static CONTROL_U8 rexmtControl;
  static uint8_t rexmtLength;
  static uint8_t rexmtFrameSentCount;

  switch (radioState)
  {
    default:
      {
        systemPrint("Unknown state: ");
        systemPrintln(radioState);
        while (1)
          petWDT();
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

      //Determine the components of the frame header and trailer
      selectHeaderAndTrailerBytes();

      //Initialize the radio
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
      if (settings.radioProtocolVersion >= 2)
      {
        //Start the V2 protocol
        if (settings.operatingMode == MODE_POINT_TO_POINT)
          changeState(RADIO_P2P_LINK_DOWN);
        else
          changeState(RADIO_MP_STANDBY);
      }
      break;

    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    //V2 - Point-to-Point Training
    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

    /*
        beginTrainingPointToPoint
                |
                | Save settings
                |
                | TX DATAGRAM_P2P_TRAINING_PING
                |
                V
        RADIO_P2P_TRAINING_WAIT_PING_DONE
                |
                V                        RX DATAGRAM_P2P_TRAINING_PARAMS
        RADIO_P2P_WAIT_FOR_TRAINING_PARAMS -----------.
                |                                     |
                | RX DATAGRAM_P2P_TRAINING_PING       |
                | TX DATAGRAM_P2P_TRAINING_PARAMS     |
                |                                     |
                V                                     |
        RADIO_P2P_WAIT_TRAINING_ACK_DONE              |
                |                                     |
                +<------------------------------------’
                |
                V
        RADIO_RESET
    */

    //Wait for the PING to complete transmission
    case RADIO_P2P_TRAINING_WAIT_PING_DONE:
      if (transactionComplete)
      {
        transactionComplete = false; //Reset ISR flag
        returnToReceiving();
        changeState(RADIO_P2P_WAIT_FOR_TRAINING_PARAMS);
      }
      break;

    case RADIO_P2P_WAIT_FOR_TRAINING_PARAMS:
      updateRSSI();

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
            triggerEvent(TRIGGER_BAD_PACKET);
            returnToReceiving();
            break;

          case DATAGRAM_P2P_TRAINING_PING:
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

            triggerEvent(TRIGGER_TRAINING_CONTROL_PACKET);

            //Send the parameters
            xmitDatagramP2pTrainingParams();
            changeState(RADIO_P2P_WAIT_TRAINING_PARAMS_DONE);
            break;

          case DATAGRAM_P2P_TRAINING_PARAMS:
            triggerEvent(TRIGGER_TRAINING_DATA_PACKET);

            //Update the parameters
            updateRadioParameters(rxData);
            endPointToPointTraining(true);
            changeState(RADIO_RESET);
        }
      }

      //If the radio is available, send any data in the serial buffer over the radio
      else if (receiveInProcess() == false)
      {
        //Check for a receive timeout
        if ((millis() - datagramTimer) > (settings.clientPingRetryInterval * 1000))
        {
          triggerEvent(TRIGGER_TRAINING_NO_ACK);
          retransmitDatagram();
          lostFrames++;
          changeState(RADIO_P2P_TRAINING_WAIT_PING_DONE);
        }
      }
      break;

    case RADIO_P2P_WAIT_TRAINING_PARAMS_DONE:
      if (transactionComplete)
      {
        transactionComplete = false; //Reset ISR flag
        endPointToPointTraining(false);
        changeState(RADIO_RESET);
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
          //Received PING
          //Compute the common clock
          currentMillis = millis();
          memcpy(&clockOffset, rxData, sizeof(currentMillis));
          roundTripMillis = rcvTimeMillis - xmitTimeMillis;
          clockOffset += currentMillis + roundTripMillis;
          clockOffset >>= 1;
          clockOffset -= currentMillis;  //The currentMillis is added in systemPrintTimestamp
          timestampOffset = clockOffset;

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
          //Received PING
          //Compute the common clock
          currentMillis = millis();
          memcpy(&clockOffset, rxData, sizeof(currentMillis));
          roundTripMillis = rcvTimeMillis - xmitTimeMillis;
          clockOffset += currentMillis + roundTripMillis;
          clockOffset >>= 1;
          clockOffset -= currentMillis;  //The currentMillis is added in systemPrintTimestamp
          timestampOffset = clockOffset;

          //Acknowledge the PING
          triggerEvent(TRIGGER_SEND_ACK1);
          xmitDatagramP2PAck1();
          changeState(RADIO_P2P_WAIT_TX_ACK_1_DONE);
        }
        else if (packetType != DATAGRAM_ACK_1)
          returnToReceiving();
        else
        {
          //Received ACK 1
          //Compute the common clock
          currentMillis = millis();
          memcpy(&clockOffset, rxData, sizeof(currentMillis));
          roundTripMillis = rcvTimeMillis - xmitTimeMillis;
          clockOffset += currentMillis + roundTripMillis;
          clockOffset >>= 1;
          clockOffset -= currentMillis;  //The currentMillis is added in systemPrintTimestamp
          timestampOffset = clockOffset;

          //Acknowledge the ACK1
          triggerEvent(TRIGGER_SEND_ACK2);
          xmitDatagramP2PAck2();
          changeState(RADIO_P2P_WAIT_TX_ACK_2_DONE);
        }
      }
      else
      {
        if ((millis() - datagramTimer) >= (frameAirTime + ackAirTime + settings.overheadTime))
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
        {
          //Received ACK 2
          //Compute the common clock
          currentMillis = millis();
          memcpy(&clockOffset, rxData, sizeof(currentMillis));
          roundTripMillis = rcvTimeMillis - xmitTimeMillis;
          clockOffset += currentMillis + roundTripMillis;
          clockOffset >>= 1;
          clockOffset -= currentMillis;  //The currentMillis is added in systemPrintTimestamp
          timestampOffset = clockOffset;

          //Bring up the link
          v2EnterLinkUp();
        }
      }
      else
      {
        if ((millis() - datagramTimer) >= (frameAirTime +  ackAirTime + settings.overheadTime))
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

      Timestamp offset synchronization:

                      System A       System B

                          PING ----> Update timestampOffset

        Update timestampOffset <---- ACK 1

                         ACK 2 ----> Update timestampOffset

                   HEARTBEAT 0 ----> Update timestampOffset

        Update timestampOffset <---- HEARTBEAT 0

                   HEARTBEAT 1 --X

        Update timestampOffset <---- HEARTBEAT 1

                   HEARTBEAT 1 ----> Update timestampOffset
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

          case DATAGRAM_BAD:
            triggerEvent(TRIGGER_BAD_PACKET);
            returnToReceiving();
            break;

          case DATAGRAM_PING:
            v2BreakLink();
            break;

          case DATAGRAM_DUPLICATE:
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

            updateRSSI(); //Adjust LEDs to RSSI level
            frequencyCorrection += radio.getFrequencyError() / 1000000.0;

            triggerEvent(TRIGGER_LINK_SEND_ACK_FOR_DUP);
            xmitDatagramP2PAck(); //Transmit ACK

            changeState(RADIO_P2P_LINK_UP_WAIT_ACK_DONE);
            break;

          case DATAGRAM_HEARTBEAT:
            //Received heartbeat while link was idle. Send ack to sync clocks.
            //Adjust the timestamp offset
            currentMillis = millis();
            memcpy(&clockOffset, rxData, sizeof(currentMillis));
            clockOffset += currentMillis + roundTripMillis;
            clockOffset >>= 1;
            clockOffset -= currentMillis;  //The currentMillis is added in systemPrintTimestamp
            timestampOffset = clockOffset;

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

            //Place the data in the serial output buffer
            serialBufferOutput(rxData, rxDataBytes);

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
        else if ((millis() - linkDownTimer) >= (P2P_LINK_BREAK_MULTIPLIER * settings.heartbeatTimeout))
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

          case DATAGRAM_BAD:
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

            updateRSSI(); //Adjust LEDs to RSSI level
            frequencyCorrection += radio.getFrequencyError() / 1000000.0;

            triggerEvent(TRIGGER_LINK_ACK_RECEIVED);
            returnToReceiving();
            changeState(RADIO_P2P_LINK_UP);
            break;

          case DATAGRAM_HEARTBEAT:
            //Received heartbeat while waiting for ack.
            //Adjust the timestamp offset
            currentMillis = millis();
            memcpy(&clockOffset, rxData, sizeof(currentMillis));
            clockOffset += currentMillis + roundTripMillis;
            clockOffset >>= 1;
            clockOffset -= currentMillis;  //The currentMillis is added in systemPrintTimestamp
            timestampOffset = clockOffset;

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

            updateRSSI(); //Adjust LEDs to RSSI level
            frequencyCorrection += radio.getFrequencyError() / 1000000.0;

            //An ACK was expected for a previous transmission that must have been
            //lost.  Save the current transmit buffer for later retransmission
            //and ACK the heartbeat.  Later perform the retransmission for the
            //datagram that was lost.
            petWDT();
            SAVE_TX_BUFFER();

            triggerEvent(TRIGGER_LINK_SEND_ACK_FOR_HEARTBEAT);
            xmitDatagramP2PAck(); //Transmit ACK
            changeState(RADIO_P2P_LINK_UP_HB_ACK_REXMT);

            break;
        }
      }

      //Check for ACK timeout
      else if ((millis() - datagramTimer) >= (frameAirTime + ackAirTime + settings.overheadTime))
        //Set at end of transmit, measures ACK timeout
      {
        if (settings.debugDatagrams)
        {
          systemPrintTimestamp();
          systemPrintln("RX: ACK Timeout");
        }

        //Retransmit the packet
        if (frameSentCount < settings.maxResends)
        {
          triggerEvent(TRIGGER_LINK_RETRANSMIT);
          if (settings.debugDatagrams)
          {
            systemPrintTimestamp();
            systemPrint("TX: Retransmit ");
            systemPrint(frameSentCount);
            systemPrint(", ");
            systemPrint(v2DatagramType[txControl.datagramType]);
            switch (txControl.datagramType)
            {
              default:
                systemPrintln();
                break;

              case DATAGRAM_DATA:
              case DATAGRAM_DATA_ACK:
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
            lostFrames++;
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
        if (rexmtFrameSentCount < settings.maxResends)
        {
          RESTORE_TX_BUFFER();
          if (settings.debugDatagrams)
          {
            systemPrintTimestamp();
            systemPrint("TX: Retransmit ");
            systemPrint(frameSentCount);
            systemPrint(", ");
            systemPrint(v2DatagramType[txControl.datagramType]);
            switch (txControl.datagramType)
            {
              default:
                systemPrintln();
                break;

              case DATAGRAM_DATA:
              case DATAGRAM_DATA_ACK:
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
          lostFrames++;
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

            //Place the data in the serial output buffer
            serialBufferOutput(rxData, rxDataBytes);

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
              changeState(RADIO_MP_WAIT_TX_DONE);
            }
          }
        }
      } //End processWaitingSerial

      //Toggle 2 LEDs if we have recently transmitted
      if (millis() - datagramTimer < 5000)
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
    //V2 - Multi-Point Client Training
    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

    /*
       beginTrainingClient
                |
                | Save current settings
                |
                V
                +<--------------------------------.
                |                                 |
                | Send client ping                |
                |                                 |
                V                                 |
        RADIO_MP_WAIT_TX_TRAINING_PING_DONE       |
                |                                 |
                V                                 | Timeout
        RADIO_MP_WAIT_RX_RADIO_PARAMETERS --------'
                |
                | Update settings
                | Send client ACK
                |
                V
        RADIO_MP_WAIT_TX_PARAM_ACK_DONE
                |
                V
     endTrainingClientServer
                |
                | Restore settings
                |
                V
    */

    case RADIO_MP_WAIT_TX_TRAINING_PING_DONE:
      updateCylonLEDs();

      //If dio0ISR has fired, we are done transmitting
      if (transactionComplete == true)
      {
        transactionComplete = false;

        //Indicate that the receive is complete
        triggerEvent(TRIGGER_TRAINING_CLIENT_TX_PING_DONE);

        //Start the receive operation
        returnToReceiving();

        //Set the next state
        changeState(RADIO_MP_WAIT_RX_RADIO_PARAMETERS);
      }
      break;

    case RADIO_MP_WAIT_RX_RADIO_PARAMETERS:
      updateCylonLEDs();

      //If dio0ISR has fired, a packet has arrived
      if (transactionComplete == true)
      {
        transactionComplete = false;
        trainingPreviousRxInProgress = false;

        //Decode the received datagram
        PacketType packetType = rcvDatagram();

        //Process the received datagram
        switch (packetType)
        {
          default:
            triggerEvent(TRIGGER_BAD_PACKET);
            returnToReceiving();
            break;

          case DATAGRAM_TRAINING_PARAMS:
            //Verify the IDs
            if ((memcmp(rxData, myUniqueId, UNIQUE_ID_BYTES) != 0)
              && (memcmp(rxData, myUniqueId, UNIQUE_ID_BYTES) != 0))
            {
              triggerEvent(TRIGGER_BAD_PACKET);
              returnToReceiving();
              break;
            }

            //Save the training partner ID
            memcpy(trainingPartnerID, &rxData[UNIQUE_ID_BYTES], UNIQUE_ID_BYTES);

            //Get the radio parameters
            updateRadioParameters(&rxData[UNIQUE_ID_BYTES * 2]);

            //Acknowledge the radio parameters
            xmitDatagramMpTrainingAck(&rxData[UNIQUE_ID_BYTES]);
            changeState(RADIO_MP_WAIT_TX_PARAM_ACK_DONE);
            break;
        }
      }

      //Determine if a receive is in progress
      else if (receiveInProcess())
      {
        if (!trainingPreviousRxInProgress)
        {
          trainingPreviousRxInProgress = true;
          triggerEvent(TRIGGER_TRAINING_CLIENT_RX_PARAMS);
        }
      }

      //Check for a receive timeout
      else if ((millis() - datagramTimer) > (settings.clientPingRetryInterval * 1000))
        xmitDatagramMpTrainingPing();
      break;

    case RADIO_MP_WAIT_TX_PARAM_ACK_DONE:
      updateCylonLEDs();

      //If dio0ISR has fired, we are done transmitting
      if (transactionComplete == true)
      {
        transactionComplete = false;
        endClientServerTraining(TRIGGER_TRAINING_CLIENT_TX_ACK_DONE);
      }
      break;

    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    //V2 - Multi-Point Server Training
    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

    /*
               beginTrainingServer
                        |
                        | Save current settings
                        |
                        V
                        +<--------------------------------.
                        |                                 |
                        V                                 |
        .------ RADIO_MP_WAIT_FOR_TRAINING_PING           |
        |               |                                 |
        |               | Send client ping                |
        |               |                                 |
        |               V                                 |
        |      RADIO_MP_WAIT_TX_RADIO_PARAMS_DONE --------'
        |
        |
        `---------------.
                        | Stop training command
                        |
                        V
             endTrainingClientServer
                        |
                        | Restore settings
                        |
                        V
    */

    case RADIO_MP_WAIT_FOR_TRAINING_PING:
      updateCylonLEDs();

      //If dio0ISR has fired, a packet has arrived
      if (transactionComplete == true)
      {
        transactionComplete = false; //Reset ISR flag
        trainingPreviousRxInProgress = false;

        //Decode the received datagram
        PacketType packetType = rcvDatagram();

        //Process the received datagram
        switch (packetType)
        {
          default:
            triggerEvent(TRIGGER_BAD_PACKET);
            returnToReceiving();
            break;

          case DATAGRAM_TRAINING_PING:
            //Save the client ID
            memcpy(trainingPartnerID, rxData, UNIQUE_ID_BYTES);
            xmitDatagramMpRadioParameters(trainingPartnerID);

            //Wait for the transmit to complete
            changeState(RADIO_MP_WAIT_TX_RADIO_PARAMS_DONE);
            break;

          case DATAGRAM_TRAINING_ACK:
            //Verify the client ID
            if (memcmp(trainingPartnerID, &rxData[UNIQUE_ID_BYTES], UNIQUE_ID_BYTES) == 0)
            {
              //Don't respond to the client ACK, just start another receive operation
              triggerEvent(TRIGGER_TRAINING_SERVER_RX_ACK);

              //Print the client trained message
              systemPrint("Client ");
              systemPrintUniqueID(trainingPartnerID);
              systemPrintln(" Trained");
            }
            returnToReceiving();
            break;
        }
      }

      //Determine if a receive is in progress
      else if (receiveInProcess())
        if (!trainingPreviousRxInProgress)
        {
          trainingPreviousRxInProgress = true;
          triggerEvent(TRIGGER_TRAINING_SERVER_RX);
        }
      break;

    case RADIO_MP_WAIT_TX_RADIO_PARAMS_DONE:
      updateCylonLEDs();

      //If dio0ISR has fired, we are done transmitting
      if (transactionComplete == true)
      {
        transactionComplete = false;

        //Indicate that the receive is complete
        triggerEvent(TRIGGER_TRAINING_SERVER_TX_PARAMS_DONE);

        //Start the receive operation
        returnToReceiving();

        //Set the next state
        changeState(RADIO_MP_WAIT_FOR_TRAINING_PING);
      }
      break;
  }
}

//Compute the number of header and trailer bytes
void selectHeaderAndTrailerBytes()
{
  //Determine the components of the frame header
  headerBytes = 0;

  //Add the netID to the header
  if ((settings.operatingMode == MODE_POINT_TO_POINT) || settings.verifyRxNetID)
    headerBytes += 1;

  //Add the control byte to the header
  headerBytes += 1;

  //Determine the maximum frame size
  if (settings.radioSpreadFactor == 6)
    headerBytes += 1;

  //Set the beginning of the data portion of the transmit buffer
  endOfTxData = &outgoingPacket[headerBytes];

  //Determine the size of the trailer
  trailerBytes = (settings.enableCRC16 ? 2 : 0);

  //Determine the minimum and maximum datagram sizes
  minDatagramSize = headerBytes + trailerBytes;
  maxDatagramSize = sizeof(outgoingPacket) - minDatagramSize;
}

//Return true if the radio is in a linked state
//This is used for determining if we can do remote AT commands or not
bool isLinked()
{
  if ((radioState >= RADIO_P2P_LINK_UP)
       && (radioState <= RADIO_P2P_LINK_UP_WAIT_ACK))
    return (true);

  return (false);
}

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

//Verify the datagram type table
void verifyV2DatagramType()
{
  if ((sizeof(v2DatagramType) / sizeof(v2DatagramType[0])) != MAX_V2_DATAGRAM_TYPE)
  {
    systemPrintln("ERROR - Please update the v2DatagramTable");
    while (1)
      petWDT();
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
  linkFailures++;
  if (settings.printLinkUpDown)
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
  rmtTxAckNumber = 0;
  rxAckNumber = 0;
  txAckNumber = 0;

  //Discard any previous data
  rxTail = rxHead;
  txTail = txHead;
  commandRXTail = commandRXHead;
  commandTXTail = commandTXHead;

  //Start the receiver
  returnToReceiving();
  changeState(RADIO_P2P_LINK_UP);
  if (settings.printLinkUpDown)
    systemPrintln("========== Link UP ==========");
}
