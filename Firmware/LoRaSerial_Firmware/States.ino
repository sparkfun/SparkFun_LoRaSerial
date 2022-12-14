#define SAVE_TX_BUFFER()                                  \
  {                                                       \
    memcpy(rexmtBuffer, outgoingPacket, MAX_PACKET_SIZE); \
    rexmtControl = txControl;                             \
    rexmtLength = txDatagramSize;                         \
    rexmtFrameSentCount = frameSentCount;                 \
  }

#define RESTORE_TX_BUFFER()                               \
  {                                                       \
    memcpy(outgoingPacket, rexmtBuffer, MAX_PACKET_SIZE); \
    txControl = rexmtControl;                             \
    txDatagramSize = rexmtLength;                         \
    frameSentCount = rexmtFrameSentCount;                 \
  }

#define P2P_SEND_ACK(trigger)                                                   \
  {                                                                             \
    /*Compute the frequency correction*/                                        \
    frequencyCorrection += radio.getFrequencyError() / 1000000.0;               \
                                                                                \
    /*Send the ACK to the remote system*/                                       \
    triggerEvent(trigger);                                                      \
    if (xmitDatagramP2PAck() == true)                                           \
    {                                                                           \
      /*We ack'd the packet so be responsible for sending the next heartbeat*/  \
      setHeartbeatShort();                                                      \
      changeState(RADIO_P2P_LINK_UP_WAIT_ACK_DONE);                             \
    }                                                                           \
  }

//Process the radio states
void updateRadioState()
{
  int8_t addressByte;
  uint8_t channel;
  unsigned long clockOffset;
  unsigned long currentMillis;
  unsigned long deltaMillis;
  uint8_t * header = outgoingPacket;
  bool heartbeatTimeout;
  int index;
  uint16_t length;
  uint8_t radioSeed;
  bool serverLinkBroken;
  VIRTUAL_CIRCUIT * vc;
  VC_RADIO_MESSAGE_HEADER * vcHeader;

  switch (radioState)
  {
    default:
      {
        systemPrint("Unknown state: ");
        systemPrintln(radioState);
        waitForever();
      }
      break;

    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    // Reset
    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

    case RADIO_RESET:
      petWDT();

      //Empty the buffers
      discardPreviousData();

      //Set all of the ACK numbers to zero
      *(uint8_t *)(&txControl) = 0;

      //Initialize the radio
      rssi = -200;
      radioSeed = radio.randomByte(); //Puts radio into standy-by state
      randomSeed(radioSeed);
      if ((settings.debug == true) || (settings.debugRadio == true))
      {
        systemPrint("RadioSeed: ");
        systemPrintln(radioSeed);
        outputSerialData(true);
      }

      convertAirSpeedToSettings(); //Update the settings based upon the air speed

      generateHopTable(); //Generate frequency table based on user settings.

      selectHeaderAndTrailerBytes(); //Determine the components of the frame header and trailer

      stopChannelTimer(); //Prevent radio from frequency hopping

      configureRadio(); //Setup radio, set freq to channel 0, calculate air times

      //Start the TX timer: time to delay before transmitting the PING
      setHeartbeatShort(); //Both radios start with short heartbeat period
      pingRandomTime = random(ackAirTime, ackAirTime * 2); //Fast ping

      sf6ExpectedSize = headerBytes + CLOCK_MILLIS_BYTES + trailerBytes; //Tell SF6 to receive PING packet

      petWDT();

      returnToReceiving(); //Start receiving

      //Stop the ACK timer
      ackTimer = 0;

      //Start the link between the radios
      if (settings.operatingMode == MODE_POINT_TO_POINT)
        changeState(RADIO_P2P_LINK_DOWN);
      else if (settings.operatingMode == MODE_VIRTUAL_CIRCUIT)
      {
        if (settings.server)
        {
          //Reserve the server's address (0)
          myVc = vcIdToAddressByte(VC_SERVER, myUniqueId);

          startChannelTimer(); //Start hopping

          //Start sending heartbeats
          xmitVcHeartbeat(myVc, myUniqueId);
          changeState(RADIO_VC_WAIT_TX_DONE);
        }
        else
        {
          //Unknown client address
          myVc = VC_UNASSIGNED;
          changeState(RADIO_DISCOVER_BEGIN);
        }
      }
      else
      {
        if (settings.server == true)
        {
          startChannelTimer(); //Start hopping
          changeState(RADIO_MP_STANDBY);
        }
        else
        {
          changeState(RADIO_DISCOVER_BEGIN);
        }
      }
      break;

    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    //No Link
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
        setRadioFrequency(false);
      }

      //Determine if a PING was received
      if (transactionComplete)
      {
        //Decode the received packet
        PacketType packetType = rcvDatagram();

        //Process the received datagram
        switch (packetType)
        {
          default:
            triggerEvent(TRIGGER_UNKNOWN_PACKET);
            if (settings.debugDatagrams)
            {
              systemPrintTimestamp();
              systemPrint("Scan: Unhandled packet type ");
              systemPrint(radioDatagramType[packetType]);
              systemPrintln();
            }
            break;

          case DATAGRAM_BAD:
            triggerEvent(TRIGGER_BAD_PACKET);
            break;

          case DATAGRAM_CRC_ERROR:
            triggerEvent(TRIGGER_CRC_ERROR);
            break;

          case DATAGRAM_PING:
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
            if (xmitDatagramP2PAck1() == true)
            {
              sf6ExpectedSize = headerBytes + CLOCK_MILLIS_BYTES + trailerBytes; //Tell SF6 we expect ACK2 to contain millis info
              changeState(RADIO_P2P_WAIT_TX_ACK_1_DONE);
            }
            break;
        }
      }

      //Is it time to send the PING to the remote system
      else if ((receiveInProcess() == false) && ((millis() - heartbeatTimer) >= pingRandomTime))
      {
        //Transmit the PING
        triggerEvent(TRIGGER_HANDSHAKE_SEND_PING);
        if (xmitDatagramP2PPing() == true)
        {
          sf6ExpectedSize = headerBytes + CLOCK_MILLIS_BYTES + trailerBytes; //Tell SF6 we expect ACK1 to contain millis info
          changeState(RADIO_P2P_WAIT_TX_PING_DONE);
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
        //Decode the received packet
        PacketType packetType = rcvDatagram();

        //Process the received datagram
        switch (packetType)
        {
          default:
            triggerEvent(TRIGGER_UNKNOWN_PACKET);
            if (settings.debugDatagrams)
            {
              systemPrintTimestamp();
              systemPrint("Scan: Unhandled packet type ");
              systemPrint(radioDatagramType[packetType]);
              systemPrintln();
            }
            break;

          case DATAGRAM_BAD:
            triggerEvent(TRIGGER_BAD_PACKET);
            break;

          case DATAGRAM_CRC_ERROR:
            triggerEvent(TRIGGER_CRC_ERROR);
            break;

          case DATAGRAM_PING:
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
            if (xmitDatagramP2PAck1() == true)
            {
              sf6ExpectedSize = headerBytes + CLOCK_MILLIS_BYTES + trailerBytes; //Tell SF6 we expect ACK2 to contain millis info
              changeState(RADIO_P2P_WAIT_TX_ACK_1_DONE);
            }
            break;

          case DATAGRAM_ACK_1:
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
            if (xmitDatagramP2PAck2() == true)
            {
              sf6ExpectedSize = MAX_PACKET_SIZE; //Tell SF6 to return to max packet length
              changeState(RADIO_P2P_WAIT_TX_ACK_2_DONE);
            }
            break;
        }
      }
      else
      {
        //If we timeout during handshake, return to link down
        if ((millis() - datagramTimer) >= (frameAirTime + ackAirTime + settings.overheadTime + getReceiveCompletionOffset()))
        {
          if (settings.debugDatagrams)
          {
            systemPrintTimestamp();
            systemPrintln("RX: ACK1 Timeout");
            outputSerialData(true);
          }

          //Start the TX timer: time to delay before transmitting the PING
          triggerEvent(TRIGGER_HANDSHAKE_ACK1_TIMEOUT);
          setHeartbeatShort();

          //Slow down pings
          if (ackAirTime < settings.maxDwellTime)
            pingRandomTime = random(settings.maxDwellTime * 2, settings.maxDwellTime * 4);
          else
            pingRandomTime = random(ackAirTime * 4, ackAirTime * 8);

          sf6ExpectedSize = headerBytes + CLOCK_MILLIS_BYTES + trailerBytes; //Tell SF6 to receive PING packet
          returnToReceiving();

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
        //Decode the received packet
        PacketType packetType = rcvDatagram();

        //Process the received datagram
        switch (packetType)
        {
          default:
            triggerEvent(TRIGGER_UNKNOWN_PACKET);
            if (settings.debugDatagrams)
            {
              systemPrintTimestamp();
              systemPrint("Scan: Unhandled packet type ");
              systemPrint(radioDatagramType[packetType]);
              systemPrintln();
            }
            break;

          case DATAGRAM_BAD:
            triggerEvent(TRIGGER_BAD_PACKET);
            break;

          case DATAGRAM_CRC_ERROR:
            triggerEvent(TRIGGER_CRC_ERROR);
            break;

          case DATAGRAM_ACK_2:
            //Received ACK 2
            //Compute the common clock
            currentMillis = millis();
            memcpy(&clockOffset, rxData, sizeof(currentMillis));
            roundTripMillis = rcvTimeMillis - xmitTimeMillis;
            clockOffset += currentMillis + roundTripMillis;
            clockOffset >>= 1;
            clockOffset -= currentMillis;  //The currentMillis is added in systemPrintTimestamp
            timestampOffset = clockOffset;

            startChannelTimer(getLinkupOffset()); //We are exiting the link last so adjust our starting Timer

            setHeartbeatLong(); //We sent ACK1 and they sent ACK2, so don't be the first to send heartbeat

            //Bring up the link
            enterLinkUp();
            break;
        }
      }
      else
      {
        //If we timeout during handshake, return to link down
        if ((millis() - datagramTimer) >= (frameAirTime +  ackAirTime + settings.overheadTime + getReceiveCompletionOffset()))
        {
          if (settings.debugDatagrams)
          {
            systemPrintTimestamp();
            systemPrintln("RX: ACK2 Timeout");
            outputSerialData(true);
          }

          //Start the TX timer: time to delay before transmitting the PING
          triggerEvent(TRIGGER_HANDSHAKE_ACK2_TIMEOUT);
          setHeartbeatShort();

          //Slow down pings
          if (ackAirTime < settings.maxDwellTime)
            pingRandomTime = random(settings.maxDwellTime * 2, settings.maxDwellTime * 4);
          else
            pingRandomTime = random(ackAirTime * 4, ackAirTime * 8);

          sf6ExpectedSize = headerBytes + CLOCK_MILLIS_BYTES + trailerBytes; //Tell SF6 to receive PING packet
          returnToReceiving();

          changeState(RADIO_P2P_LINK_DOWN);
        }
      }
      break;

    case RADIO_P2P_WAIT_TX_ACK_2_DONE:
      //Determine if a ACK 2 has completed transmission
      if (transactionComplete)
      {
        transactionComplete = false; //Reset ISR flag

        startChannelTimer(); //We are exiting the link first so do not adjust our starting Timer

        setHeartbeatShort(); //We sent the last ack so be responsible for sending the next heartbeat

        //Bring up the link
        enterLinkUp();
      }
      break;

    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    //Link Up
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

    //====================
    //Wait for the next operation (listed in priority order):
    // * Frame received
    // * Time to send HEARTBEAT
    // * Time to retransmit previous frame
    // * Remote command response to send
    // * Data to send
    // * Link timeout
    //====================
    case RADIO_P2P_LINK_UP:
      if (timeToHop == true) //If the channelTimer has expired, move to next frequency
        hopChannel();

      //Check for a received datagram
      if (transactionComplete == true)
      {
        //Decode the received datagram
        PacketType packetType = rcvDatagram();

        //Process the received datagram
        switch (packetType)
        {
          default:
            triggerEvent(TRIGGER_UNKNOWN_PACKET);
            if (settings.debugDatagrams)
            {
              systemPrintTimestamp();
              systemPrint("LinkUp: Unhandled packet type ");
              systemPrint(radioDatagramType[packetType]);
              systemPrintln();
              outputSerialData(true);
            }
            break;

          case DATAGRAM_BAD:
            triggerEvent(TRIGGER_BAD_PACKET);
            break;

          case DATAGRAM_CRC_ERROR:
            triggerEvent(TRIGGER_CRC_ERROR);
            break;

          case DATAGRAM_NETID_MISMATCH:
            triggerEvent(TRIGGER_NETID_MISMATCH);
            break;

          case DATAGRAM_PING:
            breakLink();
            break;

          case DATAGRAM_DUPLICATE:
            frequencyCorrection += radio.getFrequencyError() / 1000000.0;

            triggerEvent(TRIGGER_LINK_SEND_ACK_FOR_DUP);
            if (xmitDatagramP2PAck() == true) //Transmit ACK
            {
              setHeartbeatShort(); //We ack'd the packet (again) so be responsible for sending the next heartbeat
              changeState(RADIO_P2P_LINK_UP_WAIT_ACK_DONE);
            }
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

            //Transmit ACK
            P2P_SEND_ACK(TRIGGER_LINK_SEND_ACK_FOR_HEARTBEAT);
            break;

          case DATAGRAM_DATA:
            //Place the data in the serial output buffer
            serialBufferOutput(rxData, rxDataBytes);

            //Transmit ACK
            P2P_SEND_ACK(TRIGGER_LINK_SEND_ACK_FOR_DATA);
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

            //Transmit ACK
            P2P_SEND_ACK(TRIGGER_LINK_SEND_ACK_FOR_REMOTE_COMMAND);
            break;

          case DATAGRAM_REMOTE_COMMAND_RESPONSE:
            //Print received data. This is blocking but we do not use the serialTransmitBuffer because we're in command mode (and it's not much data to print).
            for (int x = 0 ; x < rxDataBytes ; x++)
              Serial.write(rxData[x]);

            //Transmit ACK
            P2P_SEND_ACK(TRIGGER_LINK_SEND_ACK_FOR_REMOTE_COMMAND_RESPONSE);
            break;
        }
      }

      //If the radio is available, send any data in the serial buffer over the radio
      else if (receiveInProcess() == false)
      {
        heartbeatTimeout = ((millis() - heartbeatTimer) > heartbeatRandomTime);

        //If we have data, try to send it out
        if (availableRadioTXBytes())
        {
          //Check if we are yielding for 2-way comm
          if (requestYield == false ||
              (requestYield == true && (millis() - yieldTimerStart > (settings.framesToYield * maxPacketAirTime)))
             )
          {
            //Yield has expired, allow transmit.
            requestYield = false;

            //Check for time to send serial data
            if (processWaitingSerial(heartbeatTimeout) == true)
            {
              triggerEvent(TRIGGER_LINK_DATA_XMIT);

              if (xmitDatagramP2PData() == true)
              {
                setHeartbeatLong(); //We're sending data, so don't be the first to send next heartbeat
                ackTimer = datagramTimer;

                //Save the previous transmit in case the previous ACK was lost or a
                //HEARTBEAT must be transmitted.  Restore the buffer when a retransmission
                //is necessary.
                petWDT();
                SAVE_TX_BUFFER();
                changeState(RADIO_P2P_LINK_UP_WAIT_TX_DONE);
              }
            }
          }
        }
        else if (availableTXCommandBytes()) //If we have command bytes to send out
        {
          //Load command bytes into outgoing packet
          readyOutgoingCommandPacket(0);

          triggerEvent(TRIGGER_LINK_DATA_XMIT);

          //We now have the commandTXBuffer loaded
          if (remoteCommandResponse)
          {
            if (xmitDatagramP2PCommandResponse() == true)
            {
              setHeartbeatLong(); //We're sending command, so don't be the first to send next heartbeat
              changeState(RADIO_P2P_LINK_UP_WAIT_TX_DONE);
            }
          }
          else
          {
            if (xmitDatagramP2PCommand() == true)
            {
              setHeartbeatLong(); //We're sending command, so don't be the first to send next heartbeat
              changeState(RADIO_P2P_LINK_UP_WAIT_TX_DONE);
            }
          }

        }
        else if (heartbeatTimeout)
        {
          triggerEvent(TRIGGER_HEARTBEAT);

          if (xmitDatagramP2PHeartbeat() == true)
          {
            setHeartbeatLong(); //We're sending a heartbeat, so don't be the first to send next heartbeat

            ackTimer = datagramTimer;

            //Wait for heartbeat to transmit
            changeState(RADIO_P2P_LINK_UP_WAIT_TX_DONE);
          }
        }
        else if ((millis() - linkDownTimer) >= (P2P_LINK_BREAK_MULTIPLIER * settings.heartbeatTimeout))
          //Break the link
          breakLink();
      }
      break;

    //====================
    //Wait for the ACK or HEARTBEAT to finish transmission
    //====================
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

    //====================
    //Wait for the data transmission to complete
    //====================
    case RADIO_P2P_LINK_UP_WAIT_TX_DONE:
      if (timeToHop == true) //If the channelTimer has expired, move to next frequency
        hopChannel();

      if (transactionComplete)
      {
        sf6ExpectedSize = headerBytes + CHANNEL_TIMER_BYTES + trailerBytes; //Tell SF6 to receive ACK packet

        triggerEvent(TRIGGER_LINK_WAIT_FOR_ACK);
        transactionComplete = false; //Reset ISR flag
        returnToReceiving();
        changeState(RADIO_P2P_LINK_UP_WAIT_ACK);
      }
      break;

    //====================
    //Wait for the ACK to be received
    //====================
    case RADIO_P2P_LINK_UP_WAIT_ACK:
      if (timeToHop == true) //If the channelTimer has expired, move to next frequency
        hopChannel();

      if (transactionComplete)
      {
        //Decode the received datagram
        PacketType packetType = rcvDatagram();

        //Process the received datagram
        switch (packetType)
        {
          default:
            triggerEvent(TRIGGER_UNKNOWN_PACKET);
            if (settings.debugDatagrams)
            {
              systemPrintTimestamp();
              systemPrint("RX: Unhandled packet type ");
              systemPrint(radioDatagramType[packetType]);
              systemPrintln();
              outputSerialData(true);
            }
            break;

          case DATAGRAM_BAD:
            triggerEvent(TRIGGER_BAD_PACKET);
            break;

          case DATAGRAM_CRC_ERROR:
            triggerEvent(TRIGGER_CRC_ERROR);
            break;

          case DATAGRAM_NETID_MISMATCH:
            triggerEvent(TRIGGER_NETID_MISMATCH);
            break;

          case DATAGRAM_PING:
            //Break the link
            breakLink();
            break;

          case DATAGRAM_DATA_ACK:
            //The datagram we are expecting
            syncChannelTimer(); //Adjust freq hop ISR based on remote's remaining clock

            //Stop the ACK timer
            ackTimer = 0;

            setHeartbeatLong(); //Those who send an ACK have short time to next heartbeat. Those who send a heartbeat or data have long time to next heartbeat.

            frequencyCorrection += radio.getFrequencyError() / 1000000.0;

            triggerEvent(TRIGGER_LINK_ACK_RECEIVED);
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

            frequencyCorrection += radio.getFrequencyError() / 1000000.0;

            triggerEvent(TRIGGER_LINK_SEND_ACK_FOR_HEARTBEAT);
            if (xmitDatagramP2PAck() == true) //Transmit ACK
            {
              setHeartbeatShort(); //We ack'd the packet so be responsible for sending the next heartbeat
              changeState(RADIO_P2P_LINK_UP_HB_ACK_REXMT);
            }

            break;

          case DATAGRAM_DATA:
            //Received data while waiting for ack.

            //Place the data in the serial output buffer
            serialBufferOutput(rxData, rxDataBytes);

            frequencyCorrection += radio.getFrequencyError() / 1000000.0;

            triggerEvent(TRIGGER_LINK_SEND_ACK_FOR_DATA);
            if (xmitDatagramP2PAck() == true) //Transmit ACK
            {
              setHeartbeatShort(); //We ack'd this data, so be responsible for sending the next heartbeat
              changeState(RADIO_P2P_LINK_UP_HB_ACK_REXMT);
            }
            break;
        }
      }

      //Check for ACK timeout, set at end of transmit, measures ACK timeout
      else if ((receiveInProcess() == false)
               && ((millis() - datagramTimer) >= (frameAirTime + ackAirTime + settings.overheadTime + getReceiveCompletionOffset())))
      {
        //Retransmit the packet
        if ((!settings.maxResends) || (frameSentCount < settings.maxResends))
        {
          //Throttle back retransmits based on the number of retransmits we've attempted
          //retransmitTimeout is a random number, set when the first datagram is sent
          if (millis() - datagramTimer > (frameSentCount * retransmitTimeout))
          {
            if (settings.debugDatagrams)
            {
              systemPrintTimestamp();
              systemPrintln("RX: ACK Timeout");
              outputSerialData(true);
            }

            triggerEvent(TRIGGER_LINK_RETRANSMIT);
            if (settings.debugDatagrams)
            {
              systemPrintTimestamp();
              systemPrint("TX: Retransmit ");
              systemPrint(frameSentCount);
              systemPrint(", ");
              systemPrint(radioDatagramType[txControl.datagramType]);
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
              outputSerialData(true);
            }

            if (retransmitDatagram(NULL) == true)
            {
              setHeartbeatLong(); //We're re-sending data, so don't be the first to send next heartbeat
              lostFrames++;
              changeState(RADIO_P2P_LINK_UP_WAIT_TX_DONE);
            }
          }
        }
        else
        {
          //Failed to reach the other system, break the link
          triggerEvent(TRIGGER_LINK_RETRANSMIT_FAIL);

          //Break the link
          breakLink();
        }
      }

      else if ((millis() - linkDownTimer) >= (P2P_LINK_BREAK_MULTIPLIER * settings.heartbeatTimeout))
        //Break the link
        breakLink();

      //Retransmits are not getting through in a rational time
      else if (ackTimer && ((millis() - ackTimer) >= (P2P_LINK_BREAK_MULTIPLIER * settings.heartbeatTimeout)))
        //Break the link
        breakLink();
      break;

    //====================
    //Wait for the HEARTBEAT frame to complete transmission then wait for ACK
    //and retransmit previous data frame if necessary
    //====================
    case RADIO_P2P_LINK_UP_HB_ACK_REXMT:
      if (timeToHop == true) //If the channelTimer has expired, move to next frequency
        hopChannel();

      //An ACK was expected for a previous transmission that must have been
      //lost. A heartbeat was received instead which was ACKed. Once the ACK
      //completes transmission, retransmit the previously lost datagram.
      if (transactionComplete)
      {
        transactionComplete = false; //Reset ISR flag

        //Retransmit the packet
        if ((!settings.maxResends) || (rexmtFrameSentCount < settings.maxResends))
        {
          RESTORE_TX_BUFFER();
          if (settings.debugDatagrams)
          {
            systemPrintTimestamp();
            systemPrint("TX: Retransmit ");
            systemPrint(frameSentCount);
            systemPrint(", ");
            systemPrint(radioDatagramType[txControl.datagramType]);
            outputSerialData(true);
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
                outputSerialData(true);
                break;
            }
          }

          triggerEvent(TRIGGER_LINK_HB_ACK_REXMIT);

          if (retransmitDatagram(NULL) == true)
          {
            setHeartbeatLong(); //We're re-sending data, so don't be the first to send next heartbeat
            lostFrames++;
            changeState(RADIO_P2P_LINK_UP_WAIT_TX_DONE);
          }
        }
        else
          //Failed to reach the other system, break the link
          breakLink();
      }
      break;

    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    //Multi-Point Data Exchange
    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

    //====================
    //Start searching for other radios
    //====================
    case RADIO_DISCOVER_BEGIN:
      stopChannelTimer(); //Stop hopping

      multipointChannelLoops = 0;
      multipointAttempts = 0;

      triggerEvent(TRIGGER_MP_SCAN);
      changeState(RADIO_DISCOVER_SCANNING);
      break;

    //====================
    //Walk through channel table backwards, transmitting a Ping and looking for an ACK1
    //====================
    case RADIO_DISCOVER_SCANNING:
      if (transactionComplete)
      {
        //Decode the received datagram
        PacketType packetType = rcvDatagram();

        //Process the received datagram
        switch (packetType)
        {
          default:
            triggerEvent(TRIGGER_UNKNOWN_PACKET);
            if (settings.debugDatagrams)
            {
              systemPrintTimestamp();
              systemPrint("Scan: Unhandled packet type ");
              systemPrint(radioDatagramType[packetType]);
              systemPrintln();
              outputSerialData(true);
            }
            break;

          case DATAGRAM_BAD:
            triggerEvent(TRIGGER_BAD_PACKET);
            break;

          case DATAGRAM_CRC_ERROR:
            triggerEvent(TRIGGER_CRC_ERROR);
            break;

          case DATAGRAM_NETID_MISMATCH:
            triggerEvent(TRIGGER_NETID_MISMATCH);
            break;

          case DATAGRAM_ACK_2:
          case DATAGRAM_DATA:
          case DATAGRAM_DATA_ACK:
          case DATAGRAM_PING: //Clients do not respond to pings, only the server
          case DATAGRAM_REMOTE_COMMAND:
          case DATAGRAM_REMOTE_COMMAND_RESPONSE:
            //We should not be receiving these datagrams, but if we do, just ignore
            triggerEvent(TRIGGER_BAD_PACKET);
            break;

          case DATAGRAM_ACK_1:
            triggerEvent(TRIGGER_LINK_ACK_RECEIVED);

            //Server has responded with ACK
            syncChannelTimer(); //Start and adjust freq hop ISR based on remote's remaining clock

            if (settings.operatingMode == MODE_VIRTUAL_CIRCUIT)
              channelNumber = rxVcData[0];
            else
              channelNumber = rxData[0]; //Change to the server's channel number

            if (settings.debugSync)
            {
              systemPrint("    Channel Number: ");
              systemPrintln(channelNumber);
              outputSerialData(true);
              if (timeToHop == true) //If the channelTimer has expired, move to next frequency
                hopChannel();
            }

            frequencyCorrection += radio.getFrequencyError() / 1000000.0;

            lastPacketReceived = millis(); //Reset

            if (settings.operatingMode == MODE_VIRTUAL_CIRCUIT)
            {
              //Acknowledge the ACK1
              triggerEvent(TRIGGER_SEND_ACK2);
              if (xmitVcAck2(VC_SERVER))
              {
                sf6ExpectedSize = MAX_PACKET_SIZE; //Tell SF6 to return to max packet length
                changeState(RADIO_VC_WAIT_TX_DONE);
              }
            }
            else
              changeState(RADIO_MP_STANDBY);

            break;
        }
      }

      //Nothing received
      else if (receiveInProcess() == false)
      {
        //Check for a receive timeout
        if ((millis() - datagramTimer) >= (frameAirTime + ackAirTime + settings.overheadTime + getReceiveCompletionOffset()))
        {
          if (settings.debugDatagrams)
          {
            systemPrintTimestamp();
            systemPrintln("MP: ACK1 Timeout");
            outputSerialData(true);
          }

          //Move to previous channel in table
          hopChannelReverse();

          if (channelNumber == 0) multipointChannelLoops++;

          if (multipointChannelLoops > 10)
          {
            multipointChannelLoops = 0;

            multipointAttempts++;
            //Throttle the scanning. Stop transmitting for 60s times the number of attempts.
            unsigned long startTime = millis();
            while ((millis() - startTime) < (60 * 1000 * multipointAttempts))
            {
              delay(10);
              petWDT();
            }
          }

          //Send ping
          if (settings.operatingMode == MODE_VIRTUAL_CIRCUIT)
          {
            //Send the PING datagram, first part of the 3-way handshake
            if (xmitVcPing(VC_SERVER))
            {
              changeState(RADIO_DISCOVER_WAIT_TX_PING_DONE);
            }
          }
          else
          {
            if (xmitDatagramMpPing() == true)
              changeState(RADIO_DISCOVER_WAIT_TX_PING_DONE);
          }
        }
      }

      break;

    //====================
    //Wait for the PING to complete transmission
    //====================
    case RADIO_DISCOVER_WAIT_TX_PING_DONE:
      if (transactionComplete)
      {
        transactionComplete = false; //Reset ISR flag
        returnToReceiving();
        changeState(RADIO_DISCOVER_SCANNING);
      }
      break;

    //====================
    //Wait for the ACK to complete transmission
    //====================
    case RADIO_MP_WAIT_TX_ACK_DONE:
      if (transactionComplete)
      {
        transactionComplete = false; //Reset ISR flag
        returnToReceiving();
        changeState(RADIO_MP_STANDBY);
      }
      break;

    //====================
    //Wait for the next operation (listed in priority order):
    // * Frame received
    // * Data to send
    // * Time to send HEARTBEAT
    // * Link timeout
    //====================
    case RADIO_MP_STANDBY:
      //Hop channels when required
      if (timeToHop == true)
        hopChannel();

      //Process the receive packet
      if (transactionComplete == true)
      {
        triggerEvent(TRIGGER_MP_PACKET_RECEIVED);

        //Decode the received datagram
        PacketType packetType = rcvDatagram();

        //Process the received datagram
        switch (packetType)
        {
          default:
            triggerEvent(TRIGGER_UNKNOWN_PACKET);
            if (settings.debugDatagrams)
            {
              systemPrintTimestamp();
              systemPrint("MP Standby: Unhandled packet type ");
              systemPrint(radioDatagramType[packetType]);
              systemPrintln();
              outputSerialData(true);
            }
            break;

          case DATAGRAM_BAD:
            triggerEvent(TRIGGER_BAD_PACKET);
            break;

          case DATAGRAM_CRC_ERROR:
            triggerEvent(TRIGGER_CRC_ERROR);
            break;

          case DATAGRAM_NETID_MISMATCH:
            triggerEvent(TRIGGER_NETID_MISMATCH);
            break;

          case DATAGRAM_ACK_1:
          case DATAGRAM_ACK_2:
          case DATAGRAM_DATAGRAM:
          case DATAGRAM_DATA_ACK:
          case DATAGRAM_REMOTE_COMMAND:
          case DATAGRAM_REMOTE_COMMAND_RESPONSE:
            //We should not be receiving these datagrams, but if we do, just ignore
            frequencyCorrection += radio.getFrequencyError() / 1000000.0;
            triggerEvent(TRIGGER_BAD_PACKET);
            break;

          case DATAGRAM_PING:
            //A new radio is saying hello
            if (settings.server == true)
            {
              //Ack their ping with sync data
              if (xmitDatagramMpAck() == true)
              {
                triggerEvent(TRIGGER_MP_SEND_ACK_FOR_PING);
                changeState(RADIO_MP_WAIT_TX_ACK_DONE);
              }
            }
            else
            {
              changeState(RADIO_MP_STANDBY);
            }
            break;

          case DATAGRAM_HEARTBEAT:
            //Received data or heartbeat. Sync clock, do not ack.
            syncChannelTimer(); //Adjust freq hop ISR based on remote's remaining clock

            frequencyCorrection += radio.getFrequencyError() / 1000000.0;

            lastPacketReceived = millis(); //Update timestamp for Link LED

            changeState(RADIO_MP_STANDBY);
            break;

          case DATAGRAM_DATA:
            //Received data or heartbeat. Sync clock, do not ack.
            syncChannelTimer(); //Adjust freq hop ISR based on remote's remaining clock

            setHeartbeatMultipoint(); //We're sync'd so reset heartbeat timer

            //Place any available data in the serial output buffer
            serialBufferOutput(rxData, rxDataBytes);

            frequencyCorrection += radio.getFrequencyError() / 1000000.0;

            triggerEvent(TRIGGER_MP_DATA_PACKET);
            lastPacketReceived = millis(); //Update timestamp for Link LED

            changeState(RADIO_MP_STANDBY);
            break;
        }
      }

      //If the radio is available, send any data in the serial buffer over the radio
      else if (receiveInProcess() == false)
      {
        heartbeatTimeout = ((millis() - heartbeatTimer) > heartbeatRandomTime);

        //Check for time to send serial data
        if (availableRadioTXBytes() && (processWaitingSerial(heartbeatTimeout) == true))
        {
          triggerEvent(TRIGGER_MP_DATA_PACKET);
          if (xmitDatagramMpData() == true)
          {
            setHeartbeatMultipoint(); //We're sending something with clock data so reset heartbeat timer
            changeState(RADIO_MP_WAIT_TX_DONE);
          }
        }

        //Only the server transmits heartbeats
        else if (settings.server == true)
        {
          if (heartbeatTimeout)
          {
            triggerEvent(TRIGGER_HEARTBEAT);
            if (xmitDatagramMpHeartbeat() == true)
            {
              setHeartbeatMultipoint(); //We're sending something with clock data so reset heartbeat timer
              changeState(RADIO_MP_WAIT_TX_DONE); //Wait for heartbeat to transmit
            }
          }
        }

        //If the client hasn't received a packet in too long, return to scanning
        else if (settings.server == false)
        {
          if ((millis() - lastPacketReceived) > (settings.heartbeatTimeout * 3))
          {
            changeState(RADIO_DISCOVER_BEGIN);
          }
        }
      }
      break;

    //====================
    //Wait for the frame transmission to complete
    //====================
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
      }
      break;

    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    //Client Training
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
        RADIO_TRAIN_WAIT_TX_PING_DONE             |
                |                                 |
                V                                 | Timeout
        RADIO_TRAIN_WAIT_RX_RADIO_PARAMETERS -----'
                |
                | Update settings
                | Send client ACK
                |
                V
        RADIO_TRAIN_WAIT_TX_PARAM_ACK_DONE
                |
                V
      endTrainingClientServer
                |
                | Restore settings
                |
                V
    */

    //====================
    //Wait for the PING to complete transmission
    //====================
    case RADIO_TRAIN_WAIT_TX_PING_DONE:

      //If dio0ISR has fired, we are done transmitting
      if (transactionComplete == true)
      {
        transactionComplete = false;

        //Indicate that the receive is complete
        triggerEvent(TRIGGER_TRAINING_CLIENT_TX_PING_DONE);

        //Start the receive operation
        returnToReceiving();

        //Set the next state
        changeState(RADIO_TRAIN_WAIT_RX_RADIO_PARAMETERS);
      }
      break;

    //====================
    //Wait to receive the radio parameters
    //====================
    case RADIO_TRAIN_WAIT_RX_RADIO_PARAMETERS:

      //If dio0ISR has fired, a packet has arrived
      if (transactionComplete == true)
      {
        trainingPreviousRxInProgress = false;

        //Decode the received datagram
        PacketType packetType = rcvDatagram();

        //Process the received datagram
        switch (packetType)
        {
          default:
            triggerEvent(TRIGGER_BAD_PACKET);
            break;

          case DATAGRAM_TRAINING_PARAMS:
            //Verify the IDs
            if ((memcmp(rxData, myUniqueId, UNIQUE_ID_BYTES) != 0)
                && (memcmp(rxData, myUniqueId, UNIQUE_ID_BYTES) != 0))
            {
              triggerEvent(TRIGGER_BAD_PACKET);
              break;
            }

            //Save the training partner ID
            memcpy(trainingPartnerID, &rxData[UNIQUE_ID_BYTES], UNIQUE_ID_BYTES);

            //Get the radio parameters
            updateRadioParameters(&rxData[UNIQUE_ID_BYTES * 2]);

            //Acknowledge the radio parameters
            if (xmitDatagramTrainingAck(&rxData[UNIQUE_ID_BYTES]) == true)
              changeState(RADIO_TRAIN_WAIT_TX_PARAM_ACK_DONE);
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
      {
        //If we are training with button, in P2P mode, and user has not set server mode
        //Automatically switch to server
        if (trainViaButton
            && tempSettings.operatingMode == MODE_POINT_TO_POINT
            && originalServer == false)
        {
          //Give up and change to Server automatically

          settings = tempSettings; //Return to original radio settings

          generateRandomKeysID(); //Generate random netID and AES key

          beginTrainingServer(); //Change to server
        }
        else
        {
          xmitDatagramTrainingPing(); //Continue retrying as client
        }
      }
      break;

    //====================
    //Wait for the ACK frame to complete transmission
    //====================
    case RADIO_TRAIN_WAIT_TX_PARAM_ACK_DONE:

      //If dio0ISR has fired, we are done transmitting
      if (transactionComplete == true)
      {
        transactionComplete = false;
        endClientServerTraining(TRIGGER_TRAINING_CLIENT_TX_ACK_DONE);
      }
      break;

    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    //Server Training
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
        .------ RADIO_TRAIN_WAIT_FOR_PING                 |
        |               |                                 |
        |               | Send client ping                |
        |               |                                 |
        |               V                                 |
        |      RADIO_TRAIN_WAIT_TX_RADIO_PARAMS_DONE -----'
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

    //====================
    //Wait for a PING frame from a client
    //====================
    case RADIO_TRAIN_WAIT_FOR_PING:

      //If dio0ISR has fired, a packet has arrived
      if (transactionComplete == true)
      {
        trainingPreviousRxInProgress = false;

        //Decode the received datagram
        PacketType packetType = rcvDatagram();

        //Process the received datagram
        switch (packetType)
        {
          default:
            triggerEvent(TRIGGER_BAD_PACKET);
            break;

          case DATAGRAM_TRAINING_PING:
            //Save the client ID
            memcpy(trainingPartnerID, rxData, UNIQUE_ID_BYTES);

            //Wait for the transmit to complete
            if (xmitDatagramTrainRadioParameters(trainingPartnerID) == true)
              changeState(RADIO_TRAIN_WAIT_TX_RADIO_PARAMS_DONE);
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

              //If we are training via button, and in point to point mode, and the user has not manually set the server
              //then reboot with current settings after a single client acks
              if (trainViaButton
                  && tempSettings.operatingMode == MODE_POINT_TO_POINT
                  && originalServer == false)
              {
                //Reboot the radio with the newly generated random netID/Key parameters
                petWDT();
                systemFlush();
                systemReset();
              }
            }
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

    //====================
    //Wait for the radio parameters to complete transmission
    //====================
    case RADIO_TRAIN_WAIT_TX_RADIO_PARAMS_DONE:

      //If dio0ISR has fired, we are done transmitting
      if (transactionComplete == true)
      {
        transactionComplete = false;

        //Indicate that the receive is complete
        triggerEvent(TRIGGER_TRAINING_SERVER_TX_PARAMS_DONE);

        //Start the receive operation
        returnToReceiving();

        //Set the next state
        changeState(RADIO_TRAIN_WAIT_FOR_PING);
      }
      break;

    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    //Virtual Circuit States
    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

    /*

      Radio States:

                   RADIO_RESET
                        |
                        V
              RADIO_DISCOVER_SCANNING
                        |
                        V
                        +<-------------------------.
                        |                          |
                        | Send VC_HEARTBEAT        |
                        |                          |
                        V                          |
      .-----> RADIO_VC_WAIT_TX_DONE                |
      |                 |                          |
      |                 V                          | Heartbeat timeout
      |       RADIO_VC_WAIT_RECEIVE ---------------'
      |                 |
      |     ACK Timeout | Receive serial data
      |      Retransmit | Send DATA
      |                 |
      |  Remote Cmd Rsp | Remote Cmd
      |    Send Cmd Rsp | Send remote Cmd
      |                 |
      '-----------------'

      3-way Handshake to zero ACKs:

        TX PING --> RX ACK1 --> TX ACK2
           or
        RX PING --> TX ACK1 --> RX ACK2

      VC States:

      .-------------> VC_STATE_LINK_DOWN <-------------------------------------.
      |                       |                                     HB Timeout |
      |                       | HEARTBEAT received                             |
      | HB Timeout            v                                                |
      +<----------- VC_STATE_LINK_ALIVE ------------------------.              |
      ^                       |                       RX PING   |              |
      |                       | ATC command                     |              |
      | HB Timeout            V                                 |              |
      +<------------- VC_STATE_SEND_PING <---------.            |              |
      ^                       |            ACK1 RX |            |              |
      |                       | TX PING    Timeout |            |              |
      |                       | TX Complete        |   RX       |              |
      | HB Timeout            V                    |  PING      V              |
      +<------------- VC_STATE_WAIT_ACK1 --------->+------->+-->+              |
      ^                       |                    ^        ^   |              |
      |                       | RX ACK1            |   ACK2 |   | TX ACK1      |
      |                       | TX ACK2            |     RX |   | TX Complete  |
      |                       | TX Complete        |    TMO |   V              |
      |       Zero ACK values |                    |    VC_STATE_WAIT_ACK2 ----'
      |                       |      .-------------'            |
      |                       |      | RX PING                  | RX ACK2
      | HB Timeout            V      |                          | Zero ACK values
      '-------------- VC_STATE_CONNECTED <----------------------'

    */

    //====================
    //Wait for the transmission to complete
    //====================
    case RADIO_VC_WAIT_TX_DONE:
      //Hop channels when required
      if (timeToHop == true)
        hopChannel();

      //If dio0ISR has fired, we are done transmitting
      if (transactionComplete == true)
      {
        transactionComplete = false;

        //Indicate that the transmission is complete
        triggerEvent(TRIGGER_VC_TX_DONE);

        //Start the receive operation
        returnToReceiving();

        //Set the next state
        changeState(RADIO_VC_WAIT_RECEIVE);
      }
      break;

    //====================
    //Wait for the next operation (listed in priority order):
    // * Frame received
    // * Time to send HEARTBEAT
    // * Time to retransmit previous frame
    // * Remote command response to send
    // * Data to send
    // * Link timeout
    //====================
    case RADIO_VC_WAIT_RECEIVE:
      //Hop channels when required
      if (timeToHop == true)
        hopChannel();

      //If dio0ISR has fired, a packet has arrived
      currentMillis = millis();
      if (transactionComplete == true)
      {
        //Decode the received datagram
        PacketType packetType = rcvDatagram();
        vcHeader = (VC_RADIO_MESSAGE_HEADER *)rxData;

        //Indicate receive traffic from this VC including HEARTBEATs
        if ((uint8_t)rxSrcVc < (uint8_t)MIN_RX_NOT_ALLOWED)
          virtualCircuitList[rxSrcVc & VCAB_NUMBER_MASK].lastTrafficMillis = currentMillis;

        //Process the received datagram
        switch (packetType)
        {
          default:
            triggerEvent(TRIGGER_BAD_PACKET);
            break;

          case DATAGRAM_VC_HEARTBEAT:
            vcReceiveHeartbeat(millis() - currentMillis);
            break;

          case DATAGRAM_DATA:
            //Move the data into the serial output buffer
            if (settings.debugSerial)
            {
              systemPrint("updateRadioState moving ");
              systemPrint(rxDataBytes);
              systemPrintln(" bytes from inputBuffer into serialTransmitBuffer");
              outputSerialData(true);
            }
            systemWrite(START_OF_VC_SERIAL);
            serialBufferOutput(rxData, rxDataBytes);

            //Acknowledge the data frame
            if (xmitVcAckFrame(rxSrcVc))
              changeState(RADIO_VC_WAIT_TX_DONE);
            break;

          case DATAGRAM_DATAGRAM:
            //Move the data into the serial output buffer
            if (settings.debugSerial)
            {
              systemPrint("updateRadioState moving ");
              systemPrint(rxDataBytes);
              systemPrintln(" bytes from inputBuffer into serialTransmitBuffer");
              outputSerialData(true);
            }
            systemWrite(START_OF_VC_SERIAL);
            serialBufferOutput(rxData, rxDataBytes);

            //Datagrams do NOT get ACKed
            break;

          case DATAGRAM_DATA_ACK:
            vcSendPcAckNack(rexmtTxDestVc, true);
            ackTimer = 0;
            break;

          case DATAGRAM_DUPLICATE:
            frequencyCorrection += radio.getFrequencyError() / 1000000.0;

            triggerEvent(TRIGGER_LINK_SEND_ACK_FOR_DUP);
            if (xmitVcAckFrame(rxSrcVc))
              changeState(RADIO_VC_WAIT_TX_DONE);
            break;

          //Second step in the 3-way handshake, received PING, respond with ACK1
          case DATAGRAM_PING:
            //Only respond to pings if we are server
            if (settings.server == true)
            {
              if (rxSrcVc == VC_UNASSIGNED)
              {
                //We received a ping from a previously unknown client
                if (xmitVcAck1(rxSrcVc))
                {
                  triggerEvent(TRIGGER_MP_SEND_ACK_FOR_PING);
                  changeState(RADIO_VC_WAIT_TX_DONE);
                }
              }
              else
              {
                //Known client
                if (xmitVcAck1(rxSrcVc))
                  changeState(RADIO_VC_WAIT_TX_DONE);

                vcChangeState(rxSrcVc, VC_STATE_WAIT_FOR_ACK2);
                virtualCircuitList[rxSrcVc].lastPingMillis = datagramTimer;
              }
            }

            break;

          //Third step in the 3-way handshake, received ACK1, respond with ACK2
          case DATAGRAM_ACK_1:
            if (xmitVcAck2(rxSrcVc))
            {
              changeState(RADIO_VC_WAIT_TX_DONE);
              vcZeroAcks(rxSrcVc);
              vcChangeState(rxSrcVc, VC_STATE_CONNECTED);
            }
            break;

          //Last step in the 3-way handshake, received ACK2, done
          case DATAGRAM_ACK_2:
            vcZeroAcks(rxSrcVc);
            vcChangeState(rxSrcVc, VC_STATE_CONNECTED);
            break;

          case DATAGRAM_REMOTE_COMMAND:
            rmtCmdVc = rxSrcVc;

            //Copy the command into the command buffer
            commandLength = rxDataBytes - VC_RADIO_HEADER_BYTES;
            memcpy(commandBuffer, &rxData[VC_RADIO_HEADER_BYTES], commandLength);
            if (settings.debugSerial)
            {
              systemPrint("RX: Moving ");
              systemPrint(commandLength);
              systemPrintln(" bytes into commandBuffer");
              outputSerialData(true);
              dumpBuffer((uint8_t *)commandBuffer, commandLength);
            }
            petWDT();

            frequencyCorrection += radio.getFrequencyError() / 1000000.0;
            triggerEvent(TRIGGER_LINK_SEND_ACK_FOR_REMOTE_COMMAND);

            //Transmit ACK
            if (xmitVcAckFrame(rxSrcVc))
              changeState(RADIO_VC_WAIT_TX_DONE);

            //Process the command
            petWDT();
            printerEndpoint = PRINT_TO_RF; //Send prints to RF link
            checkCommand(); //Parse the command buffer
            petWDT();
            printerEndpoint = PRINT_TO_SERIAL;
            length = availableTXCommandBytes();
            if (settings.debugSerial)
            {
              systemPrint("RX: checkCommand placed ");
              systemPrint(commandLength);
              systemPrintln(" bytes into commandTXBuffer");
              dumpCircularBuffer(commandTXBuffer, commandTXTail, sizeof(commandTXBuffer), length);
              outputSerialData(true);
              petWDT();
            }
            break;

          case DATAGRAM_REMOTE_COMMAND_RESPONSE:
            //Debug the serial path
            if (settings.debugSerial)
            {
              systemPrint("Moving ");
              systemPrint(rxDataBytes);
              systemPrintln(" from incomingBuffer to serialTransmitBuffer");
              dumpBuffer(rxData, rxDataBytes);
              outputSerialData(true);
            }

            //Place the data in to the serialTransmitBuffer
            serialOutputByte(START_OF_VC_SERIAL);
            vcHeader->destVc |= PC_REMOTE_RESPONSE;
            serialBufferOutput(rxData, rxDataBytes);

            frequencyCorrection += radio.getFrequencyError() / 1000000.0;

            //ACK the command response
            triggerEvent(TRIGGER_LINK_SEND_ACK_FOR_REMOTE_COMMAND_RESPONSE);
            if (xmitVcAckFrame(rxSrcVc)) //Transmit ACK
              changeState(RADIO_VC_WAIT_TX_DONE);
            break;
        }
      }

      //----------
      //Priority 1: Transmit a HEARTBEAT if necessary
      //----------
      else if (((currentMillis - heartbeatTimer) >= heartbeatRandomTime)
               && (receiveInProcess() == false))
      {
        //Send another heartbeat
        if (xmitVcHeartbeat(myVc, myUniqueId))
        {
          if (((uint8_t)myVc) < MAX_VC)
            virtualCircuitList[myVc].lastTrafficMillis = currentMillis;
          changeState(RADIO_VC_WAIT_TX_DONE);
        }
      }

      //----------
      //Priority 2: Wait for an outstanding ACK until it is received, don't
      //transmit any other data
      //----------
      else if (ackTimer)
      {
        //Verify that the link is still up
        txDestVc = rexmtTxDestVc;
        if ((txDestVc != VC_BROADCAST)
            && (virtualCircuitList[txDestVc & VCAB_NUMBER_MASK].vcState == VC_STATE_LINK_DOWN))
        {
          //Stop the retransmits
          ackTimer = 0;
        }

        //Check for retransmit needed
        else if ((currentMillis - ackTimer) >= (frameAirTime + ackAirTime + settings.overheadTime + getReceiveCompletionOffset()))
        {
          //Determine if another retransmit is allowed
          if ((!settings.maxResends) || (rexmtFrameSentCount < settings.maxResends))
          {
            rexmtFrameSentCount++;

            //Restore the message for retransmission
            RESTORE_TX_BUFFER();
            if (settings.debugDatagrams)
            {
              systemPrintTimestamp();
              systemPrint("TX: Retransmit ");
              systemPrint(frameSentCount);
              systemPrint(", ");
              systemPrint(radioDatagramType[txControl.datagramType]);
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
              outputSerialData(true);
            }

            //Retransmit the packet
            if (retransmitDatagram(((uint8_t)txDestVc <= MAX_VC) ? &virtualCircuitList[txDestVc] : NULL))
              changeState(RADIO_VC_WAIT_TX_DONE);

            //Since ackTimer is off when equal to zero, force it to a non-zero value
            ackTimer = datagramTimer;
            if (!ackTimer)
              ackTimer = 1;
            lostFrames++;
          }
          else
          {
            //Failed to reach the other system, break the link
            vcBreakLink(txDestVc);
          }
        }
      }

      //----------
      //Priority 3: Send the entire command response, toggle between waiting for
      //ACK above and transmitting the command response
      //----------
      else if (availableTXCommandBytes())
      {
        //Send the next portion of the command response
        vcHeader = (VC_RADIO_MESSAGE_HEADER *)&outgoingPacket[headerBytes];
        endOfTxData += VC_RADIO_HEADER_BYTES;
        vcHeader->destVc = rmtCmdVc;
        vcHeader->srcVc = myVc;
        vcHeader->length = readyOutgoingCommandPacket(VC_RADIO_HEADER_BYTES)
                           + VC_RADIO_HEADER_BYTES;
        if (xmitDatagramP2PCommandResponse())
          changeState(RADIO_VC_WAIT_TX_DONE);

        //Since ackTimer is off when equal to zero, force it to a non-zero value
        ackTimer = datagramTimer;
        if (!ackTimer)
          ackTimer = 1;

        //Save the message for retransmission
        SAVE_TX_BUFFER();
        rexmtTxDestVc = txDestVc;
      }

      //----------
      //Priority 4: Walk through the 3-way handshake
      //----------
      //      else if (vcConnecting)
      //      {
      //        for (index = 0; index < MAX_VC; index++)
      //        {
      //          if (receiveInProcess())
      //            break;
      //
      //          //Determine the first VC that is walking through connections
      //          if (vcConnecting & (1 << index))
      //          {
      //            //Determine if PING needs to be sent
      //            if (virtualCircuitList[index].vcState == VC_STATE_SEND_PING)
      //            {
      //              //Send the PING datagram, first part of the 3-way handshake
      //              if (xmitVcPing(index))
      //              {
      //                vcChangeState(index, VC_STATE_WAIT_FOR_ACK1);
      //                virtualCircuitList[index].lastPingMillis = datagramTimer;
      //                changeState(RADIO_VC_WAIT_TX_DONE);
      //              }
      //            }
      //
      //            //The ACK1 is handled with the receive code
      //            //Check for a timeout waiting for the ACK1
      //            else if (virtualCircuitList[index].vcState == VC_STATE_WAIT_FOR_ACK1)
      //            {
      //              if ((currentMillis - virtualCircuitList[index].lastPingMillis)
      //                  >= (frameAirTime + ackAirTime + settings.overheadTime + getReceiveCompletionOffset()))
      //              {
      //                //Retransmit the PING
      //                if (xmitVcPing(index))
      //                {
      //                  vcChangeState(index, VC_STATE_WAIT_FOR_ACK1);
      //                  virtualCircuitList[index].lastPingMillis = datagramTimer;
      //                  changeState(RADIO_VC_WAIT_TX_DONE);
      //                }
      //              }
      //            }
      //
      //            //The ACK2 is handled with the receive code
      //            //Check for a timeout waiting for the ACK2
      //            else if (virtualCircuitList[index].vcState == VC_STATE_WAIT_FOR_ACK2)
      //            {
      //              if ((currentMillis - virtualCircuitList[index].lastPingMillis)
      //                  >= (frameAirTime + ackAirTime + settings.overheadTime + getReceiveCompletionOffset()))
      //              {
      //                //Retransmit the ACK1
      //                if (xmitVcAck1(index))
      //                {
      //                  vcChangeState(index, VC_STATE_WAIT_FOR_ACK2);
      //                  virtualCircuitList[index].lastPingMillis = datagramTimer;
      //                  changeState(RADIO_VC_WAIT_TX_DONE);
      //                }
      //              }
      //            }
      //
      //            //Work on only one connection at a time
      //            break;
      //          }
      //        }
      //      }

      //----------
      //Lowest Priority: Check for data to send
      //----------
      else if (vcSerialMessageReceived())
      {
        if (receiveInProcess() == false)
        {
          //No need to add the VC header since the header is in the radioTxBuffer
          //Get the VC header
          vcHeader = (VC_RADIO_MESSAGE_HEADER *)&outgoingPacket[headerBytes];
          if (vcHeader->destVc == VC_BROADCAST)
            channel = 0;
          else
            channel = (vcHeader->destVc >> VCAB_NUMBER_BITS) & VCAB_CHANNEL_MASK;
          switch (channel)
          {
            case 0: //Data packets
              //Check for datagram transmission
              if (vcHeader->destVc == VC_BROADCAST)
              {
                //Broadcast this data to all VCs, no ACKs will be received
                triggerEvent(TRIGGER_VC_TX_DATA);
                xmitVcDatagram();
                break;
              }

              //Transmit the packet
              triggerEvent(TRIGGER_VC_TX_DATA);
              if (xmitDatagramP2PData() == true)
                changeState(RADIO_VC_WAIT_TX_DONE);

              //Since ackTimer is off when equal to zero, force it to a non-zero value
              ackTimer = datagramTimer;
              if (!ackTimer)
                ackTimer = 1;

              //Save the message for retransmission
              SAVE_TX_BUFFER();
              rexmtTxDestVc = txDestVc;
              break;

            case 1: //Remote command packets
              //Remote commands must not be broadcast
              if (vcHeader->destVc == VC_BROADCAST)
              {
                if (settings.debugSerial || settings.debugTransmit)
                {
                  systemPrintln("ERROR: Remote commands may not be broadcast!");
                  outputSerialData(true);
                }

                //Discard this message
                endOfTxData = &outgoingPacket[headerBytes];
                break;
              }

              //Determine if this remote command gets processed on the local node
              if ((vcHeader->destVc & VCAB_NUMBER_MASK) == myVc)
              {
                //Copy the command into the command buffer
                commandLength = endOfTxData - &outgoingPacket[headerBytes + VC_RADIO_HEADER_BYTES];
                memcpy(commandBuffer, &outgoingPacket[headerBytes + VC_RADIO_HEADER_BYTES], commandLength);
                if (settings.debugSerial)
                {
                  systemPrint("RX: Moving ");
                  systemPrint(commandLength);
                  systemPrintln(" bytes into commandBuffer");
                  outputSerialData(true);
                  dumpBuffer((uint8_t *)commandBuffer, commandLength);
                }
                petWDT();

                //Reset the buffer data pointer for the next transmit operation
                endOfTxData = &outgoingPacket[headerBytes];

                //Process the command
                petWDT();
                printerEndpoint = PRINT_TO_RF; //Send prints to RF link
                checkCommand(); //Parse the command buffer
                petWDT();
                printerEndpoint = PRINT_TO_SERIAL;
                length = availableTXCommandBytes();
                if (settings.debugSerial)
                {
                  systemPrint("RX: checkCommand placed ");
                  systemPrint(length);
                  systemPrintln(" bytes into commandTXBuffer");
                  dumpCircularBuffer(commandTXBuffer, commandTXTail, sizeof(commandTXBuffer), length);
                  outputSerialData(true);
                  petWDT();
                }

                //Break up the command response
                while (availableTXCommandBytes())
                  readyLocalCommandPacket();
                break;
              }

              //Send the remote command
              vcHeader->destVc &= VCAB_NUMBER_MASK;
              if (xmitDatagramP2PCommand() == true)
                changeState(RADIO_VC_WAIT_TX_DONE);

              //Since ackTimer is off when equal to zero, force it to a non-zero value
              ackTimer = datagramTimer;
              if (!ackTimer)
                ackTimer = 1;

              //Save the message for retransmission
              SAVE_TX_BUFFER();
              rexmtTxDestVc = txDestVc;
              break;
          }
        }
      }

      //----------
      //Check for link timeout
      //----------
      serverLinkBroken = false;
      for (index = 0; index < MAX_VC; index++)
      {
        //Don't timeout the connection to myself
        if (index == myVc)
          continue;

        //Determine if the link has timed out
        vc = &virtualCircuitList[index];
        if ((vc->vcState != VC_STATE_LINK_DOWN) && (serverLinkBroken
            || ((currentMillis - vc->lastTrafficMillis) > (VC_LINK_BREAK_MULTIPLIER * settings.heartbeatTimeout))))
        {
          if (index == VC_SERVER)
          {
            //When the server connection breaks, break all other connections and
            //wait for the server.  The server provides the time (hop) synchronization
            //between all of the nodes.  When the server fails, the clocks drift
            //and the radios loose communications because they are hopping at different
            //times.
            serverLinkBroken = true;
            changeState(RADIO_DISCOVER_BEGIN);
          }

          //Break the link
          vcBreakLink(index);
        }
      }
      break;
  }
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

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

  //Add channel timer bytes to header
  if (settings.frequencyHop == true)
    headerBytes += CHANNEL_TIMER_BYTES;

  //Add the byte containing the frame size (only needed in SF6)
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

//Verify the radio state definitions against the radioStateTable
bool verifyRadioStateTable()
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
  int valid;

  //Verify that all the entries are in the state table
  valid = true;
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
      waitForever();
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
    valid = false;
  }
  return valid;
}

//Verify the PacketType enums against the radioDatagramType
bool verifyRadioDatagramType()
{
  bool valid;

  valid = ((sizeof(radioDatagramType) / sizeof(radioDatagramType[0])) == MAX_DATAGRAM_TYPE);
  if (!valid)
    systemPrintln("ERROR - Please update the radioDatagramTable");
  return valid;
}

//Change states and print the new state
void changeState(RadioStates newState)
{
  radioState = newState;
  radioStateHistory[radioState] = millis();

  if ((settings.debug == false) && (settings.debugStates == false))
    return;

  displayState(newState);
  outputSerialData(true);
}

//Display the state transition
void displayState(RadioStates newState)
{
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
      systemPrint(radioStateTable[radioState].description);
    else
      systemPrint(radioStateTable[radioState].name);
  }

  if (newState == RADIO_P2P_LINK_UP)
  {
    unsigned int seconds = (millis() - lastLinkUpTime) / 1000;
    unsigned int minutes = seconds / 60;
    seconds -= (minutes * 60);
    unsigned int hours = minutes / 60;
    minutes -= (hours * 60);
    unsigned int days = hours / 24;
    hours -= (days * 24);

    systemPrint(" LinkUptime: ");

    if (days < 10) systemPrint(" ");
    if (days)
      systemPrint(days);
    else
      systemPrint(" ");
    systemPrint(" ");

    if (hours < 10) systemPrint(" ");
    systemPrint(hours);
    systemPrint(":");
    if (minutes < 10) systemPrint("0");
    systemPrint(minutes);
    systemPrint(":");
    if (seconds < 10) systemPrint("0");
    systemPrint(seconds);
  }

  systemPrintln();
}

//Display the radio state history
void displayRadioStateHistory()
{
  uint8_t index;
  uint8_t sortOrder[RADIO_MAX_STATE];
  uint8_t temp;

  //Set the default sort order
  petWDT();
  for (index = 0; index < RADIO_MAX_STATE; index++)
    sortOrder[index] = index;

  //Perform a bubble sort
  for (index = 0; index < RADIO_MAX_STATE; index++)
    for (int x = index + 1; x < RADIO_MAX_STATE; x++)
      if (radioStateHistory[sortOrder[index]] > radioStateHistory[sortOrder[x]])
      {
        temp = sortOrder[index];
        sortOrder[index] = sortOrder[x];
        sortOrder[x] = temp;
      }

  //Display the radio state history
  for (index = 0; index < RADIO_MAX_STATE; index++)
    if (radioStateHistory[sortOrder[index]])
    {
      systemPrint("        ");
      systemPrintTimestamp(radioStateHistory[sortOrder[index]]);
      systemPrint(": ");
      systemPrint(radioStateTable[sortOrder[index]].name);
      systemPrintln();
    }
  petWDT();
}

//Break a point-to-point link
void breakLink()
{
  //Break the link
  linkFailures++;
  if (settings.printLinkUpDown)
  {
    systemPrintln("--------- Link DOWN ---------");
    outputSerialData(true);
  }
  triggerEvent(TRIGGER_RADIO_RESET);

  //Stop the ACK timer
  ackTimer = 0;

  //Flush the buffers
  resetSerial();
  changeState(RADIO_RESET);
}

//Point-to-point link is now up, following a 3-way handshake
void enterLinkUp()
{
  VIRTUAL_CIRCUIT * vc;

  //Bring up the link
  triggerEvent(TRIGGER_HANDSHAKE_COMPLETE);
  hopChannel(); //Leave home

  //Synchronize the ACK numbers
  vc = &virtualCircuitList[0];
  vc->rmtTxAckNumber = 0;
  vc->rxAckNumber = 0;
  vc->txAckNumber = 0;

  //Discard any previous data
  discardPreviousData();

  //Stop the ACK timer
  ackTimer = 0;

  //Mark start time for uptime calculation
  lastLinkUpTime = millis();

  //Start the receiver
  changeState(RADIO_P2P_LINK_UP);
  if (settings.printLinkUpDown)
  {
    systemPrintln("========== Link UP ==========");
    outputSerialData(true);
  }
}

//Empty the remote command receive buffer
void discardPreviousData()
{
  //Output any debug messages
  outputSerialData(true);

  //Discard any previous data
  radioTxTail = radioTxHead;
  txTail = txHead;
  commandRXTail = commandRXHead;
  commandTXTail = commandTXHead;
}

//Output VC link status
void vcChangeState(int8_t vcIndex, uint8_t state)
{
  VIRTUAL_CIRCUIT * vc;
  uint32_t vcBit;

  vc = &virtualCircuitList[vcIndex];
  if (state != vc->vcState)
  {
    //Set the new state
    vc->vcState = state;

    //Display the state change
    if (settings.printLinkUpDown)
    {
      if (state == VC_STATE_CONNECTED)
      {
        systemPrint("======= VC ");
        systemPrint(vcIndex);
        systemPrintln(" CONNECTED ======");
      }
      if (state == VC_STATE_LINK_DOWN)
      {
        systemPrint("--------- VC ");
        systemPrint(vcIndex);
        systemPrintln(" Down ---------");
      }
      else if (state == VC_STATE_LINK_ALIVE)
      {
        systemPrint("-=--=--=- VC ");
        systemPrint(vcIndex);
        systemPrintln(" ALIVE =--=--=-");
      }
      else if (state == VC_STATE_NEW_CLIENT)
      {
        systemPrint("-=--=--=- VC ");
        systemPrint(vcIndex);
        systemPrintln(" NEW CLIENT =--=--=-");
      }
      outputSerialData(true);
    }

    //Determine if the VC is connecting
    vcBit = 1 << vcIndex;
    if (state == VC_STATE_NEW_CLIENT)
      vcConnecting |= vcBit;
    else
      vcConnecting &= ~vcBit;
  }
  vcSendPcStateMessage(vcIndex, state);
}

//Send the PC the state message
void vcSendPcStateMessage(int8_t vcIndex, uint8_t state)
{
  //Build the VC state message
  VC_STATE_MESSAGE message;
  message.vcState = state;

  //Build the message header
  VC_SERIAL_MESSAGE_HEADER header;
  header.start = START_OF_VC_SERIAL;
  header.radio.length = VC_RADIO_HEADER_BYTES + sizeof(message);
  header.radio.destVc = PC_LINK_STATUS;
  header.radio.srcVc = vcIndex;

  //Send the VC state message
  systemWrite((uint8_t *)&header, sizeof(header));
  systemWrite((uint8_t *)&message, sizeof(message));
}

//Break the virtual-circuit link
void vcBreakLink(int8_t vcIndex)
{
  VIRTUAL_CIRCUIT * vc;

  //Only handle real VCs
  if ((vcIndex >= PC_COMMAND) && (vcIndex <= VC_BROADCAST))
    return;
  vcIndex &= VCAB_NUMBER_MASK;

  //If waiting for an ACK and the link breaks, stop the retransmissions
  //by stopping the ACK timer.
  if (ackTimer && (txDestVc == vcIndex))
  {
    ackTimer = 0;
    vcSendPcAckNack(vcIndex, false);
  }

  //Get the virtual circuit data structure
  if ((vcIndex >= 0) && (vcIndex != myVc) && (vcIndex < MAX_VC))
  {
    //Account for the link failure
    vc = &virtualCircuitList[vcIndex];
    vc->linkFailures++;
    vcChangeState(vcIndex, VC_STATE_LINK_DOWN);
  }
  linkFailures++;

  //Flush the buffers
  outputSerialData(true);
  if (vcIndex == myVc)
    resetSerial();
}

//Place VC in LINK-UP state since it is receiving HEARTBEATs from the remote radio
int8_t vcLinkAlive(int8_t vcIndex)
{
  VIRTUAL_CIRCUIT * vc = &virtualCircuitList[vcIndex];

  //Send the status message
  if (vc->vcState == VC_STATE_LINK_DOWN)
  {
    vc->firstHeartbeatMillis = millis();

    //Update the link status
    vcChangeState(vcIndex, VC_STATE_LINK_ALIVE);
  }
  return vcIndex;
}

void vcZeroAcks(int8_t vcIndex)
{
  VIRTUAL_CIRCUIT * vc = &virtualCircuitList[vcIndex];

  //Reset the ACK counters
  vc->txAckNumber = 0;
  vc->rmtTxAckNumber = 0;
  vc->rxAckNumber = 0;
}

//Translate the UNIQUE ID value into a VC number to reduce the communications overhead
int8_t vcIdToAddressByte(int8_t srcAddr, uint8_t * id)
{
  VIRTUAL_CIRCUIT * vc;
  int8_t vcIndex;

  //Determine if the address is already in the list
  for (vcIndex = 0; vcIndex < MAX_VC; vcIndex++)
  {
    //Verify that an address is present
    vc = &virtualCircuitList[vcIndex];
    if (!vc->valid)
      continue;

    //Compare the unique ID values
    if (memcmp(vc->uniqueId, id, UNIQUE_ID_BYTES) == 0)
      //Update the link status
      return vcLinkAlive(vcIndex);
  }

  //The unique ID is not in the list
  //Fill in clients that were already running
  vcIndex = srcAddr;
  vc = &virtualCircuitList[vcIndex];

  //Only the server can assign the address bytes
  if ((srcAddr == VC_UNASSIGNED) && (!settings.server))
    return -1;

  //Assign an address if necessary
  if (srcAddr == VC_UNASSIGNED)
  {
    //Unknown client ID
    //Determine if there is a free address
    for (vcIndex = 0; vcIndex < MAX_VC; vcIndex++)
    {
      vc = &virtualCircuitList[vcIndex];
      if (!virtualCircuitList[vcIndex].valid)
        break;
    }
    if (vcIndex >= MAX_VC)
    {
      systemPrintln("ERROR: Too many clients, no free addresses!\n");
      outputSerialData(true);
      return -2;
    }
    srcAddr = vcIndex;
  }

  //Check for an address conflict
  if (vc->valid)
  {
    systemPrint("ERROR: Unknown ID with pre-assigned conflicting address: ");
    systemPrintln(srcAddr);
    systemPrint("Received ID: ");
    for (int i = 0; i < UNIQUE_ID_BYTES; i++)
      systemPrint(id[i], HEX);
    systemPrintln();
    systemPrint("Assigned ID: ");
    for (int i = 0; i < UNIQUE_ID_BYTES; i++)
      systemPrint(vc->uniqueId[i], HEX);
    systemPrintln();
    outputSerialData(true);
    return -3;
  }

  //Save the unique ID to VC assignment in NVM
  memcpy(&vc->uniqueId, id, UNIQUE_ID_BYTES);
  if (settings.server)
    nvmSaveVcUniqueId(vcIndex);

  //Mark this link as up
  vc->valid = true;
  return vcLinkAlive(vcIndex);
}

//Process a received HEARTBEAT frame from a VC
void vcReceiveHeartbeat(uint32_t rxMillis)
{
  uint32_t deltaMillis;
  int vcSrc;

  if (rxSrcVc == VC_SERVER)
    syncChannelTimer(); //Adjust freq hop ISR based on server's remaining clock

  //Update the timestamp offset
  if (rxSrcVc == VC_SERVER)
  {
    //Assume client and server are running the same level of debugging,
    //then the delay from reading the millisecond value on the server should
    //get offset by the transmit setup time and the receive overhead time.
    memcpy(&timestampOffset, &rxVcData[UNIQUE_ID_BYTES], sizeof(timestampOffset));
    timestampOffset += vcTxHeartbeatMillis + rxMillis - millis();
  }

  //Save our address
  if ((myVc == VC_UNASSIGNED) && (memcmp(myUniqueId, rxVcData, UNIQUE_ID_BYTES) == 0))
    myVc = rxSrcVc;

  //Translate the unique ID into an address byte
  vcSrc = vcIdToAddressByte(rxSrcVc, rxVcData);
  if (vcSrc < 0)
    return;

  //When the client does not know its address, it is assigned by the server
  if (settings.server && (rxSrcVc == VC_UNASSIGNED))
  {
    //Assign the address to the client
    if (xmitVcHeartbeat(vcSrc, rxVcData))
      changeState(RADIO_VC_WAIT_TX_DONE);
  }
}
