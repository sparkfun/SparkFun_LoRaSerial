#define SAVE_TX_BUFFER()                                  \
  {                                                       \
    petWDT();                                             \
    memcpy(rexmtBuffer, outgoingPacket, MAX_PACKET_SIZE); \
    rexmtControl = txControl;                             \
    rexmtLength = txDatagramSize;                         \
    rexmtFrameSentCount = frameSentCount;                 \
  }

#define RESTORE_TX_BUFFER()                               \
  {                                                       \
    petWDT();                                             \
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
      changeState(RADIO_P2P_LINK_UP_WAIT_TX_DONE);                              \
    }                                                                           \
  }

#define START_ACK_TIMER()                                                       \
  {                                                                             \
    /*Start the ACK timer*/                                                     \
    ackTimer = datagramTimer;                                                   \
                                                                                \
    /*Since ackTimer is off when equal to zero, force it to a non-zero value*/  \
    /*Subtract one so that the comparisons result in a small number*/           \
    if (!ackTimer)                                                              \
      ackTimer -= 1;                                                            \
  }

#define STOP_ACK_TIMER()      \
  {                           \
    /*Stop the ACK timer*/    \
    ackTimer = 0;             \
  }

#define COMPUTE_TIMESTAMP_OFFSET(millisBuffer, rShift)                          \
  {                                                                             \
    unsigned long deltaUsec = txTimeUsec + rxTimeUsec;                          \
    memcpy(&remoteSystemMillis, millisBuffer, sizeof(currentMillis));           \
    timestampOffset = (remoteSystemMillis + (deltaUsec / 1000) - currentMillis);\
    timestampOffset >>= rShift;                                                  \
  }

#define COMPUTE_RX_TIME(millisBuffer, rShift)                                   \
  {                                                                             \
    currentMillis = millis();                                                   \
    if (!rxFirstAck)                                                            \
    {                                                                           \
      rxTimeUsec = micros() - transactionCompleteMicros;                        \
      txRxTimeMsec = (txTimeUsec + rxTimeUsec) / 1000;                          \
                                                                                \
      /*Display the results*/                                                   \
      if (settings.debugSync)                                                   \
      {                                                                         \
        systemPrintTimestamp(radioCallHistory[RADIO_CALL_transactionCompleteISR] + timestampOffset);  \
        systemPrint(" RX Time: ");                                              \
        systemPrint(rxTimeUsec);                                                \
        systemPrintln(" uSec");                                                 \
        systemPrint(" TX + RX Time: ");                                         \
        systemPrint(txRxTimeMsec);                                              \
        systemPrintln(" mSec");                                                 \
      }                                                                         \
    }                                                                           \
                                                                                \
    /*Adjust the timestamp offset*/                                             \
    COMPUTE_TIMESTAMP_OFFSET(millisBuffer, rShift);                             \
  }

#define COMPUTE_TX_TIME()                                                       \
  {                                                                             \
    currentMillis = millis();                                                   \
    if (!txFirstAck)                                                            \
    {                                                                           \
      txTimeUsec = transactionCompleteMicros - txDatagramMicros;                \
                                                                                \
      /*Display the results*/                                                   \
      if (settings.debugSync)                                                   \
      {                                                                         \
        systemPrintTimestamp(currentMillis + timestampOffset);                  \
        systemPrint(" TX Time: ");                                              \
        systemPrint(txTimeUsec);                                                \
        systemPrintln(" uSec");                                                 \
      }                                                                         \
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

      stopChannelTimer(); //Stop frequency hopping - reset
      clockSyncReceiver = true; //Assume receiving clocks - reset

      configureRadio(); //Setup radio, set freq to channel 0, calculate air times

      //Determine the maximum frame air time
      maxFrameAirTime = calcAirTimeMsec(MAX_PACKET_SIZE);

      //Start the TX timer: time to delay before transmitting the FIND_PARTNER
      setHeartbeatShort(); //Both radios start with short heartbeat period
      randomTime = random(ackAirTime, ackAirTime * 2); //Fast FIND_PARTNER

      sf6ExpectedSize = headerBytes + CLOCK_MILLIS_BYTES + trailerBytes; //Tell SF6 to receive FIND_PARTNER packet

      petWDT();

      returnToReceiving(); //Start receiving

      //Stop the ACK timer
      STOP_ACK_TIMER();

      //Start the link between the radios
      if (settings.operatingMode == MODE_POINT_TO_POINT)
        changeState(RADIO_P2P_LINK_DOWN);
      else if (settings.operatingMode == MODE_VIRTUAL_CIRCUIT)
      {
        if (settings.server)
        {
          //Reserve the server's address (0)
          myVc = vcIdToAddressByte(VC_SERVER, myUniqueId);
          clockSyncReceiver = false; //VC server is clock source
        }
        else
          //Unknown client address
          myVc = VC_UNASSIGNED;

        //Start sending heartbeats
        xmitVcHeartbeat(myVc, myUniqueId);
        changeState(RADIO_VC_WAIT_TX_DONE);
      }
      else
      {
        if (settings.server == true)
        {
          clockSyncReceiver = false; //Multipoint server is clock source
          startChannelTimer(); //Start hopping - multipoint clock source
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
    //directions. This handshake is also used to synchronize the HOP timer and zero the ACKs.
    /*
                    System A                 System B

                     RESET                     RESET <-------------------------.
                       |                         |                             |
             Channel 0 |                         | Channel 0                   |
        Stop HOP Timer |                         | Stop HOP Timer              |
     clockSyncReceiver | = true                  | clockSyncReceiver = true    |
                       |                         |                             |
                       V                         V                             |
           .----> P2P_NO_LINK               P2P_NO_LINK <------------------.   |
           |           | Tx FIND_PARTNER         |                         |   |
           | Timeout   |                         |          Stop HOP Timer |   |
           |           V                         |       clockSyncReceiver |   |
           | P2P_WAIT_TX_FIND_PARTNER_DONE       |                  = true |   |
           |           |                         |                         |   |
           |           | Tx Complete - - - - - > | Rx FIND_PARTNER         |   |
           |           |   Start Rx              |   Start HOP Timer       |   |
           |           |   MAX_PACKET_SIZE       |   clockSyncReceiver     |   |
           |           V                         |     = false             |   |
           `---- P2P_WAIT_SYNC_CLOCKS            |                         |   |
                       |                         | Tx SYNC_CLOCKS          |   |
                       |                         V                         |   |
                       |              P2P_WAIT_TX_SYNC_CLOCKS_DONE         |   |
                       |                         |                         |   |
        Rx SYNC_CLOCKS | < - - - - - - - - - - - | Tx Complete             |   |
                       |                         |   Start Rx              |   |
                       | Start HOP Timer         |   MAX_PACKET_SIZE       |   |
                       | Sync HOP Timer          |                         |   |
                       |                         V         Timeout         |   |
                       |                P2P_WAIT_ZERO_ACKS ----------------'   |
                       |                         |                             |
                       | TX ZERO_ACKS            |                             |
                       |                         |                             |
                       V                         |                             |
           P2P_WAIT_TX_ZERO_ACKS_DONE            |                             |
                       | Tx Complete - - - - - > | Rx ZERO_ACKS                |
                       |   Start Rx              |   Start Rx                  |
                       |   MAX_PACKET_SIZE       |   MAX_PACKET_SIZE           |
                       |   Zero ACKs             |   Zero ACKs                 |
                       |                         |                             |
                       V                         V         Rx FIND_PARTNER     |
                  P2P_LINK_UP               P2P_LINK_UP -----------------------’
                       |                         |
                       | Rx Data                 | Rx Data
                       |                         |
                       V                         V

      Two timers are in use:
        datagramTimer:  Set at end of transmit, measures ACK timeout
        heartbeatTimer: Set upon entry to P2P_NO_LINK, measures time to send next FIND_PARTNER
    */
    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

    case RADIO_P2P_LINK_DOWN:
      //If we are on the wrong channel, go home
      if (channelNumber != 0)
      {
        channelNumber = 0;
        setRadioFrequency(false);
      }

      //Update the transmit and receive times until the first ACK TX and RX
      rxFirstAck = false;
      txFirstAck = false;

      //Determine if a FIND_PARTNER was received
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

          case DATAGRAM_FIND_PARTNER:
            //Received FIND_PARTNER
            //Compute the receive time
            COMPUTE_RX_TIME(rxData, 1);

            //Start the channel timer
            startChannelTimer(); //Start hopping - P2P clock source

            //This system is the source of clock synchronization
            clockSyncReceiver = false; //P2P clock source

            //Display the channelTimer source
            if (settings.debugSync)
            {
              systemPrint("Sourcing channelTimer, TX + RX mSec: ");
              systemPrintln(txRxTimeMsec);
            }

            //Acknowledge the FIND_PARTNER with SYNC_CLOCKS
            triggerEvent(TRIGGER_SEND_SYNC_CLOCKS);
            if (xmitDatagramP2PSyncClocks() == true)
            {
              sf6ExpectedSize = headerBytes + P2P_ZERO_ACKS_BYTES + trailerBytes; //Tell SF6 we expect ZERO_ACKS to contain millis info
              changeState(RADIO_P2P_WAIT_TX_SYNC_CLOCKS_DONE);
            }
            break;
        }
      }

      //Is it time to send the FIND_PARTNER to the remote system
      else if ((receiveInProcess() == false) && ((millis() - heartbeatTimer) >= randomTime))
      {
        //Transmit the FIND_PARTNER
        triggerEvent(TRIGGER_HANDSHAKE_SEND_FIND_PARTNER);
        if (xmitDatagramP2PFindPartner() == true)
        {
          sf6ExpectedSize = headerBytes + CLOCK_MILLIS_BYTES + trailerBytes; //Tell SF6 we expect SYNC_CLOCKS to contain millis info
          changeState(RADIO_P2P_WAIT_TX_FIND_PARTNER_DONE);
        }
      }
      break;

    case RADIO_P2P_WAIT_TX_FIND_PARTNER_DONE:
      //Determine if a FIND_PARTNER has completed transmission
      if (transactionComplete)
      {
        COMPUTE_TX_TIME();
        triggerEvent(TRIGGER_HANDSHAKE_SEND_FIND_PARTNER_COMPLETE);
        transactionComplete = false; //Reset ISR flag
        returnToReceiving();
        changeState(RADIO_P2P_WAIT_SYNC_CLOCKS);
      }
      break;

    case RADIO_P2P_WAIT_SYNC_CLOCKS:
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

          case DATAGRAM_FIND_PARTNER:
            //Received FIND_PARTNER
            //Compute the receive time
            COMPUTE_RX_TIME(rxData + 1, 1);

            //Acknowledge the FIND_PARTNER
            triggerEvent(TRIGGER_SEND_SYNC_CLOCKS);
            if (xmitDatagramP2PSyncClocks() == true)
            {
              sf6ExpectedSize = headerBytes + P2P_ZERO_ACKS_BYTES + trailerBytes; //Tell SF6 we expect ZERO_ACKS to contain millis info
              changeState(RADIO_P2P_WAIT_TX_SYNC_CLOCKS_DONE);
            }
            break;

          case DATAGRAM_SYNC_CLOCKS:
            //Received SYNC_CLOCKS
            //Start the channel timer
            startChannelTimer(); //Start hopping - P2P clock receiver
            syncChannelTimer();

            //Display the channelTimer sink
            if (settings.debugSync)
            {
              systemPrint("Syncing channelTimer, TX + RX mSec: ");
              systemPrintln(txRxTimeMsec);
            }

            //Compute the receive time
            COMPUTE_RX_TIME(rxData + 1, 1);

            //Acknowledge the SYNC_CLOCKS
            triggerEvent(TRIGGER_SEND_ZERO_ACKS);
            if (xmitDatagramP2PZeroAcks() == true)
            {
              sf6ExpectedSize = MAX_PACKET_SIZE; //Tell SF6 to return to max packet length
              changeState(RADIO_P2P_WAIT_TX_ZERO_ACKS_DONE);
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
            systemPrintln("RX: SYNC_CLOCKS Timeout");
            outputSerialData(true);
          }

          //Start the TX timer: time to delay before transmitting the FIND_PARTNER
          triggerEvent(TRIGGER_HANDSHAKE_SYNC_CLOCKS_TIMEOUT);
          setHeartbeatShort();

          //Slow down FIND_PARTNERs
          if (ackAirTime < settings.maxDwellTime)
            randomTime = random(settings.maxDwellTime * 2, settings.maxDwellTime * 4);
          else
            randomTime = random(ackAirTime * 4, ackAirTime * 8);

          sf6ExpectedSize = headerBytes + CLOCK_MILLIS_BYTES + trailerBytes; //Tell SF6 to receive FIND_PARTNER packet
          returnToReceiving();

          changeState(RADIO_P2P_LINK_DOWN);
        }
      }
      break;

    case RADIO_P2P_WAIT_TX_SYNC_CLOCKS_DONE:
      //Determine if a SYNC_CLOCKS has completed transmission
      if (transactionComplete)
      {
        COMPUTE_TX_TIME();
        triggerEvent(TRIGGER_HANDSHAKE_SEND_SYNC_CLOCKS_COMPLETE);
        transactionComplete = false; //Reset ISR flag
        returnToReceiving();
        changeState(RADIO_P2P_WAIT_ZERO_ACKS);
      }
      break;

    case RADIO_P2P_WAIT_ZERO_ACKS:
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

          case DATAGRAM_ZERO_ACKS:
            //Received ACK 2
            //Compute the receive time
            COMPUTE_RX_TIME(rxData, 1);

            setHeartbeatLong(); //We sent SYNC_CLOCKS and they sent ZERO_ACKS, so don't be the first to send heartbeat

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
            systemPrintln("RX: ZERO_ACKS Timeout");
            outputSerialData(true);
          }

          //Stop the channel timer
          stopChannelTimer(); //P2P_WAIT_ZERO_ACKS timeout
          clockSyncReceiver = true; //P2P link timeout

          //Start the TX timer: time to delay before transmitting the FIND_PARTNER
          triggerEvent(TRIGGER_HANDSHAKE_ZERO_ACKS_TIMEOUT);
          setHeartbeatShort();

          //Slow down FIND_PARTNERs
          if (ackAirTime < settings.maxDwellTime)
            randomTime = random(settings.maxDwellTime * 2, settings.maxDwellTime * 4);
          else
            randomTime = random(ackAirTime * 4, ackAirTime * 8);

          sf6ExpectedSize = headerBytes + CLOCK_MILLIS_BYTES + trailerBytes; //Tell SF6 to receive FIND_PARTNER packet
          returnToReceiving();

          changeState(RADIO_P2P_LINK_DOWN);
        }
      }
      break;

    case RADIO_P2P_WAIT_TX_ZERO_ACKS_DONE:
      //Determine if a ACK 2 has completed transmission
      if (transactionComplete)
      {
        transactionComplete = false; //Reset ISR flag
        COMPUTE_TX_TIME();

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

                  FIND_PARTNER ----> Update timestampOffset

        Update timestampOffset <---- SYNC_CLOCKS

                         ACK 2 ----> Update timestampOffset

                   HEARTBEAT 0 ----> Update timestampOffset

        Update timestampOffset <---- HEARTBEAT 0

                   HEARTBEAT 1 --X

        Update timestampOffset <---- HEARTBEAT 1

                   HEARTBEAT 1 ----> Update timestampOffset
    */

    //====================
    //Wait for the data transmission to complete
    //====================
    case RADIO_P2P_LINK_UP_WAIT_TX_DONE:
      if (timeToHop == true) //If the channelTimer has expired, move to next frequency
        hopChannel();

      if (transactionComplete)
      {
        transactionComplete = false; //Reset ISR flag

        //Determine if an ACK was transmitted
        if (txControl.datagramType = DATAGRAM_DATA_ACK)
        {
          COMPUTE_TX_TIME();
          txFirstAck = true;
        }

        //Determine the next packet size for SF6
        if (ackTimer)
        {
          //Waiting for an ACK
          triggerEvent(TRIGGER_LINK_WAIT_FOR_ACK);
          sf6ExpectedSize = headerBytes + CHANNEL_TIMER_BYTES + trailerBytes;
        }
        else
          sf6ExpectedSize = MAX_PACKET_SIZE;

        //Receive the next packet
        returnToReceiving();
        changeState(RADIO_P2P_LINK_UP);
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

          case DATAGRAM_SYNC_CLOCKS:
          case DATAGRAM_ZERO_ACKS:
          case DATAGRAM_BAD:
            triggerEvent(TRIGGER_BAD_PACKET);
            break;

          case DATAGRAM_CRC_ERROR:
            triggerEvent(TRIGGER_CRC_ERROR);
            break;

          case DATAGRAM_NETID_MISMATCH:
            triggerEvent(TRIGGER_NETID_MISMATCH);
            break;

          case DATAGRAM_FIND_PARTNER:
            breakLink();
            break;

          case DATAGRAM_HEARTBEAT:
            //Received heartbeat while link was idle. Send ack to sync clocks.
            //Adjust the timestamp offset
            COMPUTE_TIMESTAMP_OFFSET(rxData, 1);

            //Transmit ACK
            P2P_SEND_ACK(TRIGGER_LINK_SEND_ACK_FOR_HEARTBEAT);
            break;

          case DATAGRAM_DATA:
            //Place the data in the serial output buffer
            serialBufferOutput(rxData, rxDataBytes);

            //Transmit ACK
            P2P_SEND_ACK(TRIGGER_LINK_SEND_ACK_FOR_DATA);
            break;

          case DATAGRAM_DUPLICATE:
            P2P_SEND_ACK(TRIGGER_LINK_SEND_ACK_FOR_DUP);
            break;

          case DATAGRAM_DATA_ACK:
            //Adjust the timestamp offset
            COMPUTE_RX_TIME(rxData, 1);

            //Stop updating the rxTimeUsec and txTimeUsec after TX and RX of ACKs
            if (txFirstAck)
              rxFirstAck = true;

            //The datagram we are expecting
            syncChannelTimer(); //Adjust freq hop ISR based on remote's remaining clock

            triggerEvent(TRIGGER_LINK_ACK_RECEIVED);

            //Stop the ACK timer
            STOP_ACK_TIMER();

            setHeartbeatLong(); //Those who send an ACK have short time to next heartbeat. Those who send a heartbeat or data have long time to next heartbeat.
            frequencyCorrection += radio.getFrequencyError() / 1000000.0;
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

      else if (receiveInProcess() == false)
      {
        //----------
        //Priority 1: Transmit a HEARTBEAT if necessary
        //----------
        heartbeatTimeout = ((millis() - heartbeatTimer) > heartbeatRandomTime);
        if (heartbeatTimeout)
        {
          triggerEvent(TRIGGER_HEARTBEAT);

          if (xmitDatagramP2PHeartbeat() == true)
            changeState(RADIO_P2P_LINK_UP_WAIT_TX_DONE);

          //Save the message for retransmission
          SAVE_TX_BUFFER();
          setHeartbeatLong(); //We're sending a heartbeat, so don't be the first to send next heartbeat

          START_ACK_TIMER();
        }

        //----------
        //Priority 2: Wait for an outstanding ACK until it is received, don't
        //transmit any other data
        //----------

        //An ACK is expected when the ACK timer running
        else if (ackTimer)
        {
          //Check for ACK timeout
          if ((millis() - ackTimer) >= (frameAirTime + ackAirTime + settings.overheadTime + getReceiveCompletionOffset()))
          {
            //Determine if another retransmission is allowed
            if ((!settings.maxResends) || (frameSentCount < settings.maxResends))
            {
              //Throttle back retransmits based on the number of retransmits we've attempted
              //retransmitTimeout is a random number, set when the first datagram is sent
              if (millis() - datagramTimer > (frameSentCount * retransmitTimeout))
              {
                lostFrames++;

                //Display the retransmit
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

                //Attempt the retransmission
                RESTORE_TX_BUFFER();
                if (rexmtControl.datagramType == DATAGRAM_HEARTBEAT)
                {
                  //Never retransmit the heartbeat, always send a new version to
                  //send the updated time value
                  if (xmitDatagramP2PHeartbeat() == true)
                    changeState(RADIO_P2P_LINK_UP_WAIT_TX_DONE);
                }
                else if (retransmitDatagram(NULL) == true)
                  changeState(RADIO_P2P_LINK_UP_WAIT_TX_DONE);

                //We're re-sending data, so don't be the first to send next heartbeat
                setHeartbeatLong();
                START_ACK_TIMER();
              }
            }
            else
            {
              lostFrames++;

              //Failed to reach the other system, break the link
              triggerEvent(TRIGGER_LINK_RETRANSMIT_FAIL);

              //Break the link
              breakLink();
            }
          }
        }

        //----------
        //Priority 3: Send the entire command or response, toggle between waiting
        //for ACK above and transmitting the command or response
        //----------

        else if (availableTXCommandBytes()) //If we have command bytes to send out
        {
          //Load command bytes into outgoing packet
          readyOutgoingCommandPacket(0);

          triggerEvent(TRIGGER_LINK_DATA_XMIT);
          if (remoteCommandResponse)
          {
            //Send the command response
            if (xmitDatagramP2PCommandResponse() == true)
              changeState(RADIO_P2P_LINK_UP_WAIT_TX_DONE);
          }
          else
          {
            //Send the command
            if (xmitDatagramP2PCommand() == true)
              changeState(RADIO_P2P_LINK_UP_WAIT_TX_DONE);
          }

          //Save the message for retransmission
          SAVE_TX_BUFFER();
          setHeartbeatLong(); //We're sending command, so don't be the first to send next heartbeat

          START_ACK_TIMER();
        }

        //----------
        //Lowest Priority: Check for data to send
        //----------

        //If we have data, try to send it out
        else if (availableRadioTXBytes())
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
                changeState(RADIO_P2P_LINK_UP_WAIT_TX_DONE);

              //Save the message for retransmission
              SAVE_TX_BUFFER();
              setHeartbeatLong(); //We're sending data, so don't be the first to send next heartbeat

              START_ACK_TIMER();
            }
          }
        }
      }

      //----------
      //Always check for link timeout
      //----------
      if ((millis() - linkDownTimer) >= (P2P_LINK_BREAK_MULTIPLIER * settings.heartbeatTimeout))
        //Break the link
        breakLink();
      break;

    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    //Multi-Point Data Exchange
    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

    /*
                         Server                                  Client

                          Reset                                   Reset
                            |                                       |
                            |                                       |
                            V                                       V
    .--------------> RADIO_MP_STANDBY                      RADIO_DISCOVER_BEGIN
    |                       |                                       |
    |  No TX                |                                       |
    |  Do ???      RX other V                                       |
    +<--------+<------------+                                       V
    ^         |             |                 .--------> RADIO_DISCOVER_SCANNING
    |         | TX Resp     |                 | TX done             |
    |         |             |                 |                     |
    |         |             |     WAIT_TX_FIND_PARTNER_DONE         |
    |     .---'             |                 ^                     |
    |     |              RX |                 |                     |
    |     |    FIND_PARTNER |  < - - - - - -  | TX FIND_PARTNER     |
    |     |                 |                 |                     |
    |     |                 |                 |       Delay         |
    |     |                 |                 |       10 Sec        |
    |     |                 |                 +<----------.         |
    |     |                 |                 ^           |         |
    |     |                 |                 |  Yes      | No      |
    |     |                 |                 + <---- Loops < 10    |
    |     |                 |                 ^           ^         |
    |     |                 |              No |      Yes  |         |
    |     |                 |             Channel 0 ------'         |
    |     |                 |                 ^                     |
    |     |                 |                 | Hop reverse         |
    |     |                 |                 | RX timeout          V
    |     |                 |                 '---------------------+
    |     |                 |                                       |
    |     |                 |                                       |
    |     |              TX |                                       |
    |     |     SYNC_CLOCKS |  - - - - - - - - - - - - - - - - - >  | RX SYNC_CLOCKS
    |     |                 |                                       |
    |     |                 |                                       | Sync clocks
    |     |                 v                                       | Update channel #
    |     '-----> RADIO_MP_WAIT_TX_DONE                             |
    |                       |                                       |
    |                       v                                       |
    `-----------------------+<--------------------------------------'

    */

    //====================
    //Start searching for other radios
    //====================
    case RADIO_DISCOVER_BEGIN:
      stopChannelTimer(); //Stop hopping - multipoint discovery

      if (settings.debugSync)
      {
        systemPrintln("Start scanning");
        outputSerialData(true);
      }

      multipointChannelLoops = 0;
      multipointAttempts = 0;

      triggerEvent(TRIGGER_MP_SCAN);
      changeState(RADIO_DISCOVER_SCANNING);
      break;

    //====================
    //Walk through channel table backwards, transmitting a FIND_PARTNER and looking for an SYNC_CLOCKS
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

          case DATAGRAM_ZERO_ACKS:
          case DATAGRAM_DATA:
          case DATAGRAM_DATA_ACK:
          case DATAGRAM_FIND_PARTNER: //Clients do not respond to FIND_PARTNER, only the server
          case DATAGRAM_REMOTE_COMMAND:
          case DATAGRAM_REMOTE_COMMAND_RESPONSE:
            //We should not be receiving these datagrams, but if we do, just ignore
            triggerEvent(TRIGGER_BAD_PACKET);
            break;

          case DATAGRAM_SYNC_CLOCKS:
            triggerEvent(TRIGGER_LINK_ACK_RECEIVED);

            //Compute the common clock
            currentMillis = millis();
            COMPUTE_TIMESTAMP_OFFSET(rxData + 1, 0);

            //Server has responded with ACK
            syncChannelTimer(); //Start and adjust freq hop ISR based on remote's remaining clock

            //Change to the server's channel number
            channelNumber = rxVcData[0];

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
            systemPrintln("MP: SYNC_CLOCKS Timeout");
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

          //Send FIND_PARTNER
          if (xmitDatagramP2PFindPartner() == true)
            changeState(RADIO_DISCOVER_WAIT_TX_FIND_PARTNER_DONE);
        }
      }

      break;

    //====================
    //Wait for the FIND_PARTNER to complete transmission
    //====================
    case RADIO_DISCOVER_WAIT_TX_FIND_PARTNER_DONE:
      if (transactionComplete)
      {
        transactionComplete = false; //Reset ISR flag
        returnToReceiving();
        changeState(RADIO_DISCOVER_SCANNING);
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

          case DATAGRAM_SYNC_CLOCKS:
          case DATAGRAM_ZERO_ACKS:
          case DATAGRAM_DATAGRAM:
          case DATAGRAM_DATA_ACK:
          case DATAGRAM_REMOTE_COMMAND:
          case DATAGRAM_REMOTE_COMMAND_RESPONSE:
            //We should not be receiving these datagrams, but if we do, just ignore
            frequencyCorrection += radio.getFrequencyError() / 1000000.0;
            triggerEvent(TRIGGER_BAD_PACKET);
            break;

          case DATAGRAM_FIND_PARTNER:
            //A new radio is saying hello
            if (settings.server == true)
            {
              //Ack their FIND_PARTNER with SYNC_CLOCK
              if (xmitDatagramP2PSyncClocks() == true)
              {
                triggerEvent(TRIGGER_MP_SEND_ACK_FOR_FIND_PARTNER);
                changeState(RADIO_MP_WAIT_TX_DONE);
              }
            }
            else
            {
              changeState(RADIO_MP_STANDBY);
            }
            break;

          case DATAGRAM_HEARTBEAT:
            //Received heartbeat - do not ack.

            //Sync clock if server sent the heartbeat
            if (settings.server == false)
              syncChannelTimer(); //Adjust freq hop ISR based on remote's remaining clock

            frequencyCorrection += radio.getFrequencyError() / 1000000.0;

            lastPacketReceived = millis(); //Update timestamp for Link LED

            ledMpHeartbeatOn();
            changeState(RADIO_MP_STANDBY);
            break;

          case DATAGRAM_DATA:
            //Received data - do not ack.

            //Sync clock if server sent the datagram
            if (settings.server == false)
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

        //Only the server transmits heartbeats
        if (settings.server == true)
        {
          if (heartbeatTimeout)
          {
            triggerEvent(TRIGGER_HEARTBEAT);
            if (xmitDatagramMpHeartbeat() == true)
            {
              setHeartbeatMultipoint(); //We're sending something with clock data so reset heartbeat timer
              changeState(RADIO_MP_WAIT_TX_DONE); //Wait for heartbeat to transmit
            }
            ledMpHeartbeatOn();
          }
        }

        //Check for time to send serial data
        else if (availableRadioTXBytes() && (processWaitingSerial(heartbeatTimeout) == true))
        {
          triggerEvent(TRIGGER_MP_DATA_PACKET);
          if (xmitDatagramMpData() == true)
          {
            setHeartbeatMultipoint(); //We're sending something with clock data so reset heartbeat timer
            changeState(RADIO_MP_WAIT_TX_DONE);
          }
        }

        //If the client hasn't received a packet in too long, return to scanning
        else if (settings.server == false)
        {
          if ((millis() - lastPacketReceived) > (settings.heartbeatTimeout * 3))
          {
            if (settings.debugSync)
            {
              systemPrintln("HEARTBEAT Timeout");
              outputSerialData(true);
            }
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
                | Send FIND_PARTNER               |
                |                                 |
                V                                 |
        RADIO_TRAIN_WAIT_TX_FIND_PARTNER_DONE     |
                |                                 |
                V                                 | Timeout
        RADIO_TRAIN_WAIT_RX_RADIO_PARAMETERS -----'
                |
                | Save settings
                | Send ACK
                |
                V
        RADIO_TRAIN_WAIT_TX_ACK_DONE
                |
                V
      endTrainingClientServer
                |
                | Reboot
                |
                V
    */

    //====================
    //Wait for the FIND_PARTNER to complete transmission
    //====================
    case RADIO_TRAIN_WAIT_TX_FIND_PARTNER_DONE:

      //If dio0ISR has fired, we are done transmitting
      if (transactionComplete == true)
      {
        transactionComplete = false;

        //Indicate that the receive is complete
        triggerEvent(TRIGGER_TRAINING_CLIENT_TX_FIND_PARTNER_DONE);

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
      else if ((millis() - datagramTimer) > (settings.clientFindPartnerRetryInterval * 1000))
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
          xmitDatagramTrainingFindPartner(); //Continue retrying as client
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
        .------ RADIO_TRAIN_WAIT_FOR_FIND_PARTNER         |
        |               |                                 |
        |               | Send RADIO_PARAMS               |
        |               |                                 |
        |               V                                 |
        |      RADIO_TRAIN_WAIT_TX_RADIO_PARAMS_DONE -----'
        |
        |
        `---------------.
                        | ATZ command
                        |
                        | Reboot
                        |
                        V
    */

    //====================
    //Wait for a FIND_PARTNER frame from a client
    //====================
    case RADIO_TRAIN_WAIT_FOR_FIND_PARTNER:

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

          case DATAGRAM_TRAINING_FIND_PARTNER:
            //Save the client ID
            memcpy(trainingPartnerID, rxData, UNIQUE_ID_BYTES);

            //Find a slot in the NVM unique ID table
            for (index = 0; index < MAX_VC; index++)
              if (!nvmIsVcUniqueIdSet(index))
              {
                //Save the client unique ID
                nvmSaveUniqueId(index, trainingPartnerID);
                break;
              }

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
        changeState(RADIO_TRAIN_WAIT_FOR_FIND_PARTNER);
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

      The ACK synchronization handshake occurs between VCs that want to
      communicate.  Normally this occurs between the client and the server, but
      may also occur between two clients that want to communicate with each
      other.  One of the systems initiates the three way handshake by sending
      UNKNOWN_ACKS.  The second system responses with SYNC_ACKSZERO_ACKS causing the
      first system to respond with ZERO_ACKS.

      3-way Handshake to zero ACKs:

          TX UNKNOWN_ACKS --> RX SYNC_ACKS --> TX ZERO_ACKS
             or
          RX UNKNOWN_ACKS --> TX SYNC_ACKS --> RX ZERO_ACKS

      VC States:

      .-------------> VC_STATE_LINK_DOWN <-------------------------------------.
      |                       |                                     HB Timeout |
      |                       | HEARTBEAT received                             |
      | HB Timeout            v                                                |
      +<----------- VC_STATE_LINK_ALIVE ------------------------.              |
      ^                       |               RX UNKNOWN_ACKS   |              |
      |                       | ATC command                     |              |
      | HB Timeout            V                                 |              |
      +<--------- VC_STATE_SEND_UNKNOWN_ACKS <-.                |              |
      ^                       |                |                |              |
      |       TX UNKNOWN_ACKS |   ZERO_ACKS RX |                |              |
      |           TX Complete |        Timeout |        RX      |              |
      | HB Timeout            V                |   UNKNOWN_ACKS V              |
      +<---------- VC_STATE_WAIT_SYNC_ACKS --->+----------->+-->+              |
      ^                       |                ^            ^   | TX           |
      |                       | RX SYNC_ACKS   |  ZERO_ACKS |   | SYNC_ACKS    |
      |                       | TX ZERO_ACKS   |         RX |   | TX Complete  |
      |                       | TX Complete    |        TMO |   V              |
      |       Zero ACK values |                |     VC_STATE_WAIT_ZERO_ACKS --'
      |                       |      .---------'                |
      |                       |      | RX UNKNOWN_ACKS          | RX ZERO_ACKS
      | HB Timeout            V      |                          | Zero ACK values
      '-------------- VC_STATE_CONNECTED <----------------------'

    */

    //====================
    //Wait for a HEARTBEAT from the server
    //====================
    case RADIO_VC_WAIT_SERVER:
      if (myVc == VC_SERVER)
      {
        changeState(RADIO_VC_WAIT_RECEIVE);
        break;
      }

      //If dio0ISR has fired, a packet has arrived
      currentMillis = millis();
      if (transactionComplete == true)
      {
        //Decode the received datagram
        PacketType packetType = rcvDatagram();

        //Process the received datagram
        if ((packetType == DATAGRAM_VC_HEARTBEAT) && (rxSrcVc == VC_SERVER))
        {
          vcReceiveHeartbeat(millis() - currentMillis);
          changeState(RADIO_VC_WAIT_RECEIVE);
        }
        else
          //Ignore this datagram
          triggerEvent(TRIGGER_BAD_PACKET);
      }
      break;

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
            STOP_ACK_TIMER();
            break;

          case DATAGRAM_DUPLICATE:
            frequencyCorrection += radio.getFrequencyError() / 1000000.0;

            triggerEvent(TRIGGER_LINK_SEND_ACK_FOR_DUP);
            if (xmitVcAckFrame(rxSrcVc))
              changeState(RADIO_VC_WAIT_TX_DONE);
            break;

          //Second step in the 3-way handshake, received UNKNOWN_ACKS, respond
          //with SYNC_ACKS
          case DATAGRAM_VC_UNKNOWN_ACKS:
            if (xmitVcSyncAcks(rxSrcVc))
              changeState(RADIO_VC_WAIT_TX_DONE);
            vcChangeState(rxSrcVc, VC_STATE_WAIT_ZERO_ACKS);
            virtualCircuitList[rxSrcVc].timerMillis = datagramTimer;
            break;

          //Third step in the 3-way handshake, received SYNC_ACKS, respond with ZERO_ACKS
          case DATAGRAM_VC_SYNC_ACKS:
            if (xmitVcZeroAcks(rxSrcVc))
            {
              changeState(RADIO_VC_WAIT_TX_DONE);
              vcZeroAcks(rxSrcVc);
              vcChangeState(rxSrcVc, VC_STATE_CONNECTED);
            }
            break;

          //Last step in the 3-way handshake, received ZERO_ACKS, done
          case DATAGRAM_VC_ZERO_ACKS:
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
            tempSettings = settings;
            checkCommand(); //Parse the command buffer
            settings = tempSettings;
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

      //when not receiving process the pending transmit requests
      else if (!receiveInProcess())
      {
        //----------
        //Priority 1: Transmit a HEARTBEAT if necessary
        //----------
        if (((currentMillis - heartbeatTimer) >= heartbeatRandomTime))
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
            STOP_ACK_TIMER();
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

              START_ACK_TIMER();
              lostFrames++;
            }
            else
            {
              //HEARTBEATs have not timed out, request to reestablish the connection
              vc->flags.wasConnected = true;

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

          START_ACK_TIMER();

          //Save the message for retransmission
          SAVE_TX_BUFFER();
          rexmtTxDestVc = txDestVc;
        }

        //----------
        //Priority 4: Walk through the 3-way handshake
        //----------
        else if (vcConnecting)
        {
          for (index = 0; index < MAX_VC; index++)
          {
            if (receiveInProcess())
              break;

            //Determine the first VC that is walking through connections
            if (vcConnecting & (1 << index))
            {
              //Determine if UNKNOWN_ACKS needs to be sent
              if (virtualCircuitList[index].vcState <= VC_STATE_SEND_UNKNOWN_ACKS)
              {
                //Send the UNKNOWN_ACKS datagram, first part of the 3-way handshake
                if (xmitVcUnknownAcks(index))
                {
                  vcChangeState(index, VC_STATE_WAIT_SYNC_ACKS);
                  virtualCircuitList[index].timerMillis = datagramTimer;
                  changeState(RADIO_VC_WAIT_TX_DONE);
                }
              }

              //The SYNC_ACKS is handled with the receive code
              //Check for a timeout waiting for the SYNC_ACKS
              else if (virtualCircuitList[index].vcState == VC_STATE_WAIT_SYNC_ACKS)
              {
                if ((currentMillis - virtualCircuitList[index].timerMillis)
                    >= (frameAirTime + ackAirTime + settings.overheadTime + getReceiveCompletionOffset()))
                {
                  //Retransmit the UNKNOWN_ACKS
                  if (xmitVcUnknownAcks(index))
                  {
                    virtualCircuitList[index].timerMillis = datagramTimer;
                    changeState(RADIO_VC_WAIT_TX_DONE);
                  }
                }
              }

              //The ZERO_ACKS is handled with the receive code
              //Check for a timeout waiting for the ZERO_ACKS
              else if (virtualCircuitList[index].vcState == VC_STATE_WAIT_ZERO_ACKS)
              {
                if ((currentMillis - virtualCircuitList[index].timerMillis)
                    >= (frameAirTime + ackAirTime + settings.overheadTime + getReceiveCompletionOffset()))
                {
                  //Retransmit the SYNC_CLOCKS
                  if (xmitVcSyncAcks(index))
                  {
                    virtualCircuitList[index].timerMillis = datagramTimer;
                    changeState(RADIO_VC_WAIT_TX_DONE);
                  }
                }
              }

              //Work on only one connection at a time
              break;
            }
          }
        }

        //----------
        //Lowest Priority: Check for data to send
        //----------
        else if (vcSerialMessageReceived())
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

              START_ACK_TIMER();

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
                tempSettings = settings;
                checkCommand(); //Parse the command buffer
                settings = tempSettings;
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

              START_ACK_TIMER();

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
            changeState(RADIO_VC_WAIT_SERVER);
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
      && (radioState <= RADIO_P2P_LINK_UP_WAIT_TX_DONE))
    return (true);

  return (false);
}

//Determine if multi-point sync is achieved
bool isMultiPointSync()
{
  return ((radioState >= RADIO_MP_STANDBY) && (radioState <= RADIO_MP_WAIT_TX_DONE));
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
      systemPrintTimestamp(radioStateHistory[sortOrder[index]] + timestampOffset);
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
  STOP_ACK_TIMER();

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
  STOP_ACK_TIMER();

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
    //Display the state change
    if (settings.printLinkUpDown)
    {
      if ((state == VC_STATE_WAIT_ZERO_ACKS) && (vc->vcState == VC_STATE_CONNECTED))
      {
        systemPrint("-=-=- VC ");
        systemPrint(vcIndex);
        systemPrintln(" DISCONNECTED -=-=-");
      }
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
      outputSerialData(true);
    }

    //Set the new state
    vc->vcState = state;

    //Determine if the VC is connecting
    vcBit = 1 << vcIndex;
    if (((state > VC_STATE_LINK_ALIVE) && (state < VC_STATE_CONNECTED))
      || (vc->flags.wasConnected && (vc->vcState == VC_STATE_LINK_ALIVE)))
      vcConnecting |= vcBit;
    else
      vcConnecting &= ~vcBit;

    //Clear the connection request when connected
    if (state == VC_STATE_CONNECTED)
      vc->flags.wasConnected = false;
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
    STOP_ACK_TIMER();
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
    if (!vc->flags.valid)
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
      if (!virtualCircuitList[vcIndex].flags.valid)
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
  if (vc->flags.valid)
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
  nvmSaveUniqueId(vcIndex, id);

  //Mark this link as up
  vc->flags.valid = true;
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
