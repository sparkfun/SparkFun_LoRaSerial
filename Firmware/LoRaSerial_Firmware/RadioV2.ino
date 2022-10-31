const uint16_t crc16Table[256] =
{
  0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
  0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
  0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
  0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
  0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
  0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
  0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
  0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
  0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
  0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
  0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
  0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
  0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
  0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
  0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
  0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
  0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
  0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
  0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
  0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
  0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
  0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
  0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
  0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
  0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
  0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
  0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
  0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
  0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
  0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
  0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
  0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//Point-To-Point Training
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Ping the other radio in the point-to-point configuration
void xmitDatagramP2PTrainingPing()
{
  /*
            endOfTxData ---.
                           |
                           V
      +---------+----------+
      |         |          |
      | Control | Trailer  |
      | 8 bits  | n Bytes  |
      +---------+----------+
  */

  txControl.datagramType = DATAGRAM_P2P_TRAINING_PING;
  transmitDatagram();
}

//Build the parameters packet used for training
void xmitDatagramP2pTrainingParams()
{
  Settings params;

  //Initialize the radio parameters
  memcpy(&params, &originalSettings, sizeof(settings));
  params.operatingMode = MODE_POINT_TO_POINT;

  //Add the radio parameters
  memcpy(endOfTxData, &params, sizeof(params));
  endOfTxData += sizeof(params);

  /*
                          endOfTxData ---.
                                         |
                                         V
      +----------+---------+---  ...  ---+----------+
      | Optional |         |   Radio     | Optional |
      |  NET ID  | Control | Parameters  | Trailer  |
      |  8 bits  | 8 bits  |   n bytes   | n Bytes  |
      +----------+---------+-------------+----------+
  */

  txControl.datagramType = DATAGRAM_P2P_TRAINING_PARAMS;
  transmitDatagram();
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//Point-To-Point: Bring up the link
//
//A three way handshake is used to get both systems to agree that data can flow in both
//directions.  This handshake is also used to synchronize the HOP timer.
/*
                    System A                 System B

                     RESET                     RESET
                       |                         |
             Channel 0 |                         | Channel 0
                       V                         V
           .----> P2P_NO_LINK               P2P_NO_LINK
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

//First packet in the three way handshake to bring up the link
void xmitDatagramP2PPing()
{
  unsigned long currentMillis = millis();
  memcpy(endOfTxData, &currentMillis, sizeof(currentMillis));
  endOfTxData += sizeof(unsigned long);

  /*
                    endOfTxData ---.
                                   |
                                   V
      +--------+---------+---------+----------+
      |        |         |         |          |
      | NET ID | Control | Millis  | Trailer  |
      | 8 bits | 8 bits  | 4 bytes | n Bytes  |
      +--------+---------+---------+----------+
  */

  txControl.datagramType = DATAGRAM_PING;
  transmitDatagram();
}

//Second packet in the three way handshake to bring up the link
void xmitDatagramP2PAck1()
{
  unsigned long currentMillis = millis();
  memcpy(endOfTxData, &currentMillis, sizeof(currentMillis));
  endOfTxData += sizeof(unsigned long);

  /*
                    endOfTxData ---.
                                   |
                                   V
      +--------+---------+---------+----------+
      |        |         |         | Optional |
      | NET ID | Control | Millis  | Trailer  |
      | 8 bits | 8 bits  | 4 bytes | n Bytes  |
      +--------+---------+---------+----------+
  */

  txControl.datagramType = DATAGRAM_ACK_1;
  transmitDatagram();
}

//Last packet in the three way handshake to bring up the link
void xmitDatagramP2PAck2()
{
  unsigned long currentMillis = millis();
  memcpy(endOfTxData, &currentMillis, sizeof(currentMillis));
  endOfTxData += sizeof(unsigned long);

  /*
                    endOfTxData ---.
                                   |
                                   V
      +--------+---------+---------+----------+
      |        |         |         |          |
      | NET ID | Control | Millis  | Trailer  |
      | 8 bits | 8 bits  | 4 bytes | n Bytes  |
      +--------+---------+---------+----------+
  */

  txControl.datagramType = DATAGRAM_ACK_2;
  transmitDatagram();
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Point-to-Point Data Exchange
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Send a command datagram to the remote system
void xmitDatagramP2PCommand()
{
  /*
                       endOfTxData ---.
                                      |
                                      V
      +--------+---------+---  ...  ---+----------+
      |        |         |             | Optional |
      | NET ID | Control |    Data     | Trailer  |
      | 8 bits | 8 bits  |   n bytes   | n Bytes  |
      +--------+---------+-------------+----------+
  */

  txControl.datagramType = DATAGRAM_REMOTE_COMMAND;
  transmitDatagram();
}

//Send a command response datagram to the remote system
void xmitDatagramP2PCommandResponse()
{
  /*
                       endOfTxData ---.
                                      |
                                      V
      +--------+---------+---  ...  ---+----------+
      |        |         |             | Optional |
      | NET ID | Control |    Data     | Trailer  |
      | 8 bits | 8 bits  |   n bytes   | n Bytes  |
      +--------+---------+-------------+----------+
  */

  txControl.datagramType = DATAGRAM_REMOTE_COMMAND_RESPONSE;
  transmitDatagram();
}

//Send a data datagram to the remote system
void xmitDatagramP2PData()
{
  /*
                       endOfTxData ---.
                                      |
                                      V
      +--------+---------+---  ...  ---+----------+
      |        |         |             | Optional |
      | NET ID | Control |    Data     | Trailer  |
      | 8 bits | 8 bits  |   n bytes   | n Bytes  |
      +--------+---------+-------------+----------+
  */

  txControl.datagramType = DATAGRAM_DATA;
  transmitDatagram();
}

//Heartbeat packet to keep the link up
void xmitDatagramP2PHeartbeat()
{
  unsigned long currentMillis = millis();
  memcpy(endOfTxData, &currentMillis, sizeof(currentMillis));
  endOfTxData += sizeof(currentMillis);

  /*
                    endOfTxData ---.
                                   |
                                   V
      +--------+---------+---------+----------+
      |        |         |         | Optional |
      | NET ID | Control | Millis  | Trailer  |
      | 8 bits | 8 bits  | 4 Bytes | n Bytes  |
      +--------+---------+---------+----------+
  */

  txControl.datagramType = DATAGRAM_HEARTBEAT;
  transmitDatagram();
}

//Create short packet of 2 control bytes - do not expect ack
void xmitDatagramP2PAck()
{
  int ackLength;

  uint8_t * ackStart = endOfTxData;
  uint16_t channelTimerElapsed = millis() - timerStart;
  memcpy(endOfTxData, &channelTimerElapsed, sizeof(channelTimerElapsed));
  endOfTxData += sizeof(channelTimerElapsed);

  //Verify the ACK length
  ackLength = endOfTxData - ackStart;
  if (ackLength != ACK_BYTES)
  {
    systemPrint("ERROR - Please define ACK_BYTES = ");
    systemPrintln(ackLength);
    while (1)
      petWDT();
  }

  /*
                     endOfTxData ---.
                                    |
                                    V
      +--------+---------+----------+----------+
      |        |         | Channel  | Optional |
      | NET ID | Control |  Timer   | Trailer  |
      | 8 bits | 8 bits  | 2 bytes  | n Bytes  |
      +--------+---------+----------+----------+
  */

  txControl.datagramType = DATAGRAM_DATA_ACK;
  transmitDatagram();
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Multi-Point Data Exchange
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Send a data datagram to the remote system
void xmitDatagramMpDatagram()
{
  /*
                          endOfTxData ---.
                                         |
                                         V
      +----------+---------+---  ...  ---+----------+
      | Optional |         |             | Optional |
      |  NET ID  | Control |    Data     | Trailer  |
      |  8 bits  | 8 bits  |   n bytes   | n Bytes  |
      +----------+---------+-------------+----------+
  */

  txControl.datagramType = DATAGRAM_DATAGRAM;
  transmitDatagram();
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//Multi-Point Client Training
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Build the client ping packet used for training
void xmitDatagramMpTrainingPing()
{
  //Add the source (server) ID
  memcpy(endOfTxData, myUniqueId, UNIQUE_ID_BYTES);
  endOfTxData += UNIQUE_ID_BYTES;

  /*
                        endOfTxData ---.
                                       |
                                       V
      +----------+---------+-----------+----------+
      | Optional |         |           | Optional |
      |  NET ID  | Control | Client ID | Trailer  |
      |  8 bits  | 8 bits  |  16 Bytes | n Bytes  |
      +----------+---------+-----------+----------+
  */

  txControl.datagramType = DATAGRAM_TRAINING_PING;
  transmitDatagram();
}

//Build the client ACK packet used for training
void xmitDatagramMpTrainingAck(uint8_t * serverID)
{
  //Add the destination (server) ID
  memcpy(endOfTxData, serverID, UNIQUE_ID_BYTES);
  endOfTxData += UNIQUE_ID_BYTES;

  //Add the source (client) ID
  memcpy(endOfTxData, myUniqueId, UNIQUE_ID_BYTES);
  endOfTxData += UNIQUE_ID_BYTES;

  /*
                                    endOfTxData ---.
                                                   |
                                                   V
      +----------+---------+-----------+-----------+----------+
      | Optional |         |           |           | Optional |
      |  NET ID  | Control | Server ID | Client ID | Trailer  |
      |  8 bits  | 8 bits  |  16 Bytes |  16 Bytes | n Bytes  |
      +----------+---------+-----------+-----------+----------+
  */

  txControl.datagramType = DATAGRAM_TRAINING_ACK;
  transmitDatagram();
}

void updateRadioParameters(uint8_t * rxData)
{
  Settings params;

  //Get the parameters
  memcpy(&params, rxData, sizeof(params));

  //Update the radio parameters
  originalSettings.airSpeed = params.airSpeed;
  originalSettings.netID = params.netID;
  originalSettings.operatingMode = params.operatingMode;
  originalSettings.encryptData = params.encryptData;
  memcpy(originalSettings.encryptionKey, params.encryptionKey, sizeof(originalSettings.encryptionKey));
  originalSettings.dataScrambling = params.dataScrambling;
  originalSettings.radioBroadcastPower_dbm = params.radioBroadcastPower_dbm;
  originalSettings.frequencyMin = params.frequencyMin;
  originalSettings.frequencyMax = params.frequencyMax;
  originalSettings.numberOfChannels = params.numberOfChannels;
  originalSettings.frequencyHop = params.frequencyHop;
  originalSettings.maxDwellTime = params.maxDwellTime;
  originalSettings.radioBandwidth = params.radioBandwidth;
  originalSettings.radioSpreadFactor = params.radioSpreadFactor;
  originalSettings.radioCodingRate = params.radioCodingRate;
  originalSettings.radioSyncWord = params.radioSyncWord;
  originalSettings.radioPreambleLength = params.radioPreambleLength;
  originalSettings.serialTimeoutBeforeSendingFrame_ms = params.serialTimeoutBeforeSendingFrame_ms;
  originalSettings.heartbeatTimeout = params.heartbeatTimeout;
  originalSettings.autoTuneFrequency = params.autoTuneFrequency;
  originalSettings.maxResends = params.maxResends;
  originalSettings.verifyRxNetID = params.verifyRxNetID;
  originalSettings.radioProtocolVersion = params.radioProtocolVersion;
  originalSettings.overheadTime = params.overheadTime;
  originalSettings.enableCRC16 = params.enableCRC16;
  originalSettings.clientPingRetryInterval = params.clientPingRetryInterval;

  //Update the API parameters
  originalSettings.sortParametersByName = params.sortParametersByName;
  originalSettings.printParameterName = params.printParameterName;

  //Update the debug parameters
  if (params.copyDebug)
  {
    originalSettings.debug = params.debug;
    originalSettings.displayPacketQuality = params.displayPacketQuality;
    originalSettings.printFrequency = params.printFrequency;
    originalSettings.debugRadio = params.debugRadio;
    originalSettings.debugStates = params.debugStates;
    originalSettings.debugTraining = params.debugTraining;
    originalSettings.debugTrigger = params.debugTrigger;
    originalSettings.printRfData = params.printRfData;
    originalSettings.printPktData = params.printPktData;
    originalSettings.debugReceive = params.debugReceive;
    originalSettings.debugTransmit = params.debugTransmit;
    originalSettings.printTxErrors = params.printTxErrors;
    originalSettings.printTimestamp = params.printTimestamp;
    originalSettings.debugDatagrams = params.debugDatagrams;
    originalSettings.displayRealMillis = params.displayRealMillis;
  }

  //Update the serial parameters
  if (params.copySerial)
  {
    originalSettings.serialSpeed = params.serialSpeed;
    originalSettings.echo = params.echo;
    originalSettings.flowControl = params.flowControl;
    originalSettings.usbSerialWait = params.usbSerialWait;
    originalSettings.printLinkUpDown = params.printLinkUpDown;
  }

  //Update the trigger parameters
  if (params.copyTriggers)
  {
    originalSettings.triggerWidth = params.triggerWidth;
    originalSettings.triggerWidthIsMultiplier = params.triggerWidthIsMultiplier;
    originalSettings.triggerEnable = params.triggerEnable;
    originalSettings.triggerEnable2 = params.triggerEnable2;
  }
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//Multi-Point Server Training
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Build the server parameters packet used for training
void xmitDatagramMpRadioParameters(const uint8_t * clientID)
{
  Settings params;

  //Initialize the radio parameters
  memcpy(&params, &originalSettings, sizeof(settings));
  params.operatingMode = MODE_DATAGRAM;
  params.trainingServer = false;

  //Add the destination (client) ID
  memcpy(endOfTxData, clientID, UNIQUE_ID_BYTES);
  endOfTxData += UNIQUE_ID_BYTES;

  //Add the source (server) ID
  memcpy(endOfTxData, myUniqueId, UNIQUE_ID_BYTES);
  endOfTxData += UNIQUE_ID_BYTES;

  //Add the radio parameters
  memcpy(endOfTxData, &params, sizeof(params));
  endOfTxData += sizeof(params);

  /*
                                                  endOfTxData ---.
                                                                 |
                                                                 V
      +----------+---------+-----------+-----------+---  ...  ---+----------+
      | Optional |         |           |           |   Radio     | Optional |
      |  NET ID  | Control | Client ID | Server ID | Parameters  | Trailer  |
      |  8 bits  | 8 bits  |  16 Bytes |  16 Bytes |   n bytes   | n Bytes  |
      +----------+---------+-----------+-----------+-------------+----------+
  */

  txControl.datagramType = DATAGRAM_TRAINING_PARAMS;
  transmitDatagram();
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//Virtual Circuit frames
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void xmitVcHeartbeat(int8_t addr, uint8_t * id)
{
  uint8_t * txData;

  uint32_t currentMillis = millis();
  txData = endOfTxData;
  *endOfTxData++ = 0; //Reserve for length
  *endOfTxData++ = VC_BROADCAST;
  *endOfTxData++ = addr;
  memcpy(endOfTxData, id, UNIQUE_ID_BYTES);
  endOfTxData += UNIQUE_ID_BYTES;
  memcpy(endOfTxData, &currentMillis, sizeof(currentMillis));
  endOfTxData += sizeof(currentMillis);

  //Set the length field
  *txData = (uint8_t)(endOfTxData - txData);

  /*
                                                               endOfTxData ---.
                                                                              |
                                                                              V
      +----------+---------+--------+----------+---------+----------+---------+----------+
      | Optional |         |        |          |         |          |         | Optional |
      |  NET ID  | Control | Length | DestAddr | SrcAddr |  Src ID  | millis  | Trailer  |
      |  8 bits  | 8 bits  | 8 bits |  8 bits  | 8 bits  | 16 Bytes | 4 Bytes || n Bytes  |
      +----------+---------+--------+----------+---------+----------+---------+----------+
  */

  txControl.datagramType = DATAGRAM_VC_HEARTBEAT;
  txControl.ackNumber = 0;
  transmitDatagram();

  //Determine the time that it took to pass this frame to the radio
  //This time is used to adjust the time offset
  vcTxHeartbeatMillis = millis() - currentMillis;

  //Select a random for the next heartbeat
  setHeartbeatLong(); //Those who send a heartbeat or data have long time before next heartbeat. Those who send ACKs, have short wait to next heartbeat.
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//Datagram reception
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Determine the type of datagram received
PacketType rcvDatagram()
{
  uint8_t ackNumber;
  PacketType datagramType;
  uint8_t receivedNetID;
  CONTROL_U8 rxControl;
  VIRTUAL_CIRCUIT * vc;

  //Save the receive time
  rcvTimeMillis = millis();

  //Get the received datagram
  framesReceived++;
  radio.readData(incomingBuffer, MAX_PACKET_SIZE);
  rxDataBytes = radio.getPacketLength();
  rxData = incomingBuffer;

  /*
      |<---------------------- rxDataBytes ---------------------->|
      |                                                           |
      +----------+----------+------------+---  ...  ---+----------+
      | Optional |          |  Optional  |             | Optional |
      |  NET ID  | Control  | SF6 Length |    Data     | Trailer  |
      |  8 bits  |  8 bits  |   8 bits   |   n bytes   | n Bytes  |
      +----------+----------+------------+-------------+----------+
      ^
      |
      '---- rxData
  */

  //Display the received data bytes
  if (settings.printRfData || settings.debugReceive)
  {
    systemPrintln("----------");
    systemPrintTimestamp();
    systemPrint("RX: ");
    systemPrint((settings.dataScrambling || settings.encryptData) ? "Encrypted " : "Unencrypted ");
    systemPrint("Frame ");
    systemPrint(rxDataBytes);
    systemPrint(" (0x");
    systemPrint(rxDataBytes, HEX);
    systemPrintln(") bytes");
    if (timeToHop == true) //If the channelTimer has expired, move to next frequency
      hopChannel();
    petWDT();
    if (settings.printRfData && rxDataBytes)
      dumpBuffer(incomingBuffer, rxDataBytes);
  }

  if (settings.dataScrambling == true)
    radioComputeWhitening(incomingBuffer, rxDataBytes);

  if (settings.encryptData == true)
  {
    decryptBuffer(incomingBuffer, rxDataBytes);
    if (timeToHop == true) //If the channelTimer has expired, move to next frequency
      hopChannel();
  }

  //Display the received data bytes
  if ((settings.dataScrambling || settings.encryptData)
      && (settings.printRfData || settings.debugReceive))
  {
    systemPrintTimestamp();
    systemPrint("RX: Unencrypted Frame ");
    systemPrint(rxDataBytes);
    systemPrint(" (0x");
    systemPrint(rxDataBytes, HEX);
    systemPrintln(") bytes");
    if (timeToHop == true) //If the channelTimer has expired, move to next frequency
      hopChannel();
    petWDT();
    if (settings.printRfData && rxDataBytes)
      dumpBuffer(incomingBuffer, rxDataBytes);
  }

  //All packets must include the 2-byte control header
  if (rxDataBytes < minDatagramSize)
  {
    //Display the packet contents
    if (settings.printPktData || settings.debugReceive)
    {
      systemPrintTimestamp();
      systemPrint("RX: Bad Frame ");
      systemPrint(rxDataBytes);
      systemPrint(" (0x");
      systemPrint(rxDataBytes, HEX);
      systemPrintln(") bytes");
      if (timeToHop == true) //If the channelTimer has expired, move to next frequency
        hopChannel();
      petWDT();
      if (settings.printRfData && rxDataBytes)
        dumpBuffer(incomingBuffer, rxDataBytes);
    }
    badFrames++;
    return (DATAGRAM_BAD);
  }

  /*
      |<---------------------- rxDataBytes ---------------------->|
      |                                                           |
      +----------+----------+------------+---  ...  ---+----------+
      | Optional |          |  Optional  |             | Optional |
      |  NET ID  | Control  | SF6 Length |    Data     | Trailer  |
      |  8 bits  |  8 bits  |   8 bits   |   n bytes   | n Bytes  |
      +----------+----------+------------+-------------+----------+
      ^
      |
      '---- rxData
  */

  //Verify the netID if necessary
  if ((settings.operatingMode == MODE_POINT_TO_POINT) || settings.verifyRxNetID)
  {
    receivedNetID = *rxData++;
    if (settings.debugReceive)
    {
      systemPrintTimestamp();
      systemPrint("RX: NetID ");
      systemPrint(receivedNetID);
      systemPrint(" (0x");
      systemPrint(receivedNetID, HEX);
      systemPrint(")");
      if (timeToHop == true) //If the channelTimer has expired, move to next frequency
        hopChannel();
      if (receivedNetID != settings.netID)
      {
        systemPrint(" expecting ");
        systemPrintln(settings.netID);
        petWDT();
        if (settings.printPktData && rxDataBytes)
          dumpBuffer(incomingBuffer, rxDataBytes);
        return (DATAGRAM_NETID_MISMATCH);
      }
      systemPrintln();
    }
  }

  //Process the trailer
  petWDT();
  if (settings.enableCRC16)
  {
    uint16_t crc;
    uint8_t * data;

    //Compute the CRC-16 value
    crc = 0xffff;
    if (timeToHop == true) //If the channelTimer has expired, move to next frequency
      hopChannel();
    for (data = incomingBuffer; data < &incomingBuffer[rxDataBytes - 2]; data++)
      crc = crc16Table[*data ^ (uint8_t)(crc >> (16 - 8))] ^ (crc << 8);
    if (timeToHop == true) //If the channelTimer has expired, move to next frequency
      hopChannel();
    if ((incomingBuffer[rxDataBytes - 2] != (crc >> 8))
        && (incomingBuffer[rxDataBytes - 1] != (crc & 0xff)))
    {
      //Display the packet contents
      if (settings.printPktData || settings.debugReceive)
      {
        systemPrintTimestamp();
        systemPrint("RX: Bad CRC-16, received 0x");
        systemPrint(incomingBuffer[rxDataBytes - 2], HEX);
        systemPrint(incomingBuffer[rxDataBytes - 1], HEX);
        systemPrint(" expected 0x");
        systemPrintln(crc, HEX);
        if (timeToHop == true) //If the channelTimer has expired, move to next frequency
          hopChannel();
        petWDT();
        if (settings.printRfData && rxDataBytes)
          dumpBuffer(incomingBuffer, rxDataBytes);
      }
      badFrames++;
      return (DATAGRAM_BAD);
    }
  }

  /*
      |<---------------------- rxDataBytes ---------------------->|
      |                                                           |
      +----------+----------+------------+---  ...  ---+----------+
      | Optional |          |  Optional  |             | Optional |
      |  NET ID  | Control  | SF6 Length |    Data     | Trailer  |
      |  8 bits  |  8 bits  |   8 bits   |   n bytes   | n Bytes  |
      +----------+----------+------------+-------------+----------+
                 ^
                 |
                 '---- rxData
  */

  //Get the control byte
  rxControl = *((CONTROL_U8 *)rxData++);
  datagramType = rxControl.datagramType;
  ackNumber = rxControl.ackNumber;
  if (settings.debugReceive)
    printControl(*((uint8_t *)&rxControl));
  if (datagramType >= MAX_V2_DATAGRAM_TYPE)
  {
    badFrames++;
    return (DATAGRAM_BAD);
  }

  //Display the CRC
  if (settings.enableCRC16 && settings.debugReceive)
  {
    systemPrintTimestamp();
    systemPrint("    CRC-16: 0x");
    systemPrint(incomingBuffer[rxDataBytes - 2], HEX);
    systemPrintln(incomingBuffer[rxDataBytes - 1], HEX);
    if (timeToHop == true) //If the channelTimer has expired, move to next frequency
      hopChannel();
  }

  /*
      |<---------------------- rxDataBytes ---------------------->|
      |                                                           |
      +----------+----------+------------+---  ...  ---+----------+
      | Optional |          |  Optional  |             | Optional |
      |  NET ID  | Control  | SF6 Length |    Data     | Trailer  |
      |  8 bits  |  8 bits  |   8 bits   |   n bytes   | n Bytes  |
      +----------+----------+------------+-------------+----------+
                            ^
                            |
                            '---- rxData
  */

  //Get the spread factor 6 length
  if (settings.radioSpreadFactor == 6)
  {
    if (rxDataBytes >= (*rxData + minDatagramSize))
      rxDataBytes = *rxData++;
    else
    {
      if (settings.debugReceive)
      {
        systemPrintTimestamp();
        systemPrint("Invalid SF6 length, received SF6 length");
        systemPrint(*rxData);
        systemPrint(" > ");
        systemPrint((int)rxDataBytes - minDatagramSize);
        systemPrintln(" received bytes");
        if (timeToHop == true) //If the channelTimer has expired, move to next frequency
          hopChannel();
      }
      badFrames++;
      return (DATAGRAM_BAD);
    }
  }
  rxDataBytes -= minDatagramSize;

  //Verify the packet number last so that the expected datagram or ACK number can be updated
  rxVcData = rxData;
  if (settings.operatingMode == MODE_POINT_TO_POINT)
  {
    switch (datagramType)
    {
      default:
        break;

      case DATAGRAM_DATA_ACK:
        if (ackNumber != txAckNumber)
        {
          if (settings.debugReceive)
          {
            systemPrintTimestamp();
            systemPrint("Invalid ACK number, received ");
            systemPrint(ackNumber);
            systemPrint(" expecting ");
            systemPrintln(txAckNumber);
            if (timeToHop == true) //If the channelTimer has expired, move to next frequency
              hopChannel();
          }
          badFrames++;
          return (DATAGRAM_BAD);
        }

        //Set the next TX ACK number
        txAckNumber = (txAckNumber + 1) & 3;
        break;

      case DATAGRAM_REMOTE_COMMAND:
      case DATAGRAM_REMOTE_COMMAND_RESPONSE:
      case DATAGRAM_HEARTBEAT:
        if (ackNumber != rmtTxAckNumber)
        {
          //Determine if this is a duplicate datagram
          if (ackNumber == ((rmtTxAckNumber - 1) & 3))
          {
            linkDownTimer = millis();
            duplicateFrames++;
            return DATAGRAM_DUPLICATE;
          }

          //Not a duplicate
          if (settings.debugReceive)
          {
            systemPrintTimestamp();
            systemPrint("Invalid datagram number, received ");
            systemPrint(ackNumber);
            systemPrint(" expecting ");
            systemPrintln(rmtTxAckNumber);
            if (timeToHop == true) //If the channelTimer has expired, move to next frequency
              hopChannel();
          }
          badFrames++;
          return DATAGRAM_BAD;
        }

        //Receive this data packet and set the next expected datagram number
        rxAckNumber = rmtTxAckNumber;
        rmtTxAckNumber = (rmtTxAckNumber + 1) & 3;
        break;

      case DATAGRAM_DATA:
        if (ackNumber != rmtTxAckNumber)
        {
          //Determine if this is a duplicate datagram
          if (ackNumber == ((rmtTxAckNumber - 1) & 3))
          {
            linkDownTimer = millis();
            duplicateFrames++;
            return DATAGRAM_DUPLICATE;
          }

          //Not a duplicate
          if (settings.debugReceive)
          {
            systemPrintTimestamp();
            systemPrint("Invalid datagram number, received ");
            systemPrint(ackNumber);
            systemPrint(" expecting ");
            systemPrintln(rmtTxAckNumber);
            if (timeToHop == true) //If the channelTimer has expired, move to next frequency
              hopChannel();
          }
          badFrames++;
          return DATAGRAM_BAD;
        }

        //Verify that there is sufficient space in the serialTransmitBuffer
        if ((sizeof(serialTransmitBuffer) - availableTXBytes()) < rxDataBytes)
        {
          if (settings.debugReceive)
          {
            systemPrintTimestamp();
            systemPrintln("Insufficient space in the serialTransmitBuffer");
          }
          insufficientSpace++;

          //Apply back pressure to the other radio by dropping this packet and
          //forcing the other radio to retransmit the packet.
          return DATAGRAM_BAD;
        }

        //Receive this data packet and set the next expected datagram number
        rxAckNumber = rmtTxAckNumber;
        rmtTxAckNumber = (rmtTxAckNumber + 1) & 3;
        break;
    }
  }

  //Verify the Virtual-Circuit length
  else if (settings.operatingMode == MODE_VIRTUAL_CIRCUIT)
  {
    //Verify that the virtual circuit header is present
    if (rxDataBytes < 3)
    {
      if (settings.debugReceive)
      {
        systemPrintTimestamp();
        systemPrint("Missing VC header bytes, received only ");
        systemPrint(rxDataBytes);
        systemPrintln(" bytes, expecting at least 3 bytes");
        if (timeToHop == true) //If the channelTimer has expired, move to next frequency
          hopChannel();
      }
      badFrames++;
      return DATAGRAM_BAD;
    }

    //Parse the virtual circuit header
    rxDestVc = rxData[1];
    rxSrcVc = rxData[2];
    rxVcData = &rxData[3];

    //Validate the source VC
    vc = NULL;
    if (rxSrcVc != VC_UNASSIGNED)
    {
      if ((uint8_t)rxSrcVc >= MAX_VC)
      {
        if (settings.debugReceive)
        {
          systemPrintTimestamp();
          systemPrint("Invalid source VC: ");
          systemPrintln(rxSrcVc);
          if (timeToHop == true) //If the channelTimer has expired, move to next frequency
            hopChannel();
        }
        badFrames++;
        return DATAGRAM_BAD;
      }
      vc = &virtualCircuitList[rxSrcVc];
    }

    //Validate the length
    if (*rxData != rxDataBytes)
    {
      systemPrintTimestamp();
      systemPrint("Invalid VC length, received ");
      systemPrint(*rxData);
      systemPrint(" expecting ");
      systemPrintln(rxDataBytes);
      if (timeToHop == true) //If the channelTimer has expired, move to next frequency
        hopChannel();
      badFrames++;
      if (vc)
        vc->badLength++;
      return DATAGRAM_BAD;
    }

    //Account for this frame
    if (vc)
    {
      vc->framesReceived++;
      if (datagramType == DATAGRAM_DATA)
        vc->messagesReceived++;
    }

    //Display the virtual circuit header
    if (settings.debugReceive)
    {
      systemPrintTimestamp();
      systemPrint("    VC Length: ");
      systemPrintln(*rxData);
      systemPrint("    DestAddr: ");
      if (rxDestVc == VC_BROADCAST)
        systemPrintln("Broadcast");
      else
        systemPrintln(rxDestVc);
      systemPrint("    SrcAddr: ");
      if (rxSrcVc == VC_UNASSIGNED)
        systemPrintln("Unassigned");
      else
        systemPrintln(rxSrcVc);
      if (timeToHop == true) //If the channelTimer has expired, move to next frequency
        hopChannel();
    }
  }

  /*
                                         |<-- rxDataBytes -->|
                                         |                   |
      +----------+----------+------------+------  ...  ------+----------+
      | Optional |          |  Optional  |                   | Optional |
      |  NET ID  | Control  | SF6 Length |       Data        | Trailer  |
      |  8 bits  |  8 bits  |   8 bits   |      n bytes      | n Bytes  |
      +----------+----------+------------+-------------------+----------+
                                         ^
                                         |
                                         '---- rxData
  */

  //Display the packet contents
  if (settings.printPktData || settings.debugReceive)
  {
    systemPrintTimestamp();
    systemPrint("RX: Datagram ");
    systemPrint(rxDataBytes);
    systemPrint(" (0x");
    systemPrint(rxDataBytes, HEX);
    systemPrintln(") bytes");
    if (timeToHop == true) //If the channelTimer has expired, move to next frequency
      hopChannel();
    petWDT();
    if (settings.printPktData && rxDataBytes)
      dumpBuffer(rxData, rxDataBytes);
  }

  //Display the datagram type
  if (settings.debugDatagrams)
  {
    systemPrintTimestamp();
    systemPrint("RX: ");
    systemPrint(v2DatagramType[datagramType]);
    switch (datagramType)
    {
      default:
        systemPrintln();
        break;

      case DATAGRAM_DATA:
      case DATAGRAM_DATA_ACK:
      case DATAGRAM_REMOTE_COMMAND:
      case DATAGRAM_REMOTE_COMMAND_RESPONSE:
      case DATAGRAM_HEARTBEAT:
        if (settings.operatingMode == MODE_POINT_TO_POINT)
        {
          systemPrint(" (ACK #");
          systemPrint(ackNumber);
          systemPrint(")");
        }
        systemPrintln();
        break;
    }
  }
  if (timeToHop == true) //If the channelTimer has expired, move to next frequency
    hopChannel();

  //Process the packet
  datagramsReceived++;
  linkDownTimer = millis();
  return datagramType;
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//Datagram transmission
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Push the outgoing packet to the air
void transmitDatagram()
{
  uint8_t control;
  uint8_t * header;
  uint8_t length;
  int8_t srcVc;
  uint8_t * vcData;
  uint8_t vcLength;
  VIRTUAL_CIRCUIT * vc;

  if (timeToHop == true) //If the channelTimer has expired, move to next frequency
    hopChannel();

  //Parse the virtual circuit header
  vc = NULL;
  if (settings.operatingMode == MODE_VIRTUAL_CIRCUIT)
  {
    vcData = &outgoingPacket[headerBytes];
    vcLength = *vcData++;
    txDestVc = *vcData++;
    srcVc = *vcData++;
    if ((uint8_t)srcVc <= MAX_VC)
    {
      vc = &virtualCircuitList[srcVc];
      vc->messagesSent++;
    }
  }

  //Determine the packet size
  datagramsSent++;
  txDatagramSize = endOfTxData - outgoingPacket;
  length = txDatagramSize - headerBytes;

  //Select the ACK number
  if (txControl.datagramType == DATAGRAM_DATA_ACK)
    txControl.ackNumber = rxAckNumber;
  else
    txControl.ackNumber = txAckNumber;

  //Process the packet
  if (settings.debugDatagrams)
  {
    systemPrintTimestamp();
    systemPrint("TX: ");
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
        if (settings.operatingMode == MODE_POINT_TO_POINT)
        {
          systemPrint(" (ACK #");
          systemPrint(txControl.ackNumber);
          systemPrint(")");
        }
        systemPrintln();
        break;
    }
    if (timeToHop == true) //If the channelTimer has expired, move to next frequency
      hopChannel();
  }

  /*
                                        endOfTxData ---.
                                                       |
                                                       V
      +----------+----------+------------+---  ...  ---+----------+
      | Optional |          |  Optional  |             | Optional |
      |  NET ID  | Control  | SF6 Length |    Data     | Trailer  |
      |  8 bits  |  8 bits  |   8 bits   |   n bytes   | n Bytes  |
      +----------+----------+------------+-------------+----------+
      |                                  |             |
      |                                  |<- Length -->|
      |<--------- txDatagramSize --------------------->|
  */

  //Display the packet contents
  if (settings.printPktData || settings.debugTransmit)
  {
    systemPrintTimestamp();
    systemPrint("TX: Datagram ");
    systemPrint(length);
    systemPrint(" (0x");
    systemPrint(length, HEX);
    systemPrintln(") bytes");
    if (timeToHop == true) //If the channelTimer has expired, move to next frequency
      hopChannel();
    petWDT();
    if (settings.printPktData)
      dumpBuffer(&endOfTxData[-length], length);
  }

  //Build the datagram header
  header = outgoingPacket;
  if (headerBytes && settings.debugTransmit)
  {
    systemPrintTimestamp();
    systemPrint("TX: Header");
    systemPrint(headerBytes);
    systemPrint(" (0x");
    systemPrint(headerBytes);
    systemPrintln(") bytes");
    if (timeToHop == true) //If the channelTimer has expired, move to next frequency
      hopChannel();
  }

  //Add the netID if necessary
  if ((settings.operatingMode == MODE_POINT_TO_POINT) || settings.verifyRxNetID)
  {
    *header++ = settings.netID;

    //Display the netID value
    if (settings.debugTransmit)
    {
      systemPrintTimestamp();
      systemPrint("    NetID: ");
      systemPrint(settings.netID);
      systemPrint(" (0x");
      systemPrint(settings.netID, HEX);
      systemPrintln(")");
      if (timeToHop == true) //If the channelTimer has expired, move to next frequency
        hopChannel();
      petWDT();
    }
  }

  //Add the control byte
  control = *(uint8_t *)&txControl;
  *header++ = control;

  //Display the control value
  if (settings.debugTransmit)
    printControl(control);

  //Add the spread factor 6 length is required
  if (settings.radioSpreadFactor == 6)
  {
    *header++ = length;
    txDatagramSize = MAX_PACKET_SIZE - trailerBytes; //We're now going to transmit a full size datagram
    endOfTxData = &outgoingPacket[txDatagramSize];
    if (settings.debugTransmit)
    {
      systemPrintTimestamp();
      systemPrint("    SF6 Length: ");
      systemPrintln(length);
      if (timeToHop == true) //If the channelTimer has expired, move to next frequency
        hopChannel();
    }
  }

  //Verify the Virtual-Circuit length
  if (settings.debugTransmit && (settings.operatingMode == MODE_VIRTUAL_CIRCUIT))
  {
    systemPrintTimestamp();
    systemPrint("    Length: ");
    systemPrintln(vcLength);
    systemPrint("    DestAddr: ");
    if (txDestVc == VC_BROADCAST)
      systemPrintln("Broadcast");
    else
      systemPrintln(txDestVc);
    systemPrint("    SrcAddr: ");
    if (srcVc == VC_UNASSIGNED)
      systemPrintln("Unassigned");
    else
      systemPrintln(srcVc);
    if (timeToHop == true) //If the channelTimer has expired, move to next frequency
      hopChannel();
  }

  /*
                                        endOfTxData ---.
                                                       |
                                                       V
      +----------+----------+------------+---  ...  ---+
      | Optional |          |  Optional  |             |
      |  NET ID  | Control  | SF6 Length |    Data     |
      |  8 bits  |  8 bits  |   8 bits   |   n bytes   |
      +----------+----------+------------+-------------+
      |                                                |
      |<--------------- txDatagramSize --------------->|
  */

  //Add the datagram trailer
  if (settings.enableCRC16)
  {
    uint16_t crc;
    uint8_t * txData;

    //Compute the CRC-16 value
    crc = 0xffff;
    if (timeToHop == true) //If the channelTimer has expired, move to next frequency
      hopChannel();
    for (txData = outgoingPacket; txData < endOfTxData; txData++)
      crc = crc16Table[*txData ^ (uint8_t)(crc >> (16 - 8))] ^ (crc << 8);
    *endOfTxData++ = (uint8_t)(crc >> 8);
    *endOfTxData++ = (uint8_t)(crc & 0xff);
  }
  txDatagramSize += trailerBytes;
  if (timeToHop == true) //If the channelTimer has expired, move to next frequency
    hopChannel();

  //Display the trailer
  if (trailerBytes && settings.debugTransmit)
  {
    systemPrintTimestamp();
    systemPrint("TX: Trailer ");
    systemPrint(trailerBytes);
    systemPrint(" (0x");
    systemPrint(trailerBytes);
    systemPrintln(") bytes");
    if (timeToHop == true) //If the channelTimer has expired, move to next frequency
      hopChannel();

    //Display the CRC
    if (settings.enableCRC16 && (settings.printPktData || settings.debugReceive))
    {
      systemPrintTimestamp();
      systemPrint("    CRC-16: 0x");
      systemPrint(endOfTxData[-2], HEX);
      systemPrintln(endOfTxData[-1], HEX);
      if (timeToHop == true) //If the channelTimer has expired, move to next frequency
        hopChannel();
    }
  }

  /*
                                                   endOfTxData ---.
                                                                  |
                                                                  V
      +----------+----------+------------+---  ...  ---+----------+
      | Optional |          |  Optional  |             | Optional |
      |  NET ID  | Control  | SF6 Length |    Data     | Trailer  |
      |  8 bits  |  8 bits  |   8 bits   |   n bytes   | n Bytes  |
      +----------+----------+------------+-------------+----------+
      |                                                           |
      |<-------------------- txDatagramSize --------------------->|
  */

  //Display the transmitted packet bytes
  if (settings.printRfData || settings.debugTransmit)
  {
    systemPrintTimestamp();
    systemPrint("TX: Unencrypted Frame ");
    systemPrint(txDatagramSize);
    systemPrint(" (0x");
    systemPrint(txDatagramSize, HEX);
    systemPrintln(") bytes");
    if (timeToHop == true) //If the channelTimer has expired, move to next frequency
      hopChannel();
    petWDT();
    if (settings.printRfData)
      dumpBuffer(outgoingPacket, txDatagramSize);
  }

  //Encrypt the datagram
  if (settings.encryptData == true)
  {
    encryptBuffer(outgoingPacket, txDatagramSize);
    if (timeToHop == true) //If the channelTimer has expired, move to next frequency
      hopChannel();
  }

  //Scramble the datagram
  if (settings.dataScrambling == true)
    radioComputeWhitening(outgoingPacket, txDatagramSize);

  //Display the transmitted packet bytes
  if ((settings.printRfData || settings.debugTransmit)
      && (settings.encryptData || settings.dataScrambling))
  {
    systemPrintTimestamp();
    systemPrint("TX: Encrypted Frame ");
    systemPrint(txDatagramSize);
    systemPrint(" (0x");
    systemPrint(txDatagramSize, HEX);
    systemPrintln(") bytes");
    if (timeToHop == true) //If the channelTimer has expired, move to next frequency
      hopChannel();
    petWDT();
    if (settings.printRfData)
      dumpBuffer(outgoingPacket, txDatagramSize);
  }

  //If we are trainsmitting at high data rates the receiver is often not ready for new data. Pause for a few ms (measured with logic analyzer).
  if (settings.airSpeed == 28800 || settings.airSpeed == 38400)
    delay(2);

  //Reset the buffer data pointer for the next transmit operation
  endOfTxData = &outgoingPacket[headerBytes];

  //Compute the time needed for this frame. Part of ACK timeout.
  frameAirTime = calcAirTime(txDatagramSize);

  //Transmit this datagram
  frameSentCount = 0; //This is the first time this frame is being sent
  retransmitDatagram(vc);
}

//Print the control byte value
void printControl(uint8_t value)
{
  CONTROL_U8 * control = (CONTROL_U8 *)&value;

  systemPrintTimestamp();
  systemPrint("    Control: 0x");
  systemPrintln(value, HEX);
  if (timeToHop == true) //If the channelTimer has expired, move to next frequency
    hopChannel();
  systemPrintTimestamp();
  systemPrint("        ACK # ");
  systemPrintln(value & 3);
  if (timeToHop == true) //If the channelTimer has expired, move to next frequency
    hopChannel();
  systemPrintTimestamp();
  systemPrint("        datagramType ");
  if (control->datagramType < MAX_V2_DATAGRAM_TYPE)
    systemPrintln(v2DatagramType[control->datagramType]);
  else
  {
    systemPrint("Unknown ");
    systemPrintln(control->datagramType);
  }
  if (timeToHop == true) //If the channelTimer has expired, move to next frequency
    hopChannel();
  petWDT();
}

//The previous transmission was not received, retransmit the datagram
void retransmitDatagram(VIRTUAL_CIRCUIT * vc)
{
  /*
      +----------+----------+------------+---  ...  ---+----------+
      | Optional |          |  Optional  |             | Optional |
      |  NET ID  | Control  | SF6 Length |    Data     | Trailer  |
      |  8 bits  |  8 bits  |   8 bits   |   n bytes   | n Bytes  |
      +----------+----------+------------+-------------+----------+
      |                                                           |
      |<-------------------- txDatagramSize --------------------->|
  */

  if (timeToHop == true) //If the channelTimer has expired, move to next frequency
    hopChannel();

  //Display the transmitted frame bytes
  if (frameSentCount && (settings.printRfData || settings.debugTransmit))
  {
    systemPrintTimestamp();
    systemPrint("TX: Retransmit ");
    systemPrint((settings.encryptData || settings.dataScrambling) ? "Encrypted " : "Unencrypted ");
    systemPrint("Frame ");
    systemPrint(txDatagramSize);
    systemPrint(" (0x");
    systemPrint(txDatagramSize, HEX);
    systemPrintln(") bytes");
    if (timeToHop == true) //If the channelTimer has expired, move to next frequency
      hopChannel();
    petWDT();
    if (settings.printRfData)
      dumpBuffer(outgoingPacket, txDatagramSize);
  }

  //Transmit this frame
  if (timeToHop == true) //If the channelTimer has expired, move to next frequency
    hopChannel();
  int state = radio.startTransmit(outgoingPacket, txDatagramSize);
  if (state == RADIOLIB_ERR_NONE)
  {
    frameSentCount++;
    if (vc)
      vc->framesSent++;
    framesSent++;
    xmitTimeMillis = millis();
    frameAirTime = calcAirTime(txDatagramSize); //Calculate frame air size while we're transmitting in the background
    uint16_t responseDelay = frameAirTime / responseDelayDivisor; //Give the receiver a bit of wiggle time to respond
    if (settings.debugTransmit)
    {
      systemPrintTimestamp();
      systemPrint("TX: frameAirTime ");
      systemPrint(frameAirTime);
      systemPrintln(" mSec");
      if (timeToHop == true) //If the channelTimer has expired, move to next frequency
        hopChannel();

      systemPrintTimestamp();
      systemPrint("TX: responseDelay ");
      systemPrint(responseDelay);
      systemPrintln(" mSec");
      if (timeToHop == true) //If the channelTimer has expired, move to next frequency
        hopChannel();
    }
    frameAirTime += responseDelay;
  }
  else if (settings.debugTransmit)
  {
    systemPrintTimestamp();
    systemPrint("TX: Transmit error, state ");
    systemPrintln(state);
  }

  datagramTimer = millis(); //Move timestamp even if error
}

void startChannelTimer()
{
  channelTimer.disableTimer();
  channelTimer.setInterval_MS(settings.maxDwellTime, channelTimerHandler);
  channelTimer.enableTimer();
  timerStart = millis(); //ISR normally takes care of this but allow for correct ACK sync before first ISR
  triggerEvent(TRIGGER_HOP_TIMER_START);
}

void stopChannelTimer()
{
  channelTimer.disableTimer();
  triggerEvent(TRIGGER_HOP_TIMER_STOP);
}

//Given the remote unit's amount of channelTimer that has elapsed,
//adjust our own channelTimer interrupt to be synchronized with the remote unit
void syncChannelTimer()
{
  triggerEvent(TRIGGER_SYNC_CHANNEL);

  uint16_t channelTimerElapsed;
  memcpy(&channelTimerElapsed, &rxVcData[0], sizeof(channelTimerElapsed));
  channelTimerElapsed += ackAirTime;
  channelTimerElapsed += SYNC_PROCESSING_OVERHEAD;

  int16_t remoteRemainingTime = settings.maxDwellTime - channelTimerElapsed;

  int16_t localRemainingTime = settings.maxDwellTime - (millis() - timerStart); //The amount of time we think we have left on this channel

  //If we have just hopped channels, and a sync comes in that is very small, it will incorrectly
  //cause us to hop again, causing the clocks to be sync'd, but the channels to be one ahead.
  //So, if our localRemainingTime is very large, and remoteRemainingTime is very small, then add
  //the remoteRemainingTime to our localRemainingTime
  if (remoteRemainingTime < (settings.maxDwellTime / 16)
      && localRemainingTime > (settings.maxDwellTime - (settings.maxDwellTime / 16)) )
  {
    //    systemPrint("remoteRemainingTime: ");
    //    systemPrint(remoteRemainingTime);
    //    systemPrint(" localRemainingTime: ");
    //    systemPrint(localRemainingTime);
    //    systemPrintln();

    triggerEvent(TRIGGER_SYNC_CHANNEL); //Double trigger
    remoteRemainingTime = remoteRemainingTime + localRemainingTime;
  }

  if (remoteRemainingTime < 0)
  {
    //    systemPrint(" channelTimerElapsed: ");
    //    systemPrint(channelTimerElapsed);
    //    systemPrint("remoteRemainingTime: ");
    //    systemPrint(remoteRemainingTime);
    //    systemPrintln();
    remoteRemainingTime = 0;
  }

  partialTimer = true;
  channelTimer.disableTimer();
  channelTimer.setInterval_MS(remoteRemainingTime, channelTimerHandler); //Adjust our hardware timer to match our mate's
  channelTimer.enableTimer();
}

//This function resets the heartbeat time and re-rolls the random time
//Call when something has happened (ACK received, etc) where clocks have been sync'd
//Short/long times set to avoid two radios attempting to xmit heartbeat at same time
//Those who send an ACK have short time to next heartbeat. Those who send a heartbeat or data have long time to next heartbeat.
void setHeartbeatShort()
{
  heartbeatTimer = millis();
  heartbeatRandomTime = random(settings.heartbeatTimeout * 2 / 10, settings.heartbeatTimeout / 2); //20-50%
}

void setHeartbeatLong()
{
  heartbeatTimer = millis();
  heartbeatRandomTime = random(settings.heartbeatTimeout * 8 / 10, settings.heartbeatTimeout); //80-100%
}
