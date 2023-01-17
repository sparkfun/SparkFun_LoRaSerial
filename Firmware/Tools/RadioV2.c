#include "settings.h"

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
//Constants
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

const char * frameTypeTable[] =
{
  "VC_HEARTBEAT",
  "ADDRESS_BYTE",
};

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
//Frames
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

void xmitVcHeartbeat(int8_t addr, uint8_t * id)
{
  txData = txBuffer;
  *txData++ = FRAME_VC_HEARTBEAT;
  *txData++ = 0; //Reserve for length
  *txData++ = VC_BROADCAST;
  *txData++ = addr;
  memcpy(txData, id, UNIQUE_ID_BYTES);
  txData += UNIQUE_ID_BYTES;
  transmitDatagram((const struct sockaddr *)&broadcastAddr, sizeof(broadcastAddr));
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
//Datagram Receive
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

FRAME_TYPE rcvDatagram()
{
  FRAME_TYPE frameType;
  uint8_t length;

  //Receive a datagram from the client
  if (usingTerminal)
    rxBytes = readLoRaSerial(rxBuffer, sizeof(rxBuffer));
  else
  {
    remoteAddrLength = sizeof(remoteAddr);
    memset(&remoteAddr, 0, remoteAddrLength);
    rxBytes = recvfrom(radio, rxBuffer, sizeof(rxBuffer), MSG_WAITALL,
                       (struct sockaddr *) &remoteAddr, &remoteAddrLength);
    if (rxBytes < 0)
    {
      perror("Failed call to recvfrom!");
      exit(rxBytes);
    }
  }

  //Remove the header
  rxData = rxBuffer;
  frameType = (FRAME_TYPE)*rxData++;
  length = rxData[0];
  destAddr = rxData[1];
  srcAddr = rxData[2];

  //Display the received data
  printf("Received Frame: %d --> %d\n", srcAddr, destAddr);
  printf("Header:\n");
  printf("    FrameType: %s\n", frameTypeTable[frameType]);
  printf("    Length: %d\n", length);
  printf("    Destination Address: %d\n", destAddr);
  printf("    Source Address: %d\n", srcAddr);
  dumpBuffer(rxBuffer, rxBytes);
  rxBytes -= 1;

  //Return the frame type
  return frameType;
}

void returnToReceiving()
{
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
//Datagram Transmit
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

void transmitDatagram(const struct sockaddr * addr, int addrLen)
{
  int bytesSent;
  uint8_t * data;
  FRAME_TYPE frameType;
  uint8_t length;

  //Get the buffer length
  txBytes = txData - txBuffer;
  length = txBytes - 1;
  txBuffer[1] = length;

  //Decode the frame
  data = txBuffer;
  frameType = (FRAME_TYPE)*data++;
  length = *data++;
  destAddr = *data++;
  srcAddr = *data++;

  //Display this datagram
  printf("Transmitting Frame: %d --> %d\n", srcAddr, destAddr);
  printf("Header:\n");
  printf("    FrameType: %s\n", frameTypeTable[frameType]);
  printf("    Length: %d\n", length);
  printf("    Destination Address: %d\n", destAddr);
  printf("    Source Address: %d\n", srcAddr);
  dumpBuffer(txBuffer, txBytes);

  //Send this datagram
  if (usingTerminal)
    bytesSent = write(radio, (const void *)txBuffer, txBytes);
  else
    bytesSent = sendto(radio, (const char *)txBuffer, txBytes, MSG_CONFIRM,
                     addr, addrLen);
  if (bytesSent < 0)
    perror("Failed to send datagram!");
  else
    datagramTimer = millis();
}
