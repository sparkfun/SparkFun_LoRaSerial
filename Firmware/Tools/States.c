#include "settings.h"

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
//Constants
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

const STATE_ENTRY stateList[] =
{
  {STATE_RESET,         "RESET"},               // 0
  {STATE_WAIT_TX_DONE,  "STATE_WAIT_TX_DONE"},  // 1
  {STATE_WAIT_RECEIVE,  "STATE_WAIT_RECEIVE"},  // 2
};

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
//Main loop
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

void loop()
{
  int8_t addressByte;
  FRAME_TYPE frameType;

  switch (state)
  {
  default:
    printf("Unknown state");
    while(1)
      petWDT();
    break;

  case STATE_RESET:
    if (settings.trainingServer)
      //Reserve the server's address (0)
      myAddr = idToAddressByte(ADDR_SERVER, myUniqueId);
    else
      //Unknown client address
      myAddr = ADDR_UNASSIGNED;

    //Start sending heartbeats
    xmitVcHeartbeat(myAddr, myUniqueId);
    changeState(STATE_WAIT_TX_DONE);
    break;

  case STATE_WAIT_TX_DONE:
    returnToReceiving();
    changeState(STATE_WAIT_RECEIVE);
    break;

  case STATE_WAIT_RECEIVE:
    if (transactionComplete)
    {
      transactionComplete = false;

      //Receive a datagram from the client
      frameType = rcvDatagram();

      /*
            .------- rxData
            |
            V
            +--------+------+-----+--- ... ---+
            | Length | Dest | Src |   Data    |
            +--------+------+-----+--- ... ---+
            |                                 |
            |<----------- rxBytes ----------->|
       */

      //Process the datagram
      switch(frameType)
      {
      default:
        break;

      case FRAME_VC_HEARTBEAT:
        //Save our address
        if ((myAddr == ADDR_UNASSIGNED) && (memcmp(myUniqueId, &rxData[3], UNIQUE_ID_BYTES) == 0))
        {
          myAddr = srcAddr;
printf("myAddr: %d\n", myAddr);
        }

        //Translate the unique ID into an address byte
        addressByte = idToAddressByte(srcAddr, &rxData[3]);

        if (settings.trainingServer && (srcAddr == ADDR_UNASSIGNED) && (addressByte >= 0))
          //Assign the address to the client
          xmitVcHeartbeat(addressByte, &rxData[3]);

        changeState(STATE_WAIT_TX_DONE);
        break;

      case FRAME_VC_DATA:
        if ((destAddr != myAddr) && (destAddr != ADDR_BROADCAST))
          returnToReceiving();
        else
        {
          //place in serial output buffer
        }
        break;
      }
    }
    //Process serial data length bytes at a time
//    else if ((serialAvailable() && (available data >= length byte))
//    {
//      if ((length <= maxFrameSize) && ((dest == broadcast) or (dest == cmd) or (link up for dest)))
//      {
//        if (dest == cmd)
//        {
//          Send to command processor
//          Package response with length, dest=myAddr, src=cmd
//          Place packaged response in output serial buffer
//        }
//        else if (dest == myAddr)
//        {
//          Place in output serial buffer
//        }
//        else
//        {
//          xmitVcData();
//          changeState(STATE_WAIT_TX_DONE);
//        }
//      }
//      else
//      {
//        //Discard the frame
//      }
//    }
    else if ((millis() - datagramTimer) >= settings.heartbeatTimeout)
    {
printf("deltaTime: %d\n", millis() - datagramTimer);
      //Send another heartbeat
      xmitVcHeartbeat(myAddr, myUniqueId);
      changeState(STATE_WAIT_TX_DONE);
    }
    break;
  }
}

void changeState(int newState)
{
  printf("State: %s\n", stateList[newState].name);
  state = newState;
}
