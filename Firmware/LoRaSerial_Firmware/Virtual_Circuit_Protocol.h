#ifndef __VIRTUAL_CIRCUIT_PROTOCOL_H__
#define __VIRTUAL_CIRCUIT_PROTOCOL_H__

//------------------------------------------------------------------------------
// Constants
//------------------------------------------------------------------------------

//Virtual-Circuit source and destination index values
#define MAX_VC              8
#define VC_SERVER           0
#define VC_BROADCAST        -1
#define VC_COMMAND          -2    //Command input and command response
#define VC_UNASSIGNED       -3

//Source and destinations reserved for the local host
#define PC_COMMAND          -17   //Command input and command response
#define PC_LINK_STATUS      -18   //Asynchronous link status output

//Field offsets in the VC HEARTBEAT frame
#define VC_HB_UNIQUE_ID     0
#define VC_HB_MILLIS        (VC_HB_UNIQUE_ID + UNIQUE_ID_BYTES)
#define VC_HB_CHANNEL_TIMER (VC_HB_MILLIS + sizeof(uint32_t))
#define VC_HB_CHANNEL       (VC_HB_CHANNEL_TIMER + sizeof(uint16_t))
#define VC_HB_END           (VC_HB_CHANNEL + sizeof(uint8_t))

#define VC_LINK_BREAK_MULTIPLIER    3 //Number of missing HEARTBEAT timeouts

//ASCII characters
#define START_OF_VC_SERIAL  0x02      //From ASCII table - Start of Text

//------------------------------------------------------------------------------
// Protocol Exchanges
//------------------------------------------------------------------------------
/*
Host Interaction using Virtual-Circuits

       Host A                   LoRa A      LoRa B               Host B

All output goes to serial                                           .
                                                                    .
              +++ ---->                                             .
                  <---- OK
              .
              .
              .
                  <---- Command responses
                  <---- Debug message

         Mode: VC ---->
                  <---- OK

       (VC debug) <---- Debug message

        CMD: AT&W ---->
     (VC command) <---- OK

  CMD: LINK_RESET ---->
                         HEARTBEAT -2 ---->
                         HEARTBEAT #  <---- From server

      (VC status) <---- Link self up

                          HEARTBEAT # ---->

      (VC status) <---- Link A up             Link A Up ----> (link status)

       (VC debug) <---- Debug message

                                      <---- HEARTBEAT B
      (VC status) <---- Link B Up

       (VC debug) <---- Debug message

  MSG: Data for B ---->
                           Data for B ---->
                                      <---- ACK
                                             Data for B ----> MSG: Data for B
                                                        <---- MSG: Resp for A
                                      <---- Resp for A
                                  ACK ---->
  MSG: Resp for A <---- Resp for A

*/
//------------------------------------------------------------------------------
// Types
//------------------------------------------------------------------------------

typedef struct _VC_RADIO_MESSAGE_HEADER
{
  uint8_t length;         //Length in bytes of the VC message
  int8_t destVc;          //Destination VC
  int8_t srcVc;           //Source VC
} VC_RADIO_MESSAGE_HEADER;

#define VC_RADIO_HEADER_BYTES   (sizeof(VC_RADIO_MESSAGE_HEADER)) //Length of the radio VC header in bytes

typedef struct _VC_SERIAL_MESSAGE_HEADER
{
  uint8_t start;          //START_OF_HEADING
  VC_RADIO_MESSAGE_HEADER radio;
} VC_SERIAL_MESSAGE_HEADER;

#define VC_SERIAL_HEADER_BYTES  (sizeof(VC_SERIAL_MESSAGE_HEADER)) //Length of the serial VC header in bytes

typedef struct _VC_LINK_STATUS_MESSAGE
{
  uint8_t linkStatus;     //Link status
} VC_LINK_STATUS_MESSAGE;

#define LINK_DOWN         0
#define LINK_UP           1

#endif  //__VIRTUAL_CIRCUIT_PROTOCOL_H__
