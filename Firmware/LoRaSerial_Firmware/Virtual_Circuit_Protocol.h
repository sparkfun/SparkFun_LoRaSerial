#ifndef __VIRTUAL_CIRCUIT_PROTOCOL_H__
#define __VIRTUAL_CIRCUIT_PROTOCOL_H__

//------------------------------------------------------------------------------
// Constants
//------------------------------------------------------------------------------

//Define the virtual circuit address byte
#define VCAB_NUMBER_BITS    5
#define MAX_VC              (1 << VCAB_NUMBER_BITS)
#define VCAB_NUMBER_MASK    (MAX_VC - 1)
#define VCAB_CHANNEL_BITS   (8 - VCAB_NUMBER_BITS)
#define VCAB_CHANNEL_MASK   ((1 << VCAB_CHANNEL_BITS) - 1)

//Communications over the virtual circuit are broken down into the following
//address spaces:
//
//    0 - Data communications
//    1 - Remote commands
//    6 - Remote responses
//    7 - Special virtual circuits

//The following addresses are reserved for data communications: 0 - VCAB_NUMBER_MASK
//Address zero is reserved for the server
#define VC_SERVER           0

//Address space 7 is reserved for the following special addresses:

#define VC_RSVD_SPECIAL_VCS ((int8_t)(VCAB_CHANNEL_MASK << VCAB_NUMBER_BITS))
#define VC_BROADCAST        ((int8_t)(VC_RSVD_SPECIAL_VCS | VCAB_NUMBER_MASK))
#define VC_COMMAND          (VC_BROADCAST - 1) //Command input and command response
#define VC_UNASSIGNED       (VC_COMMAND - 1)
#define VC_IGNORE_TX        (VC_UNASSIGNED - 1)

//Source and destinations reserved for the local host
#define PC_COMMAND          VC_RSVD_SPECIAL_VCS //Command input and command response
#define PC_LINK_STATUS      (PC_COMMAND + 1)    //Asynchronous link status output
#define PC_DATA_ACK         (PC_LINK_STATUS + 1)//Indicate data delivery success
#define PC_DATA_NACK        (PC_DATA_ACK + 1)   //Indicate data delivery failure
#define PC_SERIAL_RECONNECT (PC_DATA_NACK + 1)  //Disconnect/reconnect the serial port over LoRaSerial CPU reset

//Address space 1 and 6 are reserved for the host PC interface to support remote
//command processing.  The radio removes these bits and converts them to the
//appropriate datagram type.  Upon reception of one of these messages the bit is
//added back into the VC header and the message is delivered to the host PC.
//As a result, any host may send commands to any other host!
#define MIN_RX_NOT_ALLOWED  VC_RSVD_SPECIAL_VCS
#define PC_REMOTE_RESPONSE  ((int8_t)(6 << VCAB_NUMBER_BITS))
//Address spaces 2 - 5 are not currently defined
#define MIN_TX_NOT_ALLOWED  ((int8_t)(2 << VCAB_NUMBER_BITS)) //Marker only
#define PC_REMOTE_COMMAND   ((int8_t)(1 << VCAB_NUMBER_BITS))

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

typedef struct _VC_STATE_MESSAGE
{
  uint8_t vcState;        //VC state
} VC_STATE_MESSAGE;

typedef enum
{
  VC_STATE_LINK_DOWN = 0,     //0: HEARTBEATs not received
  VC_STATE_LINK_ALIVE,        //1: Receiving HEARTBEATs, waiting for UNKNOWN_ACKS
  VC_STATE_SEND_UNKNOWN_ACKS, //2: ATC command received, sending UNKNOWN_ACKS
  VC_STATE_WAIT_SYNC_ACKS,    //3: UNKNOWN_ACKS sent, waiting for SYNC_ACKS
  VC_STATE_WAIT_ZERO_ACKS,    //4: SYNC_ACKS sent, waiting for ZERO_ACKS
  VC_STATE_CONNECTED,         //5: ZERO_ACKS received, ACKs cleared, ready to send data

  //Insert new states before this line
  VC_STATE_MAX
} VC_STATE_TYPE;

typedef struct _VC_DATA_ACK_NACK_MESSAGE
{
  uint8_t msgDestVc;      //message destination VC
} VC_DATA_ACK_NACK_MESSAGE;

//------------------------------------------------------------------------------
// Macros
//------------------------------------------------------------------------------

#define DATA_BYTES(vc_message_length)     (vc_message_length - VC_RADIO_HEADER_BYTES)
#define DATA_BUFFER(data)                 (data + VC_RADIO_HEADER_BYTES)
#define GET_CHANNEL_NUMBER(vc)            (vc & (VCAB_CHANNEL_MASK << VCAB_NUMBER_BITS))

//------------------------------------------------------------------------------
// Constants
//------------------------------------------------------------------------------

//Only one instance of this table is necessary
#ifdef ADD_VC_STATE_NAMES_TABLE
const char * const vcStateNames[] =
{ //   0            1
  "LINK-DOWN", "LINK-ALIVE",
  //     2             3            4            5
  "UNKNOWN-ACKS", "SYNC-ACKS", "ZERO-ACKS", "CONNECTED",
};
#endif  //ADD_VC_STATE_NAMES_TABLE

#endif  //__VIRTUAL_CIRCUIT_PROTOCOL_H__
