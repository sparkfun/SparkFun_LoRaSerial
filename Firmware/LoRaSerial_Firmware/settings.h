typedef enum
{
  RADIO_RESET = 0,

  // V1
  RADIO_NO_LINK_RECEIVING_STANDBY,
  RADIO_NO_LINK_TRANSMITTING,
  RADIO_NO_LINK_ACK_WAIT,
  RADIO_NO_LINK_RECEIVED_PACKET,

  RADIO_LINKED_RECEIVING_STANDBY,
  RADIO_LINKED_TRANSMITTING,
  RADIO_LINKED_ACK_WAIT,
  RADIO_LINKED_RECEIVED_PACKET,

  RADIO_BROADCASTING_RECEIVING_STANDBY,
  RADIO_BROADCASTING_TRANSMITTING,
  RADIO_BROADCASTING_RECEIVED_PACKET,

  RADIO_TRAINING_RECEIVING_HERE_FIRST,
  RADIO_TRAINING_TRANSMITTING,
  RADIO_TRAINING_ACK_WAIT,
  RADIO_TRAINING_RECEIVED_PACKET,

  // V2
  // Point-To-Point: Bring up the link
  RADIO_P2P_LINK_DOWN,
  RADIO_P2P_WAIT_TX_PING_DONE,
  RADIO_P2P_WAIT_ACK_1,
  RADIO_P2P_WAIT_TX_ACK_1_DONE,
  RADIO_P2P_WAIT_ACK_2,
  RADIO_P2P_WAIT_TX_ACK_2_DONE,
  RADIO_P2P_LINK_UP,

  RADIO_MAX_STATE,
} RadioStates;
RadioStates radioState = RADIO_NO_LINK_RECEIVING_STANDBY;

typedef struct _RADIO_STATE_ENTRY
{
  RadioStates state;
  const char * name;
  const char * description;
} RADIO_STATE_ENTRY;

//Possible types of packets received
typedef enum
{
  //V2 packet types must start at zero
  DATAGRAM_PING = 0,
  DATAGRAM_ACK_1,
  DATAGRAM_ACK_2,
  DATAGRAM_DATA,
  DATAGRAM_SF6_DATA,
  DATAGRAM_DATA_ACK,
  DATAGRAM_REMOTE_COMMAND,
  DATAGRAM_REMOTE_COMMAND_RESPONSE,

  //Add new V2 datagram types before this line
  MAX_DATAGRAM_TYPE,

  //V1 packet types are below
  PACKET_BAD,
  PACKET_NETID_MISMATCH,
  PACKET_ACK, //ack = 1
  PACKET_DUPLICATE,
  PACKET_PING, //ack = 0, len = 0
  PACKET_DATA,

  PACKET_COMMAND_ACK, //remoteCommand = 1, ack = 1
  PACKET_COMMAND_DATA, //remoteCommand = 1

  PACKET_COMMAND_RESPONSE_ACK, //remoteCommand = 1, remoteCommandResponse = 1, ack = 1
  PACKET_COMMAND_RESPONSE_DATA, //remoteCommand = 1, remoteCommandResponse = 1,

  PACKET_TRAINING_PING,
  PACKET_TRAINING_DATA,
} PacketType;

const char * const v2DatagramType[] =
{//  0       1        2        3        4           5           6          7
  "PING", "ACK-1", "ACK-2", "DATA", "SF6-DATA", "DATA-ACK", "RMT-CMD", "RMT_RESP",
};

//Train button states
typedef enum
{
  TRAIN_NO_PRESS = 0,
  TRAIN_PRESSED_2S,
  TRAIN_PRESSED_5S,
} TrainStates;
TrainStates trainState = TRAIN_NO_PRESS;

enum
{                                             //#, Width - Computed with:
                                              //        triggerWidth = 25
                                              //        triggerUseWidthAsMultiplier = true
                                              //        triggerEnable = 0xffffffff
  TRIGGER_ACK_PROCESSED = 0,                  // 0,   25us
  TRIGGER_DATA_SEND,                          // 1,   50us
  TRIGGER_RTR_2BYTE,                          // 2,   75us
  TRIGGER_RTR_255BYTE,                        // 3,  100us

  TRIGGER_LINK_SEND_PING,                     // 4,  125us
  TRIGGER_LINK_SENT_ACK_PACKET,               // 5,  150 us
  TRIGGER_LINK_NOISE_TRIGGERED_HOP,           // 6,  175us
  TRIGGER_LINK_NOISE_TRIGGERED_HOP_ACK_WAIT,  // 7,  200us
  TRIGGER_LINK_NO_ACK_GIVEUP,                 // 8,  225us
  TRIGGER_LINK_PACKET_RESEND,                 // 9,  250us
  TRIGGER_LINK_DATA_PACKET,                   //10,  275us
  TRIGGER_LINK_PACKET_RECEIVED,               //11,  300us

  TRIGGER_NOLINK_SEND_PING,                   //12,  325us
  TRIGGER_NOLINK_SEND_ACK_PACKET,             //13,  350us
  TRIGGER_NOLINK_NOISE_TRIGGERED_HOP,         //14,  375us
  TRIGGER_NOLINK_NO_ACK_GIVEUP,               //15,  400us
  TRIGGER_NOLINK_IDENT_PACKET,                //16,  425us

  TRIGGER_BROADCAST_DATA_PACKET,              //17,  450us
  TRIGGER_BROADCAST_PACKET_RECEIVED,          //18,  475us

  TRIGGER_RX_IN_PROGRESS,                     //19,  500us
  TRIGGER_LINK_BAD_PACKET,                    //20,  525us
  TRIGGER_LINK_DUPLICATE_PACKET,              //21,  550us
  TRIGGER_LINK_CONTROL_PACKET,                //22,  575us

  TRIGGER_TRAINING_BAD_PACKET,                //23,  600us
  TRIGGER_TRAINING_CONTROL_PACKET,            //24,  625us
  TRIGGER_TRAINING_DATA_PACKET,               //25,  650us
  TRIGGER_TRAINING_NO_ACK,                    //26,  675us

  TRIGGER_COMMAND_CONTROL_PACKET,             //27,  700us
  TRIGGER_COMMAND_CONTROL_PACKET_ACK,         //28,  725us
  TRIGGER_COMMAND_DATA_PACKET_ACK,            //29,  750us
  TRIGGER_COMMAND_PACKET_RECEIVED,            //30,  775us
  TRIGGER_COMMAND_SENT_ACK_PACKET,            //31,  800us
  TRIGGER_COMMAND_PACKET_RESEND,              //32,  825us
  TRIGGER_PACKET_COMMAND_DATA,                //33,  850us
};

//Control where to print command output
typedef enum
{
  PRINT_TO_SERIAL = 0,
  PRINT_TO_RF,
} PrinterEndpoints;
PrinterEndpoints printerEndpoint = PRINT_TO_SERIAL;


struct ControlTrailer
{
  uint8_t resend : 1;
  uint8_t ack : 1;
  uint8_t remoteCommand : 1;
  uint8_t remoteCommandResponse : 1;
  uint8_t train : 1;
  uint8_t filler : 3;
};
struct ControlTrailer receiveTrailer;
struct ControlTrailer responseTrailer;

typedef struct _CONTROL_U8
{
  uint8_t ackNumber : 2;
  uint8_t datagramType: 4;
  uint8_t filler : 2;
} CONTROL_U8;

//These are all the settings that can be set on Serial Terminal Radio. It's recorded to NVM.
typedef struct struct_settings {
  uint16_t sizeOfSettings = 0; //sizeOfSettings **must** be the first entry and must be int
  uint16_t strIdentifier = LRS_IDENTIFIER; // strIdentifier **must** be the second entry

  uint8_t escapeCharacter = '+';
  uint8_t maxEscapeCharacters = 3; //Number of characters needed to enter command mode
  uint8_t responseDelayDivisor = 4; //Add on to max response time after packet has been sent. Factor of 2. 8 is ok. 4 is good. A smaller number increases the delay.

  uint32_t serialSpeed = 57600; //Default to 57600bps to match RTK Surveyor default firmware
  uint32_t airSpeed = 4800; //Default to ~523 bytes per second to support RTCM. Overrides spread, bandwidth, and coding
  uint8_t netID = 192; //Both radios must share a network ID
  bool pointToPoint = true; //Receiving unit will check netID and ACK. If set to false, receiving unit doesn't check netID or ACK.
  bool encryptData = true; //AES encrypt each packet
  uint8_t encryptionKey[16] = { 0x37, 0x78, 0x21, 0x41, 0xA6, 0x65, 0x73, 0x4E, 0x44, 0x75, 0x67, 0x2A, 0xE6, 0x30, 0x83, 0x08 };
  bool dataScrambling = false; //Use IBM Data Whitening to reduce DC bias
  uint8_t radioBroadcastPower_dbm = 30; //Transmit power in dBm. Max is 30dBm (1W), min is 14dBm (25mW).
  float frequencyMin = 902.0; //MHz
  float frequencyMax = 928.0; //MHz
  uint8_t numberOfChannels = 50; //Divide the min/max freq band into this number of channels and hop between.
  bool frequencyHop = true; //Hop between frequencies to avoid dwelling on any one channel for too long
  uint16_t maxDwellTime = 400; //Max number of ms before hopping (if enabled). Useful for configuring radio to be within regulator limits (FCC = 400ms max)
  float radioBandwidth = 500.0; //kHz 125/250/500 generally. We need 500kHz for higher data.
  uint8_t radioSpreadFactor = 9; //6 to 12. Use higher factor for longer range.
  uint8_t radioCodingRate = 8; //5 to 8. Higher coding rates ensure less packets dropped.
  uint8_t radioSyncWord = 18; //18 = 0x12 is default for custom/private networks. Different sync words does *not* guarantee a remote radio will not get packet.
  uint16_t radioPreambleLength = 8; //Number of symbols. Different lengths does *not* guarantee a remote radio privacy. 8 to 11 works. 8 to 15 drops some. 8 to 20 is silent.
  uint8_t frameSize = MAX_PACKET_SIZE - 2; //Send batches of bytes through LoRa link, max (255 - control trailer) = 253.
  uint16_t serialTimeoutBeforeSendingFrame_ms = 50; //Send partial buffer if time expires
  bool debug = false; //Print basic events: ie, radio state changes
  bool echo = false; //Print locally inputted serial
  uint16_t heartbeatTimeout = 5000; //ms before sending ping to see if link is active
  bool flowControl = false; //Enable the use of CTS/RTS flow control signals
  bool autoTuneFrequency = false; //Based on the last packets frequency error, adjust our next transaction frequency
  bool displayPacketQuality = false; //Print RSSI, SNR, and freqError for received packets
  uint8_t maxResends = 2; //Attempt resends up to this number.
  bool sortParametersByName = false; //Sort the parameter list (ATI0) by parameter name
  bool printParameterName = false; //Print the parameter name in the ATSx? response
  bool printFrequency = false; //Print the updated frequency
  bool debugRadio = false; //Print radio info
  bool debugStates = false; //Print state changes
  bool debugTraining = false; //Print training info
  bool debugTrigger = false; //Print triggers
  bool usbSerialWait = false; //Wait for USB serial initialization
  bool printRfData = false; //Print RX and TX data
  bool printPktData = false; //Print data, before encryption and after decryption
  bool verifyRxNetID = false; //Verify RX netID value when not operating in point-to-point mode
  uint8_t triggerWidth = 25; //Trigger width in microSeconds or multipler for trigger width
  bool triggerWidthIsMultiplier = true; //Use the trigger width as a multiplier
  uint32_t triggerEnable = 0xffffffff; //Determine which triggers are enabled: 31 - 0
  uint32_t triggerEnable2 = 0xffffffff; //Determine which triggers are enabled: 63 - 32
  bool debugReceive = false; //Print receive processing
  bool debugTransmit = false; //Print transmit processing
  bool printTxErrors = false; //Print any transmit errors
  bool useV2 = false; //Use the V2 protocol
  bool printTimestamp = false; //Print a timestamp: days hours:minutes:seconds.milliseconds
  bool debugDatagrams = false; //Print the datagrams
} Settings;
Settings settings;

//Monitor which devices on the device are on or offline.
struct struct_online {
  bool radio = false;
  bool eeprom = false;
} online;

#include <RadioLib.h> //Click here to get the library: http://librarymanager/All#RadioLib v5.1.0

typedef void (* ARCH_BEGIN_BOARD)();
typedef void (* ARCH_BEGIN_SERIAL)(uint16_t serialSpeed);
typedef void (* ARCH_BEGIN_WDT)();
typedef void (* ARCH_EEPROM_BEGIN)();
typedef void (* ARCH_EEPROM_COMMIT)();
typedef void (* ARCH_PET_WDT)();
typedef Module * (* ARCH_RADIO)();
typedef bool (* ARCH_SERIAL_AVAILABLE)();
typedef void (* ARCH_SERIAL_FLUSH)();
typedef void (* ARCH_SERIAL_PRINT)(const char * value);
typedef uint8_t (* ARCH_SERIAL_READ)();
typedef void (* ARCH_SERIAL_WRITE)(uint8_t value);
typedef void (* ARCH_SYSTEM_RESET)();
typedef void (* ARCH_UNIQUE_ID)(uint32_t * unique128_BitID);

typedef struct _ARCH_TABLE
{
  ARCH_BEGIN_BOARD beginBoard;    //Initialize the board
  ARCH_BEGIN_SERIAL beginSerial;  //Finish initializing the serial port
  ARCH_BEGIN_WDT beginWDT;        //Initialize the watchdog timer
  ARCH_EEPROM_BEGIN eepromBegin;  //Start an EEPROM operation
  ARCH_EEPROM_COMMIT eepromCommit;//Done with the EEPROM operation
  ARCH_PET_WDT petWDT;            //Reset the expiration time for the WDT
  ARCH_RADIO radio;               //Initialize the radio
  ARCH_SERIAL_AVAILABLE serialAvailable;  //Determine if serial data is available
  ARCH_SERIAL_FLUSH serialFlush;  //Flush the serial port
  ARCH_SERIAL_PRINT serialPrint;  //Print the specified string value
  ARCH_SERIAL_READ serialRead;    //Read a byte from the serial port
  ARCH_SERIAL_WRITE serialWrite;  //Print the specified character
  ARCH_SYSTEM_RESET systemReset;  //Reset the system
  ARCH_UNIQUE_ID uniqueID;        //Get the 128 bit unique ID value
} ARCH_TABLE;
