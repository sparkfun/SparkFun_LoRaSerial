typedef enum
{
  RADIO_RESET = 0,

  //Point-To-Point: Bring up the link
  RADIO_P2P_LINK_DOWN,
  RADIO_P2P_WAIT_TX_FIND_PARTNER_DONE,
  RADIO_P2P_WAIT_SYNC_CLOCKS,
  RADIO_P2P_WAIT_TX_SYNC_CLOCKS_DONE,
  RADIO_P2P_WAIT_ZERO_ACKS,
  RADIO_P2P_WAIT_TX_ZERO_ACKS_DONE,

  //Point-to-Point: Link up, data exchange
  RADIO_P2P_LINK_UP,
  RADIO_P2P_LINK_UP_WAIT_TX_DONE,

  //Server-client discovery
  RADIO_DISCOVER_BEGIN,
  RADIO_DISCOVER_SCANNING,
  RADIO_DISCOVER_WAIT_TX_FIND_PARTNER_DONE,

  //Multi-Point: Datagrams
  RADIO_MP_STANDBY,
  RADIO_MP_WAIT_TX_DONE,

  //Training client states
  RADIO_TRAIN_WAIT_TX_FIND_PARTNER_DONE,
  RADIO_TRAIN_WAIT_RX_RADIO_PARAMETERS,
  RADIO_TRAIN_WAIT_TX_PARAM_ACK_DONE,

  //Training server states
  RADIO_TRAIN_WAIT_FOR_FIND_PARTNER,
  RADIO_TRAIN_WAIT_TX_RADIO_PARAMS_DONE,

  //Virtual-Circuit states
  RADIO_VC_WAIT_SERVER,
  RADIO_VC_WAIT_TX_DONE,
  RADIO_VC_WAIT_RECEIVE,

  RADIO_MAX_STATE,
} RadioStates;

RadioStates radioState = RADIO_RESET;

#define P2P_LINK_BREAK_MULTIPLIER       3

typedef struct _RADIO_STATE_ENTRY
{
  RadioStates state;
  bool rxState;
  const char * name;
  const char * description;
} RADIO_STATE_ENTRY;

const RADIO_STATE_ENTRY radioStateTable[] =
{
  {RADIO_RESET,                          0, "RESET",                          NULL},                         // 0

  //Point-to-Point link handshake
  //    State                           RX      Name                              Description
  {RADIO_P2P_LINK_DOWN,                  1, "P2P_LINK_DOWN",                  "P2P: [No Link] Waiting for FIND_PARTNER"}, // 1
  {RADIO_P2P_WAIT_TX_FIND_PARTNER_DONE,  0, "P2P_WAIT_TX_FIND_PARTNER_DONE",  "P2P: [No Link] Wait FIND_PARTNER TX Done"},// 2
  {RADIO_P2P_WAIT_SYNC_CLOCKS,           1, "P2P_WAIT_SYNC_CLOCKS",           "P2P: [No Link] Waiting for SYNC_CLOCKS"},  // 3
  {RADIO_P2P_WAIT_TX_SYNC_CLOCKS_DONE,   0, "P2P_WAIT_TX_SYNC_CLOCKS_DONE",   "P2P: [No Link] Wait SYNC_CLOCKS TX Done"}, // 4
  {RADIO_P2P_WAIT_ZERO_ACKS,             1, "P2P_WAIT_ZERO_ACKS",             "P2P: [No Link] Waiting for ZERO_ACKS"},    // 5
  {RADIO_P2P_WAIT_TX_ZERO_ACKS_DONE,     0, "P2P_WAIT_TX_ZERO_ACKS_DONE",     "P2P: [No Link] Wait ZERO_ACKS TX Done"},   // 6

  //Point-to-Point, link up, data exchange
  //    State                           RX      Name                              Description
  {RADIO_P2P_LINK_UP,                    1, "P2P_LINK_UP",                    "P2P: Receiving Standby"},          // 7
  {RADIO_P2P_LINK_UP_WAIT_TX_DONE,       0, "P2P_LINK_UP_WAIT_TX_DONE",       "P2P: Waiting TX done"},            // 8

  //Server-client discovery
  //    State                           RX      Name                              Description
  {RADIO_DISCOVER_BEGIN,                 0, "DISCOVER_BEGIN",                 "Disc: Setup for scanning"},                  // 9
  {RADIO_DISCOVER_SCANNING,              0, "DISCOVER_SCANNING",              "Disc: Scanning for servers"},                //10
  {RADIO_DISCOVER_WAIT_TX_FIND_PARTNER_DONE, 0, "DISCOVER_WAIT_TX_FIND_PARTNER_DONE", "Disc: Wait for FIND_PARTNER to xmit"}, //11

  //Multi-Point data exchange
  //    State                           RX      Name                              Description
  {RADIO_MP_STANDBY,                     1, "MP_STANDBY",                     "MP: Wait for TX or RX"},           //12
  {RADIO_MP_WAIT_TX_DONE,                0, "MP_WAIT_TX_DONE",                "MP: Waiting for TX done"},         //13

  //Training client states
  //    State                           RX      Name                              Description
  {RADIO_TRAIN_WAIT_TX_FIND_PARTNER_DONE, 0, "TRAIN_WAIT_TX_FIND_PARTNER_DONE", "Train: Wait TX training FIND_PARTNER done"}, //14
  {RADIO_TRAIN_WAIT_RX_RADIO_PARAMETERS, 1, "TRAIN_WAIT_RX_RADIO_PARAMETERS", "Train: Wait for radio parameters"},          //15
  {RADIO_TRAIN_WAIT_TX_PARAM_ACK_DONE,   0, "TRAIN_WAIT_TX_PARAM_ACK_DONE",   "Train: Wait for TX param ACK done"},         //16

  //Training server states
  //    State                           RX      Name                              Description
  {RADIO_TRAIN_WAIT_FOR_FIND_PARTNER,    1, "TRAIN_WAIT_FOR_FIND_PARTNER",    "Train: Wait for training FIND_PARTNER"}, //17
  {RADIO_TRAIN_WAIT_TX_RADIO_PARAMS_DONE, 0, "TRAIN_WAIT_TX_RADIO_PARAMS_DONE", "Train: Wait for TX params done"},      //18

  //Virtual circuit states
  //    State                           RX      Name                              Description
  {RADIO_VC_WAIT_SERVER,                 1, "VC_WAIT_SERVER",                 "VC: Wait for the server"},         //19
  {RADIO_VC_WAIT_TX_DONE,                0, "VC_WAIT_TX_DONE",                "VC: Wait for TX done"},            //20
  {RADIO_VC_WAIT_RECEIVE,                1, "VC_WAIT_RECEIVE",                "VC: Wait for receive"},            //21
};

//Possible types of packets received
typedef enum
{
  //Sync frequencies, HEARTBEAT timing and zero ACKs
  //P2P: Between the two LoRaSerial radios
  //VC:  Between the server radio and a client radio
  DATAGRAM_FIND_PARTNER = 0,        // 0
  DATAGRAM_SYNC_CLOCKS,             // 1
  DATAGRAM_ZERO_ACKS,               // 2

  //Point-to-Point data exchange
  DATAGRAM_DATA,                    // 3
  DATAGRAM_DATA_ACK,                // 4
  DATAGRAM_HEARTBEAT,               // 5
  DATAGRAM_REMOTE_COMMAND,          // 6
  DATAGRAM_REMOTE_COMMAND_RESPONSE, // 7

  //Multi-Point data exchange
  DATAGRAM_DATAGRAM,                // 8

  //Multi-Point training exchange
  DATAGRAM_TRAINING_FIND_PARTNER,   // 9
  DATAGRAM_TRAINING_PARAMS,         //10
  DATAGRAM_TRAINING_ACK,            //11

  //Virtual-Circuit (VC) exchange
  DATAGRAM_VC_HEARTBEAT,            //12
  DATAGRAM_VC_UNKNOWN_ACKS,         //13 Synchronize ACKs client VC to client VC
  DATAGRAM_VC_SYNC_ACKS,            //14
  DATAGRAM_VC_ZERO_ACKS,            //15

  //Add new datagram types before this line
  MAX_DATAGRAM_TYPE,

  //Add new protocol datagrams above this line

  //Common datagram types
  DATAGRAM_BAD,
  DATAGRAM_CRC_ERROR,
  DATAGRAM_NETID_MISMATCH,
  DATAGRAM_DUPLICATE,
  DATAGRAM_NOT_MINE,
} PacketType;

const char * const radioDatagramType[] =
{ //    0            1        2
  "FIND_PARTNER", "ACK-1", "ACK-2",
  // 3        4           5            6           7
  "DATA", "DATA-ACK", "HEARTBEAT", "RMT-CMD", "RMT_RESP",
  //  8
  "DATAGRAM",
  //         9                    10                11
  "TRAINING_FIND_PARTNER", "TRAINING_PARAMS", "TRAINING_ACK",
  //    12
  "VC_HEARTBEAT",
  //      13               14              15
  "VC_UNKNOWN_ACKS", "VC_SYNC_ACKS", "VC_ZERO_ACKS",
};

typedef struct _VC_FLAGS
{
  bool valid : 1;           //Unique ID is valid
  bool wasConnected : 1;    //The VC was previously connected
} VC_FLAGS;

typedef struct _VIRTUAL_CIRCUIT
{
  uint8_t uniqueId[UNIQUE_ID_BYTES];
  unsigned long firstHeartbeatMillis; //Time VC link came up
  unsigned long lastTrafficMillis; //Last time a frame was received
  unsigned long timerMillis; //Last time the timer was started, after handshake or ACK

  //Link quality metrics
  uint32_t framesSent;        //myVc --> VC, Total number of frames sent
  uint32_t framesReceived;    //myVc <-- VC, Total number of frames received
  uint32_t messagesSent;      //myVc --> VC, Total number of messages sent
  uint32_t messagesReceived;  //myVc <-- VC, Total number of messages received
  uint32_t badLength;         //myVc <-- VC, Total number of bad lengths received
  uint32_t linkFailures;      //myVc <-> VC, Total number of link failures

  //Link management

  VC_FLAGS flags;
  uint8_t vcState;            //State of VC

  /* ACK number management

              System A                              System B
             (in destVc)                           (in srcVc)

             txAckNumber
                  |
                  V
            Tx DATA Frame -----------------------> Rx DATA Frame
                                                        |
                                                        V
                                                    AckNumber == rmtTxAckNumber
                                                        |
                                                        | yes
                                                        V
                                                   rxAckNumber = rmtTxAckNumber++
                                                        |
                                                        V
          Rx DATA_Ack Frame <--------------------- Tx DATA_ACK Frame
                  |
                  V
              ackNumber == txAckNumber
                  |
                  | yes
                  V
             txAckNumber++
  */

  uint8_t rmtTxAckNumber; //Next expected ACK # from remote system in DATA frame,
  //incremented upon match to received DATA frame ACK number,
  //indicates frame was received and processed
  //Duplicate frame if received ACK # == (rmtTxAckNumber -1)
  uint8_t rxAckNumber;    //Received ACK # of the most recent acknowledged DATA frame,
  //does not get incremented, used to ACK the data frame
  uint8_t txAckNumber;    //# of next ACK to be sent by the local system in DATA frame,
  //incremented when successfully acknowledged via DATA_ACK frame
} VIRTUAL_CIRCUIT;

#define ADD_VC_STATE_NAMES_TABLE
#include "Virtual_Circuit_Protocol.h"

//Train button states
typedef enum
{
  TRAIN_NO_PRESS = 0,
  TRAIN_PRESSED,
  TRAIN_IN_PROCESS,
} TrainStates;

enum
{ //#, Width - Computed with:
  //        triggerWidth = 25
  //        triggerUseWidthAsMultiplier = true
  //        triggerEnable = 0xffffffff

  TRIGGER_BAD_PACKET,
  TRIGGER_CHANNEL_TIMER_ISR,
  TRIGGER_CRC_ERROR,
  TRIGGER_RECEIVE_IN_PROCESS,
  TRIGGER_DUMMY_READ,
  TRIGGER_FREQ_CHANGE,
  TRIGGER_HANDSHAKE_COMPLETE,
  TRIGGER_HANDSHAKE_SEND_FIND_PARTNER_COMPLETE,
  TRIGGER_HANDSHAKE_SEND_SYNC_CLOCKS_COMPLETE,
  TRIGGER_HANDSHAKE_SYNC_CLOCKS_TIMEOUT,
  TRIGGER_HANDSHAKE_ZERO_ACKS_TIMEOUT,
  TRIGGER_HOP_TIMER_START,
  TRIGGER_HOP_TIMER_STOP,
  TRIGGER_MP_PACKET_RECEIVED,
  TRIGGER_MP_SCAN,
  TRIGGER_MP_TX_DATA,
  TRIGGER_NETID_MISMATCH,
  TRIGGER_RADIO_RESET,
  TRIGGER_RETRANSMIT,
  TRIGGER_RETRANSMIT_FAIL,
  TRIGGER_RTR_255BYTE,
  TRIGGER_RTR_SHORT_PACKET,
  TRIGGER_RX_ACK,
  TRIGGER_RX_COMMAND,
  TRIGGER_RX_COMMAND_RESPONSE,
  TRIGGER_RX_DATA,
  TRIGGER_RX_DATAGRAM,
  TRIGGER_RX_FIND_PARTNER,
  TRIGGER_RX_HEARTBEAT,
  TRIGGER_RX_SPI_DONE,
  TRIGGER_RX_SYNC_CLOCKS,
  TRIGGER_RX_VC_HEARTBEAT,
  TRIGGER_RX_VC_SYNC_ACKS,
  TRIGGER_RX_VC_UNKNOWN_ACKS,
  TRIGGER_RX_VC_ZERO_ACKS,
  TRIGGER_RX_YIELD,
  TRIGGER_RX_ZERO_ACKS,
  TRIGGER_SYNC_CHANNEL_TIMER,
  TRIGGER_TRAINING_CLIENT_RX_PARAMS,
  TRIGGER_TRAINING_CLIENT_TX_ACK_DONE,
  TRIGGER_TRAINING_CLIENT_TX_FIND_PARTNER_DONE,
  TRIGGER_TRAINING_SERVER_RX,
  TRIGGER_TRAINING_SERVER_RX_ACK,
  TRIGGER_TRAINING_SERVER_TX_PARAMS_DONE,
  TRIGGER_TRANSACTION_COMPLETE,
  TRIGGER_TRANSMIT_CANCELED,
  TRIGGER_TX_ACK,
  TRIGGER_TX_COMMAND,
  TRIGGER_TX_COMMAND_RESPONSE,
  TRIGGER_TX_DATA,
  TRIGGER_TX_DATAGRAM,
  TRIGGER_TX_DONE,
  TRIGGER_TX_DUPLICATE_ACK,
  TRIGGER_TX_FIND_PARTNER,
  TRIGGER_TX_HEARTBEAT,
  TRIGGER_TX_LOAD_CHANNEL_TIMER_VALUE,
  TRIGGER_TX_SPI_DONE,
  TRIGGER_TX_SYNC_CLOCKS,
  TRIGGER_TX_VC_HEARTBEAT,
  TRIGGER_TX_VC_SYNC_ACKS,
  TRIGGER_TX_VC_UNKNOWN_ACKS,
  TRIGGER_TX_VC_ZERO_ACKS,
  TRIGGER_TX_YIELD,
  TRIGGER_TX_ZERO_ACKS,
  TRIGGER_UNKNOWN_PACKET,
};

//Control where to print command output
typedef enum
{
  PRINT_TO_SERIAL = 0,
  PRINT_TO_RF,
} PrinterEndpoints;
PrinterEndpoints printerEndpoint = PRINT_TO_SERIAL;

//Select the operating mode
typedef enum
{
  MODE_MULTIPOINT = 0,
  MODE_POINT_TO_POINT,
  MODE_VIRTUAL_CIRCUIT,
} OPERATING_MODE;

typedef struct _CONTROL_U8
{
  PacketType datagramType: 4;
  uint8_t ackNumber : 2;
  uint8_t requestYield : 1;
  uint8_t ignoreFrame : 1;
} CONTROL_U8;

typedef bool (* VALIDATION_ROUTINE)(void * value, uint32_t valMin, uint32_t valMax);

typedef struct _COMMAND_ENTRY
{
  char letter;
  char requireAll;
  bool forceRadioReset;
  uint32_t minValue;
  uint32_t maxValue;
  uint8_t digits;
  uint8_t type;
  VALIDATION_ROUTINE validate;
  const char * name;
  void * setting;
} COMMAND_ENTRY;

typedef enum
{
  LEDS_MULTIPOINT = 0,  // 0: Green1: RX, Green2: Sync, Green3: RSSI, Green4: TX
  //    Blue: Hop, Yellow: HEARTBEAT RX/TX
  LEDS_P2P,         // 1: Green: RSSI, Blue: Serial TX, Yellow: Serial RX
  LEDS_VC,          // 2; Green1: RX, Green2: Sync, Green3: RSSI, Green4: TX
  //    Blue: Hop, Yellow: HEARTBEAT RX/TX
  LEDS_RADIO_USE,   // 3: Green1: RX, Green2: Link, Green3: RSSI, Green4: TX
  //    Blue: Bad frames, Yellow: Bad CRC
  LEDS_RSSI,        // 4: Green: RSSI, Blue: Serial TX, Yellow: Serial RX
  LEDS_RESERVED_1,  // 5
  LEDS_RESERVED_2,  // 6
  LEDS_CYLON,       // 7: Display the cylon pattern on the green LEDs, others off

  //Testing
  LEDS_ALL_OFF,     // 8: All LEDs off
  LEDS_BLUE_ON,     // 9: Blue: ON, other: OFF
  LEDS_YELLOW_ON,   //10; Yellow: ON, other: OFF
  LEDS_GREEN_1_ON,  //11; Green 1: ON, other: OFF
  LEDS_GREEN_2_ON,  //12; Green 2: ON, other: OFF
  LEDS_GREEN_3_ON,  //13; Green 3: ON, other: OFF
  LEDS_GREEN_4_ON,  //14: Green 4: ON, other: OFF
  LEDS_ALL_ON,      //15: All LEDs on

  //Add user LED types from 255 working down
} LEDS_USE_TYPE;

typedef struct _CLOCK_SYNC_DATA
{
  int16_t msToNextHopRemote;
  uint16_t frameAirTimeMsec;
  uint16_t msToNextHop;
  int16_t lclHopTimeMsec;
  int16_t adjustment;
  int8_t delayedHopCount;
  bool timeToHop;
} CLOCK_SYNC_DATA;

//These are all the settings that can be set on Serial Terminal Radio. It's recorded to NVM.
typedef struct struct_settings {
  uint16_t sizeOfSettings = 0; //sizeOfSettings **must** be the first entry and must be int
  uint16_t strIdentifier = LRS_IDENTIFIER; // strIdentifier **must** be the second entry

  //----------------------------------------
  //Radio parameters
  //----------------------------------------

  float frequencyMin = 902.0; //MHz
  float frequencyMax = 928.0; //MHz
  float radioBandwidth = 500.0; //kHz 125/250/500 generally. We need 500kHz for higher data.
  uint32_t txToRxUsec = 657; //TX transactionComplete to RX transactionComplete in microseconds

  bool frequencyHop = true; //Hop between frequencies to avoid dwelling on any one channel for too long
  uint8_t numberOfChannels = 50; //Divide the min/max freq band into this number of channels and hop between.
  uint16_t maxDwellTime = 400; //Max number of ms before hopping (if enabled). Useful for configuring radio to be within regulator limits (FCC = 400ms max)

#if (ENABLE_DEVELOPER == true)
#define TX_POWER_DB     14
#else   //ENABLE_DEVELOPER
#define TX_POWER_DB     30
#endif  //ENABLE_DEVELOPER
  uint8_t radioBroadcastPower_dbm = TX_POWER_DB; //Transmit power in dBm. Max is 30dBm (1W), min is 14dBm (25mW).
  uint8_t radioCodingRate = 8; //5 to 8. Higher coding rates ensure less packets dropped.
  uint8_t radioSpreadFactor = 9; //6 to 12. Use higher factor for longer range.
  uint8_t radioSyncWord = 18; //18 = 0x12 is default for custom/private networks. Different sync words does *not* guarantee a remote radio will not get packet.

  uint16_t radioPreambleLength = 8; //Number of symbols. Different lengths does *not* guarantee a remote radio privacy. 8 to 11 works. 8 to 15 drops some. 8 to 20 is silent.
  bool autoTuneFrequency = false; //Based on the last packets frequency error, adjust our next transaction frequency

  //----------------------------------------
  //Radio protocol parameters
  //----------------------------------------

  uint8_t operatingMode = MODE_VIRTUAL_CIRCUIT; //Receiving unit will check netID and ACK. If set to false, receiving unit doesn't check netID or ACK.

  uint8_t selectLedUse = LEDS_VC; //Select LED use
  bool server = false; //Default to being a client, enable server for multipoint, VC and training
  uint8_t netID = 192; //Both radios must share a network ID
  bool verifyRxNetID = true; //Verify RX netID value when not operating in point-to-point mode

  uint8_t encryptionKey[AES_KEY_BYTES] = { 0x37, 0x78, 0x21, 0x41, 0xA6, 0x65, 0x73, 0x4E, 0x44, 0x75, 0x67, 0x2A, 0xE6, 0x30, 0x83, 0x08 };

  bool encryptData = true; //AES encrypt each packet
  bool dataScrambling = true; //Use IBM Data Whitening to reduce DC bias
  bool enableCRC16 = true; //Append CRC-16 to packet, check CRC-16 upon receive
  uint8_t framesToYield = 3; //If remote requests it, supress transmission for this number of max packet frames

  uint16_t heartbeatTimeout = 5000; //ms before sending HEARTBEAT to see if link is active
  uint16_t overheadTime = 10; //ms added to ack and datagram times before ACK timeout occurs

  uint8_t maxResends = 0; //Attempt resends up to this number, 0 = infinite retries

  //----------------------------------------
  //Serial parameters
  //----------------------------------------

  bool copySerial = false; //Copy the serial parameters to the training client
  bool invertCts = false; //Invert the input of CTS
  bool invertRts = false; //Invert the output of RTS

  uint32_t serialSpeed = 57600; //Default to 57600bps to match RTK Surveyor default firmware

  uint16_t serialTimeoutBeforeSendingFrame_ms = 50; //Send partial buffer if time expires
  bool echo = false; //Print locally inputted serial
  bool flowControl = false; //Enable the use of CTS/RTS flow control signals
#if (ENABLE_DEVELOPER == true)
#define WAIT_SERIAL_DEFAULT     true
#else   //ENABLE_DEVELOPER
#define WAIT_SERIAL_DEFAULT     false
#endif  //ENABLE_DEVELOPER
  bool usbSerialWait = WAIT_SERIAL_DEFAULT; //Wait for USB serial initialization

  //----------------------------------------
  //Training parameters
  //----------------------------------------

  uint8_t clientFindPartnerRetryInterval = 3; //Number of seconds before retransmiting the client FIND_PARTNER

  uint8_t trainingKey[AES_KEY_BYTES] = { 0x53, 0x70, 0x61, 0x72, 0x6b, 0x46, 0x75, 0x6E, 0x54, 0x72, 0x61, 0x69, 0x6e, 0x69, 0x6e, 0x67 };

  uint8_t trainingTimeout = 1; //Timeout in minutes to complete the training

  //----------------------------------------
  //Trigger parameters
  //----------------------------------------

  bool copyTriggers = false; //Copy the trigger parameters to the training client
  uint8_t triggerWidth = 10; //Trigger width in microSeconds or multipler for trigger width
  bool triggerWidthIsMultiplier = true; //Use the trigger width as a multiplier

  uint32_t triggerEnable = 0; //Determine which triggers are enabled: 31 - 0
  uint32_t triggerEnable2 = 0; //Determine which triggers are enabled: 63 - 32


  //----------------------------------------
  //Debug parameters
  //----------------------------------------

  bool copyDebug = false; //Copy the debug parameters to the training client
  bool debug = false; //Print basic events: ie, radio state changes
  bool debugDatagrams = false; //Print the datagrams
  bool debugHeartbeat = false; //Print the HEARTBEAT timing values

  bool debugNvm = false; //Debug NVM operation
  bool debugRadio = false; //Print radio info
  bool debugReceive = false; //Print receive processing
  bool debugSerial = false; //Debug the serial input

  bool debugStates = false; //Print state changes
  bool debugSync = false; //Print clock sync processing
  bool debugTraining = false; //Print training info
  bool debugTransmit = false; //Print transmit processing

  bool displayRealMillis = false; //true = Display the millis value without offset, false = use offset
  bool printAckNumbers = false; //Print the ACK numbers
  bool printChannel = false; //Print the channel number
  bool printFrequency = false; //Print the updated frequency

  bool printLinkUpDown = false; //Print the link up and link down messages
  bool printPacketQuality = false; //Print RSSI, SNR, and freqError for received packets
  bool printPktData = false; //Print data, before encryption and after decryption
  bool printRfData = false; //Print RX and TX data

  bool printTimestamp = false; //Print a timestamp: days hours:minutes:seconds.milliseconds
  bool printTxErrors = false; //Print any transmit errors

  //Add new parameters immediately before this line
  //-- Add commands to set the parameters
  //-- Add parameters to routine updateRadioParameters
} Settings;
Settings settings;

//Monitor which devices on the device are on or offline.
struct struct_online {
  bool radio = false;
  bool eeprom = false;
} online;

#include <RadioLib.h> //Click here to get the library: http://librarymanager/All#RadioLib v5.5.0

typedef void (* ARCH_BEGIN_BOARD)();
typedef void (* ARCH_BEGIN_SERIAL)(uint16_t serialSpeed);
typedef void (* ARCH_BEGIN_WDT)();
typedef void (* ARCH_EEPROM_BEGIN)();
typedef void (* ARCH_EEPROM_COMMIT)();
typedef void (* ARCH_PET_WDT)();
typedef Module * (* ARCH_RADIO)();
typedef bool (* ARCH_SERIAL_AVAILABLE)();
typedef void (* ARCH_SERIAL_FLUSH)();
typedef uint8_t (* ARCH_SERIAL_READ)();
typedef void (* ARCH_SERIAL_WRITE)(uint8_t value);
typedef void (* ARCH_SYSTEM_RESET)();
typedef void (* ARCH_UNIQUE_ID)(uint8_t * unique128_BitID);

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
  ARCH_SERIAL_READ serialRead;    //Read a byte from the serial port
  ARCH_SERIAL_WRITE serialWrite;  //Print the specified character
  ARCH_SYSTEM_RESET systemReset;  //Reset the system
  ARCH_UNIQUE_ID uniqueID;        //Get the 128 bit unique ID value
} ARCH_TABLE;

typedef struct _U8_TO_STRING
{
  uint8_t value;
  const char * string;
} U8_TO_STRING;

typedef struct _I16_TO_STRING
{
  int16_t value;
  const char * string;
} I16_TO_STRING;

//Declare the radio call types
typedef enum
{
  RADIO_CALL_configureRadio = 0,
  RADIO_CALL_setRadioFrequency,
  RADIO_CALL_returnToReceiving,
  RADIO_CALL_calcAirTimeUsec,
  RADIO_CALL_xmitDatagramP2PFindPartner,
  RADIO_CALL_xmitDatagramP2PSyncClocks,
  RADIO_CALL_xmitDatagramP2PZeroAcks,
  RADIO_CALL_xmitDatagramP2PCommand,
  RADIO_CALL_xmitDatagramP2PCommandResponse,
  RADIO_CALL_xmitDatagramP2PData,
  RADIO_CALL_xmitDatagramP2PHeartbeat,
  RADIO_CALL_xmitDatagramP2PAck,
  RADIO_CALL_xmitDatagramMpData,
  RADIO_CALL_xmitDatagramMpHeartbeat,
  RADIO_CALL_xmitDatagramTrainingFindPartner,
  RADIO_CALL_xmitDatagramTrainingAck,
  RADIO_CALL_xmitDatagramTrainRadioParameters,
  RADIO_CALL_xmitVcDatagram,
  RADIO_CALL_xmitVcHeartbeat,
  RADIO_CALL_xmitVcAckFrame,
  RADIO_CALL_xmitVcUnknownAcks,
  RADIO_CALL_xmitVcSyncAcks,
  RADIO_CALL_xmitVcZeroAcks,
  RADIO_CALL_rcvDatagram,
  RADIO_CALL_transmitDatagram,
  RADIO_CALL_retransmitDatagram,
  RADIO_CALL_startChannelTimer,
  RADIO_CALL_stopChannelTimer,
  RADIO_CALL_syncChannelTimer,
  RADIO_CALL_setHeartbeatShort,
  RADIO_CALL_setHeartbeatLong,
  RADIO_CALL_setHeartbeatMultipoint,
  RADIO_CALL_setVcHeartbeatTimer,
  RADIO_CALL_hopChannel,

  //Insert new values before this line
  RADIO_CALL_transactionCompleteISR,
  RADIO_CALL_hopISR,
  RADIO_CALL_channelTimerHandler,
#ifdef  RADIOLIB_LOW_LEVEL
  RADIO_CALL_readSX1276Register,
  RADIO_CALL_printSX1276Registers,
#endif  //RADIOLIB_LOW_LEVEL
  RADIO_CALL_MAX
} RADIO_CALLS;
