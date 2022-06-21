typedef enum
{
  RADIO_NO_LINK_RECEIVING_STANDBY = 0,
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
} RadioStates;
RadioStates radioState = RADIO_NO_LINK_RECEIVING_STANDBY;

//Possible types of packets received
typedef enum
{
  PACKET_BAD = 0,
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

//Train button states
typedef enum
{
  TRAIN_NO_PRESS = 0,
  TRAIN_PRESSED_2S,
  TRAIN_PRESSED_5S,
} TrainStates;
TrainStates trainState = TRAIN_NO_PRESS;

enum
{
  TRIGGER_ACK_PROCESSED = 25,
  TRIGGER_DATA_SEND = 50,
  TRIGGER_RTR_2BYTE = 75,
  TRIGGER_RTR_255BYTE = 100,

  TRIGGER_LINK_SEND_PING = 125,
  TRIGGER_LINK_SENT_ACK_PACKET = 150,
  TRIGGER_LINK_NOISE_TRIGGERED_HOP = 175,
  TRIGGER_LINK_NOISE_TRIGGERED_HOP_ACK_WAIT = 200,
  TRIGGER_LINK_NO_ACK_GIVEUP = 225,
  TRIGGER_LINK_PACKET_RESEND = 250,
  TRIGGER_LINK_DATA_PACKET = 275,
  TRIGGER_LINK_PACKET_RECEIVED = 300,

  TRIGGER_NOLINK_SEND_PING = 325,
  TRIGGER_NOLINK_SEND_ACK_PACKET = 350,
  TRIGGER_NOLINK_NOISE_TRIGGERED_HOP = 375,
  TRIGGER_NOLINK_NO_ACK_GIVEUP = 400,
  TRIGGER_NOLINK_IDENT_PACKET = 425,

  TRIGGER_BROADCAST_DATA_PACKET = 450,
  TRIGGER_BROADCAST_PACKET_RECEIVED = 475,

  TRIGGER_RX_IN_PROGRESS = 500,
  TRIGGER_LINK_BAD_PACKET = 525,
  TRIGGER_LINK_DUPLICATE_PACKET = 550,
  TRIGGER_LINK_CONTROL_PACKET = 575,

  TRIGGER_TRAINING_BAD_PACKET = 600,
  TRIGGER_TRAINING_CONTROL_PACKET = 625,
  TRIGGER_TRAINING_DATA_PACKET = 650,
  TRIGGER_TRAINING_NO_ACK = 675,

  TRIGGER_COMMAND_CONTROL_PACKET = 700,
  TRIGGER_COMMAND_CONTROL_PACKET_ACK = 725,
  TRIGGER_COMMAND_DATA_PACKET_ACK = 750,
  TRIGGER_COMMAND_PACKET_RECEIVED = 775,
  TRIGGER_COMMAND_SENT_ACK_PACKET = 800,
  TRIGGER_COMMAND_PACKET_RESEND = 825,
  TRIGGER_PACKET_COMMAND_DATA = 850,
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
  uint16_t radioBroadcastPower_dbm = 30; //Transmit power in dBm. Max is 30dBm (1W), min is 14dBm (25mW).
  float frequencyMin = 902.0; //MHz
  float frequencyMax = 928.0; //MHz
  uint8_t numberOfChannels = 50; //Divide the min/max freq band into this number of channels and hop between.
  bool frequencyHop = true; //Hop between frequencies to avoid dwelling on any one channel for too long
  uint16_t maxDwellTime = 400; //Max number of ms before hopping (if enabled). Useful for configuring radio to be within regulator limits (FCC = 400ms max)
  float radioBandwidth = 500.0; //kHz 125/250/500 generally. We need 500kHz for higher data.
  uint8_t radioSpreadFactor = 9; //6 to 12. Use higher factor for longer range.
  uint8_t radioCodingRate = 8; //5 to 8. Higher coding rates ensure less packets dropped.
  uint8_t radioSyncWord = 18; //18 = 0x12 is default for custom/private networks. Different sync words does *not* guarantee a remote radio will not get packet.
  uint8_t radioPreambleLength = 8; //Number of symbols. Different lengths does *not* guarantee a remote radio privacy. 8 to 11 works. 8 to 15 drops some. 8 to 20 is silent.
  uint8_t frameSize = MAX_PACKET_SIZE - 2; //Send batches of bytes through LoRa link, max (255 - control trailer) = 253.
  uint16_t serialTimeoutBeforeSendingFrame_ms = 50; //Send partial buffer if time expires
  bool debug = false; //Print basic events: ie, radio state changes
  bool echo = false; //Print locally inputted serial
  uint16_t heartbeatTimeout = 5000; //ms before sending ping to see if link is active
  bool flowControl = false; //Enable the use of CTS/RTS flow control signals
  bool autoTuneFrequency = false; //Based on the last packets frequency error, adjust our next transaction frequency
  bool displayPacketQuality = false; //Print RSSI, SNR, and freqError for received packets
  uint8_t maxResends = 2; //Attempt resends up to this number.

} Settings;
Settings settings;

//Monitor which devices on the device are on or offline.
struct struct_online {
  bool radio = false;
  bool eeprom = false;
} online;
