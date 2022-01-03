typedef enum
{
  RADIO_RECEIVING = 0,
  RADIO_TRANSMITTING,
  RADIO_RECEIVING_AVAILABLE,
} RadioStates;
volatile RadioStates radioState = RADIO_RECEIVING;

//These are all the settings that can be set on Serial Terminal Radio. It's recorded to NVM.
typedef struct struct_settings {
  int sizeOfSettings = 0; //sizeOfSettings **must** be the first entry and must be int
  int strIdentifier = STR_IDENTIFIER; // strIdentifier **must** be the second entry

  bool echo = false;
  bool debug = false;

  uint8_t escapeCharacter = '+';
  uint8_t maxEscapeCharacters = 3; //Number of characters needed to enter command mode

  uint8_t frameSize = 255; //Send batches of bytes through LoRa link, max 255.
  uint16_t serialTimeoutBeforeSendingFrame_ms = 50; //Send partial buffer if time expires

  uint32_t serialSpeed = 57600; //Default to 57600bps to match RTK Surveyor default firmware
  uint32_t airSpeed = 4800; //Default to ~523 bytes per second to support RTCM. Overrides spread, bandwidth, and coding
  uint16_t radioBroadcastPower_dbm = 20; //Max software setting is 20 but radios with built-in PA will get 30dBm(1W) with rx/tx_en pins
  float radioFrequency = 915.0; //MHz
  float radioBandwidth = 500.0; //kHz 125/250/500 generally. We need 500kHz for higher data. 
  uint8_t radioSpreadFactor = 9; //6 to 12. Use higher factor for longer range.
  uint8_t radioCodingRate = 6; //5 to 8. 6 was chosen to allow higher spread factor. Higher coding rates ensure less packets dropped.
  uint8_t radioSyncWord = 18; //18=0x12 is default for custom/private networks. Different sync words does *not* guarantee a remote radio will not get packet.
  uint8_t radioPreambleLength = 8; //Number of symbols. Different lengths does *not* guarantee a remote radio privacy. 8 to 11 works. 8 to 15 drops some. 8 to 20 is silent.

} Settings;

Settings settings;

//Monitor which devices on the device are on or offline.
struct struct_online {
  bool radio = false;
  bool eeprom = false;
} online;
