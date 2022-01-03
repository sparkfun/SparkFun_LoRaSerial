typedef enum
{
  STR_1W = 0,
  STR_FACET,
} ProductVariant;
ProductVariant productVariant = STR_1W;

typedef enum
{
  RADIO_REGION_US = 0,
  RADIO_REGION_EU433,
  RADIO_REGION_EU863,
  RADIO_REGION_AU915,
  RADIO_REGION_US902,
  RADIO_REGION_IN865,
  RADIO_REGION_DEV,
} RadioRegion;

//These are all the settings that can be set on Serial Terminal Radio. It's recorded to NVM.
typedef struct struct_settings {
  int sizeOfSettings = 0; //sizeOfSettings **must** be the first entry and must be int
  //int strIdentifier = STR_IDENTIFIER; // strIdentifier **must** be the second entry

  bool echo = true;

  uint8_t escapeCharacter = '+';
  uint8_t maxEscapeCharacters = 3; //Number of characters needed to enter command mode

  uint16_t frameSize = 178; //Send batches of bytes through LoRa link. 178 splits ~530 RTCM packet efficiently into 3 frames.
  uint16_t serialTimeoutBeforeSendingFrame_ms = 50; //Send partial buffer if time expires

  uint32_t serialSpeed = 57600; //Default to 57600bps to match RTK Surveyor default firmware
  uint16_t radioBroadcastPower_dbm = 20; //Max software setting is 20 but radios with built-in PA will get 30dBm(1W) with rx/tx_en pins
  float radioFrequency = 915.0; //MHz
  float radioBandwidth = 500.0; //kHz 125/250/500 generally. We need 500kHz for higher data. 
  uint8_t radioSpreadFactor = 9; //6 to 12. Higher factor for longer range.
  uint8_t radioCodingRate = 6; //5 to 8. Chosen to allow higher spread factor.
  uint8_t radioSyncWord = 0x12;
  uint8_t radioPreambleLength = 8; //Number of symbols
  RadioRegion radioRegion = RADIO_REGION_US; //Governs freq, spread, bandwidth.

  //radioAmplifierGain
  //radioOverCurrentLimit

  //SPI Freq to radio - 1MHz?

} Settings;

Settings settings;

//Monitor which devices on the device are on or offline.
struct struct_online {
  bool radio = false;
  bool eeprom = false;
} online;
