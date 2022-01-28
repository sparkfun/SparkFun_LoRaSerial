/*
  December 15th, 2021
  SparkFun Electronics
  Nathan Seidle

  This firmware runs the core of the SparkFun LoRaSerial Radio product.
  Primarily designed to run on a ATSAMD21G18A with the ebyte 1W LoRa Radio based on the SX1276.

  This radio is designed to operate at the physical layer of LoRa sending data directly to an end point
  as opposed to something like LoRaWAN that operates on the data and network layers. For this reason
  LoRaSerial is not intended to operate on LoRaWAN.

  The system requests an ACK for certain packet types. 
  
  Each packet contains data plus a NetID (byte) and Control byte. 
  For transmissions at SpreadFactor 6 the packet length is set to 250 bytes and an additional byte is 
  transmitted before NetID that repsents the actual data length within the packet.

  The max packet size for the SX127x is 255 bytes. With the NetID and control bytes removed, we have
  253 bytes available for data (252 when spread factor is 6).

  The AT command structure is based on the SiK Ardupilot radio.

  For a graphical view of the system state machine see:

  Processors supported:
    SAMD21
    ESP32

  For other processors the following unique features must be considered:
    Interrupt capable pins for DIO0/1
    Processor reset (command ATZ)
    EEPROM read/write/commit
    ~14k RAM for serial RX/TX and radio buffers

  Compiled with Arduino v1.8.15

  TODO:
    (done) Allow escape chars within first 2 POR seconds
    (done) Support RTS (output, low = don't give me more) and CTS (input, low = I can't send to host)
    (done) Add echo at setting
    (done) Add ability to query module for max bytes per second with current settings
    (done) Add Link and Activity LEDs
    (done) Add AirSpeed setting that automatically sets the spread, bandwidth, etc
    (done) Change channels[] to malloc or other (not hard coded)
    (done) Add min/max radio frequency at commands
    (done) move heartBeatTimeout to settings. Good for distance testing.
    (done) Add ATSx option for enable/disable flow control
    (done) Settings: Limit frame size to 256 - 2 (for trailer and control)
    (done) Add more state checking RADIOLIB_ERR_TX_TIMEOUT when we send packets out radio sendDataPacket et al
    (done) Fix ack timeout. Should be packetAirTime + controlPacketAirTime, not 2* packetAirTime
    (done) Implement check radio status flags before talk
    (done) If hopping has begun but we don't receive another dio1ISR within hop time, return to receiving
    (done) Print data to both USB and Serial1
    (done) Twinkle LEDs at poweron
    (done) Adjust link frequency based on frequency error report

    Implement low power sleep during receiving mode - ArduinoLowPower
    Implement radio power down mode
    Remote commands RTIx
    Add \r requirement to enter command mode
    Add I2C/Qwiic interface
    Add broadcast. Don't set setting.netID to 255 (as we need it for channel array generation)
      Base should set netID to 255. Rovers should accept netID 255 without pause.
    Put responseDelay divisor into settings
    Put all the platform specific functions into a header or guarded area (reset, eeprom?, etc)
    Add data size to all SAMD boards
    Read data from both USB And Serial1

    Add Broadcast boolean to settings
    Add NetID to commands and register table
    Verify ESP32 EEPROM writing
    Implement spread factor 6, pre-known packet sized transactions
    Implement train feature

    Verify that TX/RX of 2k bytes matches airspeed calculations. Set trigger at each data packet send.
    Search TODO

    Uno:
    Add processor guard to limit Uno to 16 channels (float array is a ram sink)
    Consider malloc the outgoingPacket and incomingPacket sizes due to settings.framesize
    Move radiolib to static arrays to see real memory footprint
*/

const int FIRMWARE_VERSION_MAJOR = 1;
const int FIRMWARE_VERSION_MINOR = 0;

#define ENABLE_DEVELOPER //Uncomment this line to enable special developer modes

//Define the LoRaSerial board identifier:
//  This is an int which is unique to this variant of the LoRaSerial hardware which allows us
//  to make sure that the settings in EEPROM are correct for this version of the LoRaSerial
//  (sizeOfSettings is not necessarily unique and we want to avoid problems when swapping from one variant to another)
//  It is the sum of:
//    the major firmware version * 0x10
//    the minor firmware version
#define LRS_IDENTIFIER (FIRMWARE_VERSION_MAJOR * 0x10 + FIRMWARE_VERSION_MINOR)

#define MAX_PACKET_SIZE 255 //Limited by SX127x

#include "settings.h"
#include "build.h"

//Hardware connections
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//These pins are set in beginBoard()
uint8_t pin_rst = 255;
uint8_t pin_cs = 255;
uint8_t pin_txen = 255;
uint8_t pin_rxen = 255;
uint8_t pin_dio0 = 255;
uint8_t pin_dio1 = 255;
uint8_t pin_rts = 255;
uint8_t pin_cts = 255;
uint8_t pin_linkLED = 255;
uint8_t pin_activityLED = 255;
uint8_t pin_txLED = 255;
uint8_t pin_rxLED = 255;

uint8_t pin_trigger = 255;
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//EEPROM for storing settings
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_ARCH_ESP32)

#include <EEPROM.h>
#define EEPROM_SIZE 1024 //ESP32 emulates EEPROM in non-volatile storage (external flash IC). Max is 508k.

#elif defined(ARDUINO_ARCH_SAMD)

#include <FlashAsEEPROM_SAMD.h> //Click here to get the library: http://librarymanager/All#FlashStorage_SAMD21 v1.2.1 by Khoi Hoang

#endif
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//Radio Library
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
#include <RadioLib.h> //Click here to get the library: http://librarymanager/All#RadioLib v5.1.0
SX1276 radio = NULL; //We can't instantiate here because we don't yet know what pin numbers to use

float *channels;
uint8_t currentChannel = 0; //Increments with each hop
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//Global variables - Serial
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
//Buffer to store bytes incoming from serial before broadcasting over LoRa
#if defined(ARDUINO_AVR_UNO)
//uint8_t serialReceiveBuffer[512]; //Conserve RAM due to limitations
uint8_t serialReceiveBuffer[32]; //Conserve RAM due to limitations
#else
uint8_t serialReceiveBuffer[1024 * 4]; //Buffer up to 1s of bytes at 4k
uint8_t serialTransmitBuffer[1024 * 4]; //Buffer up to 1s of bytes at 4k
#endif

uint16_t txHead = 0;
uint16_t txTail = 0;
uint16_t rxHead = 0;
uint16_t rxTail = 0;
unsigned long lastByteReceived_ms = 0; //Track when last transmission was. Send partial buffer once time has expired.

char platformPrefix[15]; //Used for printing platform specific device name
uint8_t escapeCharsReceived = 0; //Used to enter command mode
unsigned long lastEscapeReceived_ms = 0; //Tracks end of serial traffic
const long minEscapeTime_ms = 2000; //Serial traffic must stop this amount before an escape char is recognized
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//Global variables - Radio
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
uint8_t outgoingPacket[MAX_PACKET_SIZE]; //Contains the current data in route to receiver
uint8_t packetSize = 0; //Tracks how much data + control trailer
uint16_t packetAirTime = 0; //Recalc'd with each new packet transmission
uint16_t controlPacketAirTime = 0; //Recalc'd with each change of settings
uint8_t packetSent = 0; //Increases each time packet is sent
unsigned long packetTimestamp = 0;
uint16_t packetsLost = 0; //Used to determine if radio link is down
uint16_t packetsResent = 0; //Keep metrics of link quality
uint16_t totalPacketsLost = 0; //Keep metrics of link quality
uint8_t maxResends = 2; //Attempt resends up to this number. TODO - move to settings.

uint8_t lastPacket[MAX_PACKET_SIZE]; //Contains the last data received. Used for duplicate testing.
uint8_t lastPacketSize = 0; //Tracks the last packet size we received

#define MAX_LOST_PACKET_BEFORE_LINKLOST 2
bool sentFirstPing = false; //Force a ping to link at POR

volatile bool transactionComplete = false; //Used in dio0ISR
volatile bool timeToHop = true; //Used in dio1ISR
bool expectingAck = false; //Used by various send packet functions

int hopsCompleted = 0;

float frequencyCorrection = 0; //Adjust receive freq based on the last packet received freqError

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//Global variables
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
long startTime = 0; //Used for air time of TX frames
long stopTime = 0;
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

void setup()
{
  //eepromErase();

  Serial.begin(57600);

#if defined(ENABLE_DEVELOPER)
  //Wait for serial to come online for debug printing
#if defined(ARDUINO_ARCH_ESP32)
  delay(500);
#elif defined(ARDUINO_ARCH_SAMD)
  while (!Serial);
#endif
#endif

  loadSettings(); //Load settings from EEPROM

  Serial.begin(settings.serialSpeed);
#if defined(ARDUINO_ARCH_SAMD)
  Serial1.begin(settings.serialSpeed);
#endif

  beginBoard(); //Determine what hardware platform we are running on

  settings.displayPacketQuality = false;
  settings.frequencyHop = true;
  settings.autoTuneFrequency = true;
  settings.heartbeatTimeout = 2000;

  generateHopTable();

  beginLoRa(); //Start radio

  Serial.println(F("LRS"));
#if defined(ARDUINO_ARCH_SAMD)
  Serial1.println(F("LRS"));
#endif

}

void loop()
{
  updateSerial(); //Store incoming and print outgoing

  updateRadioState(); //Ping/ack/send packets as needed
}
