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

  For a graphical view of the system state machine see: https://lucid.app/lucidchart/7293b4a6-690a-493e-a3f6-92bf47025fb1/edit?invitationId=inv_9476a070-5ba9-40e6-b89f-9c0d22af9855

  Build notes:
    RadioLib should have RADIOLIB_FIX_ERRATA_SX127X turned on (uncommented)

  Processors supported:
    SAMD21
    ESP32

  For other processors the following unique features must be considered:
    Interrupt capable pins for DIO0/1
    Processor reset (command ATZ)
    EEPROM read/write/commit
    ~14k RAM for serial RX/TX and radio buffers

  Compiled with Arduino v1.8.15
*/

const int FIRMWARE_VERSION_MAJOR = 1;
const int FIRMWARE_VERSION_MINOR = 0;

//#define ENABLE_DEVELOPER //Uncomment this line to enable special developer modes

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
uint8_t pin_txLED = 255;
uint8_t pin_rxLED = 255;
uint8_t pin_trainButton = 255;
uint8_t pin_rssi1LED = 255;
uint8_t pin_rssi2LED = 255;
uint8_t pin_rssi3LED = 255;
uint8_t pin_rssi4LED = 255;
uint8_t pin_boardID = 255;

uint8_t pin_trigger = 255;
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//EEPROM for storing settings
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
#if defined(ARDUINO_ARCH_ESP32)

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
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//Encryption
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#include <Crypto.h> //From: https://github.com/rweather/arduinolibs
#include <AES.h>
#include <GCM.h>
GCM <AES128> gcm;

uint8_t AESiv[12] = {0}; //Set during hop table generation
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Watchdog
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
#if defined(ARDUINO_ARCH_SAMD)
#include <WDTZero.h> //https://github.com/javos65/WDTZero
WDTZero myWatchDog;
#endif
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//Buttons - Interrupt driven and debounce
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
#include <JC_Button.h> // http://librarymanager/All#JC_Button
Button *trainBtn = NULL; //We can't instantiate the button here because we don't yet know what pin number to use

const int trainButtonTime = 2000; //ms press and hold before entering training
const int trainWithDefaultsButtonTime = 5000; //ms press and hold before entering training
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//Global variables - Serial
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
//Buffer to store bytes incoming from serial before broadcasting over LoRa
uint8_t serialReceiveBuffer[1024 * 4]; //Bytes received from UART waiting to be RF transmitted. Buffer up to 1s of bytes at 4k
uint8_t serialTransmitBuffer[1024 * 4]; //Bytes received from RF waiting to be printed out UART. Buffer up to 1s of bytes at 4k

uint16_t txHead = 0;
uint16_t txTail = 0;
uint16_t rxHead = 0;
uint16_t rxTail = 0;

uint8_t commandRXBuffer[700]; //Bytes received from remote, waiting for printing or AT parsing
uint8_t commandTXBuffer[700]; //Bytes waiting to be transmitted to the remote unit
uint16_t commandTXHead = 0;
uint16_t commandTXTail = 0;
uint16_t commandRXHead = 0;
uint16_t commandRXTail = 0;

unsigned long lastByteReceived_ms = 0; //Track when last transmission was. Send partial buffer once time has expired.

char platformPrefix[25]; //Used for printing platform specific device name, ie "SAMD21 1W 915MHz"
uint8_t escapeCharsReceived = 0; //Used to enter command mode
unsigned long lastEscapeReceived_ms = 0; //Tracks end of serial traffic
const long minEscapeTime_ms = 2000; //Serial traffic must stop this amount before an escape char is recognized

uint16_t petTimeoutHalf = 0; //Half the amount of time before WDT. Helps reduce amount of time spent petting.
unsigned long lastPet = 0; //Remebers time of last WDT pet.
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

uint8_t lastPacket[MAX_PACKET_SIZE]; //Contains the last data received. Used for duplicate testing.
uint8_t lastPacketSize = 0; //Tracks the last packet size we received
unsigned long lastPacketReceived = 0; //Controls link LED in broadcast mode
unsigned long lastLinkBlink = 0; //Controls link LED in broadcast mode

#define MAX_LOST_PACKET_BEFORE_LINKLOST 2
bool sentFirstPing = false; //Force a ping to link at POR

volatile bool transactionComplete = false; //Used in dio0ISR
volatile bool timeToHop = true; //Used in dio1ISR
bool expectingAck = false; //Used by various send packet functions

float frequencyCorrection = 0; //Adjust receive freq based on the last packet received freqError

unsigned long lastTrainBlink = 0; //Controls LED during training

Settings originalSettings; //Create a duplicate of settings during training so that we can resort as needed
uint8_t trainNetID; //New netID passed during training
uint8_t trainEncryptionKey[16]; //New AES key passed during training

bool inCommandMode = false; //Normal data is prevented from entering serial output when in command mode
char commandBuffer[100]; //Received serial gets stored into buffer until \r or \n is received
uint8_t commandLength = 0;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//Global variables
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
long startTime = 0; //Used for air time of TX frames
long stopTime = 0;

bool confirmDeliveryBeforeRadioConfig = false; //Goes true when we have remotely configured a radio

int trainCylonNumber = 0b0001;
int trainCylonDirection = -1;
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

void setup()
{
  beginSerial(57600); //Default for debug messages before board begins

  loadSettings(); //Load settings from EEPROM

  beginSerial(settings.serialSpeed);

  beginBoard(); //Determine what hardware platform we are running on

  //settings.airSpeed = 19200;
  //settings.debug = true; //Enable trigger pin events

  beginLoRa(); //Start radio

  beginButton(); //Start watching the train button

  beginWDT(); //Start watchdog timer

  systemPrintln("LRS");
}

void loop()
{
  petWDT();

  updateButton();

  updateSerial(); //Store incoming and print outgoing

  updateRadioState(); //Ping/ack/send packets as needed
}
