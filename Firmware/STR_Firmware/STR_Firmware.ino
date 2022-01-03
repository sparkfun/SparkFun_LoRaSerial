/*
  December 15th, 2021
  SparkFun Electronics
  Nathan Seidle

  This firmware runs the core of the SparkFun STR Serial Telemetry Radio product.
  Primarily designed to run on an ATmega328 with the ebyte 1W LoRa Radio based on the SX1726.

  This radio is designed to operate at the physical layer of LoRa sending data directly to an end point
  as opposed to something like LoRaWAN that operates on the data and network layers. For this reason
  the STR is not intended to operate on LoRaWAN.

  The command structure is based on the SiK Arudpilot radio.

  Other processors supported:
    ESP32
    SAMD21

  For other processors the following unique features must be considered:
    Interrupt capable pins for DIO0/1
    Processor reset
    EEPROM read/write/commit

  Compiled with Arduino v1.8.13

  TODO:
    (done) Allow escape chars within first 2 POR seconds
    (done) Support RTS (output, low = don't give me more) and CTS (input, low = I can't send to host)
    (done) Add echo at setting
    (done) Add ability to query module for max bytes per second with current settings
    Implement low power sleep during receiving mode
    Implement radio power down mode
    Remote commands RTIx
    (done) Add Link and Activity LEDs
    Add ATSx option for enable/disable flow control
    Add \r requirement to enter command mode
    (done) Add AirSpeed setting that automatically sets the spread, bandwidth, etc
    Add I2C/Qwiic interface
*/

const int FIRMWARE_VERSION_MAJOR = 1;
const int FIRMWARE_VERSION_MINOR = 0;

//Define the STR board identifier:
//  This is an int which is unique to this variant of the STR hardware which allows us
//  to make sure that the settings in EEPROM are correct for this version of the STR
//  (sizeOfSettings is not necessarily unique and we want to avoid problems when swapping from one variant to another)
//  It is the sum of:
//    the major firmware version * 0x10
//    the minor firmware version
#define STR_IDENTIFIER (FIRMWARE_VERSION_MAJOR * 0x10 + FIRMWARE_VERSION_MINOR)

#include "settings.h"

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
uint8_t pin_link = 255;
uint8_t pin_act = 255;
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//EEPROM for storing settings
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_ARCH_ESP32)

#include <EEPROM.h>

#elif defined(ARDUINO_ARCH_SAMD)

#include <FlashAsEEPROM_SAMD.h> //Click here to get the library: http://librarymanager/All#FlashStorage_SAMD21 v1.2.1 by Khoi Hoang

#endif
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//Radio Library
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
#include <RadioLib.h> //Click here to get the library: http://librarymanager/All#RadioLib v5.1.0
SX1276 radio = NULL; //We can't instantiate here because we don't yet know what pin numbers to use
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//Global variables
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
void(* Reset_AVR) (void) = 0; //Way of resetting the ATmega

//Buffer to store bytes incoming from serial before broadcasting over LoRa
#if defined(ARDUINO_AVR_UNO)
//uint8_t serialReceiveBuffer[512]; //Conserve RAM due to limitations
uint8_t serialReceiveBuffer[128]; //Conserve RAM due to limitations
#else
uint8_t serialReceiveBuffer[1024 * 4]; //Buffer up to 1s of bytes at 4k
#endif

uint16_t head = 0;
uint16_t tail = 0;
unsigned long lastByteReceived_ms = 0; //Track when last transmission was. Send partial buffer once time has expired.

char platformPrefix[15]; //Used for printing platform specific device name
uint8_t escapeCharsReceived = 0; //Used to enter command mode
unsigned long lastEscapeReceived_ms = 0; //Tracks end of serial traffic
const long minEscapeTime_ms = 2000; //Serial traffic must stop this amount before an escape char is recognized

long startTime = 0; //Used for air time of TX frames
long stopTime = 0;
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

void setup()
{
  //eepromErase();
  Serial.begin(5760);
    
  loadSettings(); //Load settings from EEPROM

  Serial.begin(settings.serialSpeed);

  beginBoard(); //Determine what hardware platform we are running on

  beginLoRa(); //Start radio

  Serial.println(F("STR"));
}

void loop()
{
  updateSerial(); //Receive anything coming in and store into buffer

  updateSystemState();
}
