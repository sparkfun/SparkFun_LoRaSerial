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

const int FIRMWARE_VERSION_MAJOR = 2;
const int FIRMWARE_VERSION_MINOR = 0;

#define RADIOLIB_LOW_LEVEL  //Enable access to the module functions
#define ENABLE_DEVELOPER //Uncomment this line to enable special developer modes

#define UNUSED(x) (void)(x)

//Define the LoRaSerial board identifier:
//  This is an int which is unique to this variant of the LoRaSerial hardware which allows us
//  to make sure that the settings in EEPROM are correct for this version of the LoRaSerial
//  (sizeOfSettings is not necessarily unique and we want to avoid problems when swapping from one variant to another)
//  It is the sum of:
//    the major firmware version * 0x10
//    the minor firmware version
#define LRS_IDENTIFIER (FIRMWARE_VERSION_MAJOR * 0x10 + FIRMWARE_VERSION_MINOR)

#define ACK_BYTES       2   //Length of the ACK in bytes
#define MAX_PACKET_SIZE 255 //Limited by SX127x
#define AES_IV_BYTES    12  //Number of bytes for AESiv
#define AES_KEY_BYTES   16  //Number of bytes in the encryption key
#define UNIQUE_ID_BYTES 16  //Number of bytes in the unique ID

#include "settings.h"

//Hardware connections
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//These pins are set in beginBoard()
#define PIN_UNDEFINED   255

uint8_t pin_rst = PIN_UNDEFINED;
uint8_t pin_cs = PIN_UNDEFINED;
uint8_t pin_txen = PIN_UNDEFINED;
uint8_t pin_rxen = PIN_UNDEFINED;
uint8_t pin_dio0 = PIN_UNDEFINED;
uint8_t pin_dio1 = PIN_UNDEFINED;
uint8_t pin_rts = PIN_UNDEFINED;
uint8_t pin_cts = PIN_UNDEFINED;
uint8_t pin_txLED = PIN_UNDEFINED;
uint8_t pin_rxLED = PIN_UNDEFINED;
uint8_t pin_trainButton = PIN_UNDEFINED;
uint8_t pin_rssi1LED = PIN_UNDEFINED;
uint8_t pin_rssi2LED = PIN_UNDEFINED;
uint8_t pin_rssi3LED = PIN_UNDEFINED;
uint8_t pin_rssi4LED = PIN_UNDEFINED;
uint8_t pin_boardID = PIN_UNDEFINED;

uint8_t pin_trigger = PIN_UNDEFINED;

#define ALT_LED_RX_DATA     pin_rssi1LED  //Green
#define ALT_LED_RADIO_LINK  pin_rssi2LED  //Green
#define ALT_LED_RSSI        pin_rssi3LED  //Green
#define ALT_LED_TX_DATA     pin_rssi4LED  //Green
#define ALT_LED_BAD_FRAMES  pin_txLED     //Blue
#define ALT_LED_BAD_CRC     pin_rxLED     //Yellow

#define LED_ON              HIGH
#define LED_OFF             LOW

#define ALT_LED_BLINK_MILLIS    15
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//Radio Library
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
#include <RadioLib.h> //Click here to get the library: http://librarymanager/All#RadioLib v5.1.2
SX1276 radio = NULL; //We can't instantiate here because we don't yet know what pin numbers to use

float *channels;
uint8_t channelNumber = 0;
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//Encryption
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#include <Crypto.h> //From: https://github.com/rweather/arduinolibs
#include <AES.h>
#include <GCM.h>
GCM <AES128> gcm;

uint8_t AESiv[AES_IV_BYTES] = {0}; //Set during hop table generation
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//Buttons - Interrupt driven and debounce
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
#include <JC_Button.h> // http://librarymanager/All#JC_Button
Button *trainBtn = NULL; //We can't instantiate the button here because we don't yet know what pin number to use

const int trainButtonTime = 2000; //ms press and hold before entering training
const int trainWithDefaultsButtonTime = 5000; //ms press and hold before entering training
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//Hardware Timers
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#include "SAMDTimerInterrupt.h" //http://librarymanager/All#SAMD_TimerInterrupt v1.9.0 (currently) by Koi Hang
SAMDTimer channelTimer(TIMER_TCC); //Available: TC3, TC4, TC5, TCC, TCC1 or TCC2
unsigned long timerStart = 0; //Tracks how long our timer has been running since last hop
bool partialTimer = false; //After an ACK we reset and run a partial timer to sync units
const int SYNC_PROCESSING_OVERHEAD = 3; //Number of milliseconds it takes to compute clock deltas before sync'ing clocks

uint16_t petTimeoutHalf = 0; //Half the amount of time before WDT. Helps reduce amount of time spent petting.
unsigned long lastPet = 0; //Remebers time of last WDT pet.
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Global variables - Serial
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint8_t escapeCharacter = '+';
const uint8_t maxEscapeCharacters = 3; //Number of characters needed to enter command mode
const uint8_t responseDelayDivisor = 4; //Add on to max response time after packet has been sent. Factor of 2. 8 is ok. 4 is good. A smaller number increases the delay.

//Buffer to receive serial data from the USB or serial ports
uint16_t rxHead = 0;
uint16_t rxTail = 0;
uint8_t serialReceiveBuffer[1024];

//Buffer to store bytes for transmission via the long range radio
uint16_t radioTxHead = 0;
uint16_t radioTxTail = 0;
uint8_t radioTxBuffer[1024 * 3];

//Buffer to store bytes to be sent to the USB or serial ports
uint16_t txHead = 0;
uint16_t txTail = 0;
uint8_t serialTransmitBuffer[1024 * 4]; //Bytes received from RF waiting to be printed out UART. Buffer up to 1s of bytes at 4k

unsigned long lastByteReceived_ms = 0; //Track when last transmission was. Send partial buffer once time has expired.
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Global variables - Command Processing
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
char commandBuffer[100]; //Received serial gets stored into buffer until \r or \n is received
uint8_t commandRXBuffer[100]; //Bytes received from remote, waiting for printing or AT parsing
uint8_t commandTXBuffer[1024 * 4]; //Bytes waiting to be transmitted to the remote unit
uint16_t commandTXHead = 0;
uint16_t commandTXTail = 0;
uint16_t commandRXHead = 0;
uint16_t commandRXTail = 0;

uint8_t escapeCharsReceived = 0; //Used to enter command mode
unsigned long lastEscapeReceived_ms = 0; //Tracks end of serial traffic
const long minEscapeTime_ms = 2000; //Serial traffic must stop this amount before an escape char is recognized

bool inCommandMode = false; //Normal data is prevented from entering serial output when in command mode
uint8_t commandLength = 0;
bool remoteCommandResponse;

bool rtsAsserted; //When RTS is asserted, host says it's ok to send data

/* Data Flow - Point-to-Point and Multi-Point

                             USB or UART
                                  |
                                  | Flow control: RTS for UART
                                  |      Off: Buffer full
                                  |      On: Buffer drops below half full
                                  V
                         serialReceiveBuffer
                                  |
                                  | inCommandMode?
                                  |
                     true         V        false
                    .-------------+--------------------------.
                    |                                        |
                    V                                        v
              commandBuffer                            radioTxBuffer
                    |                                        |
                    | Remote Command?                        v
                    |                                 outgoingPacket
       false        V         true                           |
      .-------------+-------------.                          V
      |                           |                Send to remote system
      |                           V                          |
      |                    outgoingPacket                    V
      |                           |                   incomingBuffer
      |                           V                          |
      |                 Send to remote system                V
      |                           |                serialTransmitBuffer
      |                           V                          |
      |                    incomingBuffer                    |
      |                           |                          |
      |                           V                          |
      |                    commandRXBuffer                   |
      |                           |                          |
      |                           V                          |
      |                  Command processing                  |
      |                     checkCommand                     |
      |                           |                          |
      |                           V                          |
      |                    commandTXBuffer                   |
      |                           |                          |
      |                           V                          |
      |                    outgoingPacket                    |
      |                           |                          |
      |                           V                          |
      |               Send back to local system              |
      |                           |                          |
      |                           V                          |
      |                    incomingBuffer                    |
      |                           |                          |
      |                           V                          |
      `-------------------------->+<-------------------------'
                                  |
                                  |
                                  V
                             USB or UART

*/

/* Data Flow - Virtual Circuit

                             USB or UART
                                  |
                                  | Flow control: RTS for UART
                                  |      Off: Buffer full
                                  |      On: Buffer drops below half full
                                  V
                         serialReceiveBuffer
                                  |
                                  | Destination VC?
                                  |
                                  V                                    Other
                    .-------------+--------------->+---------------->+------> Discard
         VC_COMMAND |                         myVc |            >= 0 |
                    |                              |    VC_BROADCAST |
                    |                              |                 |
                    V                              |                 v
              commandBuffer                        |           radioTxBuffer
                    |                              |                 |
                    | Remote Command?              |                 v
                    |                              |           outgoingPacket
       false        V         true                 |                 |
      .-------------+-------------.                |                 V
      |                           |                |       Send to remote system
      |                           V                |                 |
      |                    outgoingPacket          |                 V
      |                           |                |          incomingBuffer
      |                           V                |                 |
      |                 Send to remote system      |                 |
      |                           |                |                 |
      |                           V                |                 |
      |                    incomingBuffer          |                 |
      |                           |                |                 |
      |                           V                |                 |
      |                    commandRXBuffer         |                 |
      |                           |                |                 |
      |                           V                |                 |
      |                  Command processing        |                 |
      |                     checkCommand           |                 |
      |                           |                |                 |
      |                           V                |                 |
      |                    commandTXBuffer         |                 |
      |                           |                |                 |
      |                           V                |                 |
      |                    outgoingPacket          |                 |
      |                           |                |                 |
      |                           V                |                 |
      |               Send back to local system    |                 |
      |                           |                |                 |
      |                           V                |                 |
      |                    incomingBuffer          |                 |
      |                           |                |                 |
      |                           V                V                 |
      `-------------------------->+<---------------+<----------------'
                                  |
                                  V
                        serialTransmitBuffer
                                  |
                                  | Flow control: CTS for UART
                                  |
                                  V
                             USB or UART

*/

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//Global variables - LEDs
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
int trainCylonNumber = 0b0001;
int trainCylonDirection = -1;

unsigned long lastTrainBlink = 0; //Controls LED during training
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//Global variables - Radio (General)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
uint8_t outgoingPacket[MAX_PACKET_SIZE]; //Contains the current data in route to receiver
uint8_t packetSize = 0; //Tracks how much data + control trailer
uint16_t frameAirTime = 0; //Recalc'd with each new packet transmission
uint16_t ackAirTime = 0; //Recalc'd with each change of settings
uint8_t frameSentCount = 0; //Increases each time a frame is sent

unsigned long lastPacketReceived = 0; //Controls link LED in broadcast mode
unsigned long lastLinkBlink = 0; //Controls link LED in broadcast mode

volatile bool transactionComplete = false; //Used in dio0ISR
volatile bool timeToHop = false; //Used in dio1ISR
bool expectingAck = false; //Used by various send packet functions

float frequencyCorrection = 0; //Adjust receive freq based on the last packet received freqError

volatile bool clearDIO1 = true; //Clear the DIO1 hop ISR when possible

//RSSI must be above these negative numbers for LED to illuminate
const int rssiLevelLow = -150;
const int rssiLevelMed = -70;
const int rssiLevelHigh = -50;
const int rssiLevelMax = -20;
int rssi; //Signal level

//Link quality metrics
uint32_t datagramsSent;     //Total number of datagrams sent
uint32_t datagramsReceived; //Total number of datagrams received
uint32_t framesSent;        //Total number of frames sent
uint32_t framesReceived;    //Total number of frames received
uint32_t badFrames;         //Total number of bad frames received
uint32_t duplicateFrames;   //Total number of duplicate frames received
uint32_t lostFrames;        //Total number of lost TX frames
uint32_t linkFailures;      //Total number of link failures
uint32_t insufficientSpace; //Total number of times the buffer did not have enough space
uint32_t badCrc;            //Total number of bad CRC frames

unsigned long lastLinkUpTime = 0; //Mark when link was first established
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//Global variables - V2 Protocol
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//Frame size values
uint8_t headerBytes;
uint8_t trailerBytes;
uint8_t txDatagramSize;

//Point-to-Point
unsigned long datagramTimer;
uint16_t pingRandomTime;
uint16_t heartbeatRandomTime;

/* ACK Number Management

            System A                              System B

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

//ACK management
uint8_t rmtTxAckNumber;
uint8_t rxAckNumber;
uint8_t txAckNumber;

//Receive control
uint8_t incomingBuffer[MAX_PACKET_SIZE];
uint8_t minDatagramSize;
uint8_t maxDatagramSize;
uint8_t * rxData;
uint8_t rxDataBytes;
unsigned long heartbeatTimer;
unsigned long linkDownTimer;

//Clock synchronization
unsigned long rcvTimeMillis;
unsigned long xmitTimeMillis;
unsigned long timestampOffset;
unsigned long roundTripMillis;
unsigned long vcTxHeartbeatMillis;

//Transmit control
uint8_t * endOfTxData;
CONTROL_U8 txControl;
uint32_t transmitTimer;

//Multi-point Training
bool trainingServerRunning; //Training server is running
bool trainingPreviousRxInProgress = false; //Previous RX status
float originalChannel; //Original channel from HOP table while training is in progress
uint8_t trainingPartnerID[UNIQUE_ID_BYTES]; //Unique ID of the training partner
uint8_t myUniqueId[UNIQUE_ID_BYTES]; // Unique ID of this system
unsigned long trainingTimer;

//Virtual-Circuit
int8_t cmdVc;   //VC index for ATI commands only
int8_t myVc;
int8_t rxDestVc;
int8_t rxSrcVc;
uint8_t *rxVcData;
int8_t txDestVc;
unsigned long vcAckTimer;
VIRTUAL_CIRCUIT virtualCircuitList[MAX_VC];

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//Global variables
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const Settings defaultSettings;
Settings originalSettings; //Create a duplicate of settings during training so that we can resort as needed

char platformPrefix[25]; //Used for printing platform specific device name, ie "SAMD21 1W 915MHz"
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//Architecture variables
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
void updateRTS(bool assertRTS);

#include "Arch_ESP32.h"
#include "Arch_SAMD.h"
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

void setup()
{
  beginSerial(57600); //Default for debug messages before board begins

  loadSettings(); //Load settings from EEPROM

  beginSerial(settings.serialSpeed);

  systemPrintTimestamp();
  systemPrintln("LRS");

  verifyRadioStateTable(); //Verify that the state table contains all of the states in increasing order

  verifyV2DatagramType(); //Verify that the datagram type table contains all of the datagram types

  arch.uniqueID(myUniqueId); //Get the unique ID

  beginBoard(); //Determine what hardware platform we are running on.

  beginLoRa(); //Start radio

  beginButton(); //Start watching the train button

  beginChannelTimer(); //Setup (but do not start) hardware timer for channel hopping

  arch.beginWDT(); //Start watchdog timer

  updateRTS(true); //Enable serial input

  systemPrintTimestamp();
  systemPrintln("LRS Setup Complete");

  triggerEvent(TRIGGER_RADIO_RESET);
}

void loop()
{
  petWDT();

  updateButton();

  updateSerial(); //Store incoming and print outgoing

  if (settings.alternateLedUsage)
    updateLeds();

  updateRadioState(); //Ping/ack/send packets as needed

  if (clearDIO1) //Allow DIO1 hop ISR but use it only for debug
  {
    clearDIO1 = false;
    radio.clearFHSSInt(); //Clear the interrupt
  }
}
