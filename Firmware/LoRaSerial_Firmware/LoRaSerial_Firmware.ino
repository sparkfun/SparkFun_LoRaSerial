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

#define CHANNEL_TIMER_BYTES sizeof(uint16_t) //Number of bytes used within in control header for clock sync (uint16_t msToNextHop)
#define CLOCK_MILLIS_BYTES sizeof(unsigned long) //Number of bytes used within in various packets for system timestamps sync (unsigned long currentMillis)
#define SYNC_CLOCKS_BYTES   (sizeof(uint8_t) + sizeof(unsigned long)) //Number of data bytes in the SYNC_CLOCKS frame
#define ZERO_ACKS_BYTES     sizeof(unsigned long) //Number of data bytes in the ZERO_ACKS frame
#define MAX_PACKET_SIZE 255 //Limited by SX127x
#define AES_IV_BYTES    12  //Number of bytes for AESiv
#define AES_KEY_BYTES   16  //Number of bytes in the encryption key
#define UNIQUE_ID_BYTES 16  //Number of bytes in the unique ID

//Bit 0: Signal Detected indicates that a valid LoRa preamble has been detected
//Bit 1: Signal Synchronized indicates that the end of Preamble has been detected, the modem is in lock
//Bit 3: Header Info Valid toggles high when a valid Header (with correct CRC) is detected
#define RECEIVE_IN_PROCESS_MASK   0b1011

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

#define GREEN_LED_1         pin_rssi1LED
#define GREEN_LED_2         pin_rssi2LED
#define GREEN_LED_3         pin_rssi3LED
#define GREEN_LED_4         pin_rssi4LED
#define BLUE_LED            pin_txLED
#define YELLOW_LED          pin_rxLED

#define RADIO_USE_BLINK_MILLIS    15

#define RADIO_USE_RX_DATA_LED     GREEN_LED_1 //Green
#define RADIO_USE_LINK_LED        GREEN_LED_2 //Green
#define RADIO_USE_RSSI_LED        GREEN_LED_3 //Green
#define RADIO_USE_TX_DATA_LED     GREEN_LED_4 //Green
#define RADIO_USE_BAD_FRAMES_LED  BLUE_LED    //Blue
#define RADIO_USE_BAD_CRC_LED     YELLOW_LED  //Yellow

#define CYLON_TX_DATA_LED   BLUE_LED
#define CYLON_RX_DATA_LED   YELLOW_LED

#define LED_ON              HIGH
#define LED_OFF             LOW
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
bool trainViaButton = false; //Allows auto-creation of server if client times out
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//Hardware Timers
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#include "SAMDTimerInterrupt.h" //http://librarymanager/All#SAMD_TimerInterrupt v1.9.0 (currently) by Koi Hang
SAMDTimer channelTimer(TIMER_TCC); //Available: TC3, TC4, TC5, TCC, TCC1 or TCC2
unsigned long timerStart = 0; //Tracks how long our timer has been running since last hop
bool partialTimer = false; //After an ACK we reset and run a partial timer to sync units

uint16_t petTimeout = 0; //A reduced amount of time before WDT triggers. Helps reduce amount of time spent petting.
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
bool forceRadioReset = false; //Goes true when a setting requires a link/radio reset to work
bool writeOnCommandExit = false; //Goes true if user specifies ATW command

/* Data Flow - Point-to-Point and Multi-Point

                             USB or UART
                                  |
                                  | Flow control: RTS for UART
                                  |      Off: Buffer full
                                  |      On: Buffer drops below half full
                                  | updateSerial
                                  V
                         serialReceiveBuffer
                                  |
                                  | processSerialInput
                                  |
                                  | inCommandMode or local VC command?
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
      |                 Send to remote system                |
      |                           |                          |
      |                           V                          |
      |                    incomingBuffer                    |
      |                           |                          |
      |                           V                          |
      |                    commandRXBuffer                   |
      |                           |                          |
      |                           V          VC Remote Cmd   V
      |                     commandBuffer <------------------+
      |                           |                          |
      |                           V                          |
      |                  Command processing                  |
      |                     checkCommand                     | Data or
      |                           |                          | VC Data or
      |                           V                          | VC Rmt Cmd Resp
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
                         serialTransmitBuffer
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
                                  | updateSerial
                                  V
                         serialReceiveBuffer
                                  |
                                  | vcProcessSerialInput
                                  |
                                  | Destination VC?
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
uint16_t radioBand = 0; //In MHz. Detected radio module type. Sets the upper/lower frequency bounds.
uint8_t outgoingPacket[MAX_PACKET_SIZE]; //Contains the current data in route to receiver
uint16_t frameAirTime = 0; //Recalc'd with each new packet transmission
uint16_t ackAirTime = 0; //Recalc'd with each change of settings
uint16_t maxPacketAirTime = 0; //Recalc'd with each change of settings
uint8_t frameSentCount = 0; //Increases each time a frame is sent

unsigned long lastPacketReceived = 0; //Controls link LED in broadcast mode
unsigned long lastLinkBlink = 0; //Controls link LED in broadcast mode

volatile bool transactionComplete = false; //Used in dio0ISR
volatile bool timeToHop = false; //Used in dio1ISR
uint8_t sf6ExpectedSize = MAX_PACKET_SIZE; //Used during SF6 operation to reduce packet size when needed

float radioFrequency; //Current radio frequency
float frequencyCorrection = 0; //Adjust receive freq based on the last packet received freqError

volatile bool hop = true; //Clear the DIO1 hop ISR when possible

//RSSI must be above these negative numbers for LED to illuminate
const int rssiLevelLow = -150;
const int rssiLevelMed = -120;
const int rssiLevelHigh = -100;
const int rssiLevelMax = -70;
int rssi; //Average signal level, measured during reception of a packet

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
uint32_t netIdMismatch;     //Total number of mismatched Net ID frames

unsigned long lastLinkUpTime = 0; //Mark when link was first established
unsigned long lastRxDatagram; //Remember last valid receive

//Receiver and Transmitter status
unsigned long rxSuccessMillis;
unsigned long rxFailureMillis;
int rxFailureState;

//Receive failures
unsigned long startReceiveFailureMillis;
int startReceiveFailureState;
unsigned long lastReceiveInProcessTrue;
uint8_t lastModemStatus;

unsigned long txSuccessMillis;
unsigned long txFailureMillis;
int txFailureState;

//History
unsigned long radioCallHistory[RADIO_CALL_MAX];
unsigned long radioStateHistory[RADIO_MAX_STATE];

uint8_t packetLength = 0; //Total bytes received, used for calculating clock sync times in multi-point mode
int16_t msToNextHopRemote; //Can become negative

bool requestYield = false; //Datagram sender can tell this radio to stop transmitting to enable two-way comm
unsigned long yieldTimerStart = 0;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//Global variables - Radio Protocol
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//Frame size values
uint8_t headerBytes;
uint8_t trailerBytes;
uint8_t txDatagramSize;

//Point-to-Point
unsigned long datagramTimer;
uint16_t randomTime;
uint16_t heartbeatRandomTime;

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
unsigned long nextChannelZeroTimeInMillis;

//Transmit control
uint8_t * endOfTxData;
CONTROL_U8 txControl;
unsigned long ackTimer;

//Retransmit support
uint8_t rmtCmdVc;
uint8_t rexmtBuffer[MAX_PACKET_SIZE];
CONTROL_U8 rexmtControl;
uint8_t rexmtLength;
uint8_t rexmtFrameSentCount;
uint8_t rexmtTxDestVc;

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
VIRTUAL_CIRCUIT virtualCircuitList[MAX_VC];
uint8_t serialOperatingMode;
uint32_t vcConnecting;

unsigned int multipointChannelLoops = 0; //Count the number of times Multipoint scanning has traversed the table
unsigned int multipointAttempts = 0; //Throttle back scanning when a server is not detected

unsigned long retransmitTimeout = 0; //Throttle back re-transmits

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//Global variables
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const Settings defaultSettings;
Settings tempSettings; //Create a duplicate of settings during training so that we can resort as needed
uint8_t originalEncryptionKey[AES_KEY_BYTES] = {0}; //Temp store key if we need to exit button training
uint8_t originalNetID = 0; //Temp store ID if we need to exit button training
bool originalServer = false; //Temp store server setting if we need to exit button training

char platformPrefix[25]; //Used for printing platform specific device name, ie "SAMD21 1W 915MHz"
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

/*
          Server                                Client

                  HEARTBEAT send needed
          call xmitDatagramP2PHeartbeat
    .--------------------  Get millis()
    |     Add millis to HEARTBEAT frame
    |  *                  USB Interrupt
    |  *        hopChannel if necessary
    |                      Software CRC
    | txTimeUsec        Data encryption
    |                    Data whitening
    |        Move data to radio via SPI
    |         xmitTimeMillis = millis()
    |  *          Radio arbitrary delay
    |                      TX HEARTBEAT  - - >  Detect RX in process
    |                               ...         ...
    |                           TX done         RX HEARTBEAT
    |  *          Radio arbitrary delay         Radio arbitrary delay           *
    '---  Transaction complete detected         USB Interrupt                   *
                                      .-------  DIO0 Interrupt
                                      |         Delay - finish previous loop
                           rxTimeUsec |         Transaction complete detected
                                      |         call receiveDatagram
                                      |         USB Interrupt
                                      +-------  Get millis()
                                                Move data from radio using SPI
                                                Data whitening
                                                Data decryption
                                                Software CRC
                                                Start processing HEARTBEAT datagram
                                                Get millis();

                                                CPU Clock drift

      timeStampOffset = TX millis + txTimeUsec + rxTimeUsec

                                    RX uSec   RX Var   TX uSec   Hop uSec
      None                            1379      216      1970       22
      Encryption                      1957      155      2540       13
      CRC                             1387      178      1986        5
      CRC + Encryption                1960      223      2557       15
      Whitening                       1393      231      1993       28
      Whitening + Encryption          1963      263      2562       22
      Whitening + CRC                 1415      249      2014       13
      Whitening + CRC + Encryption    1987      220      2584       27

      Options         RX      TX
      CRC:             8      16   uSec
      Encryption:    578     570   uSec
      CRC + Encr:    581     587   uSec
      Whitening       14      15   uSec
      White + Enc:   584     592   uSec
      White + CRC:    36      44   uSec
      Wh + CRC + En: 608     614   uSec

*/

//Clock synchronization
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//RX and TX time measurements
uint32_t rxTimeUsec;
uint32_t txTimeUsec;

uint32_t transactionCompleteMicros; //Timestamp at the beginning of the transactionCompleteIsr routine
uint32_t txDatagramMicros; //Timestamp at the beginning of the transmitDatagram routine
uint16_t maxFrameAirTime; //Air time of the maximum sized message

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//Architecture variables
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
void updateRTS(bool assertRTS);

#include "Arch_ESP32.h"
#include "Arch_SAMD.h"
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//Initial entrypoint following any runtime library initialization
void setup()
{
  int index;

  beginSerial(57600); //Default for debug messages before board begins

  arch.beginBoard(); //Initialize the board specific hardware, and ID platform type

  loadSettings(); //Load settings from EEPROM
  serialOperatingMode = settings.operatingMode;

  beginSerial(settings.serialSpeed);

  verifyTables(); //Verify that the enum counts match the name table lengths

  //Load the unique IDs for the virtual circuits
  //Always hand out the same VC number for a given unique ID
  if (settings.server)
  {
    for (index = 0; index < MAX_VC; index++)
      nvmLoadVcUniqueId(index);
  }

  arch.uniqueID(myUniqueId); //Get the unique ID

  beginLoRa(); //Start radio

  beginButton(); //Start watching the train button

  beginChannelTimer(); //Setup (but do not start) hardware timer for channel hopping

  arch.beginWDT(); //Start watchdog timer

  updateRTS(true); //Enable serial input

  systemPrintTimestamp();
  systemPrintln("LRS");
  outputSerialData(true);

  triggerEvent(TRIGGER_RADIO_RESET);

  blinkStartup(); //Blink LEDs to indicate the completion of system setup
}

//Idle loop for the CPU
void loop()
{
  petWDT();

  updateButton(); //Check if train button is pressed

  updateSerial(); //Store incoming and print outgoing

  updateRadioState(); //Send packets as needed for handshake, data, remote commands

  updateLeds(); //Update the LEDs on the board

  updateHopISR(); //Clear hop ISR as needed
}
