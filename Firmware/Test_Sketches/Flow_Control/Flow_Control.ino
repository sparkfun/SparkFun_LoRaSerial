/*
  October 26 2022
  SparkFun Electronics
  Lee Leahy

  Verify flow control operation

  Compiled with Arduino v1.8.15
*/

#define UNUSED(x) (void)(x)

#define INVERT_RTS_CTS      1

//Hardware connections
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//These pins are set in beginBoard()
#define PIN_UNDEFINED   255

uint8_t pin_rts = PIN_UNDEFINED;
uint8_t pin_cts = PIN_UNDEFINED;
uint8_t pin_txLED = PIN_UNDEFINED;
uint8_t pin_rxLED = PIN_UNDEFINED;
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//Global variables - Serial
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
//Buffer to store bytes incoming from serial before broadcasting over LoRa
uint8_t serialTransmitBuffer[1024 * 1]; //Bytes received from RF waiting to be printed out UART. Buffer up to 1s of bytes at 4k

uint16_t txHead = 0;
uint16_t txTail = 0;

//When RTS is asserted, host says it's ok to send data
bool rtsAsserted;

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

/* Data Flow

                             USB or UART
                                  |
                                  | Flow control: RTS for UART
                                  |      Off: Buffer full
                                  |      On: Buffer drops below half full
                                  V
                        serialTransmitBuffer
                                  |
                                  | Flow control: CTS for UART
                                  |
                                  V
                             USB or UART

*/

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//Architecture variables
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
#include "Arch_SAMD.h"
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

void setup()
{
  samdBeginSerial(57600);
  samdBeginBoard();
  randomSeed(5);

  //Enable the flow of data from the remote serial port into the SAMD
  updateRTS(true);
}

void loop()
{
  updateSerial(); //Store incoming and print outgoing
}

void systemPrint(int value)
{
  char string[20];
  sprintf(string, "%d", value);
  SerialUSB.print(string);
}

void systemPrint(const char * string)
{
  SerialUSB.print(string);
}

void systemPrintln()
{
  SerialUSB.print("\r\n");
}
