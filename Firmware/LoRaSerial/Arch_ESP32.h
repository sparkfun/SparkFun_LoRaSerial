#if defined(ARDUINO_ARCH_ESP32)
#ifndef __ESP32_H__

#include <EEPROM.h>
#define EEPROM_SIZE 1024 //ESP32 emulates EEPROM in non-volatile storage (external flash IC). Max is 508k.

#define NVM_ERASE_VALUE         0xff
#define NVM_UNIQUE_ID_OFFSET    (EEPROM_SIZE - (MAX_VC * UNIQUE_ID_BYTES))

#define PET_TIMEOUT             500 // Milliseconds

/*
  Data flow
                   +--------+
                   | ESP32  |
                   |        |       +--------------+
                   |    SPI |<----->| SX1276 Radio |<---> Antenna
                   |        |       +--------------+         ^
    USB Serial <-->| Serial |                                |
                   +--------+                                |
                                                             |
                   +--------+                                |
                   | ESP32  |                                |
                   |        |       +--------------+         V
                   |    SPI |<----->| SX1276 Radio |<---> Antenna
                   |        |       +--------------+
    USB Serial <-->| Serial |
                   +--------+
*/

//Initialize the LoRaSerial board
void esp32BeginBoard()
{
  //Lower power boards
  pin_cs = 15;
  pin_dio0 = 26; //aka A0
  pin_dio1 = 25; //aka A1
  pin_rst = 32;

  pin_trigger = 13;

  pin_trainButton = 0;

  //Debug
  pinMode(pin_trigger, OUTPUT);
  digitalWrite(pin_trigger, HIGH);

  strcpy(platformPrefix, "ESP32 100mW");
}

//Initialize the USB serial port
void esp32BeginSerial(uint16_t serialSpeed)
{
  if (settings.usbSerialWait)
    //Wait for serial to come online for debug printing
    delay(500);
}

//Initialize the watch dog timer
void esp32BeginWDT()
{
}

//Initilaize the EEPROM controller or simulation
void esp32EepromBegin()
{
  EEPROM.begin(EEPROM_SIZE);
}

//Write any remaining data to EEPROM
void esp32EepromCommit()
{
  EEPROM.commit();
}

//Perform the necessary action to "pet" the watch dog timer
void esp32PetWDT()
{
  delay(1);
}

//Initialize the radio module
Module * esp32Radio()
{
  return new Module(pin_cs, pin_dio0, pin_rst, pin_dio1);
}

//Determine if serial input data is available
bool esp32SerialAvailable()
{
  return Serial.available();
}

//Ensure that all serial output data has been sent over USB
void esp32SerialFlush()
{
  Serial.flush();
}

//Read in the serial input data
uint8_t esp32SerialRead()
{
  return (Serial.read());
}

//Provide the serial output data to the USB layer or the UART TX FIFO
void esp32SerialWrite(uint8_t value)
{
  Serial.write(value);
}

//Reset the CPU
void esp32SystemReset()
{
  ESP.restart();
}

//Get the CPU's unique ID value
void esp32UniqueID(uint8_t * unique128_BitID)
{
  memset(unique128_BitID, 0, UNIQUE_ID_BYTES);
  esp_read_mac(unique128_BitID, ESP_MAC_WIFI_STA);
}

//Provide the hardware abstraction layer (HAL) interface
const ARCH_TABLE arch = {
  esp32BeginBoard,          //beginBoard
  esp32BeginSerial,         //beginSerial
  esp32BeginWDT,            //beginWDT
  esp32EepromBegin,         //eepromBegin
  esp32EepromCommit,        //eepromCommit
  esp32PetWDT,              //petWDT
  esp32Radio,               //radio
  esp32SerialAvailable,     //serialAvailable
  esp32SerialFlush,         //serialFlush
  esp32SerialRead,          //serialRead
  esp32SerialWrite,         //serialWrite
  esp32SystemReset,         //systemReset
  esp32UniqueID,            //uniqueID
};

#endif  //__ESP32_H__
#endif  //ARDUINO_ARCH_ESP32
