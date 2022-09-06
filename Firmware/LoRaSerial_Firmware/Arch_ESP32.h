#if defined(ARDUINO_ARCH_ESP32)
#ifndef __ESP32_H__

#include <EEPROM.h>
#define EEPROM_SIZE 1024 //ESP32 emulates EEPROM in non-volatile storage (external flash IC). Max is 508k.

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

void esp32BeginSerial(uint16_t serialSpeed)
{
#if defined(ENABLE_DEVELOPER)
  //Wait for serial to come online for debug printing
  delay(500);
#endif  //ENABLE_DEVELOPER
}

void esp32BeginWDT()
{
  petTimeoutHalf = 1000 / 2;
}

void esp32EepromBegin()
{
  EEPROM.begin(EEPROM_SIZE);
}

void esp32EepromCommit()
{
  EEPROM.commit();
}

//Perform the necessary action to "pet" the watch dog timer
void esp32PetWDT()
{
  delay(1);
}

Module * esp32Radio()
{
  return new Module(pin_cs, pin_dio0, pin_rst, pin_dio1);
}

bool esp32SerialAvailable()
{
  return Serial.available();
}

void esp32SerialFlush()
{
  Serial.flush();
}

void esp32SerialPrint(const char * value)
{
  Serial.print(value);
}

uint8_t esp32SerialRead()
{
  return (Serial.read());
}

void esp32SerialWrite(uint8_t value)
{
  Serial.write(value);
}

void esp32SystemReset()
{
  ESP.restart();
}

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
  esp32SerialPrint,         //serialPrint
  esp32SerialRead,          //serialRead
  esp32SerialWrite,         //serialWrite
  esp32SystemReset          //systemReset
};

#endif  //__ESP32_H__
#endif  //ARDUINO_ARCH_ESP32
