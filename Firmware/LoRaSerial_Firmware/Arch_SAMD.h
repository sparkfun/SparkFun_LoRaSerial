#if defined(ARDUINO_ARCH_SAMD)
#ifndef __SAMD_H__

#include <FlashAsEEPROM_SAMD.h> //Click here to get the library: http://librarymanager/All#FlashStorage_SAMD21 v1.2.1 by Khoi Hoang
#include <WDTZero.h> //https://github.com/javos65/WDTZero
WDTZero myWatchDog;

void samdBeginBoard()
{
  //Use ADC to check resistor divider
  pin_boardID = A2;

  pin_cs = 5;
  pin_dio0 = 7; //aka A0
  pin_dio1 = 10; //aka A1
  pin_txen = 2;
  pin_rxen = 3;
  pin_rst = 6;
  pin_cts = 30;
  pin_rts = 38;
  pin_txLED = 31;
  pin_rxLED = A5;
  pin_rssi1LED = A3;
  pin_rssi2LED = A4;
  pin_rssi3LED = 8;
  pin_rssi4LED = 9;

  pin_trainButton = 4;

  pin_trigger = A0;

  //Flow control
  pinMode(pin_rts, OUTPUT);
  digitalWrite(pin_rts, HIGH);

  pinMode(pin_cts, INPUT_PULLUP);

  //LEDs
  pinMode(pin_rssi1LED, OUTPUT);
  digitalWrite(pin_rssi1LED, LOW);
  pinMode(pin_rssi2LED, OUTPUT);
  digitalWrite(pin_rssi2LED, LOW);
  pinMode(pin_rssi3LED, OUTPUT);
  digitalWrite(pin_rssi3LED, LOW);
  pinMode(pin_rssi4LED, OUTPUT);
  digitalWrite(pin_rssi4LED, LOW);

  pinMode(pin_txLED, OUTPUT);
  digitalWrite(pin_txLED, LOW);
  pinMode(pin_rxLED, OUTPUT);
  digitalWrite(pin_rxLED, LOW);

  //Train button input
  pinMode(pin_trainButton, INPUT_PULLUP);

  //Debug
  pinMode(pin_trigger, OUTPUT);
  digitalWrite(pin_trigger, HIGH);

  //Get average of board ID voltage divider
  int val = 0;
  for (int x = 0 ; x < 8 ; x++)
    val += analogRead(pin_boardID);
  val /= 8;

  //Convert ADC to volts
  float boardID = 3.3 * val / 1024;

  //Use ADC to check board ID resistor divider
  if (boardID > 1.64 * 0.9 && boardID < 1.64 * 1.1)
  {
    strcpy(platformPrefix, "SAMD21 1W 915MHz");
  }
  else
  {
    strcpy(platformPrefix, "SAMD21 1W");
  }
}

void samdBeginSerial(uint16_t serialSpeed)
{
  Serial1.begin(serialSpeed);

#if defined(ENABLE_DEVELOPER)
  //Wait for serial to come online for debug printing
  while (!Serial);
#endif  //ENABLE_DEVELOPER
}

void samdBeginWDT()
{
  myWatchDog.setup(WDT_HARDCYCLE250m);  // Initialize WDT with 250ms timeout
  petTimeoutHalf = 250 / 2;
}

void samdEepromBegin()
{
}

void samdEepromCommit()
{
  EEPROM.commit();
}

//Perform the necessary action to "pet" the watch dog timer
void samdPetWDT()
{
  //This takes 4-5ms to complete
  myWatchDog.clear();
}

Module * samdRadio()
{
  return new Module(pin_cs, pin_dio0, pin_rst, pin_dio1);
}

bool samdSerialAvailable()
{
  return (Serial.available() || Serial1.available());
}

void samdSerialFlush()
{
  Serial.flush();
  Serial1.flush();
}

void samdSerialPrint(const char * value)
{
  Serial.print(value);
  Serial1.print(value);
}

uint8_t samdSerialRead()
{
  byte incoming = 0;
  if (Serial.available())
    incoming = Serial.read();
  else if (Serial1.available())
    incoming = Serial1.read();
  return (incoming);
}

void samdSerialWrite(uint8_t value)
{
  Serial.write(value);
  Serial1.write(value);
}

void samdSystemReset()
{
  NVIC_SystemReset();
}

const ARCH_TABLE arch = {
  samdBeginBoard,             //beginBoard
  samdBeginSerial,            //beginSerial
  samdBeginWDT,               //beginWDT
  samdEepromBegin,            //eepromBegin
  samdEepromCommit,           //eepromCommit
  samdPetWDT,                 //petWDT
  samdRadio,                  //radio
  samdSerialAvailable,        //serialAvailable
  samdSerialFlush,            //serialFlush
  samdSerialPrint,            //serialPrint
  samdSerialRead,             //serialRead
  samdSerialWrite,            //serialWrite
  samdSystemReset             //systemReset
};

#endif  //__SAMD_H__
#endif  //ARDUINO_ARCH_SAMD
