#if defined(ARDUINO_ARCH_SAMD)
#ifndef __SAMD_H__

#include <FlashAsEEPROM_SAMD.h> //Click here to get the library: http://librarymanager/All#FlashStorage_SAMD21 v1.2.1 by Khoi Hoang
#include <WDTZero.h> //https://github.com/javos65/WDTZero
WDTZero myWatchDog;

/*
  Data flow
                   +--------------+
                   |     SAMD     |
                   |              |
    TTL Serial <-->| Serial1      |       +--------------+
                   |          SPI |<----->| SX1276 Radio |<---> Antenna
    USB Serial <-->| Serial       |       +--------------+         ^
                   +--------------+                                |
                                                                   |
                   +--------------+                                |
                   |     SAMD     |                                |
                   |              |                                |
    TTL Serial <-->| Serial1      |       +--------------+         V
                   |          SPI |<----->| SX1276 Radio |<---> Antenna
    USB Serial <-->| Serial       |       +--------------+
                   +--------------+

                          +--------------+
                          | SAMD         |
                          |              |
                          |      PB02 A5 |---> rxLED
                          |      PB23 31 |---> txLED
                          |              |
                          |      PA07  9 |---> rssi4LED
                          |      PA06  8 |---> rssi3LED
                          |      PA05 A4 |---> rssi2LED
                          |      PA04 A3 |---> rssi1LED
                          |              |
    USB_D- <------------->| 28 PA24      |
    USB_D+ <------------->| 29 PA25      |
                          |              |
                          |      PA15  5 |---> cs -----> LORA_CS/
                          |      PA17 13 |---> SCLK ---> SPI_SCK
                          |      PA16 11 |---> MOSI ---> SPI_PICO
                          |      PA19 12 |<--- MISO ---> SPI_POCI
                          |              |
    RTS-0 <------ rts <---| 38 PA13      |
    TX-0 <------- tx <----| 0  PA10      |
    RX-I_LV <---- rx ---->| 1  PA11      |
    CTS-I_LV <--- cts --->| 30 PB22      |
                          |              |
                          |      PA09  3 |---> rxen ---> LORA_RXEN
                          |      PA14  2 |---> txen ---> LORA_TXEN
                          |              |
                          |      PA18 10 |<--- dio1 <--- LORA_D1 (Freq Change)
                          |      PA21  7 |<--- dio0 <--- LORA_D0 (TX Done)
                          |              |
                          |      PA20  6 |---> rst ----> LORA_RST/
                          |              |
                          |      PA23 21 |---> SCL
                          |      PA22 20 |---> SDA
                          +--------------+
*/

void samdBeginBoard()
{
  //Use ADC to check resistor divider
  pin_boardID = A2;

  pin_cs = 5;
  pin_dio0 = 7;  //aka A0
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

void samdUniqueID(uint32_t * unique128_BitID)
{
  //Read the CPU's unique ID value
  unique128_BitID[0] = *(uint32_t *)0x0080a00c;
  unique128_BitID[1] = *(uint32_t *)0x0080a040;
  unique128_BitID[2] = *(uint32_t *)0x0080a044;
  unique128_BitID[3] = *(uint32_t *)0x0080a048;
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
  samdSystemReset,            //systemReset
  samdUniqueID,               //uniqueID
};

#endif  //__SAMD_H__
#endif  //ARDUINO_ARCH_SAMD
