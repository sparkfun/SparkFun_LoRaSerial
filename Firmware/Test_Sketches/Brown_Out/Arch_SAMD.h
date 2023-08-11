#if defined(ARDUINO_ARCH_SAMD)
#ifndef __SAMD_H__

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
  //Flow control
  pinMode(pin_rts, OUTPUT);
  digitalWrite(pin_rts, INVERT_RTS_CTS ? HIGH : LOW); //Don't give me more

  pinMode(pin_cts, INPUT_PULLUP);

  //LEDs
  pinMode(pin_blue_LED, OUTPUT);
  digitalWrite(pin_blue_LED, LOW);
}

void samdBeginSerial(uint16_t serialSpeed)
{
  //Wait for serial to come online for debug printing
  SerialUSB.begin(serialSpeed);
  while (!SerialUSB);
}

bool samdSerialAvailable()
{
  return (SerialUSB.available() || Serial1.available());
}

void samdSerialFlush()
{
  SerialUSB.flush();
}

void samdSerialPrint(const char * value)
{
  SerialUSB.print(value);
}

uint8_t samdSerialRead()
{
  byte incoming = 0;
  if (SerialUSB.available())
    incoming = SerialUSB.read();
  return (incoming);
}

void samdSerialWrite(uint8_t value)
{
  SerialUSB.write(value);
}

#endif  //__SAMD_H__
#endif  //ARDUINO_ARCH_SAMD
