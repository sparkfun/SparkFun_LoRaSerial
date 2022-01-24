//Based on hardware features, determine which hardware this is
void beginBoard()
{
  //Use ADC to check resistor divider
#if defined(ARDUINO_AVR_UNO)
  pin_cs = 7;
  pin_dio0 = 3;
  pin_dio1 = 2;
  pin_txen = 5;
  pin_rxen = 8;
  pin_rst = A2;
  pin_cts = 9;
  pin_rts = 10;
  pin_act = 6;
  pin_link = 4;

  //Flow control
  pinMode(pin_rts, OUTPUT);
  digitalWrite(pin_rts, HIGH);

  pinMode(pin_cts, INPUT_PULLUP);

  pinMode(pin_act, OUTPUT);
  digitalWrite(pin_act, LOW);

  pinMode(pin_link, OUTPUT);
  digitalWrite(pin_link, LOW);

  strcpy(platformPrefix, "ATMEGA328 1W");

#elif defined(ARDUINO_ARCH_ESP32)
  //Lower power boards
  pin_cs = 15;
  pin_dio0 = 26; //aka A0
  pin_dio1 = 25; //aka A1
  pin_txen = 255; //Not used
  pin_rxen = 255;
  pin_rst = 255;
  pin_cts = 255;
  pin_rts = 255;

  pin_trigger = 13;

  //Debug
  pinMode(pin_trigger, OUTPUT);
  digitalWrite(pin_trigger, HIGH);

  strcpy(platformPrefix, "ESP32 100mW");

#elif defined(ARDUINO_ARCH_SAMD)
  //Proto hardware TODO
  pin_cs = 5;
  pin_dio0 = 7; //aka A0
  pin_dio1 = 10; //aka A1
  pin_txen = 2;
  pin_rxen = 3;
  pin_rst = 6;
  pin_cts = 255; //TODO
  pin_rts = 255;
  pin_act = 9;
  pin_link = 8;

  pin_trigger = A2;
                                   
  //Flow control
  pinMode(pin_rts, OUTPUT);
  digitalWrite(pin_rts, HIGH);

  pinMode(pin_cts, INPUT_PULLUP);

  //LEDs
  pinMode(pin_act, OUTPUT);
  digitalWrite(pin_act, LOW);

  pinMode(pin_link, OUTPUT);
  digitalWrite(pin_link, LOW);

  //Debug
  pinMode(pin_trigger, OUTPUT);
  digitalWrite(pin_trigger, HIGH);

  strcpy(platformPrefix, "SAMD21 1W");
#endif
}

void beginLoRa()
{
  radio = new Module(pin_cs, pin_dio0, pin_rst, pin_dio1);

  int state = radio.begin(channels[0]);
  if (state != RADIOLIB_ERR_NONE)
  {
    Serial.print(F("Radio init failed with code: "));
    Serial.println(state);
    while(1); //Hard freeze
  }

  configureRadio(); //Apply current settings

  returnToReceiving();
  int seed = radio.randomByte();
  randomSeed(seed);
  LRS_DEBUG_PRINT("Ping timeout seed: ");
  LRS_DEBUG_PRINTLN(seed);
  changeState(RADIO_NO_LINK_RECEIVING_STANDBY);
}
