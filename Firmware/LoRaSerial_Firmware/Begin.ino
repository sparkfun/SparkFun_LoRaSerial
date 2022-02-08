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
  pin_activityLED = 6;
  pin_linkLED = 4;

  //Flow control
  pinMode(pin_rts, OUTPUT);
  digitalWrite(pin_rts, HIGH);

  pinMode(pin_cts, INPUT_PULLUP);

  pinMode(pin_activityLED, OUTPUT);
  digitalWrite(pin_activityLED, LOW);

  pinMode(pin_linkLED, OUTPUT);
  digitalWrite(pin_linkLED, LOW);

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
  pin_cs = 5;
  pin_dio0 = 7; //aka A0
  pin_dio1 = 10; //aka A1
  pin_txen = 2;
  pin_rxen = 3;
  pin_rst = 6;
  pin_cts = 255; //TODO
  pin_rts = 255;
  pin_activityLED = 9;
  pin_linkLED = 8;
  pin_txLED = 255; //TODO
  pin_rxLED = 255;

  pin_trigger = A2;

  //Flow control
  pinMode(pin_rts, OUTPUT);
  digitalWrite(pin_rts, HIGH);

  pinMode(pin_cts, INPUT_PULLUP);

  //LEDs
  pinMode(pin_activityLED, OUTPUT);
  digitalWrite(pin_activityLED, LOW);

  pinMode(pin_linkLED, OUTPUT);
  digitalWrite(pin_linkLED, LOW);

  //Debug
  pinMode(pin_trigger, OUTPUT);
  digitalWrite(pin_trigger, HIGH);

  strcpy(platformPrefix, "SAMD21 1W");
#endif

  //Dashbord Blink LEDs
  for (int x = 0 ; x < 2 ; x++)
  {
    digitalWrite(pin_activityLED, HIGH);
    digitalWrite(pin_linkLED, HIGH);
    digitalWrite(pin_txLED, HIGH);
    digitalWrite(pin_rxLED, HIGH);
    delay(100);

    digitalWrite(pin_activityLED, LOW);
    digitalWrite(pin_linkLED, LOW);
    digitalWrite(pin_txLED, LOW);
    digitalWrite(pin_rxLED, LOW);
    delay(100);
  }
}

void beginLoRa()
{
  radio = new Module(pin_cs, pin_dio0, pin_rst, pin_dio1);

  int state = radio.begin(channels[0]);
  if (state != RADIOLIB_ERR_NONE)
  {
    Serial.print(F("Radio init failed with code: "));
    Serial.println(state);
    while (1); //Hard freeze
  }

  randomSeed(radio.randomByte()); //Puts radio into standy-by state

  configureRadio(); //Apply current settings

  returnToReceiving();

  if (settings.pointToPoint == true)
    changeState(RADIO_NO_LINK_RECEIVING_STANDBY);
  else
    changeState(RADIO_BROADCASTING_RECEIVING_STANDBY);
}

void beginWDT()
{
#if defined(ARDUINO_ARCH_SAMD)
  myWatchDog.setup(WDT_HARDCYCLE250m);  // Initialize WDT with 250ms timeout
#endif  
}
