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
  pin_rst = 32;
  pin_cts = 255;
  pin_rts = 255;

  pin_trigger = 13;

  pin_setupButton = 0;

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

  float centerFreq = (settings.frequencyMax - settings.frequencyMin) / 2;
  centerFreq += settings.frequencyMin;

  int state = radio.begin(centerFreq); //Doesn't matter what freq we start at
  if (state != RADIOLIB_ERR_NONE)
  {
    systemPrint("Radio init failed with code: ");
    systemPrintln(state);
    while (1)
    {
      petWDT();
      delay(100);
    }
  }

  uint8_t radioSeed = radio.randomByte(); //Puts radio into standy-by state
  randomSeed(radioSeed);
  if (settings.debug == true)
  {
    systemPrint("RadioSeed: ");
    systemPrintln(radioSeed);
  }

  generateHopTable(); //Generate frequency table based on randomByte

  configureRadio(); //Generate freq table, setup radio, go to receiving, change state to standby

  returnToReceiving();

  if (settings.pointToPoint == true)
    changeState(RADIO_NO_LINK_RECEIVING_STANDBY);
  else
    changeState(RADIO_BROADCASTING_RECEIVING_STANDBY);
}

void beginButton()
{
  if (pin_setupButton != 255)
  {
    trainBtn = new Button(pin_setupButton); //Create the button
    trainBtn->begin();
  }
}

void beginWDT()
{
#if defined(ARDUINO_ARCH_SAMD)
  myWatchDog.setup(WDT_HARDCYCLE250m);  // Initialize WDT with 250ms timeout
  petTimeoutHalf = 250 / 2;
#elif defined(ARDUINO_ARCH_ESP32)
  petTimeoutHalf = 1000 / 2;
#endif
}

void beginSerial(uint16_t serialSpeed)
{
  Serial.begin(settings.serialSpeed);
#if defined(ARDUINO_ARCH_SAMD)
  Serial1.begin(settings.serialSpeed);
#endif

#if defined(ENABLE_DEVELOPER)
  //Wait for serial to come online for debug printing
#if defined(ARDUINO_ARCH_ESP32)
  delay(500);
#elif defined(ARDUINO_ARCH_SAMD)
  while (!Serial);
#endif
#endif
}

void petWDT()
{
  //Petting the dog takes a really long time on the SAMD21, 4-5ms to complete
  //so we don't always clear it, only after we've passed the half way point
  //Similarly for ESP32, we don't want to delay 1ms every loop of Serial.available()
  if (millis() - lastPet > petTimeoutHalf)
  {
    lastPet = millis();

#if defined(ARDUINO_ARCH_SAMD)
    myWatchDog.clear();
#elif defined(ARDUINO_ARCH_ESP32)
    delay(1);
#endif

  }

}
