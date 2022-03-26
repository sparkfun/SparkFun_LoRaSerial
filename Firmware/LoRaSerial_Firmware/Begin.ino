//Based on hardware features, determine which hardware this is
void beginBoard()
{
#if defined(ARDUINO_ARCH_ESP32)
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

#elif defined(ARDUINO_ARCH_SAMD)

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

#endif

  //Dashbord Blink LEDs
  for (int x = 0 ; x < 3 ; x++)
  {
    digitalWrite(pin_rssi1LED, HIGH);
    digitalWrite(pin_rssi2LED, HIGH);
    digitalWrite(pin_rssi3LED, HIGH);
    digitalWrite(pin_rssi4LED, HIGH);
    digitalWrite(pin_txLED, HIGH);
    digitalWrite(pin_rxLED, HIGH);
    delay(50);

    digitalWrite(pin_rssi1LED, LOW);
    digitalWrite(pin_rssi2LED, LOW);
    digitalWrite(pin_rssi3LED, LOW);
    digitalWrite(pin_rssi4LED, LOW);
    digitalWrite(pin_txLED, LOW);
    digitalWrite(pin_rxLED, LOW);
    delay(50);
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
  if (pin_trainButton != 255)
  {
    trainBtn = new Button(pin_trainButton); //Create the button
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

//Delay with pets of WDT when needed
void delayWDT(uint16_t delayAmount)
{
  unsigned long startTime = millis();
  while (millis() - startTime < delayAmount)
  {
    delay(1);
    petWDT();
  }
}

void beginSerial(uint16_t serialSpeed)
{
  Serial.begin(serialSpeed);
#if defined(ARDUINO_ARCH_SAMD)
  Serial1.begin(serialSpeed);
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
