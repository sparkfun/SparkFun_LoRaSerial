//=========================================================================================
//  Begin.ino
//=========================================================================================

//Blink LEDs to indicate the completion of system setup
void blinkStartup()
{
  for (int x = 0 ; x < 3 ; x++)
  {
    digitalWrite(pin_green_1_LED, HIGH);
    digitalWrite(pin_green_2_LED, HIGH);
    digitalWrite(pin_green_3_LED, HIGH);
    digitalWrite(pin_green_4_LED, HIGH);
    digitalWrite(pin_blue_LED, HIGH);
    digitalWrite(pin_yellow_LED, HIGH);
    delay(50);

    digitalWrite(pin_green_1_LED, LOW);
    digitalWrite(pin_green_2_LED, LOW);
    digitalWrite(pin_green_3_LED, LOW);
    digitalWrite(pin_green_4_LED, LOW);
    digitalWrite(pin_blue_LED, LOW);
    digitalWrite(pin_yellow_LED, LOW);
    delay(50);
  }
}

//=========================================================================================

//Initialize the radio layer
void beginLoRa()
{
  radio = arch.radio();

  float centerFreq = (settings.frequencyMax - settings.frequencyMin) / 2;
  centerFreq += settings.frequencyMin;

  int state = radio.begin(centerFreq); //Doesn't matter what freq we start at
  if (state != RADIOLIB_ERR_NONE)
  {
    systemPrint("Radio init failed with code: ");
    systemPrintln(state);
    waitForever("Radio init failed!");
  }

  changeState(RADIO_RESET);
}

//=========================================================================================

//Initialize the button driver
void beginButton()
{
  if (pin_trainButton != PIN_UNDEFINED)
  {
    trainBtn = new Button(pin_trainButton); //Create the button
    trainBtn->begin();
  }
}

//=========================================================================================

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

//=========================================================================================

//Initialize the serial drivers
void beginSerial(uint16_t serialSpeed)
{
  Serial.begin(serialSpeed);
  arch.beginSerial(serialSpeed);
}

//=========================================================================================

//Ensure the watch dog timer does not fire which would cause a CPU hardware reset
void petWDT()
{
  static unsigned long lastPet;

  //Reduce calls to pet the watchdog
  if (millis() - lastPet > PET_TIMEOUT)
  {
    lastPet = millis();
    arch.petWDT();
  }
}

//=========================================================================================

//Start the timer measuring the dwell interval and indicating that it is time to
//hop channels
void beginChannelTimer()
{
  if (channelTimer.attachInterruptInterval_MS(settings.maxDwellTime, channelTimerHandler) == false)
    systemPrintln("Error starting ChannelTimer!");

  stopChannelTimer(); //Start timer in state machine - beginChannelTimer
}

