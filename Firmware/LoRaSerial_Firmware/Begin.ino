//Blink LEDs to indicate the completion of system setup
void blinkStartup()
{
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
    outputSerialData(true);
    while (1)
    {
      petWDT();
      delay(100);
    }
  }

  changeState(RADIO_RESET);
}

//Initialize the button driver
void beginButton()
{
  if (pin_trainButton != PIN_UNDEFINED)
  {
    trainBtn = new Button(pin_trainButton); //Create the button
    trainBtn->begin();
  }
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

//Initialize the serial drivers
void beginSerial(uint16_t serialSpeed)
{
  Serial.begin(serialSpeed);
  arch.beginSerial(serialSpeed);
}

//Ensure the watch dog timer does not fire which would cause a CPU hardware reset
void petWDT()
{
  //Petting the dog takes a long time (~4.5ms on SAMD21) so it's only done after we've passed the timeout
  if (millis() - lastPet > petTimeout)
  {
    lastPet = millis();
    arch.petWDT();
  }
}

//Start the timer measuring the dwell interval and indicating that it is time to
//hop channels
void beginChannelTimer()
{
  if (channelTimer.attachInterruptInterval_MS(settings.maxDwellTime, channelTimerHandler) == false)
    Serial.println("Error starting ChannelTimer!");

  stopChannelTimer(); //Start timer only after link is up
}

//ISR that fires when channel timer expires
void channelTimerHandler()
{
  timerStart = millis(); //Record when this ISR happened. Used for calculating clock sync.

  //If the last timer was used to sync clocks, restore full timer interval
  if (reloadChannelTimer == true)
  {
    reloadChannelTimer = false;
    channelTimer.setInterval_MS(settings.maxDwellTime, channelTimerHandler);
  }

  if (settings.frequencyHop)
  {
    triggerEvent(TRIGGER_CHANNEL_TIMER_ISR);
    timeToHop = true;
  }
}
