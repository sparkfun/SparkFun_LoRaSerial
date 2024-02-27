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
