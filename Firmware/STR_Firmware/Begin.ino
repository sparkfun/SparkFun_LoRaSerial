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

  pinMode(pin_rst, OUTPUT);
  digitalWrite(pin_rst, HIGH);

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

  strcpy(platformPrefix, "ESP32 100mW");
#endif

}

void beginLoRa()
{
  radio = new Module(pin_cs, pin_dio0, pin_rst, pin_dio1);

  bool success = true;
  int state = radio.begin(settings.radioFrequency);
  if (state != RADIOLIB_ERR_NONE)
  {
    success = false;
    if (settings.debug == true)
    {
      Serial.print(F("Failed with code: "));
      Serial.println(state);
    }
  }

  configureRadio(); //Apply current settings


  //Return to receiving
  radio.startReceive();
  radioState = RADIO_RECEIVING;
}
