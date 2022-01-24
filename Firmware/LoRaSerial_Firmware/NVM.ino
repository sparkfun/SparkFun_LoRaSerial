void loadSettings()
{
#if defined(ARDUINO_ARCH_ESP32)
  EEPROM.begin(EEPROM_SIZE);
#endif

  //Check to see if EEPROM is blank
  uint32_t testRead = 0;
  if (EEPROM.get(0, testRead) == 0xFFFFFFFF)
  {
    //Serial.println(F("EEPROM is blank. Default settings applied."));
    recordSystemSettings(); //Record default settings to EEPROM. At power on, settings are in default state
  }

  //Check that the current settings struct size matches what is stored in EEPROM
  //Misalignment happens when we add a new feature or setting
  int tempSize = 0;
  EEPROM.get(0, tempSize); //Load the sizeOfSettings
  if (tempSize != sizeof(settings))
  {
    //Serial.println(F("Settings wrong size. Default settings applied."));
    recordSystemSettings(); //Record default settings to EEPROM. At power on, settings are in default state
  }

  //Check that the strIdentifier is correct
  int tempIdentifier = 0;
  EEPROM.get(sizeof(int), tempIdentifier); //Load the identifier from the EEPROM location after sizeOfSettings (int)
  if (tempIdentifier != LRS_IDENTIFIER)
  {
    //Serial.print(F("Settings are not valid for this variant of STR "));
    //Serial.print((String)platformPrefix);
    //Serial.println(F(". Default settings applied."));
    recordSystemSettings(); //Record default settings to EEPROM. At power on, settings are in default state
  }

  //Read current settings
  EEPROM.get(0, settings);

  recordSystemSettings();
}

//Record the current settings struct to EEPROM
void recordSystemSettings()
{
  settings.sizeOfSettings = sizeof(settings);
  EEPROM.put(0, settings);

#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_SAMD)
  EEPROM.commit();
#endif
}

void eepromErase()
{
  for (uint16_t i = 0 ; i < EEPROM.length() ; i++)
  {
    EEPROM.write(i, 0);

#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_SAMD)
    EEPROM.commit();
#endif
  }
}
