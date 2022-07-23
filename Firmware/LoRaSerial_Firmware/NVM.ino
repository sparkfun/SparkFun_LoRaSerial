void loadSettings()
{
  arch.eepromBegin();

  //Check to see if EEPROM is blank
  uint32_t testRead = 0;
  if (EEPROM.get(0, testRead) == 0xFFFFFFFF)
  {
    //systemPrintln("EEPROM is blank. Default settings applied.");
    recordSystemSettings(); //Record default settings to EEPROM. At power on, settings are in default state
  }

  //Check that the current settings struct size matches what is stored in EEPROM
  //Misalignment happens when we add a new feature or setting
  uint16_t tempSize = 0;
  EEPROM.get(0, tempSize); //Load the sizeOfSettings
  if (tempSize != sizeof(settings))
  {
    //systemPrintln("Settings wrong size. Default settings applied.");
    recordSystemSettings(); //Record default settings to EEPROM. At power on, settings are in default state
  }

  //Check that the strIdentifier is correct
  uint16_t tempIdentifier = 0;
  EEPROM.get(sizeof(tempSize), tempIdentifier); //Load the identifier from the EEPROM location after sizeOfSettings (int)
  if (tempIdentifier != LRS_IDENTIFIER)
  {
    //systemPrint("Settings are not valid for this variant of LoRaSerial. Default settings applied.");
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

  arch.eepromCommit();
}

void eepromErase()
{
  for (uint16_t i = 0 ; i < EEPROM.length() ; i++)
  {
    petWDT();

    EEPROM.write(i, 0);

    arch.eepromCommit();
  }
}
