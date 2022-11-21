void loadSettings()
{
  arch.eepromBegin();

  //Check to see if EEPROM is blank
  uint32_t testRead = 0;
  if (EEPROM.get(0, testRead) == 0xFFFFFFFF)
  {
    if (settings.debug)
    {
      systemPrintln("EEPROM is blank.");
      outputSerialData(true);
    }
    recordSystemSettings(); //Record default settings to EEPROM. At power on, settings are in default state
  }

  //Check that the current settings struct size matches what is stored in EEPROM
  //Misalignment happens when we add a new feature or setting
  uint16_t tempSize = 0;
  EEPROM.get(0, tempSize); //Load the sizeOfSettings
  if (tempSize != sizeof(settings))
  {
    if (settings.debug)
    {
      systemPrintln("Settings wrong size.");
      outputSerialData(true);
    }
    recordSystemSettings(); //Record default settings to EEPROM. At power on, settings are in default state
  }

  //Check that the strIdentifier is correct
  uint16_t tempIdentifier = 0;
  EEPROM.get(sizeof(tempSize), tempIdentifier); //Load the identifier from the EEPROM location after sizeOfSettings (int)
  if (tempIdentifier != LRS_IDENTIFIER)
  {
    if (settings.debug)
    {
      systemPrint("Settings are not valid for this variant of LoRaSerial.");
      outputSerialData(true);
    }
    recordSystemSettings(); //Record default settings to EEPROM. At power on, settings are in default state
  }

  //Read current settings
  if (settings.debug)
  {
    systemPrintln("Reading the settings from EEPROM");
    outputSerialData(true);
  }
  EEPROM.get(0, settings);

  recordSystemSettings();
}

//Record the current settings struct to EEPROM
void recordSystemSettings()
{
  if (settings.debug)
  {
    systemPrintln("Writing settings to EEPROM");
    outputSerialData(true);
  }
  settings.sizeOfSettings = sizeof(settings);
  EEPROM.put(0, settings);

  arch.eepromCommit();
}

void eepromErase()
{
  if (settings.debug)
  {
    systemPrintln("Erasing the EEPROM");
    outputSerialData(true);
  }
  for (uint16_t i = 0 ; i < EEPROM.length() ; i++)
  {
    petWDT();

    EEPROM.write(i, 0);

    arch.eepromCommit();
  }
}
