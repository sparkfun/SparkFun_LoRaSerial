//Read the settings from NVM into the settings structure
void loadSettings()
{
  arch.eepromBegin();

  //Check to see if EEPROM is blank
  uint32_t testRead = 0;
  if (EEPROM.get(0, testRead) == 0xFFFFFFFF)
  {
    if (settings.debugNvm)
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
  if (tempSize != sizeof(Settings))
  {
    if (settings.debugNvm)
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
    if (settings.debugNvm)
    {
      systemPrint("Settings are not valid for this variant of LoRaSerial.");
      outputSerialData(true);
    }
    recordSystemSettings(); //Record default settings to EEPROM. At power on, settings are in default state
  }

  //Read current settings
  if (settings.debugNvm)
  {
    systemPrintln("Reading the settings from EEPROM");
    outputSerialData(true);
  }
  EEPROM.get(0, settings);

  validateSettings(); //Confirm these settings are within regulatory bounds

  recordSystemSettings();
}

//Modify defaults for each radio type (915, 868, 433, etc)
//Confirm various settings are within regulatory bounds
void validateSettings()
{
  if (radioBand == 915)
  {
    //Radio limits are 900-931MHz
    //USA ISM bounds are 902-928MHz
    if (settings.frequencyMin < 900.0 || settings.frequencyMin > 931.0) settings.frequencyMin = 902.0;
    if (settings.frequencyMax > 931.0 || settings.frequencyMax < 900.0) settings.frequencyMax = 928.0;
  }
  else if (radioBand == 868)
  {
    //Radio limits are 862-893MHz
    //EU ESTI limits are 862-893MHz
    if (settings.frequencyMin < 862.0 || settings.frequencyMin > 893.0) settings.frequencyMin = 862.0;
    if (settings.frequencyMax > 893.0 || settings.frequencyMax < 862.0) settings.frequencyMax = 893.0;
  }
  else if (radioBand == 433)
  {
    //Radio limits are 410-510MHz
    //USA amateur radio limits are 420-450MHz
    if (settings.frequencyMin < 410.0 || settings.frequencyMin > 510.0) settings.frequencyMin = 420.0;
    if (settings.frequencyMax > 510.0 || settings.frequencyMax < 410.0) settings.frequencyMax = 450.0;
  }
  else
  {
    systemPrintln("Error: Unknown radioBand");
  }
}

//Record the current settings struct to EEPROM
void recordSystemSettings()
{
  if (settings.debugNvm)
  {
    systemPrintln("Writing settings to EEPROM");
    outputSerialData(true);
  }
  settings.sizeOfSettings = sizeof(Settings);
  EEPROM.put(0, settings);

  arch.eepromCommit();
}

//Erase the entire EEPROM
void nvmErase()
{
  int address;
  int length;
  uint8_t value[64];

  //Get the EEPROM length
  length = EEPROM.length();

  //Set the erase value
  memset(&value, NVM_ERASE_VALUE, sizeof(value));

  //Erase the EEPROM
  address = 0;
  while (address < length)
  {
    petWDT();
    EEPROM.put(address, value);
    address += sizeof(value);
  }

  //Finish the write
  arch.eepromCommit();
}
