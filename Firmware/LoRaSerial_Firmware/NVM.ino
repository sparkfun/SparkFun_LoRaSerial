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
  if (tempSize != sizeof(settings))
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

  recordSystemSettings();
}

//Record the current settings struct to EEPROM
void recordSystemSettings()
{
  if (settings.debugNvm)
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
  if (settings.debugNvm)
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

//Copy the unique ID for the VC from NVM into the virtualCircuitList entry
void nvmLoadVcUniqueId(int8_t vc)
{
  uint8_t id[UNIQUE_ID_BYTES];
  int index;
  int offset;

  //Read the ID from the flash
  offset = NVM_UNIQUE_ID_OFFSET + (vc * UNIQUE_ID_BYTES);
  EEPROM.get(offset, id);

  //Verify that the value was saved
  for (index = 0; index < sizeof(id); index++)
  {
    //Determine if this entry was set to a value
    if (id[index] != NVM_ERASE_VALUE)
    {
      //The unique ID was set, copy it into the VC structure
      memcpy(virtualCircuitList[vc].uniqueId, id, sizeof(id));
      virtualCircuitList[vc].valid = true;
      break;
    }
  }
}

//Save the unique ID from the virtualCircuitList entry into the NVM
void nvmSaveVcUniqueId(int8_t vc)
{
  uint8_t id[UNIQUE_ID_BYTES];
  int index;
  int offset;

  //Read the ID from the flash
  offset = NVM_UNIQUE_ID_OFFSET + (vc * UNIQUE_ID_BYTES);
  EEPROM.get(offset, id);

  //Write the ID into the flash
  if (memcmp(id, virtualCircuitList[vc].uniqueId, sizeof(id)) != 0)
  {
    EEPROM.put(offset, virtualCircuitList[vc].uniqueId);
    arch.eepromCommit();
  }
}
