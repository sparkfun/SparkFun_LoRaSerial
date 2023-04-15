//Read the settings from NVM into the settings structure
void loadSettings()
{
  arch.eepromBegin();

  //Use the default settings
  getDefaultSettings(&settings);

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

//Merge the default settings with the default radio settings
void getDefaultSettings(Settings * newSettings)
{
  const Settings defaultSettings;

  //Set the initial radio parameters
  *newSettings = defaultSettings;
  validateAirSpeed(newSettings, DEFAULT_AIR_SPEED);
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

int nvmVcOffset(int8_t vc)
{
  return NVM_UNIQUE_ID_OFFSET + (vc * UNIQUE_ID_BYTES);
}

//Erase the specified unique ID
void nvmEraseUniqueId(int8_t vc)
{
  uint8_t id[UNIQUE_ID_BYTES];
  int index;

  //Set the erase value
  for (index = 0; index < sizeof(id); index++)
    id[index] = NVM_ERASE_VALUE;

  //Erase this portion of the NVM
  nvmSaveUniqueId(vc, id);

  //Invalidate the VC structure
  memcpy(virtualCircuitList[vc].uniqueId, id, sizeof(id));
  virtualCircuitList[vc].flags.valid = false;
}

//Get the unique ID for the VC from NVM
void nvmGetUniqueId(int8_t vc, uint8_t * uniqueId)
{
  uint8_t id[UNIQUE_ID_BYTES];

  //Read the unique ID from the flash
  EEPROM.get(nvmVcOffset(vc), id);
  memcpy(uniqueId, id, UNIQUE_ID_BYTES);
}

//Determine if the unique ID is set
bool nvmIsVcUniqueIdSet(int8_t vc)
{
  uint8_t id[UNIQUE_ID_BYTES];
  int index;

  //Read the ID from the flash
  nvmGetUniqueId(vc, id);

  //Determine if a unique ID is set in the flash
  for (index = 0; index < sizeof(id); index++)
    if (id[index] != NVM_ERASE_VALUE)
      return true;
  return false;
}

//Copy the unique ID for the VC from NVM into the virtualCircuitList entry
void nvmLoadVcUniqueId(int8_t vc)
{
  uint8_t id[UNIQUE_ID_BYTES];
  int index;

  //Read the ID from the flash
  nvmGetUniqueId(vc, id);

  //Update the VC when a unique ID is in the NVM
  if (nvmIsVcUniqueIdSet(vc))
  {
    //The unique ID was set, copy it into the VC structure
    memcpy(virtualCircuitList[vc].uniqueId, id, sizeof(id));
    virtualCircuitList[vc].flags.valid = true;
  }
}

//Save the unique ID into the NVM
void nvmSaveUniqueId(int8_t vc, uint8_t * uniqueId)
{
  uint8_t id[UNIQUE_ID_BYTES];
  int index;

  //Get the unique ID value
  memcpy(id, uniqueId, sizeof(id));

  //Write the ID into the flash
  EEPROM.put(nvmVcOffset(vc), id);
  arch.eepromCommit();

  //Place the address in the VC structure
  memcpy(virtualCircuitList[vc].uniqueId, id, sizeof(id));
  virtualCircuitList[vc].flags.valid = true;
}

//Save the unique ID from the virtualCircuitList entry into the NVM
void nvmSaveVcUniqueId(int8_t vc)
{
  uint8_t id[UNIQUE_ID_BYTES];
  int index;

  //Read the ID from the flash
  nvmGetUniqueId(vc, id);

  //Write the ID into the flash
  if (memcmp(id, virtualCircuitList[vc].uniqueId, sizeof(id)) != 0)
    nvmSaveUniqueId(vc, virtualCircuitList[vc].uniqueId);
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
