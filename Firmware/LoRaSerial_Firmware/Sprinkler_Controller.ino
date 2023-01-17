/*
  January 16th, 2023
  Lee Leahy

  Sprinkler controller support routines.
*/

void updateTimeOfDay()
{
  uint32_t currentTime;
  static uint32_t previousTime;

  //Update the time
  currentTime = millis();
  timeOfDay += currentTime - previousTime;
  previousTime = currentTime;
  if (timeOfDay >= MILLISECONDS_IN_A_DAY)
  {
    startOfDay += MILLISECONDS_IN_A_DAY;
    timeOfDay -= MILLISECONDS_IN_A_DAY;
    scheduleCopied = false;
  }
}

/*
        Day
    Sun  |  Mon
  1 -----       ---------------------------------------------- scheduleCopied
  0      \_____/:
                :
  1           __:
  0 _________/  \_____________________________________________ waterToday
                :
  1             :------------------------------------------
  0 ____________/:                                         \__ scheduleActive
                 :
  1              :__
  0 _____________/  \_________________________________________ today.zoneScheduleDuration 1
                 :  :
  1              :__:_________
  0 _____________/  :         \_______________________________ today.zoneScheduleDuration 2
                 :  :         :
                 :  :         :              Zone 3
                 :  :         :             Suspended
  1              :__:_________:_________       ____
  0 _____________/  :         :         \_____/   :\__________ today.zoneScheduleDuration 3
                 :  :         :         :     :   ::
  1              :  :         :         :     :   ::
  0 ________________:_________:_________:_____:___::__________ today.zoneScheduleDuration 4
                    :         :         :     :   ::
  1                 :-------  :-------  :---  :   ::-----
  0 ________________/   1   \_/   2   \_/ 3 \_:___:/  3  \____ zoneOnDuration
                                              :   :
  1                                           :---:
  0 __________________________________________/   \___________ manualOn

*/

void updateZones()
{
  uint32_t currentTime;
  static uint32_t previousTime;
  uint32_t deltaTime;
  bool waterToday;
  int zone;

  if (!online.quadRelay)
    quadRelayOffline();

  //Verify that the previous schedule has completed.  If completed, check for
  //a watering schedule for today
  waterToday = false;
  for (zone = 0; zone < ZONE_NUMBER_MAX; zone++)
  {
    //Verify that the previous watering schedule was completed
    if (today.zoneScheduleDuration[zone])
    {
      waterToday = false;
      break;
    }

    //Determine if watering is necessary today
    if (week[dayOfWeek].zoneScheduleDuration[zone])
      waterToday = true;
  }

  //----------------------------------------
  //First finish turning on or off any of the latching relays
  //----------------------------------------
  currentTime = millis();
  if (pulseDuration)
  {
    //When the pulse duration is reached, turn off the relay
    if ((currentTime - pulseStartTime) >= pulseDuration)
      turnOffRelay();
  }

  //----------------------------------------
  //Second, process any change in manual on/off
  //----------------------------------------
  else if (zoneManualOn || (zoneManualPreviousOn != zoneManualOn))
  {
    if (zoneManualPreviousOn != zoneManualOn)
    {
      //Turn off the previous zone first
      if (zoneManualPreviousOn)
      {
        //Reduce the zone's watering schedule
        zone = zoneNumber - 1;
        if (today.zoneScheduleDuration[zone])
        {
          deltaTime = currentTime - onTime;
          today.zoneScheduleDuration[zone] = (today.zoneScheduleDuration[zone] < deltaTime)
                                           ? 0 : today.zoneScheduleDuration[zone] - deltaTime;
        }

        //Turn off this zone
        turnOffZone();
      }
      else
      {
        //No manual operation is active, if a scheduled operation is in progress
        //suspend the scheduled operation.
        if (zoneOnDuration)
        {
          //Remember the remaining time
          zone = zoneNumber - 1;
          deltaTime = zoneOnDuration - (currentTime - onTime);
          today.zoneScheduleDuration[zone] = deltaTime;
          zoneOnDuration = 0;

          //Turn off this zone
          turnOffZone();
        }
        else
        {
          //Turn on specified relay
          onTime = currentTime;
          turnOnRelay(zoneManualOn);

          //Remember the state change
          zoneManualPreviousOn = zoneManualOn;
        }
      }
    }
  }

  //----------------------------------------
  //Third, turn off the zone
  //----------------------------------------

  else if (zoneOnDuration)
  {
    if ((timeOfDay - zoneOnTime) >= zoneOnDuration)
    {
      //Turn off the zone
      turnOffZone();
      zoneOnDuration = 0;
    }
  }

  //----------------------------------------
  //Fourth, execute the schedule
  //----------------------------------------
  else if (scheduleActive)
  {
    if ((timeOfDay - startOfDay) >= today.scheduleStartTime)
    {
      zoneOnDuration = 0;
      for (zone = 0; zone < ZONE_NUMBER_MAX; zone++)
      {
        //Start this zone
        if (today.zoneScheduleDuration[zone])
        {
          //Turn on the zone
          onTime = currentTime;
          turnOnRelay(1 << zone);

          //Only water this zone once
          zoneOnTime = timeOfDay;
          zoneOnDuration = today.zoneScheduleDuration[zone];
          today.zoneScheduleDuration[zone] = 0;
          break;
        }
      }

      //Done if watering is done for all of the zones
      if (!zoneOnDuration)
        scheduleActive = false;
    }
  }

  //----------------------------------------
  //Last, copy the schedule
  //----------------------------------------
  else if (waterToday && (!scheduleCopied) && enableSprinklerController)
  {
    //Copy the schedule
    today = week[dayOfWeek];
    scheduleActive = true;
    scheduleCopied = true;
  }

  previousTime = currentTime;
}

void quadRelayOffline()
{
  //Report error back to the server
  //Delay for an hour
  //Reboot
}

uint8_t zoneMaskToZoneNumber(ZONE_MASK zoneMask)
{
  uint8_t zone;

  //Determine which zone is enabled
  if (zoneMask)
    for (zone = 0; zone < ZONE_NUMBER_MAX; zone++) {
      if (zoneMask & (1 << zone))
        return zone + 1;
    }

  //No zone enabled
  return 0;
}

void turnOnRelay(uint8_t zoneMask)
{
  ZONE_MASK previousZoneActive;

  //Get the zone number: 1 - 8
  zoneNumber = zoneMaskToZoneNumber(zoneMask);

  //Set the active zone
  previousZoneActive = zoneActive;
  zoneActive = 1 << (zoneNumber - 1);

  //Update the display
  if (latchingSolenoid & zoneActive)
    relayOn |= zoneActive;
  if (!previousZoneActive)
    zoneOn |= zoneActive;

  //Output the debug messages
  if (settings.debugSprinklers)
  {
    //Update the relay status
    if (!previousZoneActive)
      systemPrintln("--------------------------------------------------");
    systemPrintTimestamp();
    systemPrint(" Relay ");
    systemPrint(zoneNumber);
    systemPrint(" ON driving ");
    systemPrint((latchingSolenoid & zoneActive) ? "DC latching" : "AC");
    systemPrintln(" solenoid");

    if (!previousZoneActive)
    {
      //Update the zone status
      systemPrintTimestamp();
      systemPrint(" Zone ");
      systemPrint(zoneNumber);
      systemPrintln(" turning ON");
    }
  }

  //Turn on the relay
  if (online.quadRelay)
    quadRelay.turnRelayOn(zoneNumber);

  //Determine the solenoid type and pulse duration
  pulseDuration = 0;
  if (latchingSolenoid & zoneActive)
  {
    //A latching solenoid is in use, apply power for a short pulse.
    pulseDuration = settings.pulseDuration;
    pulseStartTime = millis();
  }
}

void turnOffZone()
{
  //AC solenoids require power during the entire time the zone is on
  //
  //                 ON                                OFF
  //                  __________________________________               24V
  //AC ______________/                                  \______________ 0V
  //
  //DC latching solenoids need a short pulse to turn on/off the zone
  //
  //                 ON                                OFF
  //                  ___                                ___            9V
  //DC ______________/   \______________________________/   \__________ 0V

  //Determine the solenoid type and pulse duration
  zoneManualPreviousOn = 0;
  turnWaterOff = true;

  //Update the display
  zoneOn &= ~zoneActive;

  //Turn off the zone
  if (latchingSolenoid & zoneActive)
    turnOnRelay(zoneActive);
  else
    turnOffRelay();
}

void turnOffRelay()
{
  uint8_t zone;

  //Get the zone number
  zone = zoneMaskToZoneNumber(zoneActive);

  //Turn off the relay
  if (online.quadRelay)
    quadRelay.turnRelayOff(zoneNumber);

  //Update the display
  relayOn &= ~zoneActive;

  //Output the debug messages
  if (settings.debugSprinklers)
  {
    if (turnWaterOff)
    {
      //Update the zone status
      systemPrintTimestamp();
      systemPrint(" Zone ");
      systemPrint(zone);
      systemPrintln(" turning OFF");
    }

    //Update the relay status
    systemPrintTimestamp();
    systemPrint(" Relay ");
    systemPrint(zone);
    systemPrintln(" OFF");
  }

  //Get ready for the next relay operation
  if (turnWaterOff)
  {
    turnWaterOff = false;
    zoneNumber = 0;
    zoneActive = 0;
    onTime = 0;
  }
  pulseDuration = 0;
  pulseStartTime = 0;
}
