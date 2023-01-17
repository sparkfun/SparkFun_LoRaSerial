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

