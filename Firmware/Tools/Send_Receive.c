/*
  May 4th, 2023
  SparkFun Electronics
  Lee Leahy

  Program to test LoRaSerial transmission and reception
*/

#include <malloc.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#include "Send_Receive.h"
#include "settings.h"

#define DEBUG_RX_TIMES              0
#define SEND_ENCRYPTION_KEY         1
#define DISPLAY_CMD_RESPONSE        0
#define DISPLAY_RX_DATA             0

#define SYNC_DELAY_SECONDS          30
#define RECEIVE_TIMEOUT_SEC         30

#define DISPLAY_PINWHEEL            0

#define MAX_DELAY_MSEC              250
#define MAX_DATA_BYTES              240
#define MIN_DATA_VALUE              0x20
#define MAX_DATA_VALUE              0x7e

#define HOURS_IN_A_DAY              24ull
#define MINUTES_IN_AN_HOUR          60ull
#define SECONDS_IN_A_MINUTE         60ull
#define MILLISECONDS_IN_A_SECOND    1000ull

#define SECONDS_IN_AN_HOUR          (MINUTES_IN_AN_HOUR * SECONDS_IN_A_MINUTE)

#define MILLISECONDS_IN_A_MINUTE    (SECONDS_IN_A_MINUTE * MILLISECONDS_IN_A_SECOND)
#define MILLISECONDS_IN_AN_HOUR     (MINUTES_IN_AN_HOUR * MILLISECONDS_IN_A_MINUTE)
#define MILLISECONDS_IN_A_DAY       (HOURS_IN_A_DAY * MILLISECONDS_IN_AN_HOUR)

#define MICROSECONDS_IN_A_SECOND    (1000Ull * MILLISECONDS_IN_A_SECOND)
#define NANOSECONDS_IN_A_SECOND     (1000ull * MICROSECONDS_IN_A_SECOND)

#define BYTES_PER_LINE              10

int ackBytes = 10;
const char * enterCommandMode = "+++";
int headerBytes = 6;
const int serialDelay = 3; //Must be >= 1
struct timespec startTime;
int terminalOpen;
int totalBytes;
int totalPackets;
int totalTxBytes;
int totalDelay;
const int txListEntries = sizeof(txList) / sizeof(TRANSMIT_ENTRY);

//Routine to create the header file
void createHeaderFile(int entries)
{
  int bytes;
  int count;
  time_t currentSeconds;
  struct tm * currentTime;
  uint8_t * data;
  int delay;
  int delta;
  int index;
  int j;
  TRANSMIT_ENTRY * list;
  int offset;
  int plusCount;
  char timeBuffer[256];

  //Allocate the list
  list = malloc(sizeof(TRANSMIT_ENTRY) * entries);

  //Determine the delay, number of data bytes for each entry and the total byte count
  totalBytes = 0;
  for (count = 0; count < entries; count++)
  {
    list[count].mSecDelay = rand() % MAX_DELAY_MSEC;
    bytes = (rand() % MAX_DATA_BYTES) + 1;
    bytes = (rand() % bytes) + 1;
    list[count].byteCount = bytes;
    totalBytes += bytes;
  }

  //Allocate the data array
  data = malloc(totalBytes);

  //Initialize the data array
  plusCount = 0;
  delta = MAX_DATA_VALUE + 1 - MIN_DATA_VALUE;
  for (index = 0; index < totalBytes; index++)
  {
    data[index] = (rand() % delta) + MIN_DATA_VALUE;

    //Don't allow the enter command mode sequence in the data stream
    if (data[index] != enterCommandMode[plusCount])
      plusCount = 0;
    else
    {
      //The beginning portion of the enter command sequence has matched
      plusCount += 1;

      //Determine if the entire enter command sequence has matched
      if (plusCount == (int)strlen(enterCommandMode))
      {
        //Change the last character in the sequence to avoid the match
        data[index] -= 1;
        plusCount = 0;
      }
    }
  }

  //Terminate each line with CR and LF if possible, otherwise just use CR
  offset = 0;
  for (count = 0; count < entries; count++)
  {
    list[count].data = &data[offset];
    if (list[count].byteCount == 1)
      data[offset] = '\r';
    else
    {
      data[offset + list[count].byteCount - 2] = '\r';
      data[offset + list[count].byteCount - 1] = '\n';
    }
    offset += list[count].byteCount;
  }

  //Get the local time
  time(&currentSeconds);
  currentTime = localtime(&currentSeconds);
  strftime(&timeBuffer[0], sizeof(timeBuffer), "%B %d, %Y", currentTime);

  //Add the module header
  printf("/*\n");
  printf("  %s\n", timeBuffer);
  printf("  SparkFun Electronics\n");
  printf("  Lee Leahy\n");
  printf("\n");
  printf("  Include file to test LoRaSerial transmission and reception\n");
  printf("*/\n");
  printf("\n");
  printf("#ifndef __SEND_RECEIVE_H__\n");
  printf("#define __SEND_RECEIVE_H__\n");
  printf("\n");
  printf("//Transmit data structure\n");
  printf("typedef struct _TRANSMIT_ENTRY\n");
  printf("{\n");
  printf("  uint32_t mSecDelay; //Delay in milliseconds before the transmission\n");
  printf("  uint8_t byteCount;  //Number of data bytes to send\n");
  printf("  const uint8_t * data; //Address of data to send\n");
  printf("} TRANSMIT_ENTRY;\n");
  printf("\n");

  //Output the data array
  printf("const uint8_t dataBuffer[%d] =\n", totalBytes);
  printf("{\n");
  offset = 0;
  for (index = 0; index < totalBytes; index += BYTES_PER_LINE)
  {
    for (j = index; (j < totalBytes) && (j < (index + BYTES_PER_LINE)); j++)
      printf(" 0x%02x,", data[j]);
    while (j++ < (index + BYTES_PER_LINE))
      printf("      ");
    printf(" // %9d: ", index);
    for (j = index; (j < totalBytes) && (j < (index + BYTES_PER_LINE)); j++)
      if ((data[offset + j] < ' ') || (data[offset + j] == '\\') || (data[offset + j] >= 0x7f))
        printf(" ");
      else
        printf("%c", data[offset + j]);
    printf("\n");
  }
  printf("};\n");
  printf("\n");

  //Create the transmit list
  printf("const TRANSMIT_ENTRY txList[] =\n");
  printf("{\n");
  offset = 0;
  for (count = 0; count < entries; count++)
  {
    printf("  {%4d, %3d, &dataBuffer[%d]},\n", list[count].mSecDelay, list[count].byteCount, offset);
    offset += list[count].byteCount;
  }
  printf("};\n");
  printf("\n");
  printf("#endif //__SEND_RECEIVE_H__\n");
}

//Display the approximate test time
void printTestTime(int airSpeed)
{
  uint64_t days;
  uint64_t hours;
  uint64_t milliseconds;
  uint64_t minutes;
  uint64_t seconds;

  //Assume on average the ACK occurs during the fixed delay
  milliseconds = totalTxBytes;  //Bytes
  milliseconds *= 8 * MILLISECONDS_IN_A_SECOND;  //Bits * 1000
  milliseconds /= airSpeed;  //Bits * 1000 / (bits/sec) = sec * 1000
  milliseconds += totalDelay;  //Account for forced delays
  milliseconds += SYNC_DELAY_SECONDS * MILLISECONDS_IN_A_SECOND;
  milliseconds += RECEIVE_TIMEOUT_SEC * MILLISECONDS_IN_A_SECOND;
  days = milliseconds / MILLISECONDS_IN_A_DAY;
  milliseconds -= days * MILLISECONDS_IN_A_DAY;
  hours = milliseconds / MILLISECONDS_IN_AN_HOUR;
  milliseconds -= hours * MILLISECONDS_IN_AN_HOUR;
  minutes = milliseconds / MILLISECONDS_IN_A_MINUTE;
  milliseconds -= minutes * MILLISECONDS_IN_A_MINUTE;
  seconds = milliseconds / MILLISECONDS_IN_A_SECOND;
  milliseconds -= seconds * MILLISECONDS_IN_A_SECOND;
  printf("%3ld %2ld:%02ld:%02ld.%03ld", days, hours, minutes, seconds, milliseconds);
}

//Determine the amount of data for the test
void computeTxData(int packets)
{
  int entry;
  int index;

  //Compute transmit data
  totalBytes = 0;
  totalTxBytes = 0;
  totalDelay = 0;
  totalPackets = packets;
  for (index = 0; index < packets; index++)
  {
    entry = index % txListEntries;
    totalBytes += txList[entry].byteCount;
    totalTxBytes += txList[entry].byteCount + headerBytes;
    totalDelay += txList[entry].mSecDelay + serialDelay;
  }
}

//Display a time value
void displayMilliseconds(uint64_t milliseconds)
{
  uint64_t days;
  uint64_t hours;
  uint64_t minutes;
  uint64_t seconds;

  days = milliseconds / MILLISECONDS_IN_A_DAY;
  milliseconds -= days * MILLISECONDS_IN_A_DAY;
  hours = milliseconds / MILLISECONDS_IN_AN_HOUR;
  milliseconds -= hours * MILLISECONDS_IN_AN_HOUR;
  minutes = milliseconds / MILLISECONDS_IN_A_MINUTE;
  milliseconds -= minutes * MILLISECONDS_IN_A_MINUTE;
  seconds = milliseconds / MILLISECONDS_IN_A_SECOND;
  milliseconds -= seconds * MILLISECONDS_IN_A_SECOND;
  printf("%3ld %2ld:%02ld:%02ld.%03ld", days, hours, minutes, seconds, milliseconds);
}

//Display a seconds value
void displaySeconds(uint64_t seconds)
{
  displayMilliseconds(seconds * MILLISECONDS_IN_A_SECOND);
}

//Display a timespec value
void displayTimeSpec(const struct timespec * time)
{
  uint64_t milliseconds;

  milliseconds = time->tv_sec * MILLISECONDS_IN_A_SECOND;
  milliseconds += time->tv_nsec / (NANOSECONDS_IN_A_SECOND / MILLISECONDS_IN_A_SECOND);
  displayMilliseconds(milliseconds);
}

//Display a timeval value
void displayTimeVal(const struct timeval * time)
{
  uint64_t milliseconds;

  milliseconds = time->tv_sec * MILLISECONDS_IN_A_SECOND;
  milliseconds += time->tv_usec / (MICROSECONDS_IN_A_SECOND / MILLISECONDS_IN_A_SECOND);
  displayMilliseconds(milliseconds);
}

//Analyze the table
void analyzeTable(int packets)
{
  const int airSpeedList[] = {40, 150, 400, 1200, 2400, 4800, 9600, 19200, 28800, 38400};
  int byteGraph[257];
  int bytes;
  uint64_t days;
  uint64_t delay;
  int entry;
  uint64_t hours;
  int index;
  uint64_t minutes;
  uint64_t seconds;
  uint64_t tableDelay;

  //Compute the byte count graph values
  computeTxData(packets);
  memset(byteGraph, 0, sizeof(byteGraph));
  for (index = 0; index < totalPackets; index++)
  {
    entry = index % txListEntries;
    byteGraph[txList[entry].byteCount] += 1;
  }

  //Compute the delay in the table
  tableDelay = 0;
  for (index = 0; index < txListEntries; index++)
  {
    tableDelay += txList[index].mSecDelay + serialDelay;
  }

  //Display the byte count graph
  for (bytes = 1; bytes < MAX_DATA_BYTES; bytes++)
  {
    printf("%3d: ", bytes);
    for (index = 0; index < byteGraph[bytes]; index++)
      printf("*");
    printf("\n");
  }
  printf("\n");

  //Display the table data
  printf("Packets in Table: %d\n", txListEntries);
  printf("    Total Delay: ");
  displayMilliseconds(tableDelay);
  printf("\n");
  printf("    Total Bytes: %ld\n", sizeof(dataBuffer));

  //Display the delay time
  printf("Test Packets: %d\n", totalPackets);
  printf("    Total Delay: ");
  displayMilliseconds(totalDelay);
  printf("\n");
  printf("    Total Bytes: %d\n", totalBytes);

  //Display the approximate test time
  printf("Approximate test time at air speed\n");
  for (index = 0; index < (int)(sizeof(airSpeedList) / sizeof(airSpeedList[0])); index++)
  {
    printf("    %5d: ", airSpeedList[index]);
    printTestTime(airSpeedList[index]);
    printf("\n");
  }
}

//Display the contents of the buffer in hex and ASCII
void dumpBuffer(uint8_t * data, int length)
{
  char byte;
  int bytes;
  uint8_t * dataEnd;
  uint8_t * dataStart;
  const int displayWidth = 16;
  int index;

  dataStart = data;
  dataEnd = &data[length];
  while (data < dataEnd)
  {
    // Display the offset
    printf("    0x%02x: ", (unsigned int)(data - dataStart));

    // Determine the number of bytes to display
    bytes = dataEnd - data;
    if (bytes > displayWidth)
      bytes = displayWidth;

    // Display the data bytes in hex
    for (index = 0; index < bytes; index++)
      printf(" %02x", *data++);

    // Space over to the ASCII display
    for (; index < displayWidth; index++)
      printf("   ");
    printf("  ");

    // Display the ASCII bytes
    data -= bytes;
    for (index = 0; index < bytes; index++) {
      byte = *data++;
      printf("%c", ((byte < ' ') || (byte >= 0x7f)) ? '.' : byte);
    }
    printf("\n");
  }
}

//Send data to LoRaSerial
int sendData(int bytesToSend, const uint8_t * data)
{
  int bytesSent;
  int status;

  //Send the data
  status = 0;
  do
  {
    bytesSent = write(radio, data, bytesToSend);
    if (bytesSent < 0)
    {
      status = errno;
      fflush(stdout);
      fprintf(stderr, "%s\n", strerror(status));
      fflush(stderr);
      break;
    }
    bytesToSend -= bytesSent;
    data += bytesSent;
  } while (bytesToSend);
  return status;
}

//Send a command to LoRaSerial and process the response
int sendCommand(const char * command)
{
  static char buffer[65536];
  int bytesRead;
  static char cmd[65536];
  fd_set currentfds;
  char * endOfData;
  int maxfds;
  int numfds;
  fd_set readfds;
  static char response[65536];
  int startOfData;
  int status;
  struct timeval timeout;

  do
  {
    //Send the command
    status = sendData(strlen(command), (uint8_t *)command);
    if (status)
      break;

    //Wait for a read timeout from the LoRaSerial radio
    maxfds = radio;
    FD_ZERO(&readfds);
    FD_SET(radio, &readfds);
    endOfData = buffer;
    while (1)
    {
      //Set the timeout
      timeout.tv_sec = 0;
      timeout.tv_usec = 400 * 1000;

      //Wait for receive data or timeout
      memcpy((void *)&currentfds, (void *)&readfds, sizeof(readfds));
      numfds = select(maxfds + 1, &currentfds, NULL, NULL, &timeout);
      if (numfds < 0)
      {
        fflush(stdout);
        status = errno;
        perror("ERROR: select call failed!");
        fflush(stderr);
        break;
      }

      //Determine if a timeout occurred
      if (!numfds)
        break;

      //Read the data from the radio
      bytesRead = read(radio, endOfData, sizeof(buffer) - (endOfData - buffer));
      if (bytesRead < 0)
      {
        status = errno;
        break;
      }
      endOfData += bytesRead;
    }
    if (status)
      break;

    //Verify that a response was received
    bytesRead = endOfData - buffer;
    buffer[bytesRead] = 0;
    if (!bytesRead)
    {
      fflush(stdout);
      fprintf(stderr, "ERROR: No response from LoRaSerial!\n");
      fprintf(stderr, "Please press and hold LoRaSerial button for 15 seconds to reset\n");
      fflush(stderr);
      status = -1;
      break;
    }

    //Display the response
    if (DISPLAY_CMD_RESPONSE)
      dumpBuffer((uint8_t *)buffer, bytesRead);

    //Remove the trailing CR and LF
    while (bytesRead && ((buffer[bytesRead - 1] == '\r') || (buffer[bytesRead - 1] == '\n')))
    {
      bytesRead -= 1;
      buffer[bytesRead] = 0;
    }

    //Determine more than CR and LF were returned
    if (bytesRead <= 0)
    {
      fflush(stdout);
      fprintf(stderr, "ERROR: Only CR and LF response returned from LoRaSerial!\n");
      fprintf(stderr, "Please press and hold LoRaSerial button for 15 seconds to reset\n");
      fflush(stderr);
      status = -1;
      break;
    }

    //Find the last line in the returned data
    startOfData = bytesRead - 1;
    while ((startOfData > 0) && (buffer[startOfData - 1] != '\r') && (buffer[startOfData - 1] != '\n'))
      startOfData -= 1;
    bytesRead = bytesRead - startOfData;
    strcpy(response, &buffer[startOfData]);

    //Remove CR and LF from the command
    strcpy(cmd, command);
    while (strlen(cmd) && ((cmd[strlen(cmd) - 1] == '\r') || (cmd[strlen(cmd) - 1] == '\n')))
      cmd[strlen(cmd) - 1] = 0;

    //Display the command response
    printf("    %s: %s\n", cmd, response);

    //Determine if LoRaSerial is already in command mode
    if ((strcmp(command, enterCommandMode) == 0) && (strcmp(response, enterCommandMode) == 0))
    {
      //Terminate the enter command mode string and ignore the error
      sendData(2, (uint8_t *)"\r\n");
      break;
    }

    //Determine if the command was successful
    if (strcmp(response, "OK") != 0)
    {
      fflush(stdout);
      fprintf(stderr, "ERROR: Command %s failed!\n", cmd);
      fflush(stderr);
      status = -1;
    }
  } while (0);

  //Return the command status
  return status;
}

//Compute the difference between two time values
struct timespec timeSubtract(const struct timespec * endTime, const struct timespec * startTime)
{
  struct timespec deltaTime;

  deltaTime.tv_sec = endTime->tv_sec - startTime->tv_sec;
  deltaTime.tv_nsec = endTime->tv_nsec - startTime->tv_nsec;
  if (deltaTime.tv_nsec < 0)
  {
    deltaTime.tv_nsec += NANOSECONDS_IN_A_SECOND;
    deltaTime.tv_sec -= 1;
  }
  return deltaTime;
}

//Configure the LoRaSerial radio for the test
int configureRadio(char * terminal, bool transmitter, char mode, int airSpeed)
{
  static char buffer[2048];
  struct timespec endTime;
  struct timespec startDelayTime;
  int status;

  do
  {
    //Open the terminal for use
    clock_gettime(CLOCK_MONOTONIC, &startTime);
    status = openLoRaSerial(terminal);
    if (status)
      break;
    terminalOpen = true;

    //Delay for a while to all the LoRaSerial to initialize
    sleep(2);

    //Enter command mode
    printf("LoRaSerial Configuration:\n");
    status = sendCommand(enterCommandMode);
    if (status)
      break;

    //Reset to factory defaults
    status = sendCommand("ATF\r\n");
    if (status)
      break;

    //Set server true for transmitter
    sprintf(buffer, "at-Server=%d\r\n", transmitter ? 1 : 0);
    status = sendCommand(buffer);
    if (status)
      break;

    //Set the operating mode
    switch(mode)
    {
      default:
        status = -1;
        fflush(stdout);
        fprintf(stderr, "ERROR: Invalid LoRaSerial mode!\n");
        fflush(stderr);
        break;

      case 'm':
      case 'p':
        //Set the operating mode
        mode = (mode == 'p');
        sprintf(buffer, "at-OperatingMode=%d\r\n", mode);
        status = sendCommand(buffer);
        if (status)
          break;

        //Select the LED display mode
        sprintf(buffer, "at-SelectLedUse=%d\r\n", mode);
        status = sendCommand(buffer);
        break;
    }
    if (status)
      break;

    //Reduce the transmit power
    status = sendCommand("at-TxPower=14\r\n");
    if (status)
      break;

    //Set the airSpeed
    sprintf(buffer, "at-AirSpeed=%d\r\n", airSpeed);
    status = sendCommand(buffer);
    if (status)
      break;

    //Set the test network
    status = sendCommand("at-NetId=255\r\n");
    if (status)
      break;

    //Set the test network
    if (SEND_ENCRYPTION_KEY)
    {
      status = sendCommand("at-EncryptionKey=01020304050607080910111213141516\r\n");
      if (status)
        break;
    }

    //Set the serial delay
    sprintf(buffer, "at-SerialDelay=%d\r\n", serialDelay - 1);
    status = sendCommand(buffer);
    if (status)
      break;

    //Save the parameters
    status = sendCommand("ATW\r\n");
    if (status)
      break;

    //Exit command mode
    status = sendCommand("ATO\r\n");
    if (status)
      break;

    //Remember the starting time
    clock_gettime(CLOCK_MONOTONIC, &startTime);

    //Delay while the radios sync with one another
    printf("Delaying %d seconds to allow the radios to synchronize\n", SYNC_DELAY_SECONDS);
    fflush(stdout);
    clock_gettime(CLOCK_MONOTONIC, &startDelayTime);
    do
    {
      clock_gettime(CLOCK_MONOTONIC, &endTime);
      endTime = timeSubtract(&endTime, &startDelayTime);
    }
    while (endTime.tv_sec < SYNC_DELAY_SECONDS);
    printf("\n");
  } while (0);
  return status;
}

//Display the test status
void printTestStatus(int status)
{
  if (status)
  {
    printf("\n");
    printf("    FFFFFFFFFF     AA      IIIII  L\n");
    printf("    F             A  A       I    L\n");
    printf("    F            A    A      I    L\n");
    printf("    F           A      A     I    L\n");
    printf("    FFFFFF     AAAAAAAAAA    I    L\n");
    printf("    F          A        A    I    L\n");
    printf("    F          A        A    I    L\n");
    printf("    F          A        A    I    L\n");
    printf("    F          A        A  IIIII  LLLLLLLLLL\n");
    printf("\n");
  }
  else
  {
    printf("\n");
    printf("    PPPPPPPP        AA        SSSSSS      SSSSSS\n");
    printf("    P       P      A  A      S      S    S      S\n");
    printf("    P        P    A    A    S           S\n");
    printf("    P       P    A      A    S           S\n");
    printf("    PPPPPPPP    AAAAAAAAAA    SSSSSS      SSSSSS\n");
    printf("    P           A        A          S           S\n");
    printf("    P           A        A           S           S\n");
    printf("    P           A        A   S      S    S      S\n");
    printf("    P           A        A    SSSSSS      SSSSSS\n");
    printf("\n");
  }
}

//Display the test type
void displayTestInfo(const char * testType, char mode, int airSpeed, int packets)
{
  const char * testMode;

  //Determine the amount of data being sent
  computeTxData(packets);

  //Determine the test mode
  switch (mode)
  {
    default:
      testMode = "Unknown";
      break;

    case 'm':
      testMode = "Multipoint";
      break;

    case 'p':
      testMode = "Point-to-point";
      break;
  }

  //Display the test info
  printf("%s Test\n", testType);
  printf("    Mode: %s\n", testMode);
  printf("    AirSpeed: %d\n", airSpeed);
  printf("    Packets:  %d\n", packets);
  printf("    Approximate test time: ");
  printTestTime(airSpeed);
  printf("\n");
  printf("\n");
}

//Transmit the entries in the table
int transmitEntriesInTable(char mode, int airSpeed, int packets, char * terminal)
{
  double bitsPerSecond;
  int bytesSent;
  int bytesToSend;
  int bytesTransmitted;
  int count;
  const uint8_t * data;
  uint64_t days;
  struct timespec deltaTime;
  struct timespec endTime;
  int fixedDelay;
  uint64_t hours;
  int listIndex;
  uint64_t milliseconds;
  uint64_t minutes;
  int packetsTransmitted;
  uint64_t seconds;
  int status;
  double testTime;
  uint64_t timeDelay;
  struct timespec timeout;

  do
  {
    //Initialize the test
    displayTestInfo("Transmit", mode, airSpeed, packets);
    status = configureRadio(terminal, true, mode, airSpeed);
    if (status)
      break;
    bytesTransmitted = 0;
    packetsTransmitted = 0;

    //Start the test
    printf("Running the test\n");
    fflush(stdout);

    //Transmit processing
    //
    //1. Get transmit parameters from command line
    //      Transmit speed
    //      Port
    //2. Start at head of list
    //3. While more entries to send
    //      a. Delay the nessary time
    //      b. Send the data
    //      c. Wait for approximate transmit duration
    //4. Send end of transmission
    //5. Display summary
    //      packets/bytes transmitted

    for (count = 0; count < packets; count++)
    {
      if (DISPLAY_PINWHEEL)
      {
        //Display the pinwheel
        switch(count % 4)
        {
          case 0:
            printf("\r|");
            break;
          case 1:
            printf("\r/");
            break;
          case 2:
            printf("\r-");
            break;
          case 3:
            printf("\r\\");
            break;
        }
        fflush(stdout);
      }

      //Delay for the required time before the transmission
      listIndex = count % txListEntries;
      milliseconds = txList[listIndex].mSecDelay + serialDelay;
      timeout.tv_sec = milliseconds / 1000;
      timeout.tv_nsec = (milliseconds % 1000) * 1000 * 1000;
      do
        status = nanosleep(&timeout, &timeout);
      while (status == EINTR);
      if (status)
        break;

      //Send the data
      status = sendData(txList[listIndex].byteCount, txList[listIndex].data);
      if (status)
        break;
      bytesTransmitted += txList[listIndex].byteCount;
      packetsTransmitted += 1;

      //Wait for the approximate transmit duration
      timeDelay = txList[listIndex].byteCount * 8;
      timeDelay *= NANOSECONDS_IN_A_SECOND;
      timeDelay = (timeDelay + (airSpeed / 2)) / airSpeed;
      timeout.tv_sec = timeDelay / NANOSECONDS_IN_A_SECOND;
      timeout.tv_nsec = timeDelay % NANOSECONDS_IN_A_SECOND;
      do
        status = nanosleep(&timeout, &timeout);
      while (status == EINTR);
      if (status)
        break;
    }
    printf("\r");

    //Display the delta time
    clock_gettime(CLOCK_MONOTONIC, &endTime);
    if (endTime.tv_nsec < startTime.tv_nsec)
    {
      endTime.tv_nsec += NANOSECONDS_IN_A_SECOND;
      endTime.tv_sec -= 1;
    }
    deltaTime = timeSubtract(&endTime, &startTime);
    printf("Test Duration:   ");
    displayTimeSpec(&deltaTime);

    //Compute the fixed delay
    fixedDelay = 0;
    for (count = 0; count < packetsTransmitted; count++)
    {
      listIndex = count % txListEntries;
      fixedDelay += txList[listIndex].mSecDelay + serialDelay;
    }

    //Display the sync time
    printf("    Sync Time:   ");
    displaySeconds(SYNC_DELAY_SECONDS);
    printf("\n");

    //Display the fixed delay
    printf("    Fixed Delay: ");
    displayMilliseconds(fixedDelay);
    printf("\n");

    //Remove the sync and fixed delay times
    endTime.tv_sec -= SYNC_DELAY_SECONDS;
    endTime.tv_sec -= fixedDelay / MILLISECONDS_IN_A_SECOND;
    endTime.tv_nsec -= (fixedDelay % MILLISECONDS_IN_A_SECOND) * MICROSECONDS_IN_A_SECOND;
    if (endTime.tv_nsec < 0)
    {
      endTime.tv_nsec += NANOSECONDS_IN_A_SECOND;
      endTime.tv_sec -= 1;
    }
    if (endTime.tv_nsec < startTime.tv_nsec)
    {
      endTime.tv_nsec += NANOSECONDS_IN_A_SECOND;
      endTime.tv_sec -= 1;
    }

    //Compute the test time
    deltaTime = timeSubtract(&endTime, &startTime);
    testTime = (double)deltaTime.tv_nsec;
    testTime /= (double)NANOSECONDS_IN_A_SECOND;
    testTime += (double)deltaTime.tv_sec;

    //Display the transmission time
    deltaTime = timeSubtract(&endTime, &startTime);
    printf("    TX Time:     ");
    displayTimeSpec(&deltaTime);
    printf("\n");

    //Display the approximate data rate
    //Assume on average the ACK occurs during the fixed delay
    bitsPerSecond = ((double)(bytesTransmitted + (packetsTransmitted * headerBytes)) * 8.) / testTime;
    printf("Transmission Bit Rate: %.0f bits/second\n", bitsPerSecond);

    //Display the approximate data rate
    //Assume on average the ACK occurs during the fixed delay
    bitsPerSecond = ((double)bytesTransmitted * 8.) / testTime;
    printf("User bytes sent: %d\n", bytesTransmitted);
    printf("User Data Rate: %.0f bits/second\n", bitsPerSecond);
  } while (0);

  printTestStatus(status);
  return status;
}

//Receive data from the transmitter
int receiveData(char mode, int airSpeed, int packets, char * terminal)
{
  int availableBytes;
  double bitsPerSecond;
  static char buffer[65536];
  int bytesRead;
  int compareStatus;
  fd_set currentfds;
  uint64_t days;
  struct timespec deltaTime;
  struct timespec endTime;
  const uint8_t * expectedData;
  bool firstPacket;
  int fixedDelay;
  uint64_t hours;
  int index;
  int maxfds;
  uint64_t milliseconds;
  uint64_t minutes;
  int numfds;
  fd_set readfds;
  static int rxBytes;
  int rxPackets;
  struct timespec rxStartTime;
  struct timespec rxEndTime;
  uint64_t seconds;
  int status;
  struct timespec syncTime;
  double testTime;
  struct timeval timeout;
  struct timespec timeoutTime;
  int timeoutCount;
  struct timespec waitStart;
  struct timespec waitTime;

  do
  {
    //Initialize the test
    displayTestInfo("Receive", mode, airSpeed, packets);
    status = configureRadio(terminal, false, mode, airSpeed);
    clock_gettime(CLOCK_MONOTONIC, &waitStart);
    rxStartTime = waitStart;
    rxEndTime = waitStart;
    endTime = waitStart;
    if (status)
      break;
    expectedData = dataBuffer;
    firstPacket = true;
    rxBytes = 0;
    timeoutCount = 0;

    //Start the test
    printf("Starting receive test\n");
    fflush(stdout);

    //Receive processing
    //
    //1. Receive the packet
    //2. Search packet list for received packet starting at next expected packet
    //3. If any packets lost, indicate lost packets
    //4. At end of transmission or timeout, Display summary:
    //      packets/bytes received
    //      packets/bytes lost
    //      Received packet percentage

    //Wait for a read timeout from the LoRaSerial radio
    maxfds = radio;
    FD_ZERO(&readfds);
    FD_SET(radio, &readfds);
    while (1)
    {
      //Set the timeout
      timeout.tv_sec = 0;
      timeout.tv_usec = 50 * 1000;

      //Wait for receive data or timeout
      memcpy((void *)&currentfds, (void *)&readfds, sizeof(readfds));
      numfds = select(maxfds + 1, &currentfds, NULL, NULL, &timeout);
      if (numfds < 0)
      {
        perror("ERROR: select call failed!");
        status = errno;
        break;
      }

      //Determine if data was received
      if (numfds)
      {
        //Read the data from the radio
        bytesRead = read(radio, buffer, sizeof(buffer));
        if (bytesRead < 0)
        {
          status = errno;
          clock_gettime(CLOCK_MONOTONIC, &endTime);
          fflush(stdout);
          fprintf(stderr, "%s\n", strerror(status));
          fflush(stderr);
          break;
        }
        if (firstPacket)
        {
          clock_gettime(CLOCK_MONOTONIC, &rxStartTime);
          firstPacket = false;
        }
        clock_gettime(CLOCK_MONOTONIC, &rxEndTime);

        //Display the response
        if (DISPLAY_RX_DATA)
          dumpBuffer((uint8_t *)buffer, bytesRead);


        //Display the pinwheel
        if (timeoutCount > 5)
        {
          if (DISPLAY_PINWHEEL)
          {
            static int pinwheelDirection;

            switch(pinwheelDirection++ % 4)
            {
              case 0:
                printf("\r|");
                break;
              case 1:
                printf("\r/");
                break;
              case 2:
                printf("\r-");
                break;
              case 3:
                printf("\r\\");
                break;
            }
            fflush(stdout);
          }
        }

        //Reset the timeout count
        timeoutCount = 0;

        //Compare the data
        availableBytes = sizeof(dataBuffer) - (expectedData - &dataBuffer[0]);
        if (availableBytes > bytesRead)
          availableBytes = bytesRead;
        compareStatus = memcmp(buffer, expectedData, availableBytes);
        if (compareStatus == 0)
        {
          rxBytes += availableBytes;
          expectedData += availableBytes;
          if (expectedData >= &dataBuffer[sizeof(dataBuffer)])
            expectedData = dataBuffer;
          if (availableBytes < bytesRead)
          {
            compareStatus = memcmp(&buffer[availableBytes], expectedData, bytesRead - availableBytes);
            if (compareStatus == 0)
              rxBytes += bytesRead - availableBytes;
            else
            {
              //Display the failure
              printf("\r");
              printf("RX failed at byte %d\n", rxBytes);
              printf("Expected:\n");
              dumpBuffer((uint8_t *)expectedData, bytesRead - availableBytes);
              printf("\n");
              printf("Received:\n");
              dumpBuffer((uint8_t *)buffer, bytesRead - availableBytes);
              status = -1;
              break;
            }
          }
        }
        else
        {
          //Display the failure
          printf("\r");
          printf("RX failed at byte %d\n", rxBytes);
          printf("Expected:\n");
          dumpBuffer((uint8_t *)expectedData, availableBytes);
          printf("\n");
          printf("Received:\n");
          dumpBuffer((uint8_t *)buffer, availableBytes);
          status = -1;
          break;
        }
      }
      else
      {
        //Timeout for pinwheel update
        timeoutCount += 1;

        //Determine if a timeout occurred
        clock_gettime(CLOCK_MONOTONIC, &deltaTime);
        if (deltaTime.tv_sec - rxEndTime.tv_sec > RECEIVE_TIMEOUT_SEC)
        {
          if (rxBytes != totalBytes)
            status = -1;
          break;
        }
      }
    }
  } while (0);
  clock_gettime(CLOCK_MONOTONIC, &endTime);
  printf("\r");

  //Count the received packets and determine the fixed delay
  availableBytes = rxBytes;
  fixedDelay = 0;
  rxPackets = 0;
  index = 0;
  while (availableBytes > 0)
  {
    fixedDelay += txList[index].mSecDelay + serialDelay;
    rxPackets += 1;
    availableBytes -= txList[index++].byteCount;
    index %= sizeof(txList) / sizeof(TRANSMIT_ENTRY);
  }


  //Display the time line
  printf("\n");
  printf("Start           RX Start                             RX End        End\n");
  printf("  |______|__________|_____________|_____________________|___________|\n");
  printf("    Sync     Wait        RX            Fixed Delay        Timeout\n");
  printf("\n");

  //Display the test time
  deltaTime = timeSubtract(&endTime, &startTime);
#if DEBUG_RX_TIMES
  printf("endTime:   %9ld.%09ld\n", endTime.tv_sec, waitStart.tv_nsec);
  printf("startTime: %9ld.%09ld\n", startTime.tv_sec, startTime.tv_nsec);
#endif //DEBUG_RX_TIMES
  printf("Test Duration:   ");
  displayTimeSpec(&deltaTime);
  printf("\n");

  //Display the sync time
  syncTime = timeSubtract(&waitStart, &startTime);
#if DEBUG_RX_TIMES
  printf("waitStart: %9ld.%09ld\n", waitStart.tv_sec, waitStart.tv_nsec);
  printf("startTime: %9ld.%09ld\n", startTime.tv_sec, startTime.tv_nsec);
#endif //DEBUG_RX_TIMES
  printf("    Sync Time:   ");
  displayTimeSpec(&syncTime);
  printf("\n");

  //Display the wait time
  waitTime = timeSubtract(&rxStartTime, &waitStart);
  if (rxBytes)
    waitTime.tv_nsec -= (txList[0].mSecDelay + serialDelay) * MICROSECONDS_IN_A_SECOND;
  if (waitTime.tv_nsec < 0)
  {
    waitTime.tv_nsec += NANOSECONDS_IN_A_SECOND;
    waitTime.tv_sec -= 1;
  }
#if DEBUG_RX_TIMES
  printf("rxStartTime: %9ld.%09ld\n", rxStartTime.tv_sec, rxStartTime.tv_nsec);
  printf("waitStart:   %9ld.%09ld\n", waitStart.tv_sec, waitStart.tv_nsec);
#endif //DEBUG_RX_TIMES
  printf("    Wait time:   ");
  displayTimeSpec(&waitTime);
  printf("\n");

  //Display the timeout
  timeoutTime = timeSubtract(&endTime, &rxEndTime);
#if DEBUG_RX_TIMES
  printf("endTime:   %9ld.%09ld\n", endTime.tv_sec, waitStart.tv_nsec);
  printf("rxEndTime: %9ld.%09ld\n", rxEndTime.tv_sec, rxEndTime.tv_nsec);
#endif //DEBUG_RX_TIMES
  printf("    Timeout:     ");
  displayTimeSpec(&timeoutTime);
  printf("\n");

  //Display the fixed delay
  printf("    Fixed Delay: ");
  displayMilliseconds(fixedDelay);
  printf("\n");

  // Start           RX Start                             RX End        End
  //   |______|__________|_____________|_____________________|___________|
  //     Sync     Wait        RX            Fixed Delay        Timeout
  //
  //Remove the sync time, wait time, fixed delay and receive timeout
  //Assume on average the ACK occurs during the fixed delay
  deltaTime = timeSubtract(&endTime, &waitStart);
  if (memcmp(&startTime, &waitStart, sizeof(startTime)) != 0)
    deltaTime.tv_sec -= SYNC_DELAY_SECONDS;
  deltaTime.tv_sec -= waitTime.tv_sec;
  deltaTime.tv_nsec -= waitTime.tv_nsec;
  deltaTime.tv_sec -= fixedDelay / MILLISECONDS_IN_A_SECOND;
  deltaTime.tv_nsec -= (fixedDelay % MILLISECONDS_IN_A_SECOND) * MICROSECONDS_IN_A_SECOND;
  deltaTime.tv_sec -= timeoutTime.tv_sec;
  deltaTime.tv_nsec -= timeoutTime.tv_nsec;
  while (deltaTime.tv_nsec < 0)
  {
    deltaTime.tv_nsec += NANOSECONDS_IN_A_SECOND;
    deltaTime.tv_sec -= 1;
  }
  if (deltaTime.tv_nsec < rxStartTime.tv_nsec)
  {
    deltaTime.tv_nsec += NANOSECONDS_IN_A_SECOND;
    deltaTime.tv_sec -= 1;
  }

  //Display the receive time
  deltaTime = timeSubtract(&rxEndTime, &rxStartTime);
#if DEBUG_RX_TIMES
  printf("rxEndTime: %9ld.%09ld\n", rxEndTime.tv_sec, rxEndTime.tv_nsec);
  printf("rxStartTime: %9ld.%09ld\n", rxStartTime.tv_sec, rxStartTime.tv_nsec);
#endif //DEBUG_RX_TIMES
  printf("    RX Time:     ");
  displayTimeSpec(&deltaTime);
  printf("\n");

  //Display the test data
  printf("Expected packets: %d\n", totalPackets);
  printf("Packets received: %d\n", rxPackets);
  printf("Expected bytes: %d\n", totalBytes);
  printf("Bytes received: %d\n", rxBytes);
  printf("Percent of data received: %.3f %%\n", ((double)rxBytes) * 100. / (double)totalBytes);

  //Ignore the first packet
  rxBytes -= txList[0].byteCount;
  rxPackets -= 1;

  //Display the bit rates
  if (rxBytes > 0)
  {
    //Compute the test RX time
    testTime = (double)deltaTime.tv_nsec;
    testTime /= (double)NANOSECONDS_IN_A_SECOND;
    testTime += (double)deltaTime.tv_sec;

    //Display the approximate data rate
    bitsPerSecond = (((double)(rxBytes + (rxPackets * headerBytes))) * 8) / testTime;
    printf("Receive Bit Rate: %.0f bits/second\n", bitsPerSecond);

    //Display the approximate data rate
    bitsPerSecond = ((double)rxBytes * 8) / testTime;
    printf("User Data Rate: %.0f bits/second\n", bitsPerSecond);
  }
  printTestStatus(status);
  return status;
}

//Program to test LoRaSerial
int main(int argc, char **argv)
{
  int airSpeed;
  int displayHelp;
  int entries;
  int index;
  char mode;
  int packets;
  int status;
  static char terminal[16384];

  displayHelp = true;
  status = -1;
  do
  {
    if (argc != 2)
      break;

    //Check for table generation
    if ((sscanf(argv[1], "--table=%d\n", &entries) == 1) && (entries > 0) && (entries <= 100000))
    {
      displayHelp = false;
      createHeaderFile(entries);
      status = 0;
      break;
    }

    //Check for table analysis
    if (sscanf(argv[1], "--analyze=%d", &packets) == 1)
    {
      displayHelp = false;
      analyzeTable(packets);
      status = 0;
      break;
    }

    //Check for transmit
    if (sscanf(argv[1], "--transmit=%d,%c,%d,%s", &packets, &mode, &airSpeed, &terminal[0]) == 4)
    {
      displayHelp = false;
      status = transmitEntriesInTable(mode, airSpeed, packets, terminal);
      break;
    }

    //Check for receive
    if (sscanf(argv[1], "--receive=%d,%c,%d,%s", &packets, &mode, &airSpeed, &terminal[0]) == 4)
    {
      displayHelp = false;
      status = receiveData(mode, airSpeed, packets, terminal);
      break;
    }
  } while (0);

  if (terminalOpen)
    close(radio);

  if (displayHelp)
  {
    printf("%s   [option]\n", argv[0]);
    printf("    --analyze=xxx - Analyze the TX list\n");
    printf("            xxx - Number of TX entries to analyze\n");
    printf("    --receive=xxx,mode,airSpeed,port - Receive data\n");
    printf("            xxx - Expected packets to be received\n");
    printf("            mode - m: multipoint\n");
    printf("                   p: point-to-point\n");
    printf("            airSpeed - One of (40, 150, 400, 1200, 2400, 4800, 9600, 19200, 28800, 38400\n");
    printf("            port - LoRaSerial COM port\n");
    printf("    --table=xxx - Generate transmit table with xxx entries\n");
    printf("    --transmit=xxx,mode,airSpeed,port - Transmit data\n");
    printf("            xxx - Number of packets to send\n");
    printf("            mode - m: multipoint\n");
    printf("                   p: point-to-point\n");
    printf("            airSpeed - One of (40, 150, 400, 1200, 2400, 4800, 9600, 19200, 28800, 38400\n");
    printf("            port - LoRaSerial COM port\n");
  }

  fflush(stdout);
  return status;
}
