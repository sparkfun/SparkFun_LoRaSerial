#define ADD_VC_STATE_NAMES_TABLE
#include <sys/resource.h>
#include <sys/time.h>
#include "settings.h"

#ifndef POLL_TIMEOUT_USEC
#define POLL_TIMEOUT_USEC       1000
#endif  // POLL_TIMEOUT_USEC

#define ONE_SECOND_COUNT        20 // (1000000 / POLL_TIMEOUT_USEC)
#define COMMAND_POLL_COUNT      (ONE_SECOND_COUNT / 10) //100 mSec
#define STALL_CHECK_COUNT       15 * ONE_SECOND_COUNT  //15 Sec

#define BUFFER_SIZE             2048
#define INPUT_BUFFER_SIZE       BUFFER_SIZE
#define MAX_MESSAGE_SIZE        32
#define STDIN                   0
#define STDOUT                  1
#define STDERR                  2

#define BREAK_LINKS_COMMAND     "atb"
#define GET_DEVICE_INFO         "ati"
#define GET_MY_VC_ADDRESS       "ati30"
#define GET_UNIQUE_ID           "ati8"
#define GET_VC_STATE            "ati31"
#define GET_VC_STATUS           "ata"
#define LINK_RESET_COMMAND      "atz"
#define MY_VC_ADDRESS           "myVc: "
#define START_3_WAY_HANDSHAKE   "atc"

#define DEBUG_LOCAL_COMMANDS      0
#define DEBUG_PC_CMD_ISSUE        0
#define DEBUG_PC_TO_RADIO         0
#define DEBUG_RADIO_TO_PC         0
#define DISPLAY_COMMAND_COMPLETE  0
#define DISPLAY_DATA_ACK          0
#define DISPLAY_DATA_NACK         1
#define DISPLAY_STATE_TRANSITION  0
#define DISPLAY_UNKNOWN_COMMANDS  0
#define DISPLAY_VC_STATE          0
#define DUMP_RADIO_TO_PC          0

#define QUEUE_T                   uint32_t
#define QUEUE_T_BITS              ((int)(sizeof(QUEUE_T) * 8))
#define QUEUE_T_MASK              (QUEUE_T_BITS - 1)
#define COMMAND_QUEUE_SIZE        ((CMD_LIST_SIZE + QUEUE_T_MASK) / QUEUE_T_BITS)

#define SECS_IN_MINUTE            60
#define SECS_IN_HOUR              (60 * SECS_IN_MINUTE)
#define SECS_IN_DAY               (24 * SECS_IN_HOUR)

#define MILLIS_IN_SECOND          1000
#define MILLIS_IN_MINUTE          (60 * MILLIS_IN_SECOND)
#define MILLIS_IN_HOUR            (60 * MILLIS_IN_MINUTE)
#define MILLIS_IN_DAY             (24 * MILLIS_IN_HOUR)

#define ARRAY_SIZE(x)           (sizeof(x) / sizeof(x[0]))

#define COMMAND_COMPLETE(queue, active)                               \
{                                                                     \
  if (COMMAND_PENDING(queue, active))                                 \
  {                                                                   \
    queue[active / QUEUE_T_BITS] &= ~(1 << (active & QUEUE_T_MASK));  \
    active = CMD_LIST_SIZE;                                           \
  }                                                                   \
}

#define COMMAND_ISSUE(queue, pollCount, cmd)              \
{                                                         \
  /* Place the command in the queue */                    \
  queue[cmd / QUEUE_T_BITS] |= 1 << (cmd & QUEUE_T_MASK); \
                                                          \
  /* Timeout the command processor */                     \
  if (!commandProcessorRunning)                           \
    commandProcessorRunning = STALL_CHECK_COUNT;          \
                                                          \
  /* Remember when this command was issued */             \
  if (!pollCount)                                         \
  {                                                       \
    if (timeoutCount)                                     \
      pollCount = timeoutCount;                           \
    else                                                  \
      pollCount = 1;                                      \
  }                                                       \
}

#define COMMAND_PENDING(queue,cmd)  ((queue[cmd / QUEUE_T_BITS] >> (cmd & QUEUE_T_MASK)) & 1)

typedef enum
{
  //List commands in priority order
  CMD_ATI30 = 0,              //Get myVC
  CMD_ATB,                    //Break all VC links
  CMD_ATA,                    //Get VC status

  //Connect to the remote radio
  CMD_AT_CMDVC,               //Select target VC
  CMD_ATC,                    //Start the 3-way handshake
  CMD_WAIT_CONNECTED,         //Wait until the client is connected

  //Get remote radio connection status, type and ID
  CMD_AT_CMDVC_2,             //Select target VC
  CMD_ATI31,                  //Get the VC state
  CMD_ATI,                    //Get the device type
  CMD_ATI8,                   //Get the radio's unique ID

  //Last in the list
  CMD_LIST_SIZE
} COMMANDS;

const char * const commandName[] =
{
  "ATI30", "ATIB", "ATA", "AT-CMDVC", "ATC",
  "WAIT_CONNECT",
  "AT-CMDVC_2", "ATI31", "ATI", "ATI8",
};

typedef struct _VIRTUAL_CIRCUIT
{
  int vcState;
  uint32_t activeCommand;
  QUEUE_T commandQueue[COMMAND_QUEUE_SIZE];
  uint32_t commandTimer;
  uint8_t uniqueId[UNIQUE_ID_BYTES];
  bool valid;
} VIRTUAL_CIRCUIT;

uint32_t commandProcessorRunning;
bool commandStatus;
bool findMyVc;
uint8_t inputBuffer[INPUT_BUFFER_SIZE];
int myVc = VC_SERVER;
uint8_t outputBuffer[VC_SERIAL_HEADER_BYTES + BUFFER_SIZE];
uint32_t pcActiveCommand = CMD_LIST_SIZE;
char pcCommandBuffer[128];
QUEUE_T pcCommandQueue[COMMAND_QUEUE_SIZE];
uint32_t pcCommandTimer;
int pcCommandVc = MAX_VC;
uint8_t remoteCommandVc;
int remoteVc;
uint32_t timeoutCount;
char vcCommandBuffer[MAX_VC][128];
VIRTUAL_CIRCUIT virtualCircuitList[MAX_VC];
volatile bool waitingForCommandComplete;

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

int cmdToRadio(uint8_t * buffer, int length)
{
  int bytesSent;
  int bytesWritten;
  VC_SERIAL_MESSAGE_HEADER header;

  //Build the virtual circuit serial header
  header.start = START_OF_VC_SERIAL;
  header.radio.length = VC_RADIO_HEADER_BYTES + length;
  header.radio.destVc = VC_COMMAND;
  header.radio.srcVc = PC_COMMAND;

  //Display the data being sent to the radio
  if (DEBUG_PC_TO_RADIO)
  {
    dumpBuffer((uint8_t *)&header, VC_SERIAL_HEADER_BYTES);
    dumpBuffer(buffer, length);
  }
  if (DEBUG_LOCAL_COMMANDS)
    printf("Sending LoRaSerial command: %s\n", buffer);

  //Send the header
  bytesWritten = write(radio, (uint8_t *)&header, VC_SERIAL_HEADER_BYTES);
  if (bytesWritten < (int)VC_SERIAL_HEADER_BYTES)
  {
    perror("ERROR: Write of header to radio failed!");
    return -1;
  }

  //Send the message
  bytesSent = 0;
  while (bytesSent < length)
  {
    bytesWritten = write(radio, &buffer[bytesSent], length - bytesSent);
    if (bytesWritten < 0)
    {
      perror("ERROR: Write of data to radio failed!");
      return bytesWritten;
    }
    bytesSent += bytesWritten;
  }

  //Return the amount of the buffer that was sent
  return length;
}

int hostToRadio(uint8_t destVc, uint8_t * buffer, int length)
{
  int bytesSent;
  int bytesWritten;
  VC_SERIAL_MESSAGE_HEADER header;

  //Build the virtual circuit serial header
  header.start = START_OF_VC_SERIAL;
  header.radio.length = VC_RADIO_HEADER_BYTES + length;
  header.radio.destVc = destVc;
  header.radio.srcVc = myVc;

  //Display the data being sent to the radio
  if (DEBUG_PC_TO_RADIO)
  {
    dumpBuffer((uint8_t *)&header, VC_SERIAL_HEADER_BYTES);
    dumpBuffer(buffer, length);
  }

  //Send the header
  bytesWritten = write(radio, (uint8_t *)&header, VC_SERIAL_HEADER_BYTES);
  if (bytesWritten < (int)VC_SERIAL_HEADER_BYTES)
  {
    perror("ERROR: Write of header to radio failed!");
    return -1;
  }

  //Send the message
  bytesSent = 0;
  while (bytesSent < length)
  {
    bytesWritten = write(radio, &buffer[bytesSent], length - bytesSent);
    if (bytesWritten < 0)
    {
      perror("ERROR: Write of data to radio failed!");
      return bytesWritten;
    }
    bytesSent += bytesWritten;
  }

  //Return the amount of the buffer that was sent
  return length;
}

int stdinToRadio()
{
  int bytesRead;
  int bytesSent;
  int bytesToSend;
  int bytesWritten;
  int length;
  int maxfds;
  int status;
  struct timeval timeout;
  static int index;

  status = 0;
  if (!waitingForCommandComplete)
  {
    if (remoteVc == VC_COMMAND)
    {
      do
      {
        do
        {
          //Read the console input data into the local buffer.
          bytesRead = read(STDIN, &inputBuffer[index], 1);
          if (bytesRead < 0)
          {
            perror("ERROR: Read from stdin failed!");
            status = bytesRead;
            break;
          }
          index += bytesRead;
        } while (bytesRead && (inputBuffer[index - bytesRead] != '\n'));

        //Check for end of data
        if (!bytesRead)
          break;

        //Send this command the VC
        bytesWritten = cmdToRadio(inputBuffer, index);
        waitingForCommandComplete = true;
        remoteCommandVc = myVc;
        index = 0;
      } while (0);
    }
    else
      do
      {
        //Read the console input data into the local buffer.
        bytesRead = read(STDIN, inputBuffer, BUFFER_SIZE);
        if (bytesRead < 0)
        {
          perror("ERROR: Read from stdin failed!");
          status = bytesRead;
          break;
        }

        //Determine if this is a remote command
        if ((remoteVc >= PC_REMOTE_COMMAND) && (remoteVc < THIRD_PARTY_COMMAND))
        {
          remoteCommandVc = remoteVc & VCAB_NUMBER_MASK;
          waitingForCommandComplete = true;
        }

        //Send this data over the VC
        bytesSent = 0;
        while (bytesSent < bytesRead)
        {
          //Break up the data if necessary
          bytesToSend = bytesRead - bytesSent;
          if (bytesToSend > MAX_MESSAGE_SIZE)
            bytesToSend = MAX_MESSAGE_SIZE;

          //Send the data
          bytesWritten = hostToRadio(remoteVc, &inputBuffer[bytesSent], bytesToSend);
          if (bytesWritten < 0)
          {
            perror("ERROR: Write to radio failed!");
            status = bytesWritten;
            break;
          }

          //Account for the bytes written
          bytesSent += bytesWritten;
        }
      } while (0);
  }
  return status;
}

int hostToStdout(VC_SERIAL_MESSAGE_HEADER * header, uint8_t * data, uint8_t bytesToSend)
{
  uint8_t * buffer;
  uint8_t * bufferEnd;
  int bytesSent;
  int bytesWritten;
  static uint8_t compareBuffer[4 * BUFFER_SIZE];
  static int offset;
  int status;
  int vcNumber;

  //Locate myVc if necessary
  if (findMyVc)
  {
    //Place the data into the compare buffer
    buffer = compareBuffer;
    memcpy(&compareBuffer[offset], data, bytesToSend);
    offset += bytesToSend;
    bufferEnd = &buffer[offset];

    //Walk through the buffer
    while (buffer < bufferEnd)
    {
      if ((strncmp((const char *)buffer, MY_VC_ADDRESS, strlen(MY_VC_ADDRESS)) == 0)
        && (&buffer[strlen(MY_VC_ADDRESS) + 3] <= bufferEnd))
      {
        if ((sscanf((const char *)&buffer[strlen(MY_VC_ADDRESS)], "%d", &vcNumber) == 1)
          && ((uint8_t)vcNumber < MAX_VC))
        {
          findMyVc = false;

          //Set the local radio's VC number
          myVc = (int8_t)vcNumber;
          printf("myVc: %d\n", myVc);

          //Complete this command
          COMMAND_COMPLETE(pcCommandQueue, pcActiveCommand);
          break;
        }
      }

      //Skip to the end of the line
      while ((buffer < bufferEnd) && (*buffer != '\n'))
        buffer++;

      if ((buffer < bufferEnd) && (*buffer == '\n'))
      {
        //Skip to the next line
        while ((buffer < bufferEnd) && (*buffer == '\n'))
          buffer++;

        //Move this data to the beginning of the buffer
        offset = bufferEnd - buffer;
        memcpy(compareBuffer, buffer, offset);
        buffer = compareBuffer;
        bufferEnd = &buffer[offset];
      }
    }
  }

  //Write this data to stdout
  bytesSent = 0;
  status = 0;
  while (bytesSent < bytesToSend)
  {
    bytesWritten = write(STDOUT, &data[bytesSent], bytesToSend - bytesSent);
    if (bytesWritten < 0)
    {
      perror("ERROR: Write to stdout!");
      status = bytesWritten;
      break;
    }

    //Account for the bytes written
    bytesSent += bytesWritten;
  }
  return status;
}

void radioToPcLinkStatus(VC_SERIAL_MESSAGE_HEADER * header, uint8_t * data, uint8_t length)
{
  int newState;
  int previousState;
  int srcVc;
  uint8_t uniqueId[UNIQUE_ID_BYTES];
  VC_STATE_MESSAGE * vcMsg;

  //Remember the previous state
  srcVc = header->radio.srcVc;
  previousState = virtualCircuitList[srcVc].vcState;
  vcMsg = (VC_STATE_MESSAGE *)&header[1];

  //Set the new state
  newState = vcMsg->vcState;
  virtualCircuitList[srcVc].vcState = newState;

  //Save the LoRaSerial radio's unique ID
  //Determine if the PC's value is valid
  memset(uniqueId, UNIQUE_ID_ERASE_VALUE, sizeof(uniqueId));
  if (!virtualCircuitList[srcVc].valid)
  {
    //Determine if the radio knows the value
    if (memcmp(vcMsg->uniqueId, uniqueId, sizeof(uniqueId)) != 0)
    {
      //The radio knows the value, save it in the PC
      memcpy(virtualCircuitList[srcVc].uniqueId, vcMsg->uniqueId, sizeof(vcMsg->uniqueId));
      virtualCircuitList[srcVc].valid = true;

      //Display this ID value
      printf("VC %d unique ID: %02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n",
             srcVc,
             vcMsg->uniqueId[0], vcMsg->uniqueId[1], vcMsg->uniqueId[2], vcMsg->uniqueId[3],
             vcMsg->uniqueId[4], vcMsg->uniqueId[5], vcMsg->uniqueId[6], vcMsg->uniqueId[7],
             vcMsg->uniqueId[8], vcMsg->uniqueId[9], vcMsg->uniqueId[10], vcMsg->uniqueId[11],
             vcMsg->uniqueId[12], vcMsg->uniqueId[13], vcMsg->uniqueId[14], vcMsg->uniqueId[15]);
    }
  }
  else
  {
    //Determine if the radio has changed for this VC
    if ((memcmp(vcMsg->uniqueId, virtualCircuitList[srcVc].uniqueId, sizeof(vcMsg->uniqueId)) != 0)
        && (memcmp(vcMsg->uniqueId, uniqueId, sizeof(uniqueId)) != 0))
    {
      //The radio knows the value, save it in the PC
      memcpy(virtualCircuitList[srcVc].uniqueId, vcMsg->uniqueId, sizeof(vcMsg->uniqueId));
      virtualCircuitList[srcVc].valid = true;

      //Display this ID value
      printf("VC %d unique ID: %02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n",
             srcVc,
             vcMsg->uniqueId[0], vcMsg->uniqueId[1], vcMsg->uniqueId[2], vcMsg->uniqueId[3],
             vcMsg->uniqueId[4], vcMsg->uniqueId[5], vcMsg->uniqueId[6], vcMsg->uniqueId[7],
             vcMsg->uniqueId[8], vcMsg->uniqueId[9], vcMsg->uniqueId[10], vcMsg->uniqueId[11],
             vcMsg->uniqueId[12], vcMsg->uniqueId[13], vcMsg->uniqueId[14], vcMsg->uniqueId[15]);
    }
  }

  //Display the state if requested
  if (DISPLAY_STATE_TRANSITION)
    printf("VC%d: %s --> %s\n", srcVc, vcStateNames[previousState], vcStateNames[newState]);
  switch (newState)
  {
  default:
    if (DEBUG_PC_CMD_ISSUE)
      printf("VC %d unknown state!\n", srcVc);
    if (DISPLAY_VC_STATE)
      printf("------- VC %d State %3d ------\n", srcVc, vcMsg->vcState);
    break;

  case VC_STATE_LINK_DOWN:
    if (DEBUG_PC_CMD_ISSUE)
      printf("VC %d DOWN\n", srcVc);
    if (DISPLAY_VC_STATE)
      printf("--------- VC %d DOWN ---------\n", srcVc);
    break;

  case VC_STATE_LINK_ALIVE:
    //Upon transition to ALIVE, if is the server or the source VC matches the
    //target VC or myVc, bring up the connection
    if ((previousState != newState)
      && ((myVc == VC_SERVER) || (srcVc == remoteVc) || (srcVc == myVc)))
    {
      if (DEBUG_PC_CMD_ISSUE)
        printf("VC %d ALIVE\n", srcVc);
      COMMAND_ISSUE(virtualCircuitList[srcVc].commandQueue, virtualCircuitList[srcVc].commandTimer, CMD_AT_CMDVC);
      COMMAND_ISSUE(virtualCircuitList[srcVc].commandQueue, virtualCircuitList[srcVc].commandTimer, CMD_ATC);
      COMMAND_ISSUE(virtualCircuitList[srcVc].commandQueue, virtualCircuitList[srcVc].commandTimer, CMD_WAIT_CONNECTED);
    }

    if (DISPLAY_VC_STATE)
      printf("-=--=--=- VC %d ALIVE =--=--=-\n", srcVc);
    break;

  case VC_STATE_SEND_UNKNOWN_ACKS:
    if (DEBUG_PC_CMD_ISSUE)
      printf("VC %d SEND_UNKNOWN_ACKS\n", srcVc);
    if (DISPLAY_VC_STATE)
      printf("-=--=-- VC %d ALIVE UA --=--=-\n", srcVc);
    break;

  case VC_STATE_WAIT_SYNC_ACKS:
    if (DEBUG_PC_CMD_ISSUE)
      printf("VC %d WAIT_SYNC_ACKS\n", srcVc);
    if (DISPLAY_VC_STATE)
      printf("-=--=-- VC %d ALIVE SA --=--=-\n", srcVc);
    break;

  case VC_STATE_WAIT_ZERO_ACKS:
    if (DEBUG_PC_CMD_ISSUE)
      printf("VC %d WAIT_ZERO_ACKS\n", srcVc);
    if (DISPLAY_VC_STATE)
    {
      if (previousState == VC_STATE_CONNECTED)
        printf("-=-=- VC %d DISCONNECTED -=-=-", srcVc);
      printf("-=--=-- VC %d ALIVE ZA --=--=-\n", srcVc);
    }
    break;

  case VC_STATE_CONNECTED:
    if (DEBUG_PC_CMD_ISSUE)
      printf("VC %d CONNECTED\n", srcVc);
    if (myVc == VC_SERVER)
    {
      if (virtualCircuitList[srcVc].activeCommand == CMD_ATC)
        COMMAND_COMPLETE(virtualCircuitList[srcVc].commandQueue, virtualCircuitList[srcVc].activeCommand);

      //Get the device information
      if (srcVc == VC_SERVER)
      {
        COMMAND_ISSUE(pcCommandQueue, pcCommandTimer, CMD_ATI);
        COMMAND_ISSUE(pcCommandQueue, pcCommandTimer, CMD_ATI8);
      }
      else
      {
        COMMAND_ISSUE(virtualCircuitList[srcVc].commandQueue, virtualCircuitList[srcVc].commandTimer, CMD_AT_CMDVC_2);
        COMMAND_ISSUE(virtualCircuitList[srcVc].commandQueue, virtualCircuitList[srcVc].commandTimer, CMD_ATI31);
        COMMAND_ISSUE(virtualCircuitList[srcVc].commandQueue, virtualCircuitList[srcVc].commandTimer, CMD_ATI);
        COMMAND_ISSUE(virtualCircuitList[srcVc].commandQueue, virtualCircuitList[srcVc].commandTimer, CMD_ATI8);
      }
    }
    COMMAND_COMPLETE(pcCommandQueue, pcActiveCommand);
    if (DISPLAY_VC_STATE)
      printf("======= VC %d CONNECTED ======\n", srcVc);
    break;
  }

  //Clear the waiting for command complete if the link fails
  if (waitingForCommandComplete && (newState != VC_STATE_CONNECTED)
    && (srcVc == remoteCommandVc))
  {
    commandStatus = VC_CMD_ERROR;
    waitingForCommandComplete = false;
  }
}

void radioDataAck(VC_SERIAL_MESSAGE_HEADER * header, uint8_t * data, uint8_t length)
{
  VC_DATA_ACK_NACK_MESSAGE * vcMsg;

  vcMsg = (VC_DATA_ACK_NACK_MESSAGE *)data;
  if (DISPLAY_DATA_ACK)
    printf("ACK from VC %d\n", vcMsg->msgDestVc);
}

void radioDataNack(VC_SERIAL_MESSAGE_HEADER * header, uint8_t * data, uint8_t length)
{
  int index;
  int vcIndex;
  VC_DATA_ACK_NACK_MESSAGE * vcMsg;

  vcMsg = (VC_DATA_ACK_NACK_MESSAGE *)data;
  vcIndex = vcMsg->msgDestVc & VCAB_NUMBER_MASK;
  if (DISPLAY_DATA_NACK)
    printf("NACK from VC %d\n", vcIndex);

  //Clear the command queue for this VC
  for (index = 0; index < COMMAND_QUEUE_SIZE; index++)
    virtualCircuitList[vcIndex].commandQueue[index] = 0;
  virtualCircuitList[vcIndex].activeCommand = CMD_LIST_SIZE;

  //Stop the command timer
  virtualCircuitList[vcIndex].commandTimer = 0;

  //Set the VC state to down
  virtualCircuitList[vcIndex].vcState = VC_STATE_LINK_DOWN;
}

void radioCommandComplete(VC_SERIAL_MESSAGE_HEADER * header, uint8_t * data, uint8_t length)
{
  VC_COMMAND_COMPLETE_MESSAGE * vcMsg;
  uint8_t srcVc;

  //The command processor is still running
  commandProcessorRunning = STALL_CHECK_COUNT;

  //Done with this command
  srcVc = header->radio.srcVc;
  if (srcVc == myVc)
  {
    if (pcActiveCommand < CMD_LIST_SIZE)
    {
      if (pcCommandVc < MAX_VC)
      {
        COMMAND_COMPLETE(virtualCircuitList[pcCommandVc].commandQueue, virtualCircuitList[pcCommandVc].activeCommand);
      }
      COMMAND_COMPLETE(pcCommandQueue, pcActiveCommand);
    }
    else if (virtualCircuitList[pcCommandVc].activeCommand < CMD_LIST_SIZE)
    {
      //This was a VC command
      COMMAND_COMPLETE(virtualCircuitList[srcVc].commandQueue, virtualCircuitList[srcVc].activeCommand);
    }
  }
  else
  {
    //This was a VC command
    COMMAND_COMPLETE(virtualCircuitList[srcVc].commandQueue, virtualCircuitList[srcVc].activeCommand);
  }

  vcMsg = (VC_COMMAND_COMPLETE_MESSAGE *)data;
  if (DISPLAY_COMMAND_COMPLETE)
    printf("Command complete from VC %d: %s\n", srcVc,
           (vcMsg->cmdStatus == VC_CMD_SUCCESS) ? "OK" : "ERROR");
  commandStatus = vcMsg->cmdStatus;
  waitingForCommandComplete = false;
}

int radioToHost()
{
  int bytesRead;
  int bytesSent;
  int bytesToRead;
  int bytesToSend;
  static uint8_t * data = outputBuffer;
  static uint8_t * dataStart = outputBuffer;
  static uint8_t * dataEnd = outputBuffer;
  int8_t destAddr;
  static VC_SERIAL_MESSAGE_HEADER * header = (VC_SERIAL_MESSAGE_HEADER *)outputBuffer;
  int length;
  int maxfds;
  int status;
  int8_t srcAddr;
  struct timeval timeout;

  status = 0;
  do
  {
    //Read the virtual circuit header into the local buffer.
    bytesToRead = &outputBuffer[sizeof(outputBuffer)] - dataEnd;
    bytesRead = read(radio, dataEnd, bytesToRead);
    if (bytesRead == 0)
      break;
    if (bytesRead < 0)
    {
      perror("ERROR: Read from radio failed!");
      status = bytesRead;
      break;
    }
    dataEnd += bytesRead;

    //Display the data received from the radio
    if (DUMP_RADIO_TO_PC)
      if (bytesRead)
        dumpBuffer(dataStart, dataEnd - dataStart);

    //The data read is a mix of debug serial output and virtual circuit messages
    //Any data before the VC_SERIAL_MESSAGE_HEADER is considered debug serial output
    data = dataStart;
    while ((data < dataEnd) && (*data != START_OF_VC_SERIAL))
      data++;

    //Process any debug data
    length = data - dataStart;
    if (length)
      //Output the debug data
      hostToStdout(NULL, dataStart, length);

    //Determine if this is the beginning of a virtual circuit message
    length = dataEnd - data;
    if (length <= 0)
    {
      //Refill the empty buffer
      dataStart = outputBuffer;
      data = dataStart;
      dataEnd = data;
      break;
    }

    //This is the beginning of a virtual circuit message
    //Move it to the beginning of the buffer to make things easier
    if (data != outputBuffer)
      memcpy(outputBuffer, data, length);
    dataEnd = &outputBuffer[length];
    dataStart = outputBuffer;
    data = dataStart;

    //Determine if the VC header is in the buffer
    if (length < (int)VC_SERIAL_HEADER_BYTES)
      //Need more data
      break;

    //Determine if the entire message is in the buffer
    if (length < (header->radio.length + 1))
      //Need more data
      break;

    //Set the beginning of the message
    data = &outputBuffer[VC_SERIAL_HEADER_BYTES];
    length = header->radio.length - VC_RADIO_HEADER_BYTES;

    //Display the VC header and message
    if (DEBUG_RADIO_TO_PC)
    {
      printf("VC Header:\n");
      printf("    length: %d\n", header->radio.length);
      printf("    destVc: %d (0x%02x)\n", (uint8_t)header->radio.destVc, (uint8_t)header->radio.destVc);
      printf("    srcVc: %d (0x%02x)\n", header->radio.srcVc, header->radio.srcVc);
      if (length > 0)
        dumpBuffer(data, length);
    }

    //------------------------------
    //Process the message
    //------------------------------

    //Display link status
    if (header->radio.destVc == PC_LINK_STATUS)
      radioToPcLinkStatus(header, data, VC_SERIAL_HEADER_BYTES + length);

    //Display remote command response
    else if (header->radio.destVc == (PC_REMOTE_RESPONSE | myVc))
      status = hostToStdout(header, data, length);

    //Display command completion status
    else if (header->radio.destVc == PC_COMMAND_COMPLETE)
      radioCommandComplete(header, data, length);

    //Display ACKs for transmitted messages
    else if (header->radio.destVc == PC_DATA_ACK)
      radioDataAck(header, data, length);

    //Display NACKs for transmitted messages
    else if (header->radio.destVc == PC_DATA_NACK)
      radioDataNack(header, data, length);

    //Display received messages
    else if ((header->radio.destVc == myVc) || (header->radio.destVc == VC_BROADCAST))
    {
      //Output this message
      status = hostToStdout(header, data, length);
    }

    //Unknown messages
    else
    {
      if (DISPLAY_UNKNOWN_COMMANDS)
      {
        printf("Unknown message, VC Header:\n");
        printf("    length: %d\n", header->radio.length);
        printf("    destVc: %d (0x%02x)\n", (uint8_t)header->radio.destVc, (uint8_t)header->radio.destVc);
        printf("    srcVc: %d (0x%02x)\n", header->radio.srcVc, header->radio.srcVc);
        if (length > 0)
          dumpBuffer(data, length);
      }
    }

    //Continue processing the rest of the data in the buffer
    if (length > 0)
      data += length;
    dataStart = data;
  } while(0);
  return status;
}

void issuePcCommands()
{
  int cmd;
  int index;

  //Wait until this command completes
  if (pcActiveCommand >= CMD_LIST_SIZE)
  {
    for (index = 0; index < CMD_LIST_SIZE; index += QUEUE_T_BITS)
    {
      if (pcCommandQueue[index / QUEUE_T_BITS])
      {
        for (cmd = index; (cmd < CMD_LIST_SIZE) && (cmd < (index + QUEUE_T_BITS)); cmd++)
        {
          if (COMMAND_PENDING(pcCommandQueue, cmd))
          {
            pcActiveCommand = cmd;
            switch (cmd)
            {
              case CMD_ATB: //Break all of the VC links
                if (DEBUG_PC_CMD_ISSUE)
                  printf("Issuing ATB command\n");
                cmdToRadio((uint8_t *)BREAK_LINKS_COMMAND, strlen(BREAK_LINKS_COMMAND));
                return;

              case CMD_ATI30: //Get myVC
                if (myVc == VC_UNASSIGNED)
                {
                  //Get myVc address
                  if (DEBUG_PC_CMD_ISSUE)
                    printf("Issuing ATI30 command\n");
                  findMyVc = true;
                  cmdToRadio((uint8_t *)GET_MY_VC_ADDRESS, strlen(GET_MY_VC_ADDRESS));
                  return;
                }
                if (DEBUG_PC_CMD_ISSUE)
                  printf("Skipping ATI30 command, myVC already known\n");
                COMMAND_COMPLETE(pcCommandQueue, pcActiveCommand);
                break;

              case CMD_ATA: //Get all the VC states
                if (DEBUG_PC_CMD_ISSUE)
                  printf("Issuing ATA command\n");
                cmdToRadio((uint8_t *)GET_VC_STATUS, strlen(GET_VC_STATUS));
                return;

              case CMD_AT_CMDVC: //Select the VC to target
              case CMD_AT_CMDVC_2:
                //Select the VC to use
                sprintf(pcCommandBuffer, "at-CmdVc=%d", pcCommandVc);
                if (DEBUG_PC_CMD_ISSUE)
                  printf("Issuing %s command\n", pcCommandBuffer);
                cmdToRadio((uint8_t *)pcCommandBuffer, strlen(pcCommandBuffer));
                return;

              case CMD_ATC: //Perform the 3-way handshake
                //Bring up the VC connection to this remote system
                if (DEBUG_PC_CMD_ISSUE)
                  printf("Issuing ATC command\n");
                cmdToRadio((uint8_t *)START_3_WAY_HANDSHAKE, strlen(START_3_WAY_HANDSHAKE));
                return;

              case CMD_ATI31:
                //Get the VC state
                if (DEBUG_PC_CMD_ISSUE)
                  printf("Issuing ATI31 command\n");
                cmdToRadio((uint8_t *)GET_VC_STATE, strlen(GET_VC_STATE));
                return;

              case CMD_ATI:
                if (DEBUG_PC_CMD_ISSUE)
                  printf("Issuing ATI command\n");
                cmdToRadio((uint8_t *)GET_DEVICE_INFO, strlen(GET_DEVICE_INFO));
                return;

              case CMD_ATI8:
                if (DEBUG_PC_CMD_ISSUE)
                  printf("Issuing ATI8 command\n");
                cmdToRadio((uint8_t *)GET_UNIQUE_ID, strlen(GET_UNIQUE_ID));
                return;
            }
          }
        }
      }
    }
    pcActiveCommand = CMD_LIST_SIZE;
  }
}

int sendVcCommand(const char * commandString, int destVc)
{
  int bytesSent;
  int bytesWritten;
  VC_SERIAL_MESSAGE_HEADER header;
  int length;

  //Build the virtual circuit serial header
  length = strlen(commandString);
  header.start = START_OF_VC_SERIAL;
  header.radio.length = VC_RADIO_HEADER_BYTES + length;
  header.radio.destVc = PC_REMOTE_COMMAND | destVc;
  header.radio.srcVc = myVc;

  //Display the data being sent to the radio
  if (DEBUG_PC_TO_RADIO)
  {
    dumpBuffer((uint8_t *)&header, VC_SERIAL_HEADER_BYTES);
    dumpBuffer((uint8_t *)commandString, length);
  }
  if (DEBUG_LOCAL_COMMANDS)
    printf("Sending LoRaSerial command: %s\n", commandString);
  if (DEBUG_PC_CMD_ISSUE)
    printf("Sending %s command to VC %d\n", commandString, destVc);

  //Send the header
  bytesWritten = write(radio, (uint8_t *)&header, VC_SERIAL_HEADER_BYTES);
  if (bytesWritten < (int)VC_SERIAL_HEADER_BYTES)
  {
    perror("ERROR: Write of header to radio failed!");
    return -1;
  }

  //Send the message
  bytesSent = 0;
  while (bytesSent < length)
  {
    bytesWritten = write(radio, &commandString[bytesSent], length - bytesSent);
    if (bytesWritten < 0)
    {
      perror("ERROR: Write of data to radio failed!");
      return bytesWritten;
    }
    bytesSent += bytesWritten;
  }

  //Return the amount of the buffer that was sent
  return length;
}

bool commandProcessorBusy()
{
  bool busy;
  int index;

  //Determine if the command processor is using the VC
  busy = (pcActiveCommand < CMD_LIST_SIZE);
  for (index = 0; index < COMMAND_QUEUE_SIZE; index++)
    if (pcCommandQueue[index])
    {
      busy = 1;
      break;
    }
  return busy;
}

bool commandProcessorIdle(int vcIndex)
{
  bool idle;

  //Determine if the command processor is using the VC
  idle = !commandProcessorBusy();
  if (idle)
    //Set the command VC
    pcCommandVc = vcIndex;

  //Return the idle status
  return idle;
}

bool issueVcCommands(int vcIndex)
{
  int cmd;
  uint32_t dayBit;
  int dayIndex;
  int entry;
  int index;
  time_t now;
  struct tm * timeStruct;
  uint32_t zoneBit;
  int zoneIndex;

  if (virtualCircuitList[vcIndex].activeCommand >= CMD_LIST_SIZE)
  {
    for (index = 0; index < CMD_LIST_SIZE; index += QUEUE_T_BITS)
    {
      if (virtualCircuitList[vcIndex].commandQueue[index / QUEUE_T_BITS])
      {
        for (cmd = index; (cmd < CMD_LIST_SIZE) && (cmd < (index + QUEUE_T_BITS)); cmd++)
        {
          if (COMMAND_PENDING(virtualCircuitList[vcIndex].commandQueue, cmd))
          {
            virtualCircuitList[vcIndex].activeCommand = cmd;
            switch (cmd)
            {
              case CMD_AT_CMDVC:
                //Determine if the local command processor is idle
                if (commandProcessorIdle(vcIndex))
                {
                  if (DEBUG_PC_CMD_ISSUE)
                    printf("Migrating AT-CMDVC=%d and ATC commands to PC command queue\n", vcIndex);
                  COMMAND_ISSUE(pcCommandQueue, pcCommandTimer, CMD_AT_CMDVC);
                  if (COMMAND_PENDING(virtualCircuitList[vcIndex].commandQueue, CMD_ATC))
                    COMMAND_ISSUE(pcCommandQueue, pcCommandTimer, CMD_ATC);
                  return true;
                }
                virtualCircuitList[vcIndex].activeCommand = CMD_LIST_SIZE;
                return true;

              case CMD_ATC:
                return true;

              case CMD_WAIT_CONNECTED:
                if ((virtualCircuitList[vcIndex].vcState != VC_STATE_CONNECTED)
                    || commandProcessorBusy(vcIndex))
                {
                  //Mark the list as empty to allow this entry to be executed again
                  virtualCircuitList[vcIndex].activeCommand = CMD_LIST_SIZE;
                  return true;
                }

                //The wait is complete, this VC is now connected to the sprinkler server
                COMMAND_COMPLETE(virtualCircuitList[vcIndex].commandQueue,
                                 virtualCircuitList[vcIndex].activeCommand);

                //Get the VC state
                COMMAND_ISSUE(pcCommandQueue, pcCommandTimer, CMD_AT_CMDVC_2);
                COMMAND_ISSUE(pcCommandQueue, pcCommandTimer, CMD_ATI31);
                break;

              case CMD_AT_CMDVC_2:
                //Determine if the local command processor is idle
                if (commandProcessorIdle(vcIndex))
                {
                  if (DEBUG_PC_CMD_ISSUE)
                    printf("Migrating AT-CMDVC and ATI31 commands to PC command queue\n");
                  COMMAND_ISSUE(pcCommandQueue, pcCommandTimer, CMD_AT_CMDVC_2);
                  if (COMMAND_PENDING(virtualCircuitList[vcIndex].commandQueue, CMD_ATI31))
                    COMMAND_ISSUE(pcCommandQueue, pcCommandTimer, CMD_ATI31);
                  return true;
                }
                virtualCircuitList[vcIndex].activeCommand = CMD_LIST_SIZE;
                return true;

              case CMD_ATI31:
                return true;

              case CMD_ATI:
                sendVcCommand(GET_DEVICE_INFO, vcIndex);
                return true;

              case CMD_ATI8:
                sendVcCommand(GET_UNIQUE_ID, vcIndex);
                return true;
            }
          }
        }
        virtualCircuitList[vcIndex].activeCommand = CMD_LIST_SIZE;
      }
    }
  }
  return false;
}

int main(int argc, char **argv)
{
  bool breakLinks;
  int cmd;
  fd_set currentfds;
  bool displayTitle;
  int maxfds;
  int numfds;
  bool reset;
  int status;
  char * terminal;
  struct timeval timeout;
  int vcIndex;

  maxfds = STDIN;
  status = 0;
  do
  {
    //Verify the command table length
    if (ARRAY_SIZE(commandName) != CMD_LIST_SIZE)
    {
      fprintf(stderr, "ERROR: Fix commandName length of %d != %d\n", (uint32_t)ARRAY_SIZE(commandName), CMD_LIST_SIZE);
      return -1;
    }

    //Display the help text if necessary
    if (argc < 3)
    {
      printf("%s   terminal   target_VC   [options]\n", argv[0]);
      printf("\n");
      printf("terminal - Name or path to the terminal device for the radio\n");
      printf("target_VC:\n");
      printf("    Server: 0\n");
      printf("    Client: 1 - %d\n", MAX_VC - 1);
      printf("    Loopback: my_VC\n");
      printf("    Broadcast: %d\n", VC_BROADCAST);
      printf("    Command: %d\n", VC_COMMAND);
      printf("Options:\n");
      printf("    --reset    Reset the LoRaSerial radio and break the links\n");
      printf("    --break    Use ATB command to break the links\n");
      status = -1;
      break;
    }

    //Get the path to the terminal
    terminal = argv[1];

    //Determine the remote VC address
    if ((sscanf(argv[2], "%d", &remoteVc) != 1)
      || ((remoteVc > PC_LINK_STATUS) && (remoteVc < VC_COMMAND)))
    {
      fprintf(stderr, "ERROR: Invalid target VC address, please use one of the following:\n");
      if (myVc)
        fprintf(stderr, "    Server: 0\n");
      fprintf(stderr, "    Client: 1 - %d\n", MAX_VC - 1);
      fprintf(stderr, "    Loopback: my_VC\n");
      fprintf(stderr, "    Broadcast: %d\n", VC_BROADCAST);
      fprintf(stderr, "    Command: %d\n", VC_COMMAND);
      status = -1;
      break;
    }

    //Open the terminal
    status = openLoRaSerial(terminal);
    if (status)
      break;

    //Determine the options
    reset = false;
    if ((argc == 4) && (strcmp("--reset", argv[3]) == 0))
      reset = true;

    breakLinks = reset;
    if ((argc == 4) && (strcmp("--break", argv[3]) == 0))
      breakLinks = true;

    //Reset the LoRaSerial radio if requested
    if (reset)
    {
      //Delay a while to let the radio complete its reset operation
      sleep(2);

      //Break the links to this node
      cmdToRadio((uint8_t *)LINK_RESET_COMMAND, strlen(LINK_RESET_COMMAND));

      //Allow the device to reset
      close(radio);
      do
      {
        sleep(1);

        //Open the terminal
        status = openLoRaSerial(terminal);
      } while (status);

      //Delay a while to let the radio complete its reset operation
      sleep(2);
    }

    //Initialize the virtual circuits
    for (vcIndex = 0; vcIndex < MAX_VC; vcIndex++)
      virtualCircuitList[vcIndex].activeCommand = CMD_LIST_SIZE;

    //Perform the initialization commands
    pcCommandTimer = 1;
    COMMAND_ISSUE(pcCommandQueue, pcCommandTimer, CMD_ATI30); //Get myVC
    COMMAND_ISSUE(pcCommandQueue, pcCommandTimer, CMD_ATA);   //Get all the VC states

    //Break the links if requested
    if (breakLinks)
      COMMAND_ISSUE(pcCommandQueue, pcCommandTimer, CMD_ATB); //Break all the VC links

    //Initialize the fd_sets
    if (maxfds < radio)
      maxfds = radio;
    FD_ZERO(&readfds);
    FD_SET(STDIN, &readfds);
    FD_SET(radio, &readfds);

    while (1)
    {
      //Set the timeout
      timeout.tv_sec = 0;
      timeout.tv_usec = POLL_TIMEOUT_USEC;

      //Wait for receive data or timeout
      memcpy((void *)&currentfds, (void *)&readfds, sizeof(readfds));
      numfds = select(maxfds + 1, &currentfds, NULL, NULL, &timeout);
      if (numfds < 0)
      {
        perror("ERROR: select call failed!");
        status = errno;
        break;
      }

      //Determine if console input is available
      if (FD_ISSET(STDIN, &currentfds))
      {
        //Send the console input to the radio
        status = stdinToRadio();
        if (status)
          break;
      }

      if (FD_ISSET(radio, &currentfds))
      {
        //Process the incoming data from the radio
        status = radioToHost();
        if (status)
          break;
      }

      //----------------------------------------
      // Check for timeout
      //----------------------------------------

      if (numfds == 0)
      {
        timeoutCount++;

        //Check for time to process PC commands
        if (pcCommandTimer && (pcActiveCommand >= CMD_LIST_SIZE)
          && ((timeoutCount - pcCommandTimer) >= COMMAND_POLL_COUNT))
        {
          pcCommandTimer = timeoutCount;
          if (!pcCommandTimer)
            pcCommandTimer = 1;
          issuePcCommands();
        }

        //Check for time to process VC commands
        for (vcIndex = 0; vcIndex < MAX_VC; vcIndex++)
        {
          if (virtualCircuitList[vcIndex].commandTimer
            && ((timeoutCount - virtualCircuitList[vcIndex].commandTimer) >= COMMAND_POLL_COUNT))
          {
            virtualCircuitList[vcIndex].commandTimer = timeoutCount;
            if (!virtualCircuitList[vcIndex].commandTimer)
              virtualCircuitList[vcIndex].commandTimer = 1;
            if (virtualCircuitList[vcIndex].activeCommand < CMD_LIST_SIZE)
              break;
            if (issueVcCommands(vcIndex))
            {
  continue;
//              break;
            }
          }
        }

        //----------------------------------------
        // Check the resource usage
        //----------------------------------------

        if (!(timeoutCount % (ONE_SECOND_COUNT * 60)))
        {
          struct rusage usage;
          int s_days;
          int s_hours;
          int s_mins;
          int s_secs;
          uint64_t s_millis;
          int u_days;
          int u_hours;
          int u_mins;
          int u_secs;
          uint64_t u_millis;

          //Get the sprinkler server resource useage
          getrusage(RUSAGE_SELF, &usage);
          u_millis = usage.ru_utime.tv_usec / 1000;
          u_secs = usage.ru_utime.tv_sec;
          u_days = u_secs / SECS_IN_DAY;
          u_secs -= u_days * SECS_IN_DAY;
          u_hours = u_secs / SECS_IN_HOUR;
          u_secs -= u_hours * SECS_IN_HOUR;
          u_mins = u_secs / SECS_IN_MINUTE;
          u_secs -= u_mins * SECS_IN_MINUTE;

          s_millis = usage.ru_stime.tv_usec / 1000;
          s_secs = usage.ru_stime.tv_sec;
          s_days = s_secs / SECS_IN_DAY;
          s_secs -= s_days * SECS_IN_DAY;
          s_hours = s_secs / SECS_IN_HOUR;
          s_secs -= s_hours * SECS_IN_HOUR;
          s_mins = s_secs / SECS_IN_MINUTE;
          s_secs -= s_mins * SECS_IN_MINUTE;

          printf("K: %d:%02d:%02d.%03d, U: %d:%02d:%02d.%03d, Mem: %ld, PF: %ld: IN: %ld, OUT: %ld\n",
                  u_hours, u_mins, u_secs, (int)u_millis,
                  s_hours, s_mins, s_secs, (int)s_millis,
                  usage.ru_maxrss, usage.ru_majflt, usage.ru_inblock, usage.ru_oublock);

/*
          getrusage(RUSAGE_CHILDREN, &usage);
          u_millis = usage.ru_utime.tv_usec / 1000;
          u_secs = usage.ru_utime.tv_sec;
          u_days = u_secs / SECS_IN_DAY;
          u_secs -= u_days * SECS_IN_DAY;
          u_hours = u_secs / SECS_IN_HOUR;
          u_secs -= u_hours * SECS_IN_HOUR;
          u_mins = u_secs / SECS_IN_MINUTE;
          u_secs -= u_mins * SECS_IN_MINUTE;

          s_millis = usage.ru_stime.tv_usec / 1000;
          s_secs = usage.ru_stime.tv_sec;
          s_days = s_secs / SECS_IN_DAY;
          s_secs -= s_days * SECS_IN_DAY;
          s_hours = s_secs / SECS_IN_HOUR;
          s_secs -= s_hours * SECS_IN_HOUR;
          s_mins = s_secs / SECS_IN_MINUTE;
          s_secs -= s_mins * SECS_IN_MINUTE;

          printf("K: %d:%02d:%02d.%03d, U: %d:%02d:%02d.%03d, Mem: %ld, PF: %ld: IN: %ld, OUT: %ld\n",
                  u_hours, u_mins, u_secs, (int)u_millis,
                  s_hours, s_mins, s_secs, (int)s_millis,
                  usage.ru_maxrss, usage.ru_majflt, usage.ru_inblock, usage.ru_oublock);
*/
        }
      }

      //----------------------------------------
      // Check for stalled commands
      //----------------------------------------

      //Deterine if the command processor is running
      if (commandProcessorRunning)
      {
        //Determine if it is time to check for a command processor stall
        if (commandProcessorRunning)
          commandProcessorRunning--;
        if (!commandProcessorRunning)
        {
          //The command processor is either stalled or complete
          //Determine if there are any outsanding commands to be processed
          displayTitle = true;
          for (int cmdBlock = 0; cmdBlock < COMMAND_QUEUE_SIZE; cmdBlock++)
          {
            if (pcCommandQueue[cmdBlock])
            {
              //Display the next outstanding command
              int cmdBase = cmdBlock * QUEUE_T_BITS;
              for (cmd = cmdBase; cmd < (cmdBase + QUEUE_T_BITS); cmd++)
              {
                if (COMMAND_PENDING(pcCommandQueue, cmd))
                {
                  if (displayTitle)
                  {
                    displayTitle = false;
                    printf("Stalled commands:\n");
                  }
                  printf("    PC: %d (%s, %s)\n",
                         cmd,
                         commandName[cmd],
                         (pcActiveCommand < CMD_LIST_SIZE) ? "Active" : "Pending");
                  break;
                }
              }

              //Check for a stalled command
              if (cmd < (cmdBase + QUEUE_T_BITS))
                break;
            }
          }

          //Determine if the local radio command queue is idle
          if (displayTitle)
            printf("PC: Idle\n");

          //Determine if there are any outstanding VC commands
          for (vcIndex = 0; vcIndex < MAX_VC; vcIndex++)
          {
            for (int cmdBlock = 0; cmdBlock < COMMAND_QUEUE_SIZE; cmdBlock++)
            {
              if (virtualCircuitList[vcIndex].commandQueue[cmdBlock])
              {
                //Display the next outstanding command
                int cmdBase = cmdBlock * QUEUE_T_BITS;
                for (cmd = cmdBase; cmd < (cmdBase + QUEUE_T_BITS); cmd++)
                {
                  if (COMMAND_PENDING(virtualCircuitList[vcIndex].commandQueue, cmd))
                  {
                    if (displayTitle)
                    {
                      displayTitle = false;
                      printf("Stalled commands:\n");
                    }
                    printf("    VC %d (%s): %d (%s, %s)\n",
                           vcIndex,
                           vcStateNames[virtualCircuitList[vcIndex].vcState],
                           cmd,
                           commandName[cmd],
                           (virtualCircuitList[vcIndex].activeCommand < CMD_LIST_SIZE) ? "Active" : "Pending");
                    break;
                  }
                }

                //Check for a stalled command
                if (cmd < (cmdBase + QUEUE_T_BITS))
                  break;
              }
            }
          }
        }
      }
    }
  } while (0);

  //Done with the radio
  close(radio);
  return status;
}
