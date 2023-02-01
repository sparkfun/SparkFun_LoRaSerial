#define ADD_VC_STATE_NAMES_TABLE
#include "settings.h"

#define BUFFER_SIZE           2048
#define INPUT_BUFFER_SIZE     BUFFER_SIZE
#define MAX_MESSAGE_SIZE      32
#define STDIN                 0
#define STDOUT                1

#define BREAK_LINKS_COMMAND   "atb"
#define GET_MY_VC_ADDRESS     "atI11"
#define GET_VC_STATUS         "ata"
#define LINK_RESET_COMMAND    "atz"
#define MY_VC_ADDRESS         "myVc: "

#define DEBUG_LOCAL_COMMANDS      0
#define DEBUG_PC_TO_RADIO         0
#define DEBUG_RADIO_TO_PC         0
#define DISPLAY_COMMAND_COMPLETE  0
#define DISPLAY_DATA_ACK          0
#define DISPLAY_DATA_NACK         1
#define DISPLAY_VC_STATE          0
#define SEND_ATC_COMMAND          1
#define DISPLAY_STATE_TRANSITION  0

typedef struct _VIRTUAL_CIRCUIT
{
  int vcState;
} VIRTUAL_CIRCUIT;

bool commandStatus;
bool findMyVc;
int myVc = VC_UNASSIGNED;
int remoteVc;
uint8_t inputBuffer[INPUT_BUFFER_SIZE];
uint8_t outputBuffer[VC_SERIAL_HEADER_BYTES + BUFFER_SIZE];
int timeoutCount;
VIRTUAL_CIRCUIT virtualCircuitList[MAX_VC];
volatile bool waitingForCommandComplete;
uint8_t remoteCommandVc;

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
    bytesWritten = write(radio, buffer, length);
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
    bytesWritten = write(radio, buffer, length);
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

int hostToStdout(uint8_t * data, uint8_t bytesToSend)
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

          //Request the status for all of the VCs
          cmdToRadio((uint8_t *)GET_VC_STATUS, strlen(GET_VC_STATUS));
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

void radioToPcLinkStatus(VC_SERIAL_MESSAGE_HEADER * header, uint8_t length)
{
  char cmdBuffer[128];
  int newState;
  int previousState;
  int srcVc;
  VC_STATE_MESSAGE * vcMsg;

  //Remember the previous state
  srcVc = header->radio.srcVc;
  previousState = virtualCircuitList[srcVc].vcState;
  vcMsg = (VC_STATE_MESSAGE *)&header[1];

  //Set the new state
  newState = vcMsg->vcState;
  virtualCircuitList[srcVc].vcState = newState;

  //Display the state if requested
  if (DISPLAY_STATE_TRANSITION)
    printf("VC%d: %s --> %s\n", srcVc, vcStateNames[previousState], vcStateNames[newState]);
  switch (newState)
  {
  default:
    if (DISPLAY_VC_STATE)
      printf("------- VC %d State %3d ------\n", srcVc, vcMsg->vcState);
    break;

  case VC_STATE_LINK_DOWN:
    if (DISPLAY_VC_STATE)
      printf("--------- VC %d DOWN ---------\n", srcVc);
    break;

  case VC_STATE_LINK_ALIVE:
    //Upon transition to ALIVE, if is the server or the source VC matches the
    //target VC or myVc, bring up the connection
    if (SEND_ATC_COMMAND && (previousState != newState)
      && ((myVc == VC_SERVER) || (srcVc == remoteVc) || (srcVc == myVc)))
    {
      //Select the VC to use
      sprintf(cmdBuffer, "at-CmdVc=%d", srcVc);
      cmdToRadio((uint8_t *)cmdBuffer, strlen(cmdBuffer));

      //Bring up the VC connection to this remote system
      strcpy(cmdBuffer, "atC");
      cmdToRadio((uint8_t *)cmdBuffer, strlen(cmdBuffer));
    }

    if (DISPLAY_VC_STATE)
      printf("-=--=--=- VC %d ALIVE =--=--=-\n", srcVc);
    break;

  case VC_STATE_SEND_UNKNOWN_ACKS:
    if (DISPLAY_VC_STATE)
      printf("-=--=-- VC %d ALIVE UA --=--=-\n", srcVc);
    break;

  case VC_STATE_WAIT_SYNC_ACKS:
    if (DISPLAY_VC_STATE)
      printf("-=--=-- VC %d ALIVE SA --=--=-\n", srcVc);
    break;

  case VC_STATE_WAIT_ZERO_ACKS:
    if (DISPLAY_VC_STATE)
    {
      if (previousState == VC_STATE_CONNECTED)
        printf("-=-=- VC %d DISCONNECTED -=-=-", srcVc);
      printf("-=--=-- VC %d ALIVE ZA --=--=-\n", srcVc);
    }
    break;

  case VC_STATE_CONNECTED:
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

void radioDataAck(uint8_t * data, uint8_t length)
{
  VC_DATA_ACK_NACK_MESSAGE * vcMsg;

  vcMsg = (VC_DATA_ACK_NACK_MESSAGE *)data;
  if (DISPLAY_DATA_ACK)
    printf("ACK from VC %d\n", vcMsg->msgDestVc);
}

void radioDataNack(uint8_t * data, uint8_t length)
{
  VC_DATA_ACK_NACK_MESSAGE * vcMsg;

  vcMsg = (VC_DATA_ACK_NACK_MESSAGE *)data;
  if (DISPLAY_DATA_NACK)
    printf("NACK from VC %d\n", vcMsg->msgDestVc);
}

void radioCommandComplete(uint8_t srcVc, uint8_t * data, uint8_t length)
{
  VC_COMMAND_COMPLETE_MESSAGE * vcMsg;

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
//    if (bytesRead) dumpBuffer(dataStart, dataEnd - dataStart);

    //The data read is a mix of debug serial output and virtual circuit messages
    //Any data before the VC_SERIAL_MESSAGE_HEADER is considered debug serial output
    data = dataStart;
    while ((data < dataEnd) && (*data != START_OF_VC_SERIAL))
      data++;

    //Process any debug data
    length = data - dataStart;
    if (length)
      //Output the debug data
      hostToStdout(dataStart, length);

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
      printf("    destVc: %d\n", header->radio.destVc);
      printf("    srcVc: %d\n", header->radio.srcVc);
      if (length > 0)
        dumpBuffer(data, length);
    }

    //------------------------------
    //Process the message
    //------------------------------

    //Display link status
    if (header->radio.destVc == PC_LINK_STATUS)
      radioToPcLinkStatus(header, VC_SERIAL_HEADER_BYTES + length);

    //Display remote command response
    else if (header->radio.destVc == (PC_REMOTE_RESPONSE | myVc))
      status = hostToStdout(data, length);

    //Display command completion status
    else if (header->radio.destVc == PC_COMMAND_COMPLETE)
      radioCommandComplete(header->radio.srcVc, data, length);

    //Display ACKs for transmitted messages
    else if (header->radio.destVc == PC_DATA_ACK)
      radioDataAck(data, length);

    //Display NACKs for transmitted messages
    else if (header->radio.destVc == PC_DATA_NACK)
      radioDataNack(data, length);

    //Display received messages
    else if ((header->radio.destVc == myVc) || (header->radio.destVc == VC_BROADCAST))
    {
      //Output this message
      status = hostToStdout(data, length);
    }

    //Unknown messages
    else
    {
      printf("Unknown message, VC Header:\n");
      printf("    length: %d\n", header->radio.length);
      printf("    destVc: %d\n", header->radio.destVc);
      printf("    srcVc: %d\n", header->radio.srcVc);
      if (length > 0)
        dumpBuffer(data, length);
    }

    //Continue processing the rest of the data in the buffer
    if (length > 0)
      data += length;
    dataStart = data;
  } while(0);
  return status;
}

int
main (
  int argc,
  char ** argv
)
{
  bool breakLinks;
  fd_set currentfds;
  int maxfds;
  int numfds;
  bool reset;
  int status;
  char * terminal;
  struct timeval timeout;

  maxfds = STDIN;
  status = 0;
  do
  {
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

    //Break the links if requested
    findMyVc = true;
    if (breakLinks)
      cmdToRadio((uint8_t *)BREAK_LINKS_COMMAND, strlen(BREAK_LINKS_COMMAND));

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
      timeout.tv_usec = 1000;

      //Wait for receive data or timeout
      memcpy((void *)&currentfds, (void *)&readfds, sizeof(readfds));
      numfds = select(maxfds + 1, &currentfds, NULL, NULL, &timeout);
      if (numfds < 0)
      {
        perror("ERROR: select call failed!");
        status = errno;
        break;
      }

      //Check for timeout
      if ((numfds == 0) && (timeoutCount++ >= 1000))
      {
        timeoutCount = 0;
        if (myVc == VC_UNASSIGNED)
        {
          //Get myVc address
          findMyVc = true;
          cmdToRadio((uint8_t *)GET_MY_VC_ADDRESS, strlen(GET_MY_VC_ADDRESS));
        }
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
    }
  } while (0);
  return status;
}
