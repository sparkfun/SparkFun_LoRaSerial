#define ADD_VC_STATE_NAMES_TABLE
#include "settings.h"

#define BUFFER_SIZE           2048
#define MAX_MESSAGE_SIZE      32
#define STDIN                 0
#define STDOUT                1

#define BREAK_LINKS_COMMAND   "atb"
#define GET_MY_VC_ADDRESS     "atI11"
#define LINK_RESET_COMMAND    "atz"
#define MY_VC_ADDRESS         "myVc: "

#define DEBUG_PC_TO_RADIO         0
#define DEBUG_RADIO_TO_PC         0
#define DISPLAY_DATA_ACK          1
#define DISPLAY_DATA_NACK         1
#define DISPLAY_VC_STATE          0
#define DISPLAY_STATE_TRANSITION  1

typedef struct _VIRTUAL_CIRCUIT
{
  int vcState;
} VIRTUAL_CIRCUIT;

bool findMyVc;
int myVc = VC_UNASSIGNED;
int remoteVc;
uint8_t inputBuffer[BUFFER_SIZE];
uint8_t outputBuffer[VC_SERIAL_HEADER_BYTES + BUFFER_SIZE];
int timeoutCount;
VIRTUAL_CIRCUIT virtualCircuitList[MAX_VC];

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

  //Send the header
  bytesWritten = write(tty, (uint8_t *)&header, VC_SERIAL_HEADER_BYTES);
  if (bytesWritten < (int)VC_SERIAL_HEADER_BYTES)
  {
    perror("ERROR: Write of header to radio failed!");
    return -1;
  }

  //Send the message
  bytesSent = 0;
  while (bytesSent < length)
  {
    bytesWritten = write(tty, buffer, length);
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
  bytesWritten = write(tty, (uint8_t *)&header, VC_SERIAL_HEADER_BYTES);
  if (bytesWritten < (int)VC_SERIAL_HEADER_BYTES)
  {
    perror("ERROR: Write of header to radio failed!");
    return -1;
  }

  //Send the message
  bytesSent = 0;
  while (bytesSent < length)
  {
    bytesWritten = write(tty, buffer, length);
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

  status = 0;
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
      if (remoteVc == VC_COMMAND)
        bytesWritten = cmdToRadio(&inputBuffer[bytesSent], bytesToSend);
      else
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
          myVc = (int8_t)vcNumber;
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
    if (DISPLAY_VC_STATE)
      printf("-=--=--=- VC %d ALIVE =--=--=-\n", srcVc);
    break;

  case VC_STATE_SEND_PING:
    if (DISPLAY_VC_STATE)
      printf("-=--=-- VC %d ALIVE P1 --=--=-\n", srcVc);
    break;

  case VC_STATE_WAIT_FOR_ACK1:
    if (DISPLAY_VC_STATE)
      printf("-=--=-- VC %d ALIVE WA1 -=--=-\n", srcVc);
    break;

  case VC_STATE_WAIT_FOR_ACK2:
    if (DISPLAY_VC_STATE)
      printf("-=--=-- VC %d ALIVE WA2 -=--=-\n", srcVc);
    break;

  case VC_STATE_CONNECTED:
    if (DISPLAY_VC_STATE)
      printf("======= VC %d CONNECTED ======\n", srcVc);
    break;
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
    printf("ACK from VC %d\n", vcMsg->msgDestVc);
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
  uint8_t length;
  int maxfds;
  int status;
  int8_t srcAddr;
  struct timeval timeout;

  status = 0;
  do
  {
    //Read the virtual circuit header into the local buffer.
    bytesToRead = &outputBuffer[sizeof(outputBuffer)] - dataEnd;
    bytesRead = read(tty, dataEnd, bytesToRead);
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
    if (length < VC_SERIAL_HEADER_BYTES)
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

    //Display ACKs for transmitted messages
    else if (header->radio.destVc == PC_DATA_ACK)
      radioDataAck(data, length);

    //Display NACKs for transmitted messages
    else if (header->radio.destVc == PC_DATA_NACK)
      radioDataAck(data, length);

    //Display received messages
    else
    {
      if ((header->radio.destVc == myVc) || (header->radio.destVc == VC_BROADCAST))
        //Output this message
        status = hostToStdout(data, length);
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
  int maxfds;
  bool reset;
  int status;
  char * terminal;
  struct timeval timeout;
  uint8_t * vcData;

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
    status = openTty(terminal);
    if (status)
      break;
    if (maxfds < tty)
      maxfds = tty;

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
      close(tty);
      do
      {
        sleep(1);

        //Open the terminal
        status = openTty(terminal);
      } while (status);

      //Delay a while to let the radio complete its reset operation
      sleep(2);
    }

    //Get myVc address
    findMyVc = true;
    cmdToRadio((uint8_t *)GET_MY_VC_ADDRESS, strlen(GET_MY_VC_ADDRESS));

    //Break the links if requested
    if (breakLinks)
      cmdToRadio((uint8_t *)BREAK_LINKS_COMMAND, strlen(BREAK_LINKS_COMMAND));

    //Initialize the fd_sets
    if (maxfds < tty)
      maxfds = tty;
    FD_ZERO(&exceptfds);
    FD_ZERO(&readfds);
    FD_ZERO(&writefds);

    while (1)
    {
      //Set the timeout
      timeout.tv_sec = 0;
      timeout.tv_usec = 1000;

      //Wait for receive data or timeout
      FD_SET(STDIN, &readfds);
      FD_SET(tty, &readfds);
      status = select(maxfds + 1, &readfds, &writefds, &exceptfds, &timeout);

      //Check for timeout
      if ((status == 0) && (timeoutCount++ >= 1000))
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
      if (FD_ISSET(STDIN, &readfds))
      {
        //Send the console input to the radio
        status = stdinToRadio();
        if (status)
          break;
      }

      if (FD_ISSET(tty, &readfds))
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
