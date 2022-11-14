#include "settings.h"

#define BUFFER_SIZE           2048
#define MAX_MESSAGE_SIZE      32
#define STDIN                 0
#define STDOUT                1

#define LINK_RESET_COMMAND    "atb"

#define DEBUG_PC_TO_RADIO     0
#define DEBUG_RADIO_TO_PC     0

int myVc;
int remoteVc;
uint8_t inputBuffer[BUFFER_SIZE];
uint8_t outputBuffer[VC_SERIAL_HEADER_BYTES + BUFFER_SIZE];

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
  int bytesSent;
  int bytesWritten;
  int status;

  //Write this data to stdout
  bytesSent = 0;
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
  VC_LINK_STATUS_MESSAGE * linkStatus;

  linkStatus = (VC_LINK_STATUS_MESSAGE *)&header[1];
  if (linkStatus->linkStatus == LINK_UP)
    printf("========== Link %d UP ==========\n", header->radio.srcVc);
  else
    printf("--------- Link %d DOWN ---------\n", header->radio.srcVc);
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

    //Process the message
    switch (header->radio.destVc)
    {
    default:
      //Discard this message
      break;

    case PC_LINK_STATUS:
      radioToPcLinkStatus(header, VC_SERIAL_HEADER_BYTES + length);
      break;
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
  int maxfds;
  int status;
  char * terminal;
  struct timeval timeout;
  uint8_t * vcData;

  status = 0;
  do
  {
    //Display the help text if necessary
    if (argc != 4)
    {
      printf("%s   terminal   my_VC   target_VC\n", argv[0]);
      printf("\n");
      printf("terminal - Name or path to the terminal device for the radio\n");
      printf("my_VC:\n");
      printf("    Server: 0\n");
      printf("    Client: 1 - %d\n", MAX_VC - 1);
      printf("target_VC:\n");
      printf("    Server: 0\n");
      printf("    Client: 1 - %d\n", MAX_VC - 1);
      printf("    Loopback: my_VC\n");
      printf("    Broadcast: %d\n", VC_BROADCAST);
      printf("    Command: %d\n", VC_COMMAND);
      status = -1;
      break;
    }

    //Get the path to the terminal
    terminal = argv[1];

    //Determine the local VC address
    if ((sscanf(argv[2], "%d", &myVc) != 1)
      || ((myVc < VC_SERVER) || (myVc >= MAX_VC)))
    {
      fprintf(stderr, "ERROR: Invalid my VC address, please use one of the following:\n");
      fprintf(stderr, "    Server: 0\n");
      fprintf(stderr, "    Client: 1 - %d\n", MAX_VC - 1);
      status = -1;
      break;
    }

    //Determine the remote VC address
    if ((sscanf(argv[3], "%d", &remoteVc) != 1)
      || (remoteVc < VC_COMMAND) || (remoteVc >= MAX_VC))
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
    maxfds = STDIN;
    status = openTty(terminal);
    if (status)
      break;
    if (maxfds < tty)
      maxfds = tty;

    //Display myVc
    printf("myVc: %d\n", myVc);

    //Delay a while to let the radio complete its reset operation
    sleep(2);

    //Break the links to this node
    cmdToRadio((uint8_t *)LINK_RESET_COMMAND, strlen(LINK_RESET_COMMAND));

    //Initialize the fd_sets
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
