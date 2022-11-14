#include "settings.h"

#define BUFFER_SIZE           2048
#define MAX_MESSAGE_SIZE      32
#define STDIN                 0
#define STDOUT                1

int myVcAddr;
int remoteVcAddr;
uint8_t inputBuffer[VC_SERIAL_HEADER_BYTES + BUFFER_SIZE];
uint8_t outputBuffer[VC_SERIAL_HEADER_BYTES + BUFFER_SIZE];

int stdinToRadio()
{
  int bytesRead;
  int bytesSent;
  int bytesToSend;
  int maxfds;
  int status;
  struct timeval timeout;
  uint8_t * vcData;

  status = 0;
  do
  {
    //Read the console input data into the local buffer.
    vcData = inputBuffer;
    bytesRead = read(STDIN, &inputBuffer[VC_SERIAL_HEADER_BYTES], BUFFER_SIZE);
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

      //Build the virtual circuit serial header
      vcData[0] = START_OF_VC_SERIAL;
      vcData[1] = bytesToSend + VC_RADIO_HEADER_BYTES;
      vcData[2] = remoteVcAddr;
      vcData[3] = myVcAddr;

      //Send the data
      bytesToSend = write(tty, vcData, vcData[1] + 1);
      if (bytesToSend < 0)
      {
        perror("ERROR: Write to radio failed!");
        status = bytesToSend;
        break;
      }

      //Account for the bytes written
      bytesSent += bytesToSend;
    }
  } while (0);
  return status;
}

int radioToStdout()
{
  int bytesRead;
  int bytesSent;
  int bytesToSend;
  int8_t destAddr;
  uint8_t length;
  int maxfds;
  int status;
  int8_t srcAddr;
  struct timeval timeout;
  uint8_t * vcData;

  status = 0;
  do
  {
    //Read the virtual circuit header into the local buffer.
    vcData = outputBuffer;
    bytesRead = sizeof(outputBuffer);
    bytesRead = read(tty, outputBuffer, bytesRead);
    if (bytesRead < 0)
    {
      perror("ERROR: Read from radio failed!");
      status = bytesRead;
      break;
    }

/*
    //Display the VC header
    length = vcData[0];
    destAddr = vcData[1];
    srcAddr = vcData[2];
    printf("length: %d\n", length);
    printf("destAddr: %d\n", destAddr);
    printf("srcAddr: %d\n", srcAddr);

    //Read the message
    vcData = outputBuffer;
    bytesRead = read(STDIN, outputBuffer, length);
    if (bytesRead < 0)
    {
      perror("ERROR: Read from radio failed!");
      status = bytesRead;
      break;
    }
*/

    //Send this data over the VC
    bytesSent = 0;
    while (bytesSent < bytesRead)
    {
      //Send the data
      bytesToSend = bytesRead - bytesSent;
      if (bytesToSend > MAX_MESSAGE_SIZE)
        bytesToSend = MAX_MESSAGE_SIZE;
      bytesToSend = write(STDOUT, &vcData[bytesSent], bytesToSend);
      if (bytesToSend < 0)
      {
        perror("ERROR: Write to radio failed!");
        status = bytesToSend;
        break;
      }

      //Account for the bytes written
      bytesSent += bytesToSend;
    }
  } while (0);
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
  struct timeval timeout;
  uint8_t * vcData;

  status = 0;
  do
  {
    //Display the help text if necessary
    if ((argc != 2) || (sscanf(argv[1], "/dev/ttyACM%d", &myVcAddr) != 1))
    {
      printf("%s   terminal\n", argv[0]);
      printf("\n");
      printf("terminal - Name or path to the terminal device for the radio\n");
      status = -1;
      break;
    }

    //Determine the remote VC
    if (myVcAddr)
      remoteVcAddr = 0;
    else
      remoteVcAddr = 1;

    //Update STDIN and STDOUT
/*
    status = updateTerm(STDIN);
    if (status)
      break;
    status = updateTerm(STDOUT);
    if (status)
      break;
*/

    //Open the terminal
    maxfds = STDIN;
    status = openTty(argv[1]);
    if (status)
      break;
    if (maxfds < tty)
      maxfds = tty;

    //Display myVCAddr
    printf("myVcAddr: %d\n", myVcAddr);

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
        //Write the radio data to console output
        status = radioToStdout();
        if (status)
          break;
      }
    }
  } while (0);
  return status;
}
