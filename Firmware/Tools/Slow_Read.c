#include "settings.h"

#define INPUT_BUFFER_SIZE       2048
#define OUTPUT_BUFFER_SIZE      32
#define POLL_TIMEOUT_USEC       100000
#define STDIN                   0
#define STDOUT                  1
#define STDERR                  2

uint8_t inputBuffer[INPUT_BUFFER_SIZE];
uint8_t outputBuffer[OUTPUT_BUFFER_SIZE];
uint32_t timeoutCount;
suseconds_t timer;

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
  do
  {
    //Read the console input data into the local buffer.
    bytesRead = read(STDIN, inputBuffer, sizeof(inputBuffer) - 3);
    if (bytesRead < 0)
    {
      perror("ERROR: Read from stdin failed!");
      status = bytesRead;
      break;
    }

    //Terminate the line
    inputBuffer[bytesRead++] = '\r';
    inputBuffer[bytesRead++] = '\n';
    inputBuffer[bytesRead] = 0;

    //Send this data over the VC
    bytesSent = 0;
    while (bytesSent < bytesRead)
    {
      //Send the data
      bytesWritten = write(radio, &inputBuffer[bytesSent], bytesRead - bytesSent);
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

int radioToHost()
{
  int bytesRead;
  int bytesSent;
  int bytesToSend;
  int bytesWritten;
  int status;

  status = 0;
  do
  {
    //Read the virtual circuit header into the local buffer.
    bytesRead = read(radio, outputBuffer, sizeof(outputBuffer) - 1);
    if (bytesRead == 0)
      break;
    if (bytesRead < 0)
    {
      perror("ERROR: Read from radio failed!");
      status = bytesRead;
      break;
    }

    //Write this data to stdout
    bytesSent = 0;
    status = 0;
    while (bytesSent < bytesRead)
    {
      bytesWritten = write(STDOUT, &outputBuffer[bytesSent], bytesRead - bytesSent);
      if (bytesWritten < 0)
      {
        perror("ERROR: Write to stdout!");
        status = bytesWritten;
        break;
      }

      //Account for the bytes written
      bytesSent += bytesWritten;
    }
  } while(0);
  return status;
}

int main(int argc, char **argv)
{
  fd_set currentfds;
  suseconds_t delta_usec;
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
    if (argc != 2)
    {
      printf("%s   terminal\n", argv[0]);
      printf("\n");
      printf("terminal - Name or path to the terminal device for the radio\n");
      status = -1;
      break;
    }

    //Get the path to the terminal
    terminal = argv[1];

    //Open the terminal
    status = openLoRaSerial(terminal);
    if (status)
      break;

    FD_ZERO(&readfds);
    FD_SET(STDIN, &readfds);
    if (maxfds < radio)
      maxfds = radio;
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

      //----------------------------------------
      // Terminal --> Radio
      //----------------------------------------

      //Determine if console input is available
      if (FD_ISSET(STDIN, &currentfds))
      {
        //Send the console input to the radio
        status = stdinToRadio();
        if (status)
          break;
      }

      //----------------------------------------
      // Detect radio data and start delay
      //----------------------------------------

      if ((!timer) && FD_ISSET(radio, &currentfds))
      {
        //Start the timer to read the data
        gettimeofday(&timeout, NULL);
        timer = timeout.tv_usec;
        if (!timer)
          timer = 1;
      }

      //----------------------------------------
      // Radio --> Terminal, after delay
      //----------------------------------------

      //Check for time to receive more data
      gettimeofday(&timeout, NULL);
      delta_usec = (1000000 + timeout.tv_usec - timer) % 1000000;
      if (timer && (delta_usec >= POLL_TIMEOUT_USEC))
      {
        status = radioToHost();
        if (status)
          break;
        timer = 0;
      }

      //----------------------------------------
      // Delay for a while
      //----------------------------------------

      if (numfds == 0)
        timeoutCount++;
    }
  } while (0);

  //Done with the radio
  close(radio);
  return status;
}
