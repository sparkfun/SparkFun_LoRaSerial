#include "settings.h"

int openLoRaSerial(const char *terminal)
{
#if defined(_WIN32) || defined(_WIN64)
  // Open serial port
  // https://stackoverflow.com/questions/15794422/serial-port-rs-232-connection-in-c

  char terminalString[20];
  sprintf(terminalString, "\\\\.\\%s", terminal);

  radioHandle =
      CreateFile(terminalString, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);

  if (radioHandle == INVALID_HANDLE_VALUE)
  {
      printf("ERROR: Failed to open the terminal: %s\n", terminalString);
      //listPorts();

      return (0);
  }

  // Do some basic settings
  DCB serialParams = {0};
  serialParams.DCBlength = sizeof(serialParams);

  GetCommState(radioHandle, &serialParams);
  serialParams.BaudRate = 115200;
  serialParams.ByteSize = 8;
  serialParams.StopBits = ONESTOPBIT;
  serialParams.Parity = NOPARITY;
  SetCommState(radioHandle, &serialParams);

  // Set timeouts
  COMMTIMEOUTS timeout = {0};
  timeout.ReadIntervalTimeout = 5;
  timeout.ReadTotalTimeoutConstant = 5;
  timeout.ReadTotalTimeoutMultiplier = 50;
  timeout.WriteTotalTimeoutConstant = 50;
  timeout.WriteTotalTimeoutMultiplier = 10;

  SetCommTimeouts(radioHandle, &timeout);

  printf("%s open at %dbps\n", terminal, serialParams.BaudRate);

#elif defined(__linux__)
  struct termios params;
  int status;

  radio = open (terminal, O_RDWR);
  if (radio < 0)
  {
    perror ("ERROR: Failed to open the terminal");
    return errno;
  }

  if (tcgetattr(radio, &params) != 0) {
      perror("ERROR: tcgetattr failed!");
      return errno;
  }

  //Initialize the terminal parameters
  params.c_cflag &= ~(CSIZE | CSTOPB | PARENB);
  params.c_cflag |= CS8 | CRTSCTS | CREAD | CLOCAL;

  params.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);

  params.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

  params.c_oflag &= ~(OPOST | ONLCR);

  cfsetispeed(&params, B57600);
  cfsetospeed(&params, B57600);

  if (tcsetattr(radio, TCSANOW, &params) != 0) {
      perror("ERROR: tcsetattr failed!");
      return errno;
  }
#endif

  usingTerminal = true;
  return 0;
}

//Platform agnostic radio read
int radioRead(int radio, uint8_t * buffer, int bufferLength)
{
  int bytes;

#if defined(_WIN32) || defined(_WIN64)
  DWORD bytesRead = 0;
  ReadFile(radioHandle, buffer, bufferLength, &bytesRead, NULL);
  bytes = (int)bytesRead;
#elif defined(__linux__)
  bytes = read(radio, buffer, bufferLength);
#endif
  return (bytes);
}

int readLoRaSerial(uint8_t * buffer, int bufferLength)
{
  int bytes;
  int length;

  //Read the frame type
  bytes = radioRead(radio, buffer, 1);
  if (bytes < 0)
  {
    perror("Failed to read frame type!");
    exit(bytes);
  }
  rxBytes = bytes;

  //Read the length in bytes
  bytes = radioRead(radio, &buffer[rxBytes], 1);
  if (bytes < 0)
  {
    perror("Failed to read length byte!");
    exit(bytes);
  }

  //Get the length in bytes
  length = buffer[rxBytes];
  rxBytes += bytes;

  //Read the length in bytes
  while (length - rxBytes)
  {
    bytes = radioRead(radio, &buffer[rxBytes], length - rxBytes);
    if (bytes < 0)
    {
      perror("Failed to read remaining data!");
      exit(bytes);
    }
    rxBytes += bytes;
  }

  return rxBytes;
}

// int updateTerm(int fd)
// {
//   struct termios params;
//   int status;

//   if (tcgetattr(fd, &params) != 0) {
//       perror("ERROR: STDIN tcgetattr failed!");
//       return errno;
//   }

//   //Initialize the terminal parameters
//   params.c_iflag &= ~(IXON | IXOFF | ISTRIP);

//   if (tcsetattr(fd, TCSANOW, &params) != 0) {
//       perror("ERROR: STDIN tcsetattr failed!");
//       return errno;
//   }

//   return 0;
// }
