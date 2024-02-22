#include "settings.h"

int openLoRaSerial(const char *terminal)
{
  struct termios params;
  int status;

  radio = open (terminal, O_RDWR);
  if (radio < 0)
  {
    status = errno;
    fflush(stdout);
    fprintf(stderr, "ERROR: Failed to open the terminal");
    fprintf(stderr, "%s\n", strerror(status));
    fflush(stderr);
    return status;
  }

  if (tcgetattr(radio, &params) != 0) {
      status = errno;
      fflush(stdout);
      fprintf(stderr, "ERROR: tcgetattr failed!");
      fprintf(stderr, "%s\n", strerror(status));
      fflush(stderr);
      return status;
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
      status = errno;
      fflush(stdout);
      fprintf(stderr, "ERROR: tcsetattr failed!");
      fprintf(stderr, "%s\n", strerror(status));
      fflush(stderr);
      return status;
  }

  usingTerminal = true;
  return 0;
}

int readLoRaSerial(uint8_t * buffer, int bufferLength)
{
  int bytes;
  int length;

  //Read the frame type
  bytes = read(radio, buffer, 1);
  if (bytes < 0)
  {
    perror("Failed to read frame type!");
    exit(bytes);
  }
  rxBytes = bytes;

  //Read the length in bytes
  bytes = read(radio, &buffer[rxBytes], 1);
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
    bytes = read(radio, &buffer[rxBytes], length - rxBytes);
    if (bytes < 0)
    {
      perror("Failed to read remaining data!");
      exit(bytes);
    }
    rxBytes += bytes;
  }

  return rxBytes;
}

int updateTerm(int fd)
{
  struct termios params;
  int status;

  if (tcgetattr(fd, &params) != 0) {
      perror("ERROR: STDIN tcgetattr failed!");
      return errno;
  }

  //Initialize the terminal parameters
  params.c_iflag &= ~(IXON | IXOFF | ISTRIP);

  if (tcsetattr(fd, TCSANOW, &params) != 0) {
      perror("ERROR: STDIN tcsetattr failed!");
      return errno;
  }

  return 0;
}
