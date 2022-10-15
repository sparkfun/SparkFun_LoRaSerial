#include "settings.h"

int openSocket()
{
  const int broadcastEnable = 1;
  const int non_blocking = 1;
  int status;

  //Open the socket
  tty = socket(AF_INET, SOCK_DGRAM, 0);
  if (tty < 0)
  {
    status = errno;
    perror("Failed to open the socket!");
    return status;
  }

  //Set the broadcast IPv4 address
  memset(&broadcastAddr, 0, sizeof(broadcastAddr));
  broadcastAddr.sin_family = AF_INET;
  broadcastAddr.sin_addr.s_addr = BROADCAST_IPV4_ADDRESS;
  broadcastAddr.sin_port = htons(PORT);

  //Enable reception via any IPv4 address
  memset(&localAddr, 0, sizeof(localAddr));
  localAddr.sin_family = AF_INET;
  localAddr.sin_addr.s_addr = INADDR_ANY;
  localAddr.sin_port = htons(PORT);

  // Bind the socket with the server address
  status = bind(tty, (const struct sockaddr *)&localAddr, sizeof(localAddr));
  if (status < 0 )
  {
    perror("Failed to bind the socket!");
    return status;
  }

  //Support broadcasting
  status = setsockopt (tty, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable));
  if (status < 0)
  {
    perror("setsockopt failed");
    close(tty);
    return status;
  }

  //Set the socket to non-blocking
  status = ioctl(tty, FIONBIO, (char *)&non_blocking);
  if (status < 0)
  {
    perror("ioctl failed");
    close(tty);
    return status;
  }
  return 0;
}
