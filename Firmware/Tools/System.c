#include "settings.h"

void defaultSettings(Settings * settings)
{
  settings->trainingServer = false;
  settings->heartbeatTimeout = 5000;
}

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

uint32_t millis()
{
  return currentTime;
}

int8_t idToAddressByte(int8_t srcAddr, uint8_t * id)
{
  int8_t index;
  RESERVED_ADDRESS * addr;

  //Determine if the address is already in the list
  for (index = 0; index < MAX_CLIENT_ADDRESS; index++)
  {
    //Verify that an address is present
    if ((freeAddresses & (1 << index)) == 0)
      continue;

    //Compare the unique ID values
    if (memcmp(addressList[index]->uniqueId, id, UNIQUE_ID_BYTES) == 0)
    {
printf ("Address %d already assigned to ", index);
for (int i = 0; i < UNIQUE_ID_BYTES; i++)
printf("%02x", id[i]);
printf("\n");
      if ((linkUp & (1 << index)) == 0)
      {
        linkUp |= 1 << index;
        printf("==========  Link %d up  ==========\n", srcAddr);
      }
      return index;
    }
  }

  //Fill in clients that were already running
  index = srcAddr;

  //Only the server can assign the address bytes
  if ((srcAddr == VC_UNASSIGNED) && (!settings.trainingServer))
    return -1;

  //Assign an address if necessary
  if (srcAddr == VC_UNASSIGNED)
  {
    //Unknown client ID
    //Determine if there is a free address
    if (freeAddresses == (ADDRESS_MASK)(-1))
    {
      printf ("ERROR: Too many clients, no free addresses!\n");
      return -2;
    }

    //Look for the next free address byte
    for (index = 0; index < MAX_CLIENT_ADDRESS; index++)
      if ((freeAddresses & (1 << index)) == 0)
        break;

printf ("Server assigning address %d to : ", index);
for (int i = 0; i < UNIQUE_ID_BYTES; i++)
printf("%02x", id[i]);
printf("\n");
  }

  //Check for an address conflict
  if (addressList[index])
  {
    printf("ERROR: Unknown ID with pre-assigned conflicting address: %d\n", srcAddr);
    printf("Received ID: ");
    for (int i = 0; i < UNIQUE_ID_BYTES; i++)
      printf("%02x", id[i]);
    printf("\n");
    printf("Assigned ID: ");
    for (int i = 0; i < UNIQUE_ID_BYTES; i++)
      printf("%02x", addressList[srcAddr]->uniqueId[i]);
    printf("\n");
    return -3;
  }

  //Allocate the address structure
  addr = malloc(sizeof(*addr));
  if (!addr)
  {
    printf ("ERROR: No free memory, malloc of RESERVED_ADDRESS failed!\n");
    return -4;
  }

  //Reserve this address
  freeAddresses |= 1 << index;
  addr->addressByte = index;
  memcpy(&addr->uniqueId, id, UNIQUE_ID_BYTES);
  addressList[index] = addr;

  //Mark this link as up
  linkUp |= 1 << index;
  printf("==========  Link %d up  ==========\n", index);

  //Returned the assigned address
  return index;
}

void petWDT()
{
}

int processData()
{
  int deltaMillis;
  struct timeval start;
  int status;
  struct timeval stop;
  struct timeval timeout;

  //Initialize the fd_sets
  FD_ZERO(&exceptfds);
  FD_ZERO(&readfds);
  FD_ZERO(&writefds);

  //Implement the protocol
  changeState(STATE_RESET);
  gettimeofday(&start, NULL);
  while (1)
  {
    //Set the timeout
    timeout.tv_sec = 0;
    timeout.tv_usec = 1000;

    //Wait for receive data or timeout
    FD_SET(tty, &readfds);
    status = select(tty + 1, &readfds, &writefds, &exceptfds, &timeout);

    //Update the milliseconds since started
    gettimeofday(&stop, NULL);
    deltaMillis = ((stop.tv_sec - start.tv_sec) * 1000000) + stop.tv_usec - start.tv_usec;
    deltaMillis /= 1000;
    if (deltaMillis)
    {
      currentTime += deltaMillis;
      start = stop;
    }

    //Indicate receive interrupt
    if (status == 1)
      transactionComplete = 1;
    loop();
  }
  return 0;
}
