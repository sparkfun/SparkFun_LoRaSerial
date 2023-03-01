#ifndef __settings_h__
#define __settings_h__

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include "../LoRaSerial/Virtual_Circuit_Protocol.h"

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
//Defines
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

#define PORT                7272
#define UNIQUE_ID_BYTES     16

//#define false               0
//#define true                1

#define BROADCAST_IPV4_ADDRESS  inet_addr("192.168.86.255") //INADDR_BROADCAST

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
//Enums
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

typedef enum {
  FRAME_VC_HEARTBEAT = 0,
  FRAME_VC_DATA,
} FRAME_TYPE;

typedef enum
{
  STATE_RESET = 0,      // 0
  STATE_WAIT_TX_DONE,   // 1
  STATE_WAIT_RECEIVE,   // 2
} STATE;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
//Types
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

typedef uint8_t ADDRESS_MASK;

typedef struct _RESERVED_ADDRESS
{
  uint8_t uniqueId[UNIQUE_ID_BYTES];
  uint8_t addressByte;
} RESERVED_ADDRESS;

typedef struct _STATE_ENTRY
{
  STATE state;
  const char * name;
} STATE_ENTRY;

typedef struct _SETTINGS
{
  uint32_t heartbeatTimeout;
  bool trainingServer;
} Settings;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
//Globals
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

uint32_t currentTime;
uint32_t datagramTimer;
int8_t destAddr;
fd_set exceptfds;
int8_t myAddr;
uint8_t myUniqueId[UNIQUE_ID_BYTES];
int radio;
fd_set readfds;
uint8_t rxBuffer[255];
int rxBytes;
uint8_t * rxData;
Settings settings;
int8_t srcAddr;
int state;
bool transactionComplete;
uint8_t txBuffer[255];
int txBytes;
uint8_t * txData;
bool usingTerminal;
fd_set writefds;

struct sockaddr_in localAddr;
struct sockaddr_in remoteAddr;
struct sockaddr_in broadcastAddr;
socklen_t remoteAddrLength;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
//Address Byte Management
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

ADDRESS_MASK freeAddresses;
ADDRESS_MASK linkUp;
#define MAX_CLIENT_ADDRESS      ((uint8_t)(sizeof(freeAddresses) * 8))
RESERVED_ADDRESS * addressList[MAX_CLIENT_ADDRESS];

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
//Forward Declarations
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//RadioV2
FRAME_TYPE rcvDatagram();
void returnToReceiving();
void transmitDatagram(const struct sockaddr * addr, int addrLen);
void xmitVcHeartbeat(int8_t addr, uint8_t * id);

//Sockets
int openSocket();

//States
void changeState(int newState);
void loop();

//Systems
void defaultSettings(Settings * settings);
void dumpBuffer(uint8_t * data, int length);
int8_t idToAddressByte(int8_t srcAddr, uint8_t * id);
uint32_t millis();
void petWDT();
int processData();

//Terminal
int openLoRaSerial(const char *ttyName);
int readLoRaSerial(uint8_t * buffer, int bufferLength);
int updateTerm(int fd);

#endif  // __settings_h__
