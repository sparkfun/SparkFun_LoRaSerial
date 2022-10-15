#include "settings.h"

int
main (
  int argc,
  char ** argv
)
{
  int status;

  if (argc == 2)
    //Open the terminal
    status = openTty(argv[1]);
  else
    //Open the socket
    status = openSocket();
  if (status)
    return status;

  //Set the ID
  myUniqueId[0] = 1;

  //Implement the protocol
  defaultSettings(&settings);
  return processData();
}
