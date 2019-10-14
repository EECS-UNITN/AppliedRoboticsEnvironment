#ifndef CLIENT_UTILS_H
#define CLIENT_UTILS_H

#include "lego/client_bt.h"

typedef struct
{
  /*
   *  This is the 6-bytes encoding variable type defined by the Libbluetooth.
   */
  bdaddr_t mac;
} client_options;

int opts_parse_config(client_options *options, int argc, char* argv[]);

#endif
