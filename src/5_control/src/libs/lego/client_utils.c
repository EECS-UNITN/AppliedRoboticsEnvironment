#include "lego/client_utils.h"

#include <stdio.h>
#include <getopt.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>

extern char *optarg;
extern int optind, opterr, optopt;


/*  This variable contains every option we might pass via command line and also
 *  if that option should have an argument or not (The ":" means that the
 *  precedent option needs an argument).
 */
static const char optstring[] = ":";

/*  This is almost the same as optstring[], but it contains the extended options
 *  (The ones that can be called using "--" instead of "-") and what they mean.
 */
static const struct option longopts[] = {
  {"mac", 1, NULL, 'm'},            // MAC address
  {"list-devices", 0, NULL, 'l'},   // List devices
  {NULL, 0, NULL, 0}
};

/*  This function checks if a MAC address passed via command line is correct
 *  and then converts it to a bdaddr_t for future uses.
 */
static
int chk_mac_addr (char * target, bdaddr_t * mac) {

    unsigned b0, b1, b2, b3, b4, b5;
    char buffer[18];

    if (sscanf(target, "%2x:%2x:%2x:%2x:%2x:%2x",
               &b0, &b1, &b2, &b3, &b4, &b5) == 6) {
        sprintf(buffer, "%02x:%02x:%02x:%02x:%02x:%02x",
                b0, b1, b2, b3, b4, b5);
        return str2ba(buffer, mac);
    } else {
        return -1;
    }
};

int opts_parse_config(client_options *options, int argc, char* argv[])
{
    int opt;
    brick_bt_device_t *devices[MAX_BT_DEVICES];
    size_t ndevs;
    int i;
    char conv_mac[18];
    
    int valid_mac = 0;

    while ((opt = getopt_long(argc, argv, optstring, longopts, NULL)) != -1)
    {
      switch (opt) {
        case 'm':
          if (chk_mac_addr(optarg, &options->mac) != -1) {
            valid_mac = 1;
	        }
	        break;
	      case 'l':
          ndevs = brick_bt_scan_devices(devices);
          fprintf(stderr, "Bluetooth Devices in Range:\n\n");
          for (i = 0; i < ndevs; i++) {
            ba2str(&devices[i]->mac, conv_mac);
            fprintf(stderr, "%d: %s [%s]\n", i+1, devices[i]->name, conv_mac);
	        }
	        return -1;
      }
    }
    
    optind = 1; // reset index, to allow a new parsing of other command line arguments

    if (!valid_mac) {
      fprintf(stderr, "Invalid mac address\n");
      return -1;
    }
    
    return 0;
}
