#ifndef CLIENT_BT_H
#define CLIENT_BT_H

#include <stdint.h>
#include <sys/socket.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>


/** Maximum nuber of BlueTooth Devices discoverable. */
#define MAX_BT_DEVICES 255

/** Lenght of the BlueTooth Inquiry.
 *
 *  This time is not in seconds but in 1.28 seconds. Beats me why they have
 *  chosen to this, however it is discussed on the BlueZ tutorial here:
 *  http://people.csail.mit.edu/albert/bluez-intro/c404.html
 */
#define BT_INQUIRY_LEN 8

/** Structure for representing a BlueTooth Device.
 *
 *  It contains the data needed for connection and for the interaction with
 *  the user.
 */
typedef struct {
    /** Human Readable Name (As set on the Device). */
    char       name[248];
    /** Mac address.
     *
     *  The MAC address here is not stored as a Human Readable String, but
     *  as a @c bdaddr_t. This type is declared inside the BlueTooth
     *  Library (@c BlueZ or @c Libbluetooth) as an array of 6 @c uint8_t (Since
     *  a MAC address is a 6 integer tuple, all of which can have a maximum
     *  value of 0xFF or 255). This kind of variable can be transformed in
     *  a Human Readable String with the use of ba2str(const bdaddr_t*,
     *  char*) function, part of the Libbluetooth.
     */
    bdaddr_t   mac;
} brick_bt_device_t;

/** Discover all the BT Devices in range and then put their details in an array.
 *
 *  @param devices[MAX_BT_DEVICES] Array that will contains al the Devices Found.
 *  @return Number of Devices found.
 *  @retval -1 if any connection error emerged.
 */
size_t brick_bt_scan_devices (brick_bt_device_t *devices[MAX_BT_DEVICES]);
int brick_bt_connect_device(int *brick_socket, bdaddr_t mac_addr);
ssize_t brick_bt_send(int brick_socket, const uint8_t data[], size_t size);
ssize_t brick_bt_recv(int brick_socket, uint8_t incoming_data[], size_t size, int to);
int brick_bt_close_connection(int brick_socket);


#endif
