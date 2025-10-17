// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>

// Error Handling

void errorExit(const char *error_message);

// Transmitter

#define DATA_PACKET_START 0x01
#define DATA_PACKET_DATA 0x02
#define DATA_PACKET_END 0x03

int transmitter(const char *filename);
int buildControlPacket(unsigned char *packet, const char control_field, unsigned int filesize, const char *filename);
int buildDataPacket(unsigned char *packet, unsigned char *data_field, int data_size);

// Receiver

int receiver(const char *filename);

// Application Layer main function

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    // Link layer connection parameters
    LinkLayer connect_params = {serialPort, role, baudRate, nTries, timeout};

    // open Data Link connection
    if (llopen(connect_params) != 0) errorExit("Failed to open Data Link.\n");

    // Transmitter and Receiver have different processes
    if (strcmp("tx", role) == 0) {
        // TODO transmitter
    }
    else
    if (strcmp("rx", role) != 0) {
        // TODO receiver
    }
    else errorExit("Unexpected value for Data Link role.\nRole must be \"tx\" or \"rx\".\n");

    // close Data Link connection
    if (llclose() != 0) errorExit("Failed to close Data Link.\n");

    return;
}

// Error Handling function that Exits program with value 1 (EXIT_FAILURE)
// Arguments:
//  error_message: message to printf() before exiting the program.
void errorExit(const char *error_message) {
    printf("\nCritical Error ocurred.\n");
    printf(error_message);
    printf("Exiting program.\n");
    exit(1);
}

int transmitter(const char *filename) {

    // TODO
    printf("\"transmitter()\" function not yet implemented.\n");

    return 1;
}

// Builder function for Control Packets
// Arguments:
//  packet: empty buffer to write control packet.
//  control_field: octet of value 0x01 (START) or 0x03 (END)
//  data_size: size of the data provided in the data_field for this data packet. Must range between 0 and min(65535, MAX_PAYLOAD_SIZE - 3).
// Return 0 on success or -1 on error.
int buildControlPacket(unsigned char *packet, const char control_field, unsigned int filesize, const char *filename) {

    // Check if control_field is expected value
    if (control_field != 0x01 || control_field != 0x03) {
        printf("Unexpected value of control field for control packet. Must be either 0x01 (START) or 0x03 (END).\n");
        return 1;
    }

    // Control Packet Parameter positions
    int filesize_octet = 1;
    int filename_octet = 7;

    // Check if size of filename is within expected values
    if (sizeof(filename) > MAX_PAYLOAD_SIZE - filename_octet) {
        printf("Size of Filename exceed limits.\n");
        return 1;
    }

    // Control Field
    strncpy(packet[0], control_field, 1);

    // File Size parameter
    strncpy(packet[filesize_octet + 0], 0x00, 1);
    strncpy(packet[filesize_octet + 1], 0x04, 1);
    strncpy(packet[filesize_octet + 2], filesize, 4);

    // File Name parameter
    strncpy(packet[filename_octet + 0], 0x00, 1);
    strncpy(packet[filename_octet + 1], 0x04, 1);
    strncpy(packet[filename_octet + 2], filename, sizeof(filename));

    return 0;
}

// Builder Function for Data Packets
// Arguments:
//  packet: empty buffer to write data packet.
//  data_field: location of data to write into the data field portion of the data packet.
//  data_size: size of the data provided in the data_field for this data packet. Must range between 0 and min(65535, MAX_PAYLOAD_SIZE - 3).
// Return 0 on success or -1 on error.
int buildDataPacket(unsigned char *packet, unsigned char *data_field, int data_size) {

    // Check if data_size is within expected values
    if (data_size < 0 || data_size > 65535 || data_size > MAX_PAYLOAD_SIZE - 3) {
        printf("Parameter \"data_size\" was outside expected range [0, min(65535, MAX_PAYLOAD_SIZE - 3)].\n");
        return 1;
    }

    // Control Field
    strncpy(packet[0], DATA_PACKET_DATA, 1);
    strncpy(packet[1], (unsigned short) data_size, 2);
    strncpy(packet[3], data_field, data_size);

    return 0;
}

int receiver(const char *filename) {

    // TODO
    printf("\"receiver()\" function not yet implemented.\n");

    return 1;
}
