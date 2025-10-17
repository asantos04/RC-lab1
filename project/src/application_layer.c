// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>

void errorExit(const char *error_message);

int transmitter(const char *filename);
int buildControlPacket(unsigned char *packet, unsigned int filesize, const char *filename);
int buildDataPacket(unsigned char *packet, unsigned char *data_field, unsigned int data_size);

int receiver(const char *filename);

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

int buildControlPacket(unsigned char *packet, unsigned int filesize, const char *filename) {

    // TODO
    printf("\"buildControlPacket()\" function not yet implemented.\n");

    return 1;
}

int buildDataPacket(unsigned char *packet, unsigned char *data_field, unsigned int data_size) {

    // TODO
    printf("\"buildDataPacket()\" function not yet implemented.\n");

    return 1;
}

int receiver(const char *filename) {

    // TODO
    printf("\"receiver()\" function not yet implemented.\n");

    return 1;
}
