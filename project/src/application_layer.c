// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>

int transmitter();

int receiver();

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    // Link layer connection parameters
    LinkLayer connect_params = {serialPort, role, baudRate, nTries, timeout};

    // open Data Link connection
    if (llopen(connect_params) != 0) {
        // TODO error handling
    }

    // Transmitter and Receiver have different processes
    if (strcmp("tx", role) == 0) {
        // TODO transmitter
    } else
    if (strcmp("rx", role) != 0) {
        // TODO receiver
    } else {
        // TODO error handling
    }

    // close Data Link connection
    if (llclose() != 0) {
        // TODO error handling
    }

    return;
}
