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
        if (transmitter(filename) != 0) errorExit("Error encountered during file transmission.\n");
    }
    else
    if (strcmp("rx", role) != 0) {
        if (receiver(filename) != 0) errorExit("Error encountered during file reception.\n");
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

// Main function for the Transmitter's Application Layer
// Arguments:
//  filename: name of the file to be transmitted.
// Return 0 on success or -1 on error.
int transmitter(const char *filename) {

    // OPEN FILE

    printf("Attempting to open the file \"%s\".\n", filename);
    FILE* file;
    file = fopen(filename, "r");

    // Check if file is opened successfully
    if (file == NULL) {
        printf("The file \"%s\" could not be opened.\n", filename);
        return -1;
    }
    printf("The file \"%s\" was opened successfully.\n", filename);

    // Get file size
    fseek(file, 0L, SEEK_END);
    int filesize = ftell(file);
    fseek(file, 0, SEEK_SET);

    // SEND START CONTROL PACKET

    // Allocate buffer for Control Packet
    unsigned char *control_packet = (unsigned char *)calloc(9 + sizeof(filename), sizeof(unsigned char));
    if (control_packet == NULL) {
        printf("Memory Allocation for control_packet failed.\n");
        return -1;
    }

    // Build Start Control Packet
    if (buildControlPacket(control_packet, DATA_PACKET_START, (unsigned int) filesize, filename) != 0) {
        printf("Error encountered while building Control Packet.\n");
        return -1;
    }

    // Transmit Start Control Packet
    if (llwrite(control_packet, sizeof(control_packet)) != 0) {
        printf("Error occurred while transmitting Control Packet.\n");
        return -1;
    }

    // Free Allocated memory to control_packet
    free(control_packet);
    control_packet = NULL;

    // TRANSMIT FILE

    // Allocate buffer for reading file data
    unsigned char *data = (unsigned char *)calloc(MAX_PAYLOAD_SIZE - 3, sizeof(unsigned char));
    if (data == NULL) {
        printf("Memory Allocation for data failed.\n");
        return -1;
    }
    
    // Allocate buffer for Data Packet
    unsigned char *data_packet = (unsigned char *)calloc(MAX_PAYLOAD_SIZE - 3, sizeof(unsigned char));
    if (data_packet == NULL) {
        printf("Memory Allocation for data_packet failed.\n");
        return -1;
    }

    while (feof(file) == 0) {

        // Reading portion of file
        size_t count = fread(data, 1, sizeof(data), file);
        printf("Read %d Bytes from file.\n");

        // Check if file stream encountered error
        if (count != sizeof(data)) {
            if (ferror(file) != 0) {
                printf("Unexpected error encountered while reading file.\n");
                return -1;
            }

        } else {

            // Build Data Packet
            if (buildDataPacket(data_packet, data, sizeof(data)) != 0) {
                printf("Error encountered while building Data Packet.\n");
                return -1;
            }

            // Transmit Data Packet
            if (llwrite(data_packet, sizeof(data_packet)) != 0) {
                printf("Error ocurred while transmitting Data Packet.n");
                return -1;
            }
        }
    }

    printf("Finished reading and transmitting file.");

    // Free Allocated memory to data
    free(data);
    data = NULL;
    
    // Free Allocated memory to data_packet
    free(data_packet);
    data_packet = NULL;

    // SEND END CONTROL PACKET

    // Allocate buffer for Control Packet
    unsigned char *control_packet = (unsigned char *)calloc(9 + sizeof(filename), sizeof(unsigned char));
    if (control_packet == NULL) {
        printf("Memory Allocation for control_packet failed.\n");
        return -1;
    }

    // Build End Control Packet
    if (buildControlPacket(control_packet, DATA_PACKET_END, (unsigned int) filesize, filename) != 0) {
        printf("Error encountered while building Control Packet.\n");
        return -1;
    }

    // Transmit End Control Packet
    if (llwrite(control_packet, sizeof(control_packet)) != 0) {
        printf("Error occurred while transmitting Control Packet.\n");
        return -1;
    }

    // Free memory allocated to control_packet
    free(control_packet);
    control_packet = NULL;

    // CLOSE FILE

    if (fclose(file) != 0) {
        printf("Unexpected Error while closing file stream.\n");
        return -1;
    }
    printf("File \"%s\" closed.\n", filename);

    return 0;
}

// Builder function for Control Packets
// Arguments:
//  packet: empty buffer to write control packet.
//  control_field: octet of value 0x01 (START) or 0x03 (END).
//  filesize: size of the file to be transmitted in number of Bytes.
//  filename: name of the file to be transmitted. If the size of the name would put the control packet's size beyond MAX_PAYLOAD_SIZE, the function exits with error.
// Return 0 on success or -1 on error.
int buildControlPacket(unsigned char *packet, const char control_field, unsigned int filesize, const char *filename) {

    // Check if control_field is expected value
    if (control_field != 0x01 || control_field != 0x03) {
        printf("Unexpected value of control field for control packet. Must be either 0x01 (START) or 0x03 (END).\n");
        return -1;
    }

    // Control Packet Parameter positions
    int filesize_octet = 1;
    int filename_octet = 7;

    // Check if size of filename is within expected values
    if (sizeof(filename) > MAX_PAYLOAD_SIZE - 9) {
        printf("Size of Filename exceed limits.\n");
        return -1;
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
//  data_size: size of the data provided in the data_field for this data packet. Must be within range [0, MAX_PAYLOAD_SIZE - 3].
// Return 0 on success or -1 on error.
int buildDataPacket(unsigned char *packet, unsigned char *data_field, int data_size) {

    // Check if data_size is within expected values
    if (data_size < 0 || data_size > 65535 || data_size > MAX_PAYLOAD_SIZE - 3) {
        printf("Parameter \"data_size\" was outside expected range [0, MAX_PAYLOAD_SIZE - 3].\n");
        return -1;
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
