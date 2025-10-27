// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define PACKET_START 0x01
#define PACKET_DATA 0x02
#define PACKET_END 0x03

#define CONTROL_FILESIZE_LENGTH 4
#define CONTROL_FILESIZE_OCTET 1
#define CONTROL_FILENAME_OCTET 7

#define DATA_FIELD_SIZE (MAX_PAYLOAD_SIZE - 3)

// Error Handling

void errorExit(const char *error_message);

// Transmitter

int transmitter(const char *filename);
int buildControlPacket(unsigned char *packet, const char control_field, unsigned int filesize, const char *filename);
int buildDataPacket(unsigned char *packet, unsigned char *data_field, int data_size);

// Receiver

int receiver(const char *filename);
int extractControlPacket(unsigned char *packet, int packet_size, unsigned int *filesize, char *filename);
int extractDataPacket(unsigned char *packet, int packet_size, unsigned char *data_field);

// Application Layer main function

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    // Connection Role
    LinkLayerRole llr;
    if (strcmp(role, "tx") == 0) llr = LlTx;
    else if (strcmp(role, "rx") == 0) llr = LlRx;
    else errorExit("Unexpected value for Data Link role.\nRole must be \"tx\" or \"rx\".\n");

    // Link layer connection parameters
    LinkLayer connect_params;
    memset(&connect_params, 0, sizeof(LinkLayer));

    strcpy(connect_params.serialPort, serialPort);
    connect_params.role = llr;
    connect_params.baudRate = baudRate;
    connect_params.nRetransmissions = nTries;
    connect_params.timeout = timeout;


    // open Data Link connection
    if (llopen(connect_params) != 0) errorExit("Failed to open Data Link.\n");

    // Transmitter and Receiver have different processes
    switch (llr) {
        case LlTx:
            if (transmitter(filename) != 0) errorExit("Error encountered during file transmission.\n");
            break;
        case LlRx:
            if (receiver(filename) != 0) errorExit("Error encountered during file reception.\n");
            break;
        default:
            errorExit("Unexpected value for Data Link role.\nRole must be \"tx\" or \"rx\".\n");
            break;
    }

    // close Data Link connection
    if (llclose() != 0) errorExit("Failed to close Data Link.\n");

    return;
}

// Error Handling function that Exits program with value 1 (EXIT_FAILURE)
// Arguments:
//  error_message: message to printf() before exiting the program.
void errorExit(const char *error_message) {
    printf("\nCritical Error ocurred.\n");
    printf("%s", error_message);
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
    unsigned char *start_control_packet = (unsigned char *)calloc(MAX_PAYLOAD_SIZE, sizeof(unsigned char));
    if (start_control_packet == NULL) {
        printf("Memory Allocation for control_packet failed.\n");
        return -1;
    }

    // Build Start Control Packet
    if (buildControlPacket(start_control_packet, PACKET_START, (unsigned int) filesize, filename) != 0) {
        printf("Error encountered while building Control Packet.\n");
        return -1;
    }

    // Transmit Start Control Packet
    if (llwrite(start_control_packet, 9 + sizeof(filename)) < 0) {
        printf("Error occurred while transmitting Control Packet.\n");
        return -1;
    }

    // Free Allocated memory to control_packet
    free(start_control_packet);
    start_control_packet = NULL;

    // TRANSMIT FILE

    // Allocate buffer for reading file data
    unsigned char *data = (unsigned char *)calloc(DATA_FIELD_SIZE, sizeof(unsigned char));
    if (data == NULL) {
        printf("Memory Allocation for data failed.\n");
        return -1;
    }
    
    // Allocate buffer for Data Packet
    unsigned char *data_packet = (unsigned char *)calloc(MAX_PAYLOAD_SIZE, sizeof(unsigned char));
    if (data_packet == NULL) {
        printf("Memory Allocation for data_packet failed.\n");
        return -1;
    }

    while (feof(file) == 0) {

        // Reading portion of file
        size_t count = fread(data, 1, DATA_FIELD_SIZE, file);
        printf("Read %d Bytes from file.\n", (int)count);

        // Check if file stream encountered error
        if (count != DATA_FIELD_SIZE) {
            if (ferror(file) != 0) {
                printf("Unexpected error encountered while reading file.\n");
                return -1;
            }

        } else {

            // Build Data Packet
            if (buildDataPacket(data_packet, data, DATA_FIELD_SIZE) != 0) {
                printf("Error encountered while building Data Packet.\n");
                return -1;
            }

            // Transmit Data Packet
            if (llwrite(data_packet, DATA_FIELD_SIZE + 3) < 0) {
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
    unsigned char *end_control_packet = (unsigned char *)calloc(MAX_PAYLOAD_SIZE, sizeof(unsigned char));
    if (end_control_packet == NULL) {
        printf("Memory Allocation for control_packet failed.\n");
        return -1;
    }

    // Build End Control Packet
    if (buildControlPacket(end_control_packet, PACKET_END, (unsigned int) filesize, filename) != 0) {
        printf("Error encountered while building Control Packet.\n");
        return -1;
    }

    // Transmit End Control Packet
    if (llwrite(end_control_packet, sizeof(end_control_packet)) < 0) {
        printf("Error occurred while transmitting Control Packet.\n");
        return -1;
    }

    // Free memory allocated to control_packet
    free(end_control_packet);
    end_control_packet = NULL;

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
//  filename: name of the file to be transmitted. If the size of the file name is above 255, the function exits with error.
// Return 0 on success or -1 on error.
int buildControlPacket(unsigned char *packet, const char control_field, unsigned int filesize, const char *filename) {

    // Check if control_field is expected value
    if (control_field != 0x01 && control_field != 0x03) {
        printf("Unexpected value of control field for control packet. Must be either 0x01 (START) or 0x03 (END).\n");
        return -1;
    }

    // Check if size of filename is within expected values
    if (strlen(filename) > 255) {
        printf("Size of Filename exceed limits.\n");
        return -1;
    }

    // Clear packet memory
    memset(packet, 0x00, MAX_PAYLOAD_SIZE);

    // Control Field
    packet[0] = control_field;

    // File Size parameter
    packet[CONTROL_FILESIZE_OCTET + 0] = 0x00;  
    packet[CONTROL_FILESIZE_OCTET + 1] = 0x04;  
    memcpy(&packet[CONTROL_FILESIZE_OCTET + 2], &filesize, 4);  
    // File Name parameter
    packet[CONTROL_FILENAME_OCTET + 0] = 0x01; 
    packet[CONTROL_FILENAME_OCTET + 1] = (unsigned char)strlen(filename);
    memcpy(&packet[CONTROL_FILENAME_OCTET + 2], filename, strlen(filename));

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

    // Clear packet memory
    memset(packet, 0x00, MAX_PAYLOAD_SIZE);

    // Control Field
    packet[0] = PACKET_DATA; 
    packet[1] = (data_size >> 8) & 0xFF;  
    packet[2] = data_size & 0xFF;        
    memcpy(&packet[3], data_field, data_size); 

    return 0;
}

int receiver(const char *filename) {

    // Variables reused through the whole function
    int packet_size;

    // Allocate Memory for the packets to be received
    unsigned char *packet = (unsigned char *)calloc(MAX_PAYLOAD_SIZE, sizeof(unsigned char));
    if (packet == NULL) {
        printf("Memory Allocation for packet failed.\n");
        return -1;
    }

    // RECEIVE START CONTROL PACKET

    // Call Link Layer to provide packet read
    if ((packet_size = llread(packet)) == -1) {
        printf("Error encountered while reading Start Control Packet.\n");
        return -1;
    }

    // Check if Packet is a Start Control Packet
    if (packet[0] != PACKET_START) {
        printf("Error: Unexpected Packet type. Expected Packet was \"%c\" but read Packet was \"%c\".\n", PACKET_START, packet[0]);
        return -1;
    }

    // Extract File Information from Start Control Packet
    unsigned int received_filesize;
    char received_filename[255];
    if (extractControlPacket(packet, packet_size, &received_filesize, received_filename) != 0) {
        printf("Error ocurred while extracting file info from Start Control Packet.\n");
        return -1;
    }

    // CREATE FILE

    printf("Attempting to create the file \"%s\".\n", filename);
    FILE* file;
    file = fopen(filename, "w");

    // Check if file is created successfully
    if (file == NULL) {
        printf("The file \"%s\" could not be created.\n", filename);
        return -1;
    }
    printf("The file \"%s\" was created successfully.\n", filename);

    // RECEIVE FILE

    // Prepare memory buffer for Data Field in Data Packet
    unsigned char *data_field = (unsigned char *)calloc(DATA_FIELD_SIZE, sizeof(unsigned char));
    if (packet == NULL) {
        printf("Memory Allocation for data_field failed.\n");
        return -1;
    }

    // Loop requests file packets until it hits the expected file size
    unsigned int characters_read = 0;
    int packet_count = 0;
    while (characters_read < received_filesize) {

        // Clear memory buffer for receiving Data Packet
        memset(packet, 0x00, MAX_PAYLOAD_SIZE);

        // Request Data Packet
        if ((packet_size = llread(packet)) == -1) {
            printf("Error encountered while reading Data Packet number \"%d\".\n", packet_count);
            return -1;
        }

        // Check if Packet is a Start Control Packet
        if (packet[0] != PACKET_DATA) {
            printf("Error: Unexpected Packet type. Expected Packet was \"%c\" but read Packet was \"%c\".\n", PACKET_DATA, packet[0]);
            return -1;
        }

        printf("Read Data Packet number %d with %d Bytes.\n", packet_count, packet_size);

        // Extract File Data from Data Packet
        if (extractDataPacket(packet, packet_size, data_field) != 0) {
            printf("Error occurred while extracting Data Field from Data Packet.\n");
            return -1;
        }

        // Write Data into new File
        // Check if packet contains trailing zeros
        if (characters_read + packet_size <= received_filesize) {

            // Check if data_field was written into file correctly
            if (fwrite(data_field, sizeof(char), packet_size, file) != packet_size) {
                printf("Error writing data field into file.\n");
                return -1;
            }
        } else {

            // Check if data_field was written into file correctly
            if (fwrite(data_field, sizeof(char), received_filesize - characters_read, file) != received_filesize - characters_read) {
                printf("Error writing data field into file.\n");
                return -1;
            }
        }
        
        characters_read += packet_size;
        packet_count++;
    }

    // Free memory allocated to data_field
    free(data_field);
    data_field = NULL;

    // RECEIVE END CONTROL PACKET

    // Clear memory buffer for receiving End Control Packet
    memset(packet, 0x00, MAX_PAYLOAD_SIZE);

    // Call Link Layer to provide packet read
    if ((packet_size = llread(packet)) == -1) {
        printf("Error encountered while reading End Control Packet.\n");
        return -1;
    }

    // Check if Packet is a End Control Packet
    if (packet[0] != PACKET_END) {
        printf("Error: Unexpected Packet type. Expected Packet was \"%c\" but read Packet was \"%c\".\n", PACKET_END, packet[0]);
        return -1;
    }

    // Extract File Information from End Control Packet
    unsigned int received_filesize_end;
    char received_filename_end[255];
    if (extractControlPacket(packet, packet_size, &received_filesize_end, received_filename_end) != 0) {
        printf("Error ocurred while extracting file info from End Control Packet.\n");
        return -1;
    }

    // Compare File Information from Start Control Packet and End Control Packet
    if (received_filesize != received_filesize_end || strcmp(received_filename, received_filename_end) != 0) {
        printf("Error: File Information from Start Control Packet is inconsistent with End Control Packet.\n");
        return -1;
    }

    // Free memory allocated to packet
    free(packet);
    packet = NULL;

    // CLOSE FILE

    if (fclose(file) != 0) {
        printf("Unexpected Error while closing file stream.\n");
        return -1;
    }
    printf("File \"%s\" closed.\n", filename);

    return 0;
}


int extractControlPacket(unsigned char *packet, int packet_size, unsigned int *filesize, char *filename) {
    
    // Extract File size
    memcpy(filesize, &packet[CONTROL_FILESIZE_OCTET + 2], CONTROL_FILESIZE_LENGTH);

    // Extract File name size
    int filename_size = packet[CONTROL_FILENAME_OCTET + 1];

    // Extract File name
    memcpy(filename, &packet[CONTROL_FILENAME_OCTET + 2], filename_size);
    filename[filename_size] = '\0';

    return 0;
}

int extractDataPacket(unsigned char *packet, int packet_size, unsigned char *data_field) {

    // Clear data_field buffer
    memset(data_field, 0x00, DATA_FIELD_SIZE);

    // Extract Data Field size
    int data_field_size = (packet[1] << 8) | packet[2];

    // Copy Data Field into data_field buffer
    memcpy(data_field, &packet[3], data_field_size);

    return 0;
}
