// Example of how to read from the serial port in non-canonical mode
//
// Modified by: Eduardo Nuno Almeida [enalmeida@fe.up.pt]

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

#define _POSIX_SOURCE 1 // POSIX compliant source

#define FALSE 0
#define TRUE 1

#define BAUDRATE 38400
#define BUF_SIZE 256

#define FLAG    0x7E
#define A_TX    0x03  // comandos do Transmitter e respostas do Receiver 
#define A_RX    0x01  // comandos do Receiver e respostas do Transmitter
#define C_SET   0x03
#define C_UA    0x07

int fd = -1;           // File descriptor for open serial port
struct termios oldtio; // Serial port settings to restore on closing
volatile int STOP = FALSE;

int openSerialPort(const char *serialPort, int baudRate);
int closeSerialPort();
int readByteSerialPort(unsigned char *byte);
int writeBytesSerialPort(const unsigned char *bytes, int nBytes);

// ---------------------------------------------------
// MAIN
// ---------------------------------------------------
int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        printf("Incorrect program usage\n"
               "Usage: %s <SerialPort>\n"
               "Example: %s /dev/ttyS0\n",
               argv[0],
               argv[0]);
        exit(1);
    }

    // Open serial port device for reading and writing, and not as controlling tty
    // because we don't want to get killed if linenoise sends CTRL-C.
    //
    // NOTE: See the implementation of the serial port library in "serial_port/".
    const char *serialPort = argv[1];

    if (openSerialPort(serialPort, BAUDRATE) < 0)
    {
        perror("openSerialPort");
        exit(1);
    }

    printf("Serial port %s opened\n", serialPort);

    /*
    State machine to detect SET and then send UA
    Expected SET: [FLAG][A_TX][C_SET][A_TX ^ C_SET][FLAG]
    On success (STOP_ST) -> send UA: [FLAG][A_RX][C_UA][A_RX ^ C_UA][FLAG]
    */

    enum { START, FLAG_RCV, A_RCV, C_RCV, BCC_OK, STOP_ST } st = START;

    while (STOP == FALSE)
    {
        unsigned char b = 0;
        int r = readByteSerialPort(&b);
        if (r < 0) { perror("read"); closeSerialPort(); exit(1); }
        if (r == 0) continue;       

        switch (st)
        {
            case START:
                if (b == FLAG) st = FLAG_RCV;
                break;

            case FLAG_RCV:
                if      (b == FLAG) st = FLAG_RCV;
                else if (b == A_TX) st = A_RCV;
                else                st = START;
                break;

            case A_RCV:
                if      (b == FLAG)   st = FLAG_RCV;
                else if (b == C_SET)  st = C_RCV;
                else                  st = START;
                break;

            case C_RCV:
                if      (b == FLAG)                           st = FLAG_RCV;
                else if (b == (unsigned char)(A_TX ^ C_SET))  st = BCC_OK;
                else                                          st = START;
                break;

            case BCC_OK:
                if (b == FLAG) st = STOP_ST;
                else           st = START;
                break;

            case STOP_ST:
            default:
                break;
        }

        if (st == STOP_ST)
        {
            printf("SET received and validated\n");
            unsigned char UA[5] = { FLAG, A_RX, C_UA, (unsigned char)(A_RX ^ C_UA), FLAG };
            int bytes = writeBytesSerialPort(UA, 5);
            if (bytes < 0) { perror("writeBytesSerialPort"); closeSerialPort(); exit(1); }
            printf("%d bytes (UA) written to serial port\n", bytes);
            STOP = TRUE;  
        }

    }

    // Close serial port
    if (closeSerialPort() < 0)
    {
        perror("closeSerialPort");
        exit(1);
    }

    printf("Serial port %s closed\n", serialPort);

    return 0;
}

// ---------------------------------------------------
// SERIAL PORT LIBRARY IMPLEMENTATION
// ---------------------------------------------------

// Open and configure the serial port.
// Returns -1 on error.
int openSerialPort(const char *serialPort, int baudRate)
{
    // Open with O_NONBLOCK to avoid hanging when CLOCAL
    // is not yet set on the serial port (changed later)
    int oflags = O_RDWR | O_NOCTTY | O_NONBLOCK;
    fd = open(serialPort, oflags);
    if (fd < 0)
    {
        perror(serialPort);
        return -1;
    }

    // Save current port settings
    if (tcgetattr(fd, &oldtio) == -1)
    {
        perror("tcgetattr");
        return -1;
    }

    // Convert baud rate to appropriate flag

    // Baudrate settings are defined in <asm/termbits.h>, which is included by <termios.h>
#define CASE_BAUDRATE(baudrate) \
    case baudrate:              \
        br = B##baudrate;       \
        break;

    tcflag_t br;
    switch (baudRate)
    {
        CASE_BAUDRATE(1200);
        CASE_BAUDRATE(1800);
        CASE_BAUDRATE(2400);
        CASE_BAUDRATE(4800);
        CASE_BAUDRATE(9600);
        CASE_BAUDRATE(19200);
        CASE_BAUDRATE(38400);
        CASE_BAUDRATE(57600);
        CASE_BAUDRATE(115200);
    default:
        fprintf(stderr, "Unsupported baud rate (must be one of 1200, 1800, 2400, 4800, 9600, 19200, 38400, 57600, 115200)\n");
        return -1;
    }
#undef CASE_BAUDRATE

    // New port settings
    struct termios newtio;
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = br | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    // Set input mode (non-canonical, no echo,...)
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0; // Block reading
    newtio.c_cc[VMIN] = 1;  // Byte by byte

    tcflush(fd, TCIOFLUSH);

    // Set new port settings
    if (tcsetattr(fd, TCSANOW, &newtio) == -1)
    {
        perror("tcsetattr");
        close(fd);
        return -1;
    }

    // Clear O_NONBLOCK flag to ensure blocking reads
    oflags ^= O_NONBLOCK;
    if (fcntl(fd, F_SETFL, oflags) == -1)
    {
        perror("fcntl");
        close(fd);
        return -1;
    }

    return fd;
}

// Restore original port settings and close the serial port.
// Returns 0 on success and -1 on error.
int closeSerialPort()
{
    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
    {
        perror("tcsetattr");
        return -1;
    }

    return close(fd);
}

// Wait up to 0.1 second (VTIME) for a byte received from the serial port.
// Must check whether a byte was actually received from the return value.
// Save the received byte in the "byte" pointer.
// Returns -1 on error, 0 if no byte was received, 1 if a byte was received.
int readByteSerialPort(unsigned char *byte)
{
    return read(fd, byte, 1);
}

// Write up to numBytes from the "bytes" array to the serial port.
// Must check how many were actually written in the return value.
// Returns -1 on error, otherwise the number of bytes written.
int writeBytesSerialPort(const unsigned char *bytes, int nBytes)
{
    return write(fd, bytes, nBytes);
}
