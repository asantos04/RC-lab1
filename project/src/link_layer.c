// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

// Standard Libraries
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <signal.h>


#define FLAG      0x7E
#define ESC       0x7D
#define ESC_FLAG  0x5E   
#define ESC_ESC   0x5D  

// Address Field
#define A_TX 0x03        
#define A_RX 0x01        

// Control Field 
#define C_SET  0x03
#define C_UA   0x07
#define C_DISC 0x0B
#define C_RR(r)   ((unsigned char)((r) ? 0xAB : 0xAA))
#define C_REJ(r)  ((unsigned char)((r) ? 0x55 : 0x54))
#define C_I(s)    ((unsigned char)((s) ? 0x80 : 0x00))

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

// BCC Helpers
#define BCC1(a,c) ((unsigned char)((a) ^ (c)))



static unsigned char bcc2_compute(const unsigned char *data, int len)
{
    unsigned char acc = 0x00;
    for (int i = 0; i < len; i++) acc ^= data[i];
    return acc;
}

// byte stuffing
static int stuff(const unsigned char *in, int len, unsigned char *out, int outMax)
{
    int o = 0;
    for (int i = 0; i < len; i++)
    {
        unsigned char b = in[i];
        if (b == FLAG || b == ESC)
        {
            if (o + 2 > outMax) return -1; 
            out[o++] = ESC;
            out[o++] = (b == FLAG) ? ESC_FLAG : ESC_ESC;
        }
        else
        {
            if (o + 1 > outMax) return -1;
            out[o++] = b;
        }
    }
    return o;
}

// byte destuffing
static int destuff(const unsigned char *in, int len, unsigned char *out, int outMax)
{
    int o = 0;
    for (int i = 0; i < len; i++)
    {
        unsigned char b = in[i];
        if (b == ESC)
        {
            if (i + 1 >= len) return -1;     
            unsigned char n = in[++i];
            unsigned char orig;

            if (n == ESC_FLAG) orig = FLAG;
            else if (n == ESC_ESC) orig = ESC;
            else return -1;  

            if (o + 1 > outMax) return -1;
            out[o++] = orig;
        }
        else
        {
            if (o + 1 > outMax) return -1;
            out[o++] = b;
        }
    }
    return o;
}

int write_frame(const unsigned char *frame, int len) {
    //to do
    return writeBytesSerialPort(frame, len);
}

int read_frame_SM(unsigned char *frame, int maxLen) {
    //to do 
    return 0; 
}

// Static variables
static int fd = -1;          
static int txSeq = 0;        
static int rxSeq = 0;        
static int timeout = 0;      
static int nRetransmissions; 
static LinkLayerRole currentRole;

// Alarm handling variables
volatile int alarmEnabled = FALSE;
volatile int alarmCount = 0;

void alarmHandler(int sig)
{
    alarm(0);
    alarmEnabled = FALSE;
    alarmCount++;
    printf("Alarm #%d triggered\n", alarmCount);
}

// Forward declarations
static int llopen_transmitter(void);
static int llopen_receiver(void);

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    fd = openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate);
    if (fd < 0) {
        perror("openSerialPort");
        return -1;
    }

    timeout = connectionParameters.timeout;
    nRetransmissions = connectionParameters.nRetransmissions;
    currentRole = connectionParameters.role;

    if (currentRole == LlTx) {
        return llopen_transmitter();
    } else {
        return llopen_receiver();
    }
}

int llopen_transmitter()
{
    struct sigaction act = {0};
    act.sa_handler = alarmHandler;
    if (sigaction(SIGALRM, &act, NULL) == -1) {
        perror("sigaction");
        return -1;
    }

    unsigned char SET[5] = { FLAG, A_TX, C_SET, BCC1(A_TX, C_SET), FLAG };
    unsigned char b = 0;
    int tries = 0;
    int UA_received = 0;

    for (tries = 0; tries <= nRetransmissions && !UA_received; tries++) {
        alarm(0);
        alarmEnabled = FALSE;
        printf("Sending SET (try %d/%d)\n", tries + 1, nRetransmissions + 1);
        writeBytesSerialPort(SET, 5);

        alarm(timeout); 
        alarmEnabled = TRUE;

        while (alarmEnabled && !UA_received) {
            int r = readByteSerialPort(&b);
            if (r < 0) { perror("readByteSerialPort"); return -1; }
            if (r == 0) continue;
            if (b != FLAG) continue;

            unsigned char next[4];
            int got = 0;
            while (got < 4) {
                r = readByteSerialPort(&next[got]);
                if (r < 0) { perror("readByteSerialPort"); return -1; }
                if (r == 0) continue;
                got += r;
            }

            if (next[0] == A_RX &&
                next[1] == C_UA &&
                next[2] == BCC1(A_RX, C_UA) &&
                next[3] == FLAG)
            {
                printf("UA received and validated\n");
                alarm(0);
                alarmEnabled = FALSE;
                UA_received = 1;
            }
        }
    }

    if (!UA_received) {
        printf("Connection failed after %d attempts\n", nRetransmissions);
        closeSerialPort();
        return -1;
    }

    printf("Link established successfully.\n");
    return 0;
}

int llopen_receiver()
{
    printf("Waiting for SET...\n");

    enum { START, FLAG_RCV, A_RCV, C_RCV, BCC_OK, STOP_ST } st = START;
    int STOP = FALSE;

    while (STOP == FALSE)
    {
        unsigned char b = 0;
        int r = readByteSerialPort(&b);
        if (r < 0) { perror("read"); closeSerialPort(); return -1; }
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

            default:
                break;
        }

        if (st == STOP_ST)
        {
            printf("SET received and validated\n");
            unsigned char UA[5] = { FLAG, A_RX, C_UA, (unsigned char)(A_RX ^ C_UA), FLAG };
            int bytes = writeBytesSerialPort(UA, 5);
            if (bytes < 0) { perror("writeBytesSerialPort"); closeSerialPort(); return -1; }
            printf("%d bytes (UA) written to serial port\n", bytes);
            STOP = TRUE;
        }
    }

    printf("Link established successfully (Receiver)\n");
    return 0;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    // TODO: Implement this function

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    // TODO: Implement this function

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose()
{
    // TODO: Implement this function

    return 0;
}
