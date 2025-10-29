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

        alarmEnabled = TRUE; 
        alarm(timeout);

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

        while (st != STOP_ST)
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
                unsigned char UA[5] = { FLAG, A_RX, C_UA, BCC1(A_RX, C_UA), FLAG };
                int bytes = writeBytesSerialPort(UA, 5);
                if (bytes < 0) { perror("writeBytesSerialPort"); closeSerialPort(); return -1; }
                printf("%d bytes (UA) written to serial port\n", bytes);
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
    if (buf == NULL || bufSize <= 0 || bufSize > MAX_PAYLOAD_SIZE)
    {
        fprintf(stderr, "llwrite: invalid buffer or size\n");
        return -1;
    }

    unsigned char bcc2 = bcc2_compute(buf, bufSize);

    unsigned char frame_raw[6 + MAX_PAYLOAD_SIZE];
    int pos = 0;

    frame_raw[pos++] = FLAG;
    frame_raw[pos++] = A_TX;
    frame_raw[pos++] = C_I(txSeq);
    frame_raw[pos++] = BCC1(A_TX, C_I(txSeq));
    memcpy(&frame_raw[pos], buf, bufSize);
    pos += bufSize;
    frame_raw[pos++] = bcc2;
    frame_raw[pos++] = FLAG;

    unsigned char frame_stuffed[2 * (6 + MAX_PAYLOAD_SIZE)];
    frame_stuffed[0] = FLAG;
    int stuffed_len = stuff(&frame_raw[1], pos - 2,
                            &frame_stuffed[1], sizeof(frame_stuffed) - 2);
    if (stuffed_len < 0)
    {
        fprintf(stderr, "llwrite: byte stuffing failed\n");
        return -1;
    }
    frame_stuffed[1 + stuffed_len] = FLAG;
    int total_len = stuffed_len + 2;

    int ack_received = 0;
    int tries = 0;
    unsigned char resp[5];

    while (tries <= nRetransmissions && !ack_received)
    {
        printf("\n[llwrite] Sending I-frame (seq=%d, try %d/%d)\n",
               txSeq, tries + 1, nRetransmissions + 1);

        int bytes = writeBytesSerialPort(frame_stuffed, total_len);
        if (bytes < 0)
        {
            perror("writeBytesSerialPort");
            return -1;
        }

        alarmEnabled = TRUE;
        alarm(timeout);

        enum { START, FLAG_RCV, A_RCV, C_RCV, BCC_OK, STOP_ST } st = START;
        unsigned char b = 0;

        while (alarmEnabled && st != STOP_ST)
        {
            int r = readByteSerialPort(&b);
            if (r < 0) { perror("readByteSerialPort"); return -1; }
            if (r == 0) continue;

            switch (st)
            {
                case START:
                    if (b == FLAG) st = FLAG_RCV;
                    break;

                case FLAG_RCV:
                    if      (b == FLAG) st = FLAG_RCV;
                    else if (b == A_RX) st = A_RCV;
                    else                st = START;
                    break;

                case A_RCV:
                    if      (b == FLAG) st = FLAG_RCV;
                    else                { resp[2] = b; st = C_RCV; }
                    break;

                case C_RCV:
                    if      (b == FLAG) st = FLAG_RCV;
                    else if (b == (A_RX ^ resp[2])) st = BCC_OK;
                    else st = START;
                    break;

                case BCC_OK:
                    if (b == FLAG)
                    {
                        st = STOP_ST; 
                        resp[0] = FLAG;
                        resp[1] = A_RX;
                        resp[3] = A_RX ^ resp[2];
                        resp[4] = FLAG;
                    }
                    else st = START;
                    break;
                
                default:
                    break;
            }

            if (st == STOP_ST)
            {
                if (resp[2] == C_RR(1 - txSeq))
                {
                    printf("[llwrite] RR received, transmission OK\n");
                    ack_received = 1;
                    alarm(0);
                    alarmEnabled = FALSE;
                }
                else if (resp[2] == C_REJ(txSeq))
                {
                    printf("[llwrite] REJ received, retransmitting frame\n");
                    alarm(0);
                    alarmEnabled = FALSE;
                }
                else
                {
                    printf("[llwrite] Unknown supervision frame: 0x%02X\n", resp[2]);
                }
            }
        }

        if (!ack_received)
        {
            tries++;
            printf("[llwrite] Timeout or REJ -> retransmitting (attempt %d)\n", tries + 1);
        }
    }

    if (!ack_received)
    {
        printf("[llwrite] ERROR: no ACK after %d attempts\n", nRetransmissions);
        return -1;
    }

    printf("[llwrite] Frame acknowledged successfully (seq=%d)\n", txSeq);

    txSeq = 1 - txSeq; 
    return bufSize;
}
////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    if (packet == NULL)
    {
        fprintf(stderr, "llread: invalid packet pointer\n");
        return -1;
    }

    unsigned char buffer[2 * (6 + MAX_PAYLOAD_SIZE)];
    int buf_len = 0;
    unsigned char b = 0;

    enum { START, FLAG_RCV, A_RCV, C_RCV, BCC1_OK, DATA, STOP_ST } st = START;

    printf("[llread] Waiting for I-frame...\n");

    while (st != STOP_ST)
    {
        int r = readByteSerialPort(&b);
        if (r < 0) { perror("readByteSerialPort"); return -1; }
        if (r == 0) continue; 

        switch (st)
        {
            case START:
                if (b == FLAG) { st = FLAG_RCV; buf_len = 0; buffer[buf_len++] = b; }
                break;

            case FLAG_RCV:
                if (b == FLAG) { st = FLAG_RCV; buf_len = 1; }  
                else if (b == A_TX) { st = A_RCV; buffer[buf_len++] = b; }
                else st = START;
                break;

            case A_RCV:
                if (b == FLAG) { st = FLAG_RCV; buf_len = 1; }
                else if (b == C_I(0) || b == C_I(1)) { st = C_RCV; buffer[buf_len++] = b; }
                else st = START;
                break;

            case C_RCV:
                if (b == FLAG) { st = FLAG_RCV; buf_len = 1; }
                else if (b == (buffer[1] ^ buffer[2])) { st = BCC1_OK; buffer[buf_len++] = b; }
                else st = START; 
                break;

            case BCC1_OK:
                if (b == FLAG) { st = START; }
                else { st = DATA; buffer[buf_len++] = b; }
                break;

            case DATA:
                buffer[buf_len++] = b;
                if (b == FLAG) { st = STOP_ST; }
                else if (buf_len >= sizeof(buffer))
                {
                    fprintf(stderr, "[llread] Frame too long!\n");
                    return -1;
                }
                else st = DATA;
                break;

            default:
                break;
        }
    }

    printf("[llread] Frame received (%d bytes)\n", buf_len);

    if (buf_len < 6) return -1; 
    int stuffed_len = buf_len - 2;
    unsigned char destuffed[6 + MAX_PAYLOAD_SIZE];
    int data_len = destuff(&buffer[1], stuffed_len, destuffed, sizeof(destuffed));
    if (data_len < 0)
    {
        fprintf(stderr, "[llread] Byte destuffing failed\n");
        return -1;
    }

    unsigned char A = destuffed[0];
    unsigned char C = destuffed[1];
    unsigned char BCC1 = destuffed[2];
    if (BCC1 != (A ^ C))
    {
        fprintf(stderr, "[llread] Header BCC1 error\n");
        return -1;
    }

    int payload_len = data_len - 4; 
    if (payload_len < 0)
    {
        fprintf(stderr, "[llread] Invalid payload length\n");
        return -1;
    }

    unsigned char BCC2 = destuffed[data_len - 1];
    unsigned char calc_bcc2 = bcc2_compute(&destuffed[3], payload_len);

    int is_new = (C == C_I(rxSeq));

    if (calc_bcc2 != BCC2)
    {
        printf("[llread] Data BCC2 error (expected 0x%02X, got 0x%02X)\n",
               calc_bcc2, BCC2);

        // Send REJ if new frame, RR if duplicate
        unsigned char rej_frame[5] = { FLAG, A_RX, C_REJ(rxSeq), BCC1(A_RX, C_REJ(rxSeq)), FLAG };
        unsigned char rr_dup[5] = { FLAG, A_RX, C_RR(1 - rxSeq), BCC1(A_RX, C_RR(1 - rxSeq)), FLAG };

        if (is_new)
        {
            writeBytesSerialPort(rej_frame, 5);
            printf("[llread] Sent REJ(%d)\n", rxSeq);
        }
        else
        {
            writeBytesSerialPort(rr_dup, 5);
            printf("[llread] Duplicate with error, sent RR(%d)\n", 1 - rxSeq);
        }

        return -1;
    }

    // --- If BCC2 OK ---
    if (is_new)
    {
        memcpy(packet, &destuffed[3], payload_len);
        rxSeq = 1 - rxSeq;
        unsigned char rr_frame[5] = { FLAG, A_RX, C_RR(rxSeq), BCC1(A_RX, C_RR(rxSeq)), FLAG };
        writeBytesSerialPort(rr_frame, 5);
        printf("[llread] Sent RR(%d)\n", rxSeq);
        return payload_len;
    }
    else
    {
        // Duplicate frame, resend RR
        unsigned char rr_dup[5] = { FLAG, A_RX, C_RR(rxSeq), BCC1(A_RX, C_RR(rxSeq)), FLAG };
        writeBytesSerialPort(rr_dup, 5);
        printf("[llread] Duplicate frame, resent RR(%d)\n", rxSeq);
        return 0; // no new data delivered to app
    }
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose()
{
    // TODO: Implement this function

    return 0;
}
