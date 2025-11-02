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
static int rr_count = 0;
static int rej_count = 0;

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
    txSeq = 0;
    rxSeq = 0;
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
    const int TOTAL_ATTEMPTS = nRetransmissions + 1;

    for (tries = 0; tries <= nRetransmissions && !UA_received; tries++) {
        alarm(0);
        alarmEnabled = FALSE;
        printf("Sending SET (attempt %d/%d)%s\n",
            tries + 1, TOTAL_ATTEMPTS,
            (tries == 0 ? " [initial]" : " [retransmission]"));
        writeBytesSerialPort(SET, 5);

        alarmEnabled = TRUE; 
        alarm(timeout);

        while (alarmEnabled && !UA_received) {
            int r = readByteSerialPort(&b);
            if (r < 0) {
                if (errno == EINTR) continue;
                perror("readByteSerialPort");
                return -1;
            }
            if (r == 0) continue;
            if (b != FLAG) continue;

            unsigned char next[4];
            int got = 0;
            while (got < 4) {
                r = readByteSerialPort(&next[got]);
                if (r < 0) {
                    if (errno == EINTR) break;  
                    perror("readByteSerialPort");
                    return -1;
                }
                if (r == 0) continue;
                got += r;
            }

            if (next[0] == A_TX &&
                next[1] == C_UA &&
                next[2] == BCC1(A_TX, C_UA) &&
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
        alarm(0);
        alarmEnabled = FALSE;
        printf("Connection failed: no UA after %d attempts (1 initial + %d retransmissions)\n", TOTAL_ATTEMPTS, nRetransmissions);
        closeSerialPort();
        return -1;
    }

    printf("Link established successfully.\n");
    return 0;
}

    int llopen_receiver()
    {
        struct sigaction act = {0};
        act.sa_handler = alarmHandler;
        if (sigaction(SIGALRM, &act, NULL) == -1) {
            perror("sigaction");
            return -1;
        }

        printf("Waiting for SET...\n");

        enum { START, FLAG_RCV, A_RCV, C_RCV, BCC_OK, STOP_ST } st = START;
        
        alarmEnabled = TRUE;
        alarm(10 * timeout);

        while (st != STOP_ST && alarmEnabled)
        {
            unsigned char b = 0;
            int r = readByteSerialPort(&b);
            if (r < 0) {
                if (errno == EINTR) {
                    printf("[llopen_receiver] Timeout expired — no SET received.\n");
                    alarmEnabled = FALSE;
                    break; 
                }

                perror("readByteSerialPort");
                alarm(0);
                alarmEnabled = FALSE;
                closeSerialPort();
                return -1;
            }
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
                unsigned char UA[5] = { FLAG, A_TX, C_UA, BCC1(A_TX, C_UA), FLAG };
                int bytes = writeBytesSerialPort(UA, 5);
                if (bytes < 0) { perror("writeBytesSerialPort"); alarm(0); alarmEnabled = FALSE; closeSerialPort(); return -1; }
                printf("%d bytes (UA) written to serial port\n", bytes);
            }
        }

        alarm(0);
        alarmEnabled = FALSE;

        if (st != STOP_ST)
        {
            printf("[llopen_receiver] Timeout: transmitter never sent SET. Closing.\n");
            closeSerialPort();
            return -1;
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
    const int TOTAL_ATTEMPTS = nRetransmissions + 1;

    while (tries <= nRetransmissions && !ack_received)
    {
        printf("\n[llwrite] Sending I-frame (seq=%d, attempt %d/%d)%s\n",
            txSeq, tries + 1, TOTAL_ATTEMPTS,
            (tries == 0 ? " [initial]" : " [retransmission]"));

        int bytes = writeBytesSerialPort(frame_stuffed, total_len);
        if (bytes < 0)
        {
            perror("writeBytesSerialPort");
            return -1;
        }
        int outcome = 0;
        resp[2] = 0xFF;

        alarmEnabled = TRUE;
        alarm(timeout);

        enum { START, FLAG_RCV, A_RCV, C_RCV, BCC_OK, STOP_ST } st = START;
        unsigned char b = 0;

        while (alarmEnabled && st != STOP_ST)
        {
            int r = readByteSerialPort(&b);
            if (r < 0) {
                if (errno == EINTR || errno == EIO || errno == EAGAIN || errno == EWOULDBLOCK) continue;
                perror("readByteSerialPort");
                return -1;
            }
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
                    if      (b == FLAG) st = FLAG_RCV;
                    else                { resp[2] = b; st = C_RCV; }
                    break;

                case C_RCV:
                    if      (b == FLAG) st = FLAG_RCV;
                    else if (b == (A_TX ^ resp[2])) st = BCC_OK;
                    else st = START;
                    break;

                case BCC_OK:
                    if (b == FLAG)
                    {
                        st = STOP_ST; 
                        resp[0] = FLAG;
                        resp[1] = A_TX;
                        resp[3] = A_TX ^ resp[2];
                        resp[4] = FLAG;
                    }
                    else st = START;
                    break;
                
                default:
                    break;
            }

            if (st == STOP_ST) {
                if (resp[2] == C_RR(1 - txSeq)) {
                    printf("[llwrite] RR received, transmission OK\n");
                    outcome = 1;             
                    ack_received = 1;
                    alarm(0);
                    alarmEnabled = FALSE;
                }
                else if (resp[2] == C_REJ(txSeq)) {
                    printf("[llwrite] REJ received -> will retransmit same attempt (tries unchanged)\n");
                    outcome = -1;             
                    alarm(0);
                    alarmEnabled = FALSE;
                }
                else {
                    printf("[llwrite] Unknown supervision frame: 0x%02X\n", resp[2]);
                    st = START;
                }
            }
        }
        alarm(0);
        alarmEnabled = FALSE;

        if (!ack_received)
        {
            if (outcome == -1) {
                printf("[llwrite] REJ -> retransmitting same attempt (tries unchanged)\n");
                continue; 
            }

            tries++;
            if (tries > nRetransmissions) {
                printf("[llwrite] ERROR: no ACK after %d attempts (1 initial + %d retransmissions)\n",
                    TOTAL_ATTEMPTS, nRetransmissions);
                return -1;
            }
            printf("[llwrite] Timeout -> retransmitting (attempt %d/%d)\n", tries + 1, TOTAL_ATTEMPTS);
        }
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

    // enabling alarm window to receive frame
    alarmEnabled = TRUE;
    alarm(timeout);

    while (st != STOP_ST && alarmEnabled)
    {
        int r = readByteSerialPort(&b);
        if (r < 0)
        {
            if (errno == EINTR) {
                printf("[llread] Timeout expired — no frame received for too long.\n");
                alarm(0);
                alarmEnabled = FALSE;
                return -2; 
            }

            alarm(0);
            alarmEnabled = FALSE;
            perror("readByteSerialPort");
            return -1;
        }
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
                    // closing alarm window for frame reception
                    alarm(0);
                    alarmEnabled = FALSE;

                    fprintf(stderr, "[llread] Frame too long!\n");
                    return -1;
                }
                else st = DATA;
                break;

            default:
                break;
        }
    }

    // closing alarm window for frame reception
    alarm(0);
    alarmEnabled = FALSE;

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

        unsigned char rej_frame[5] = { FLAG, A_TX, C_REJ(rxSeq), BCC1(A_TX, C_REJ(rxSeq)), FLAG };
        unsigned char rr_dup[5]    = { FLAG, A_TX, C_RR(rxSeq),  BCC1(A_TX, C_RR(rxSeq)),  FLAG };                                          

        if (is_new)
        {
            writeBytesSerialPort(rej_frame, 5);
            printf("[llread] Sent REJ(%d)\n", rxSeq);
            rej_count++;
        }
        else
        {
            writeBytesSerialPort(rr_dup, 5);
            printf("[llread] Duplicate with error, sent RR(%d)\n", rxSeq);
        }

        return 0;
    }

    int frame_seq = (C == C_I(1)) ? 1 : 0;

    if (frame_seq == rxSeq)
    {
        memcpy(packet, &destuffed[3], payload_len);
        unsigned char rr_frame[5] = { FLAG, A_TX, C_RR(1 - rxSeq), BCC1(A_TX, C_RR(1 - rxSeq)), FLAG };
        writeBytesSerialPort(rr_frame, 5);
        printf("[llread] Accepted/retransmitted frame seq=%d, sent RR(%d)\n", frame_seq, 1 - rxSeq);
        rr_count++;
        rxSeq = 1 - rxSeq; 
        return payload_len;
    }
    else
    {
        unsigned char rr_dup[5] = { FLAG, A_TX, C_RR(rxSeq), BCC1(A_TX, C_RR(rxSeq)), FLAG };
        writeBytesSerialPort(rr_dup, 5);
        printf("[llread] Duplicate frame ignored, resent RR(%d)\n", rxSeq);
        return 0;
    }


}

// forward declarations

int sendSUframe ( unsigned char address_field, unsigned char control_field );

int receiveSUframe ( unsigned char address_field, unsigned char control_field );

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose()
{

    int s = 0;
    int tries = 0;
    int reply_valid = FALSE;

    // TRANSMITTER
    if (currentRole == LlTx)
    {
        printf("Transmitter is attempting to close connection.\n");

        // Repeat connection termination sequence until success, error, or the number of attempts exceeds nRetransmissions
        while ( tries < nRetransmissions && !reply_valid ) {

            // Send DISC frame to receiver
            s = sendSUframe( A_TX, C_DISC );

            // Check if error was encountered while sending frame to serial port
            if ( s < 0 ) {
                printf("Unexpected error sending DISC frame to serial port.\n");
                return -1;
            }

            printf("Wrote DISC frame to Receiver (try #%d /%d)\n", tries + 1, nRetransmissions);
            
            printf("Attempting to read DISC frame from Receiver (try #%d /%d)\n", tries + 1, nRetransmissions);

            // Receive DISC frame from receiver
            s = receiveSUframe( A_RX, C_DISC );

            // Check if error was encountered while receiving frame from serial port
            if ( s < 0 ) {
                printf("Unexpected error receiving DISC frame from serial port.\n");
                return -1;
            }

            if (s == 1) { 
                printf("[llclose] Timeout: no response (DISC from receiver not received).\n");
                break; 
            }

            // Check if valid frame was received before timeout
            if ( s == 0 ) {

                reply_valid = TRUE;

                printf("Read DISC frame from Receiver (try #%d /%d)\n", tries + 1, nRetransmissions);

                // Send UA frame response to receiver
                s = sendSUframe( A_RX, C_UA );

                // Check if error was encountered while sending frame to serial port
                if ( s < 0 ) {
                    printf("Unexpected error sending DISC frame to serial port.\n");
                    return -1;
                }

                printf("Wrote UA frame to Receiver\n");
            }

            // Increase counter for number of attempts
            tries++;
        }
    }

    // RECEIVER
    else if (currentRole == LlRx)
    {
        printf("Receiver is attempting to close connection.\n");

        // Repeat connection termination sequence until success, error, or the number of attempts exceeds nRetransmissions
        while ( tries < nRetransmissions && !reply_valid ) {

            printf("Attempting to read DISC frame from Transmitter (try #%d /%d).\n", tries + 1, nRetransmissions);

            // Receive DISC frame
            s = receiveSUframe( A_TX, C_DISC );

            // Check if error was encountered while receiving frame from serial port
            if ( s == -1 )
            {
                printf("Unexpected error receiving DISC frame from serial port.\n");
                return -1;
            }

            // Check if no valid frame was received in time
            else if ( s == 1 )
            {
                printf("Failed to read DISC frame from Transmitter in time (try #%d /%d)\n", tries + 1, nRetransmissions);

                tries++;
                continue;
            }

            // Check if valid frame was received before timeout
            else if ( s == 0 )
            {
                reply_valid = TRUE;

                printf("Read DISC frame from Transmitter (try #%d /%d)\n", tries + 1, nRetransmissions);

                // Send DISC frame
                s = sendSUframe( A_RX, C_DISC );

                // Check if error was encountered while sending frame to serial port
                if ( s != 0 ) {
                    printf("Unexpected error sending DISC frame to serial port.\n");
                    return -1;
                }

                printf("Sent DISC frame to Transmitter\n");

                // Receive UA frame
                s = receiveSUframe( A_RX, C_UA );

                if ( s < 0 ) {
                    printf("Unexpected error receiving UA frame from serial port.\n");
                    return -1;
                }
            }
            
            // Unexpected value returned from receiveSUframe()
            else
            {
                printf("Error: unexpected value '%d' returned from receiveSUframe()\n", s);
                return -1;
            }

            tries++;
        }
    }

    // Unexpected value for currentRole
    else
    {
        printf("Unexpected value for currentRole '%x'. Must be either '%x' or '%x'.\n", currentRole, LlTx, LlRx);
        return -1;
    }

    alarm(0);
    alarmEnabled = FALSE;

    if (currentRole == LlRx) {
        printf("\n===== Link Statistics =====\n");
        printf("RR frames sent:  %d\n", rr_count);
        printf("REJ frames sent: %d\n", rej_count);
        printf("===============================\n\n");
    }

    if ( closeSerialPort() != 0 ) {
        printf("Error encountered closing serial port.\n");
        return -1;
    }

    printf("Closed serial port.\n");
    return 0;
}

// Function for sending Supervision and Unnumbered Frames to the Serial Port
// Arguments:
//  address_field: value of the address field of the frame to be sent.
//  control_field: value of the control field of the frame to be sent.
// Return -1 on error, 0 on success.
int sendSUframe ( unsigned char address_field, unsigned char control_field ) {

    // build frame
    unsigned char frame[5] = { FLAG, address_field, control_field, BCC1(address_field, control_field), FLAG };

    // send frame
    int bytes_sent = writeBytesSerialPort( frame, 5 );

    // Check if serial port encountered an error
    if ( bytes_sent < 0 ) { perror("Serial port function writeBytesSerialPort() returned error."); return -1; }

    // verify correct amount of bytes were sent
    if ( bytes_sent != 5 ) {
        printf("Frame was not written to serial port correctly.\n");
        return -1;
    }

    printf("Wrote frame '0x%02x%02x%02x%02x%02x' to serial port.\n", frame[0], frame[1], frame[2], frame[3], frame[4]);

    return 0;
}

// Function for receiving and validating Supervision and Unnumbered Frames from the Serial Port
// Arguments:
//  address_field: expected value for the address field of the frame to be received.
//  control_field: expected value for the control field of the frame to be received.
// Return -1 on error, 0 on success, 1 if no valid frame was received before timeout.
int receiveSUframe ( unsigned char address_field, unsigned char control_field ) {

    // state
    enum { START, FLAG_RCV, A_RCV, C_RCV, BCC1_OK, STOP } state = START;

    // buffer for byte received from serial port's readByteSerialPort()
    unsigned char byte_read = 0; 

    alarmEnabled = TRUE;
    alarm(timeout);  

    // loop to receive frame byte byt byte and validate it
    while ( state != STOP && alarmEnabled ) {

        // Read byte from serial port
        int r = readByteSerialPort(&byte_read);

        // Check if serial port encountered an error
        if (r == -1) {
            if (errno == EINTR) continue;
            perror("Serial port function readByteSerialPort() returned error.");
            alarm(0);
            return -1;
        }

        // Check if serial port returned a byte
        // If value is 0 (no byte) skip current loop iteration and try again.
        if (r == 0) continue;

        //printf("%02x", byte_read); debug

        // state machine to validate received byte of expected frame
        switch (state)
        {
        // Starting point for the frame
        // Expecting a FLAG (0x7E) byte
        case START:
            if ( byte_read == FLAG ) state = FLAG_RCV;
            break;

        // Received FLAG (0x7E) byte
        // Expecting an Adress Field byte equal to function parameter 'address_field'
        // If a FLAG (0x7E) byte is received, discard the previous and accept the current
        case FLAG_RCV:
            if      ( byte_read == address_field ) state = A_RCV;
            else if ( byte_read == FLAG )          state = FLAG_RCV;
            else                                    state = START;
            break;

        // Received correct Address Field byte
        // Expecting a Control Field byte equal to function parameter 'control_field'
        case A_RCV:
            if      ( byte_read == FLAG )          state = FLAG_RCV;
            else if ( byte_read == control_field ) state = C_RCV;
            else                                    state = START;
            break;

        // Received correct Control Field byte
        // Expecting a BCC1 byte equal to the XOR operation between the 'address_field' and the 'control_field' parameters
        case C_RCV:
            if ( byte_read == ( unsigned char )( address_field ^ control_field ) ) state = BCC1_OK;
            else                                                                   state = START;
            break;

        // Received correct BCC1 byte
        // Expecting a FLAG (0x7E) byte
        case BCC1_OK:
            if ( byte_read == FLAG ) state = STOP;
            else                     state = START;
            break;
        
        // If current value of 'state' is not any of the above, an error has ocurred
        default:
            alarm(0);
            printf("Unexpected state '%d'.\n", state);
            return -1;
        }
    }

    // check if alarm was triggered before any frame was accepted
    if ( !alarmEnabled ) {
        printf("Timeout while reading from serial port.\n");
        return 1;
    }

    // check if state of frame acceptance was reached and return success if true
    if ( state == STOP )
    {
        alarm(0);
        printf("\nFrame was read with success.\n");
        return 0;
    }

    // if frame was not accepted before the alarm was triggered return a timeout failure
    else
    {
        alarm(0);
        printf("\nNo frame was read with success.\n");
        return 1;
    }
}
