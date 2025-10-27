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


////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    // TODO: Implement this function

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
