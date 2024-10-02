// Read from serial port in non-canonical mode
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

// Baudrate settings are defined in <asm/termbits.h>, which is
// included by <termios.h>
#define BAUDRATE B38400
#define _POSIX_SOURCE 1 // POSIX compliant source

#define FALSE 0
#define TRUE 1

////////////////////////////////
////////////////////////////////
#define BUF_SIZE 5 // Frame size
#define FLAG 0x7E // 01111110 (0x7E)
// A (Address)
#define A_SENDER 0x03 // 00000011 (0x03) – frames sent by the Sender or answers from the Receiver
#define A_RECEIVER 0x01 // 00000001 (0x01) – frames sent by the Receiver or answers from the Sender
// C (Control)
#define C_SET 0x03 // SET: 00000011 (0x03)
#define C_UA 0x07 // UA: 00000111 (0x07)
////////////////////////////////
////////////////////////////////

volatile int STOP = FALSE;

int main(int argc, char *argv[]) {
    // Program usage: Uses either COM1 or COM2
    const char *serialPortName = argv[1];

    if (argc < 2) {
        printf("Incorrect program usage\n"
               "Usage: %s <SerialPort>\n"
               "Example: %s /dev/ttyS1\n",
               argv[0],
               argv[0]);
        exit(1);
    }

    // Open serial port device for reading and writing and not as controlling tty
    // because we don't want to get killed if linenoise sends CTRL-C.
    int fd = open(serialPortName, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        perror(serialPortName);
        exit(-1);
    }

    struct termios oldtio;
    struct termios newtio;

    // Save current port settings
    if (tcgetattr(fd, &oldtio) == -1) {
        perror("tcgetattr");
        exit(-1);
    }

    // Clear struct for new port settings
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    // Set input mode (non-canonical, no echo,...)
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0; // Inter-character timer unused
    newtio.c_cc[VMIN] = 5;  // Blocking read until 5 chars received

    // VTIME e VMIN should be changed in order to protect with a
    // timeout the reception of the following character(s)

    // Now clean the line and activate the settings for the port
    // tcflush() discards data written to the object referred to
    // by fd but not transmitted, or data received but not read,
    // depending on the value of queue_selector:
    //   TCIFLUSH - flushes data received but not read.
    tcflush(fd, TCIOFLUSH);

    // Set new port settings
    if (tcsetattr(fd, TCSANOW, &newtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }

    printf("New termios structure set\n");

    ////////////////////////////////
    ////////////////////////////////
    while (STOP == FALSE) {
        // Read the transmitter SET frame
        unsigned char received_set_frame[BUF_SIZE];
        int bytes = read(fd, received_set_frame, BUF_SIZE);

        // Check if the frame is a valid SET
        if (bytes == BUF_SIZE && received_set_frame[0] == FLAG && 
            received_set_frame[1] == A_SENDER && received_set_frame[2] == C_SET &&
            received_set_frame[3] == (received_set_frame[1] ^ received_set_frame[2]) &&
            received_set_frame[4] == FLAG)
        {
            printf("SET frame received successfully!\n");
            printf("Received SET frame:\n");
            for (int i = 0; i < BUF_SIZE; i++)
            {
                printf("0x%02X ", received_set_frame[i]);
            }
            printf("\n");

            // Set up and send the UA frame
            unsigned char ua_frame[BUF_SIZE];
            ua_frame[0] = FLAG;               // FLAG
            ua_frame[1] = A_RECEIVER;          // A (Address)
            ua_frame[2] = C_UA;                // C (UA)
            ua_frame[3] = ua_frame[1] ^ ua_frame[2]; // BCC1 (A ^ C)
            ua_frame[4] = FLAG;               // FLAG

            // Send the UA frame back to the transmitter
            write(fd, ua_frame, BUF_SIZE);
            printf("UA frame sent.\n");

            // Stop the loop after sending UA
            STOP = TRUE;
        }
        else
        {
            printf("Invalid frame received or not enough bytes (expected 5, received %d).\n", bytes);
        }
    }
    ////////////////////////////////
    ////////////////////////////////

    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);
    return 0;
}
