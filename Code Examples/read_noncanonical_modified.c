// Read from serial port in non-canonical mode
//
// Modified by: Eduardo Nuno Almeida [enalmeida@fe.up.pt]

// TO RUN LAB 1 IN WSL:
// STEP 1: sudo apt install socat

// STEP 2: socat -d -d pty,raw,echo=0 pty,raw,echo=0

// STEP 3: Run these commands in the same terminal
//         stty -F /dev/pts/1 9600 cs8 -cstopb -parenb
//         stty -F /dev/pts/2 9600 cs8 -cstopb -parenb

// STEP 4: Open two new terminals and run each line in each terminal
//         ./read_noncanonical_modified /dev/pts/2
//         ./write_noncanonical_modified /dev/pts/1


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

// Define state machine states
typedef enum {
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    STOP
} State;

volatile int stop_flag = FALSE;


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
    newtio.c_cc[VTIME] = 1; // Inter-character timer unused
    newtio.c_cc[VMIN] = 1;  // Blocking read until 1 chars received

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

    State currentState = START;
    unsigned char byte;
    unsigned char address = 0;
    unsigned char control = 0;
    unsigned char bcc = 0;

    while (stop_flag == FALSE) {
        // Read one byte at a time from the serial port
        int bytes = read(fd, &byte, 1);

        if (bytes > 0) { // Proceed only if a byte was read
            switch (currentState) {
                case START:
                    if (byte == FLAG) {
                        currentState = FLAG_RCV;
                    }
                    printf("byte: 0x%02X next state: %d \n", byte, currentState);
                    break;

                case FLAG_RCV:
                    // Expecting the address byte
                    if (byte == A_SENDER) {
                        address = byte;
                        currentState = A_RCV;
                    } else if (byte != FLAG) { // If not FLAG, reset to START (Other_RCV)
                        currentState = START;
                    }
                    printf("byte: 0x%02X next state: %d \n", byte, currentState);
                    break;

                case A_RCV:
                    // Expecting the control byte
                    if (byte == C_SET) {
                        control = byte;
                        currentState = C_RCV;
                    } else if (byte == FLAG) {
                        currentState = FLAG_RCV; // If FLAG, go back to FLAG_RCV
                    } else { // Any other byte resets to START (Other_RCV)
                        currentState = START;
                    }
                    printf("byte: 0x%02X next state: %d \n", byte, currentState);
                    break;

                case C_RCV:
                    // Expecting BCC (A ^ C)
                    bcc = address ^ control;
                    if (byte == bcc) {
                        currentState = BCC_OK;
                    } else if (byte == FLAG) {
                        currentState = FLAG_RCV; // If FLAG, go back to FLAG_RCV
                    } else { // If BCC doesn't match or any other byte, reset to START
                        currentState = START;
                    }
                    printf("byte: 0x%02X next state: %d \n", byte, currentState);
                    break;

                case BCC_OK:
                    // Expecting end flag
                    if (byte == FLAG) {
                        currentState = STOP; // Full frame received successfully
                        stop_flag = TRUE; // End the loop
                    } else { // If not FLAG, reset to START
                        currentState = START;
                    }
                    printf("byte: 0x%02X next state: %d \n", byte, currentState);
                    break;

                default:
                    currentState = START;
                    break;
            }
        }
    }

    // If STOP is reached, the frame was successfully received
    printf("SET frame received successfully!\n");

    // Set up and send the UA frame
    unsigned char ua_frame[BUF_SIZE] = { 
        FLAG, 
        A_RECEIVER, 
        C_UA,
        (A_RECEIVER ^ C_UA),
        FLAG
    };
    write(fd, ua_frame, BUF_SIZE);
    printf("UA frame sent.\n");

    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);
    return 0;
}
