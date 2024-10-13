// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
#include <signal.h>
#include <unistd.h>
#include <stdio.h> // for printf
#include <stdlib.h> // for malloc, free


// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

#define BUF_SIZE 50 // Frame size
#define FLAG 0x7E // 01111110 (0x7E)
// A (Address)
#define A_SENDER 0x03   // 00000011 (0x03) – frames sent by the Sender or answers from the Receiver
#define A_RECEIVER 0x01 // 00000001 (0x01) – frames sent by the Receiver or answers from the Sender
// C (Control)
#define C_SET 0x03 // SET: 00000011 (0x03)
#define C_UA 0x07  // UA: 00000111 (0x07)

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


int alarmEnabled = FALSE;
volatile int alarmCount = 0;


void alarmHandler(int signal) {
    alarmEnabled = FALSE;
    printf("Alarm triggered, retransmission count: %d\n", alarmCount+1);
    alarmCount++;
}

// int check_frame(int fd, LinkLayerRole role) {
//     State currentState = START;
//     unsigned char byte;
//     unsigned char address = 0;
//     unsigned char control = 0;
//     unsigned char bcc = 0;

//     while (currentState != STOP) {
//         // Read one byte at a time from the serial port
//         int bytes = readByteSerialPort(&byte);
//         if (bytes > 0) { // Proceed only if a byte was read
//             switch (currentState) {
//                 case START:
//                     if (byte == FLAG) {
//                         currentState = FLAG_RCV;
//                     }
//                     break;

//                 case FLAG_RCV:
//                     // Expecting the address byte
//                     if (role == LlTx && byte == A_RECEIVER) { // Expect receiver's response
//                         address = byte;
//                         currentState = A_RCV;
//                     } else if (role == LlRx && byte == A_SENDER) { // Expect sender's frame
//                         address = byte;
//                         currentState = A_RCV;
//                     } else if (byte != FLAG) { // If not FLAG, reset to START
//                         currentState = START;
//                     }
//                     break;

//                 case A_RCV:
//                     // Expecting the control byte based on role
//                     if (role == LlTx && (byte == C_UA)) { // Later we should implement RR/REJ
//                         control = byte;  // Receiver replies with UA
//                         currentState = C_RCV;
//                     } else if (role == LlRx && byte == C_SET) { 
//                         control = byte;  // Sender sends SET frame
//                         currentState = C_RCV;
//                     } else if (byte == FLAG) { // If FLAG, go back to FLAG_RCV
//                         currentState = FLAG_RCV;
//                     } else { // Any other byte resets to START
//                         currentState = START;
//                     }
//                     break;

//                 case C_RCV:
//                     // Expecting BCC (A ^ C)
//                     bcc = address ^ control;
//                     if (byte == bcc) {
//                         currentState = BCC_OK;
//                     } else if (byte == FLAG) {
//                         currentState = FLAG_RCV; // If FLAG, go back to FLAG_RCV
//                     } else { // If BCC doesn't match, reset to START
//                         currentState = START;
//                     }
//                     break;

//                 case BCC_OK:
//                     // Expecting end flag
//                     if (byte == FLAG) {
//                         currentState = STOP; // Full frame received successfully
//                     } else { // If not FLAG, reset to START
//                         currentState = START;
//                     }
//                     break;

//                 default:
//                     currentState = START;
//                     break;
//             }
//         }
//     }

//     return 0; // Return 0 when a valid frame is received
// }

int check_frame(int fd, LinkLayerRole role) {

    State currentState = START;
    unsigned char byte;
    unsigned char address = 0;
    unsigned char control = 0;
    unsigned char bcc = 0;

    while (stop_flag == FALSE) {
        // Read one byte at a time from the serial port
        int bytes = readByteSerialPort(&byte);
        if (bytes > 0) { // Proceed only if a byte was read
            // printf("byte: 0x%02X current state: %d \n", byte, currentState);
            if (role == LlTx) {
                printf("Role is LlTx\n");  // This will be printed because role == LlTx
            } else if (role == LlRx) {
                printf("Role is LlRx\n");
            }
            printf("Received byte: 0x%02X, current state: %d\n", byte, currentState);
            switch (currentState) {
                
                case START:
                    if (byte == FLAG) {
                        currentState = FLAG_RCV;
                    }
                    break;

                case FLAG_RCV:
                    // Expecting the address byte
                    if ((byte == A_RECEIVER && role == LlTx) || (byte == A_SENDER && role == LlRx)) {
                        address = byte;
                        currentState = A_RCV;
                    } else if (byte != FLAG) { // If not FLAG, reset to START (Other_RCV)
                        currentState = START;
                    }
                    // printf("byte: 0x%02X next state: %d \n", byte, currentState);
                    break;

                case A_RCV:
                    // Expecting the control byte
                    if (role == LlTx && (byte == C_UA)) { // Later we should implement RR/REJ
                        control = byte;  // Receiver replies with UA
                        currentState = C_RCV;
                    } else if (role == LlRx && byte == C_SET) { 
                        control = byte;  // Sender sends SET frame
                        currentState = C_RCV;
                    } else if (byte == FLAG) {
                        currentState = FLAG_RCV; // If FLAG, go back to FLAG_RCV
                    } else { // Any other byte resets to START (Other_RCV)
                        currentState = START;
                    }
                    // printf("byte: 0x%02X next state: %d \n", byte, currentState);
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
                    // printf("byte: 0x%02X next state: %d \n", byte, currentState);
                    break;

                case BCC_OK:
                    // Expecting end flag
                    if (byte == FLAG) {
                        currentState = STOP; // Full frame received successfully
                        stop_flag = TRUE; // End the loop
                    } else { // If not FLAG, reset to START
                        currentState = START;
                    }
                    // printf("byte: 0x%02X next state: %d \n", byte, currentState);
                    break;

                default:
                    currentState = START;
                    break;
            }
        }
    }
    return 0;
}



int send_frame(int fd, LinkLayerRole role, const unsigned char *frame, int frame_size) {
    // Install the alarm handler for retransmissions
    if (signal(SIGALRM, alarmHandler) == SIG_ERR) {
        perror("Unable to catch SIGALRM");
        return -1;
    }

    // Initialize the retry counter and ensure alarmCount starts at 0
    alarmCount = 0;

    // Loop for retransmissions, allowing up to 3 attempts
    while (alarmCount < 3) {
        // Send the frame via the serial port
        printf("Attempting to send frame, attempt #%d...\n", alarmCount + 1);
        if (writeBytesSerialPort(frame, frame_size) == -1) {
            perror("Error writing frame to serial port");
            return -1;
        }
        printf("Frame sent.\n");

        // Enable the alarm for timeout in 3 seconds
        alarm(3);
        alarmEnabled = TRUE;

        // Wait for the acknowledgment (reply frame)
        if (check_frame(fd, role) == 0) {
            // Frame was successfully acknowledged
            printf("Reply frame received successfully!\n");
            alarm(0);  // Disable the alarm since we got a valid reply
            return 0;  // Transmission was successful, exit the function
        }

        // If the frame was not acknowledged, wait for the alarm to trigger before retrying
        pause(); // Pauses execution until the alarm triggers or a signal is caught

        // Increment the retry count when alarm triggers (handled by alarmHandler)
        if (alarmEnabled == FALSE) {
            alarmCount++;
            printf("Retrying transmission...\n");
        }
    }

    // If we've retried 3 times without success, return an error
    printf("Failed to receive reply after 3 attempts. Exiting...\n");
    return -1;
}


////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters) {
    int fd = openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate);
    if (fd < 0) {
        printf("Error opening serial port\n");
        return -1;
    }

    // Setup connection
    if(connectionParameters.role == LlTx) {
        char BBC1 = A_SENDER ^ C_SET;
        const unsigned char set_frame[5] = {
            FLAG,
            A_SENDER,
            C_SET,
            BBC1,
            FLAG,
        };
        if(send_frame(fd, LlTx, set_frame, 5) != 0) {
            printf("Error sending frame or receiving reply\n");
            return -1;
        }
    } else if (connectionParameters.role == LlRx) {

        if(check_frame(fd, LlRx) != 0) {
            printf("Error reading frame\n");
            return -1;
        }

        char BBC1 = A_RECEIVER ^ C_UA;
        const unsigned char ua_frame[5] = {
            FLAG,
            A_RECEIVER,
            C_UA,
            BBC1,
            FLAG,
        };
        if(send_frame(fd, LlRx, ua_frame, 5) != 0) {
            printf("Error sending frame or receiving reply\n");
            return -1;
        }
    } else {
        printf("Invalid role\n");
        return -1;
    }

    return 0;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize) {

    return 1;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet) {

    return 1;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics) {
    // TODO

    int clstat = closeSerialPort();
    return clstat;
}
