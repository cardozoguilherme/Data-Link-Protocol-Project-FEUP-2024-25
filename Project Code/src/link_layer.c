// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
#include <signal.h>
#include <unistd.h>
#include <stdio.h> // for printf
#include <stdlib.h> // for malloc, free

#include <string.h> // for memcpy


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

int check_frame(LinkLayerRole role) {

    State currentState = START;
    unsigned char byte;
    unsigned char address = 0;
    unsigned char control = 0;
    unsigned char bcc = 0;

    while (stop_flag == FALSE) {
        // Read one byte at a time from the serial port
        int bytes = readByteSerialPort(&byte);
        if (bytes > 0) { // Proceed only if a byte was read
            
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
                    break;

                case A_RCV:
                    // Expecting the control byte
                    if (role == LlTx && (byte == C_UA)) { // Later we should implement RR/REJ/Stop and Wait
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
                    break;

                case BCC_OK:
                    // Expecting end flag
                    if (byte == FLAG) {
                        currentState = STOP; // Full frame received successfully
                        stop_flag = TRUE; // End the loop
                    } else { // If not FLAG, reset to START
                        currentState = START;
                    }
                    break;

                default:
                    currentState = START;
                    break;
            }
        }
    }
    return 0;
}



int send_frame(LinkLayerRole role, const unsigned char *frame, int frame_size) {
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
        if (check_frame(role) == 0) {
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
        if(send_frame(LlTx, set_frame, 5) != 0) {
            printf("Error sending frame or receiving reply\n");
            return -1;
        }
    } else if (connectionParameters.role == LlRx) {

        if(check_frame(LlRx) != 0) {
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
        if(send_frame(LlRx, ua_frame, 5) != 0) {
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
unsigned char calculateBCC2(const unsigned char *data, int length) {
    unsigned char bcc2 = 0;
    for (int i = 0; i < length; i++) {
        bcc2 ^= data[i];  // XOR all the bytes
    }
    return bcc2;
}

unsigned char *byteStuffing(const unsigned char *frame, int frameSize, int *stuffedFrameSize) {
    // Allocate memory for the worst-case scenario (each byte being stuffed)
    unsigned char *stuffedFrame = malloc(frameSize * 2 * sizeof(unsigned char));
    if (!stuffedFrame) {
        printf("Failed to allocate memory for byte stuffing.\n");
        return NULL;
    }

    int j = 0;  // Index for the stuffed frame
    for (int i = 0; i < frameSize; i++) {
        if (frame[i] == 0x7E) {
            // If the byte is a flag, perform byte stuffing
            stuffedFrame[j++] = 0x7D;
            stuffedFrame[j++] = 0x5E;
        } else if (frame[i] == 0x7D) {
            // If the byte is an escape, perform byte stuffing
            stuffedFrame[j++] = 0x7D;
            stuffedFrame[j++] = 0x5D;
        } else {
            // Otherwise, copy the byte as is
            stuffedFrame[j++] = frame[i];
        }
    }

    *stuffedFrameSize = j;  // Update the stuffed frame size
    return stuffedFrame;
}

int llwrite(const unsigned char *buf, int bufSize) {
    if (buf == NULL || bufSize <= 0 || bufSize > MAX_PAYLOAD_SIZE) {
        printf("Invalid buffer or buffer size.\n");
        return -1;
    }

    unsigned char address = A_SENDER;  // Address of the sender
    static unsigned char control = 0x00;  // Control field for alternating sequence numbers (Stop-and-Wait)
    unsigned char bcc1 = address ^ control;  // Calculate BCC1 by XORing address and control fields

    int frameSize = 5 + bufSize;  // Total frame size (header + data + BCC2 + flags)
    unsigned char *frame = malloc((frameSize + 2) * sizeof(unsigned char));  // Allocate memory for the frame

    if (!frame) {
        printf("Failed to allocate memory for frame.\n");
        return -1;
    }

    // Construct the frame
    frame[0] = FLAG;               // Start flag
    frame[1] = address;            // Address field
    frame[2] = control;            // Control field
    frame[3] = bcc1;               // BCC1
    memcpy(&frame[4], buf, bufSize);  // Copy the data into the frame
    frame[4 + bufSize] = calculateBCC2(buf, bufSize);  // Calculate BCC2 for data and append it
    frame[5 + bufSize] = FLAG;      // End flag

    // Apply byte stuffing to ensure transparency (escape special characters)
    int stuffedFrameSize;
    unsigned char *stuffedFrame = byteStuffing(frame, frameSize + 2, &stuffedFrameSize);

    free(frame);  // Free the original frame as the stuffed frame will be sent
    send_frame(stuffedFrame, stuffedFrameSize);
    int attempts = 0;
    // while (attempts < 3) {  // Try sending the frame up to 3 times
    //     // Send the stuffed frame
    //     if (send_frame(LlTx, stuffedFrame, stuffedFrameSize) != 0) {
    //         printf("Error sending frame.\n");
    //         free(stuffedFrame);
    //         return -1;
    //     }
    //     // Wait for acknowledgment (ACK or NACK)
    //     int result = check_frame(LlTx);
    //     if (result == 0) {  // If ACK is received
    //         printf("Frame acknowledged.\n");
    //         free(stuffedFrame);
    //         control ^= 0x80;  // Alternate the control field for the next frame (Stop-and-Wait)

    //         // STOP AND WAIT...

    //         return bufSize;   // Success, return the number of bytes written
    //     } else if (result == -1) {  // If NACK (REJ) is received, retransmit
    //         printf("Frame rejected, retransmitting...\n");
    //     } else {  // If timeout occurs, retransmit
    //         printf("Timeout, retransmitting...\n");
    //     }
    //     attempts++;  // Increment the number of attempts
    // }

    // If all attempts failed, return error
    printf("Failed to send frame after 3 attempts.\n");
    free(stuffedFrame);
    return -1;
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
