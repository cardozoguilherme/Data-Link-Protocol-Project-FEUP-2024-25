// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
#include <signal.h>
#include <unistd.h>
#include <stdio.h> // for printf
#include <stdlib.h> // for malloc, free

#include <string.h> // for memcpy
#include <stdbool.h> // for bool, true, false


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

// Communications statistics (llclose)
int totalFramesSent = 0;       // Total frames sent
int totalRetransmissions = 0;  // Total retransmissions
int totalTimeouts = 0;         // Total timeouts

// Define state machine states for setup frames
typedef enum {
    SETUP_START,
    SETUP_FLAG_RCV,
    SETUP_A_RCV,
    SETUP_C_RCV,
    SETUP_BCC_OK,
    SETUP_STOP
} Setup_frame_state;

// Define state machine states for information frames
typedef enum {
    INFO_START,
    INFO_FLAG_RCV,
    INFO_A_RCV,
    INFO_C_RCV,
    INFO_BCC1_OK,
    INFO_DATA_RCV,
    INFO_BCC2_OK,
    INFO_STOP
} Information_frame_state;

volatile int stop_flag = FALSE;


int alarmEnabled = FALSE;
volatile int alarmCount = 0;

void alarmHandler(int signal) {
    alarmEnabled = FALSE;
    printf("Alarm triggered, retransmission count: %d\n", alarmCount+1);
    alarmCount++;
}

int check_setup_frame(LinkLayerRole role) {

    Setup_frame_state currentState = SETUP_START;
    unsigned char byte;
    unsigned char address = 0;
    unsigned char control = 0;
    unsigned char bcc = 0;

    while (stop_flag == FALSE) {
        int bytes = readByteSerialPort(&byte);
        if (bytes > 0) { 
            
            printf("Received byte: 0x%02X, current state: %d\n", byte, currentState);
            switch (currentState) {
                
                case SETUP_START:
                    if (byte == FLAG) {
                        currentState = SETUP_FLAG_RCV;
                    }
                    break;

                case SETUP_FLAG_RCV:
                    // Expecting the address byte
                    if ((byte == A_RECEIVER && role == LlTx) || (byte == A_SENDER && role == LlRx)) {
                        address = byte;
                        currentState = SETUP_A_RCV;
                    } else if (byte != FLAG) { 
                        currentState = SETUP_START;
                    }
                    break;

                case SETUP_A_RCV:
                    // Expecting the control byte
                    if (role == LlTx && (byte == C_UA)) { 
                        control = byte;
                        currentState = SETUP_C_RCV;
                    } else if (role == LlRx && byte == C_SET) { 
                        control = byte;
                        currentState = SETUP_C_RCV;
                    } else if (byte == FLAG) {
                        currentState = SETUP_FLAG_RCV; 
                    } else { 
                        currentState = SETUP_START;
                    }
                    break;

                case SETUP_C_RCV:
                    // Expecting BCC (A ^ C)
                    bcc = address ^ control;
                    if (byte == bcc) {
                        currentState = SETUP_BCC_OK;
                    } else if (byte == FLAG) {
                        currentState = SETUP_FLAG_RCV;
                    } else { 
                        currentState = SETUP_START;
                    }
                    break;

                case SETUP_BCC_OK:
                    // Expecting end flag
                    if (byte == FLAG) {
                        currentState = SETUP_STOP; 
                        stop_flag = TRUE;
                    } else { 
                        currentState = SETUP_START;
                    }
                    break;

                default:
                    currentState = SETUP_START;
                    break;
            }
        }
    }
    return 0;
}

int check_information_frame(LinkLayerRole role, unsigned char *data_buffer, int *data_length) {
    Information_frame_state currentState = INFO_START;
    unsigned char byte;
    unsigned char address = 0;
    unsigned char control = 0;
    unsigned char bcc1 = 0;
    unsigned char bcc2 = 0;
    int data_index = 0;
    bool is_escaped = false;

    while (stop_flag == FALSE) {
        int bytes = readByteSerialPort(&byte);
        if (bytes > 0) {
            printf("Received byte: 0x%02X, current state: %d\n", byte, currentState);
            
            // Handle byte stuffing
            if (byte == 0x7D && !is_escaped) {
                is_escaped = true;
                continue;
            }
            
            if (is_escaped) {
                byte = byte ^ 0x20;
                is_escaped = false;
            }

            switch (currentState) {
                case INFO_START:
                    if (byte == FLAG) {
                        currentState = INFO_FLAG_RCV;
                        data_index = 0;
                        bcc2 = 0;
                    }
                    break;

                case INFO_FLAG_RCV:
                    if ((byte == A_RECEIVER && role == LlTx) || 
                        (byte == A_SENDER && role == LlRx)) {
                        address = byte;
                        currentState = INFO_A_RCV;
                    }
                    else if (byte != FLAG) {
                        currentState = INFO_START;
                    }
                    break;

                case INFO_A_RCV:
                    // Check for I-frame control field (0 or 1 in N(s))
                    if ((byte == 0x00) || (byte == 0x80)) {  // N(s) = 0 or 1
                        control = byte;
                        currentState = INFO_C_RCV;
                    }
                    else if (byte == FLAG) {
                        currentState = INFO_FLAG_RCV;
                    }
                    else {
                        currentState = INFO_START;
                    }
                    break;

                case INFO_C_RCV:
                    bcc1 = address ^ control;
                    if (byte == bcc1) {
                        currentState = INFO_BCC1_OK;
                    }
                    else if (byte == FLAG) {
                        currentState = INFO_FLAG_RCV;
                    }
                    else {
                        currentState = INFO_START;
                    }
                    break;

                case INFO_BCC1_OK:
                    if (byte == FLAG) {
                        // Empty I-frame
                        if (data_index == 0) {
                            currentState = INFO_STOP;
                            stop_flag = TRUE;
                        }
                        else {
                            // Check if last byte was BCC2
                            if (bcc2 == data_buffer[data_index - 1]) {
                                *data_length = data_index - 1;
                                currentState = INFO_STOP;
                                stop_flag = TRUE;
                            }
                            else {
                                currentState = INFO_START;
                            }
                        }
                    }
                    else {
                        // Start receiving data
                        data_buffer[data_index] = byte;
                        bcc2 ^= byte;
                        data_index++;
                        currentState = INFO_DATA_RCV;
                    }
                    break;

                case INFO_DATA_RCV:
                    if (byte == FLAG) {
                        // Check if last byte was BCC2
                        if (bcc2 == data_buffer[data_index - 1]) {
                            *data_length = data_index - 1;
                            currentState = INFO_STOP;
                            stop_flag = TRUE;
                        }
                        else {
                            currentState = INFO_START;
                        }
                    }
                    else {
                        data_buffer[data_index] = byte;
                        bcc2 ^= byte;
                        data_index++;
                    }
                    break;

                default:
                    currentState = INFO_START;
                    break;
            }
        }
    }

    // Return status based on frame validation
    if (currentState == INFO_STOP) {
        return 0;  // Success
    }
    return -1;  // Error
}


int send_frame(LinkLayerRole role, const unsigned char *frame, int frame_size) {

    if (writeBytesSerialPort(frame, frame_size) == -1) {
        perror("Error writing frame to serial port");
        return -1;
    }

    return 0;
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
            printf("Error sending frame");
            return -1;
        }
    } else if (connectionParameters.role == LlRx) {

        if(check_setup_frame(LlRx) != 0) {
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
            printf("Error sending frame");
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

    static unsigned char control = 0x00;
    
    unsigned char address = A_SENDER;  
    unsigned char bcc1 = address ^ control;

    int frameSize = 5 + bufSize; 
    unsigned char *frame = malloc((frameSize + 2) * sizeof(unsigned char)); 

    if (!frame) {
        printf("Failed to allocate memory for frame.\n");
        return -1;
    }

    frame[0] = FLAG;                  
    frame[1] = address;               
    frame[2] = control;               
    frame[3] = bcc1;                  
    memcpy(&frame[4], buf, bufSize);  
    frame[4 + bufSize] = calculateBCC2(buf, bufSize);
    frame[5 + bufSize] = FLAG;

    int stuffedFrameSize;
    unsigned char *stuffedFrame = byteStuffing(frame, frameSize + 2, &stuffedFrameSize);

    free(frame);

    if (!stuffedFrame) {
        printf("Failed to allocate memory for stuffed frame.\n");
        return -1;
    }

    if (signal(SIGALRM, alarmHandler) == SIG_ERR) {
        perror("Unable to catch SIGALRM");
        free(stuffedFrame);
        return -1;
    }

    int attempts = 0;
    while (attempts < 3) {
        // Send the stuffed frame
        printf("Attempt #%d: Sending frame...\n", attempts + 1);
        if (send_frame(LlTx, stuffedFrame, stuffedFrameSize) != 0) {
            printf("Error sending frame.\n");
            free(stuffedFrame);
            return -1;
        }

        // Increment total frames sent
        totalFramesSent++;

        // Set the alarm for timeout (3 seconds)
        alarm(3);
        alarmEnabled = TRUE;

        // Wait for acknowledgment
        int result = check_information_frame(LlTx, stuffedFrame, &stuffedFrameSize);
        if (result == 0) {  // ACK received
            printf("Frame acknowledged.\n");
            alarm(0);
            free(stuffedFrame);
            control ^= 0x80;
            return bufSize;
        } else if (result == -1) {  // NACK received, retransmit
            printf("Frame rejected, retransmitting...\n");
            totalRetransmissions++;  // Increment retransmission count
        } else {  // Timeout occurred, retransmit
            printf("Timeout, retransmitting...\n");
            totalTimeouts++;  // Increment timeout count
        }

        pause();

        if (!alarmEnabled) {
            attempts++;
            printf("Retrying transmission...\n");
        }
    }

    printf("Failed to send frame after 3 attempts.\n");
    free(stuffedFrame);
    return -1;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet) {
    unsigned char data_buffer[BUF_SIZE];  // Buffer for the received data
    int data_length = 0;

    // Continuously read until a valid frame is received
    while (1) {
        int result = check_information_frame(LlRx, data_buffer, &data_length);

        if (result == 0) {  // Successfully received a valid information frame
            printf("Information frame received successfully.\n");

            // Copy the data from data_buffer to the packet
            memcpy(packet, data_buffer, data_length);

            // Send RR (Receiver Ready) frame to acknowledge reception
            unsigned char rr_frame[5];
            unsigned char address = A_RECEIVER;
            unsigned char control = (data_buffer[2] == 0x00) ? 0x05 : 0x85;  // RR0 or RR1
            unsigned char bcc1 = address ^ control;

            rr_frame[0] = FLAG;
            rr_frame[1] = address;
            rr_frame[2] = control;
            rr_frame[3] = bcc1;
            rr_frame[4] = FLAG;

            if (send_frame(LlRx, rr_frame, 5) != 0) {
                printf("Error sending RR frame.\n");
                return -1;
            }

            return data_length;  // Return the length of the received data
        } else {
            // If we encounter a frame error, send REJ frame
            printf("Error in information frame, sending REJ frame.\n");

            unsigned char rej_frame[5];
            unsigned char address = A_RECEIVER;
            unsigned char control = (data_buffer[2] == 0x00) ? 0x01 : 0x81;  // REJ0 or REJ1
            unsigned char bcc1 = address ^ control;

            rej_frame[0] = FLAG;
            rej_frame[1] = address;
            rej_frame[2] = control;
            rej_frame[3] = bcc1;
            rej_frame[4] = FLAG;

            if (send_frame(LlRx, rej_frame, 5) != 0) {
                printf("Error sending REJ frame.\n");
                return -1;
            }
        }
    }

    return -1;
}


////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////

int llclose(int showStatistics) {
    // Define the DISC frame for disconnection
    unsigned char address = A_SENDER;
    unsigned char control = 0x0B;  // DISC control field (00001011)
    unsigned char bcc1 = address ^ control;
    const unsigned char disc_frame[5] = {
        FLAG,
        address,
        control,
        bcc1,
        FLAG
    };

    // Set up the alarm handler for retransmissions
    if (signal(SIGALRM, alarmHandler) == SIG_ERR) {
        perror("Unable to catch SIGALRM");
        return -1;
    }

    int attempts = 0;
    while (attempts < 3) {
        printf("Attempt #%d: Sending DISC frame...\n", attempts + 1);
        if (send_frame(LlTx, disc_frame, sizeof(disc_frame)) != 0) {
            printf("Error sending DISC frame.\n");
            return -1;
        }

        alarm(3);
        alarmEnabled = TRUE;

        if (check_setup_frame(LlTx) == 0) {  // UA received
            printf("Received UA frame, closing connection.\n");
            alarm(0);  
            closeSerialPort();  

            // Always print statistics
            printf("Transmission statistics:\n");
            printf("Total frames sent: %d\n", totalFramesSent);
            printf("Total retransmissions: %d\n", totalRetransmissions);
            printf("Total timeouts: %d\n", totalTimeouts);

            return 0;
        } else {
            printf("Timeout or error receiving UA, retrying DISC frame...\n");
        }

        pause();

        if (!alarmEnabled) {
            attempts++;
            printf("Retrying DISC transmission...\n");
        }
    }

    printf("Failed to close connection after 3 attempts.\n");
    return -1;
}
