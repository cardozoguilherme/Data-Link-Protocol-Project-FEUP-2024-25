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

#define FLAG 0x7E // 01111110 (0x7E)
// A (Address)
#define A_SENDER 0x03   // 00000011 (0x03) – frames sent by the Sender or answers from the Receiver
#define A_RECEIVER 0x01 // 00000001 (0x01) – frames sent by the Receiver or answers from the Sender
// C (Control)
#define C_SET 0x03 // SET: 00000011 (0x03)
#define C_UA 0x07  // UA: 00000111 (0x07)

#define C_0 0x00 // 00000000 (0x00)
#define C_1 0x80 // 01000000 (0x80)

#define RR_0 0xAA
#define RR_1 0xAB
#define REJ0 0x54
#define REJ1 0x55
#define DISC 0x0B

// Communications statistics (llclose)
int totalFramesSent = 0;       // Total frames sent
int totalRetransmissions = 0;  // Total retransmissions
int totalTimeouts = 0;         // Total timeouts

static unsigned char control = C_0;


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

unsigned char calculateBCC2(const unsigned char *data, int length) {
    unsigned char bcc2 = 0;
    for (int i = 0; i < length; i++) {
        bcc2 ^= data[i];  // XOR all the bytes
    }
    return 0xef;
    // return bcc2;
}

int destuffBytes(const unsigned char* input, int inputSize, unsigned char* output) {
    int outputSize = 0;
    int i = 0;

    while(i < inputSize) {
        if(input[i] == 0x7D) { // Escape byte detected
            i++; // Move to next byte

            if(i >= inputSize) {
                printf("Error: Truncated escape sequence at end of input.\n");
                return -1; // Indicate error
            }

            if(input[i] == 0x5E) {
                output[outputSize++] = 0x7E; // Replace with 0x7e
            } else if(input[i] == 0x5D) {
                output[outputSize++] = 0x7D; // Replace with 0x7d
            } else {
                printf("Error: Invalid escape sequence 0x7d 0x%02x.\n", input[i]);
                return -1; // Indicate error
            }
        } else {
            output[outputSize++] = input[i];
        }

        i++;
    }

    return outputSize;
}


int alarmEnabled = FALSE;
volatile int alarmCount = 0;

void alarmHandler(int signal) {
    alarmEnabled = FALSE;
    printf("Alarm triggered, retransmission count: %d\n", alarmCount+1);
    alarmCount++;
}

int check_setup_frame(LinkLayerRole role, unsigned char control_expected) {

    Setup_frame_state currentState = SETUP_START;
    unsigned char byte;
    unsigned char address = 0;
    unsigned char bcc = 0;
    volatile int stop_flag = FALSE;
    unsigned char control_received;

    while (stop_flag == FALSE) {
        int bytes = readByteSerialPort(&byte);
        if (bytes > 0) { 
            
            switch (currentState) {
                
                case SETUP_START:
                    if (byte == FLAG) {
                        currentState = SETUP_FLAG_RCV;
                    }
                    break;

                case SETUP_FLAG_RCV:
                    // Expecting the address byte
                    if (byte == A_SENDER) {
                        address = byte;
                        currentState = SETUP_A_RCV;
                    } else { 
                        return -1;
                    }
                    break;

                case SETUP_A_RCV:
                    // Expecting the control byte
                    if (byte == control_expected) { 
                        control_received = byte;
                        currentState = SETUP_C_RCV;
                    } else { 
                        return -1;
                    }
                    break;

                case SETUP_C_RCV:
                    // Expecting BCC (A ^ C)
                    bcc = address ^ control_received;
                    if (byte == bcc) {
                        currentState = SETUP_BCC_OK;
                    } else { 
                        return -1;
                    }
                    break;

                case SETUP_BCC_OK:
                    // Expecting end flag
                    if (byte == FLAG) {
                        currentState = SETUP_STOP; 
                        stop_flag = TRUE;
                    } else { 
                        return -1;
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
    unsigned char I_control = 0;
    unsigned char bcc1 = 0;
    int data_index = -1;
    volatile int stop_flag = FALSE;

    unsigned char data_frame[MAX_PAYLOAD_SIZE*2];

    while (!stop_flag) {
        int bytes = readByteSerialPort(&byte);
        if (bytes > 0) {
            
            switch (currentState) {
                case INFO_START:
                    if (byte == FLAG) {
                        currentState = INFO_FLAG_RCV;
                    }
                    break;

                case INFO_FLAG_RCV:
                    if (byte == A_SENDER) {
                        address = byte;
                        currentState = INFO_A_RCV;
                    } else if (byte != FLAG) {
                        return -1;
                    }
                    break;

                case INFO_A_RCV:
                    if (byte == C_0 || byte == C_1) {  // Information frame numbers (0 or 1)
                        I_control = byte;
                        currentState = INFO_C_RCV;
                    } else {
                        return -1;
                    }
                    break;

                case INFO_C_RCV:
                    bcc1 = address ^ I_control;
                    if (byte == bcc1) {
                        currentState = INFO_BCC1_OK;
                    } else {
                        return -1;
                    }
                    break;

                case INFO_BCC1_OK:
                    if (byte == FLAG) {
                        if (data_index == -1) {  // Empty I-frame, no data field
                            return -2;
                        } 
                        else {
                            return -1;
                        }
                    } else {
                        data_index = 0;
                        currentState = INFO_DATA_RCV;
                    }
                    break;

                case INFO_DATA_RCV:
                    
                    if(byte==FLAG){            
                        currentState = INFO_STOP;
                        stop_flag = TRUE;
                        break;
                    }

                    data_frame[data_index] = byte;
                    data_index++;
                    break;
                default:
                    currentState = INFO_START;
                    break;
            }
        }
    }

    *data_length = destuffBytes(data_frame, data_index, data_buffer);

    if(*data_length == -1) {
        printf("Error during byte destuffing.\n");
        return -1;
    }

    return (currentState == INFO_STOP) ? 0 : -1;
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
            printf("Error sending SET frame");
            return -1;
        }
        printf("SET Frame sent\n");

        if(check_setup_frame(LlTx, C_UA) != 0) {
            printf("Error reading UA frame\n");
            return -1;
        }

        printf("UA frame read\n");

    } else if (connectionParameters.role == LlRx) {

        if(check_setup_frame(LlRx, C_SET) != 0) {
            printf("Error reading SET frame\n");
            return -1;
        }
        printf("SET frame read\n");

        char BBC1 = A_SENDER ^ C_UA;
        const unsigned char ua_frame[5] = {
            FLAG,
            A_SENDER,
            C_UA,
            BBC1,
            FLAG,
        };
        if(send_frame(LlRx, ua_frame, 5) != 0) {
            printf("Error sending UA frame");
            return -1;
        }
        printf("UA Frame sent\n");
    } else {
        printf("Invalid role\n");
        return -1;
    }

    return 0;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////


unsigned char *byteStuffing(const unsigned char *frame, int frameSize, int *stuffedFrameSize) {
    // Allocate memory for the worst-case scenario (each byte being stuffed)
    unsigned char *stuffedFrame = malloc(frameSize * 2 * sizeof(unsigned char));
    if (!stuffedFrame) {
        printf("Failed to allocate memory for byte stuffing.\n");
        return NULL;
    }

    int j = 0;  // Index for the stuffed frame
    for (int i = 0; i < frameSize; i++) {
        if (frame[i] == FLAG) {
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
    
    unsigned char address = A_SENDER;  
    unsigned char bcc1 = address ^ control;

    int frameSize = bufSize; 
    unsigned char *frame = malloc((frameSize + 2) * sizeof(unsigned char)); 

    if (!frame) {
        printf("Failed to allocate memory for frame.\n");
        return -1;
    }

    free(frame);

    frame[0] = FLAG;
    frame[1] = address;
    frame[2] = control;
    frame[3] = bcc1;

    int stuffedFrameSize;
    unsigned char *stuffedFrame = byteStuffing(buf, bufSize, &stuffedFrameSize);

    memcpy(&frame[4], stuffedFrame, stuffedFrameSize);

    frame[4 + stuffedFrameSize] = calculateBCC2(buf, bufSize);
    frame[5 + stuffedFrameSize] = FLAG;

    if (!stuffedFrame) {
        printf("Failed to allocate memory for stuffed frame.\n");
        return -1;
    }

    if (signal(SIGALRM, alarmHandler) == SIG_ERR) {
        perror("Unable to catch SIGALRM");
        return -1;
    }

    int attempts = 0;
    while (attempts < 3) {
        // Send the stuffed frame
        if (send_frame(LlTx, frame, stuffedFrameSize+6) != 0) {
            printf("Error sending frame.\n");
            return -1;
        }
        unsigned char res;

        if(control == C_1) {
            res = RR_0;
        } else {
            res = RR_1;
        }

        // Increment total frames sent
        totalFramesSent++;
        // printf("\nTotal frames sent: %d\n", totalFramesSent);
        // Set the alarm for timeout (3 seconds)
        alarm(3);
        alarmEnabled = TRUE;

        // Wait for acknowledgment
        int result = check_setup_frame(LlTx, res);
        if (result == 0) {  // ACK received
            printf("Frame acknowledged.\n");
            alarm(0);
            control ^= C_1;
            return bufSize;
        } else if (result == -2) {  // NACK received, retransmit
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
            printf("Attempt #%d\n", attempts);

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
    unsigned char data_buffer[MAX_PAYLOAD_SIZE*2];  // Buffer for the received data
    int data_length = 0;

    int result = check_information_frame(LlRx, data_buffer, &data_length);

    if (result == 0) {  // Successfully received a valid information frame
        printf("Information frame received successfully.\n");

        // Copy the data from data_buffer to the packet
        memcpy(packet, data_buffer, data_length);

        printf("control global variable - %02x\n", control);

        // Send RR (Receiver Ready) frame to acknowledge reception
        unsigned char rr_frame[5];
        unsigned char address = A_SENDER;
        unsigned char res_control = control ? RR_0 : RR_1;  // RR0 or RR1
        unsigned char bcc1 = address ^ res_control;

        rr_frame[0] = FLAG;
        rr_frame[1] = address;
        rr_frame[2] = res_control;
        rr_frame[3] = bcc1;
        rr_frame[4] = FLAG;

        if (send_frame(LlRx, rr_frame, 5) != 0) {
            printf("Error sending RR frame.\n");
            return -1;
        }
        printf("RR frame sent successfully.\n");

        control ^= 0x80;

        return data_length;  // Return the length of the received data
    } else if (result == -2) {  // Frame error detected
        // If we encounter a frame error, send REJ frame
        printf("Error in information frame, sending REJ frame.\n");

        unsigned char rej_frame[5];
        unsigned char address = A_SENDER;
        unsigned char res_control = (data_buffer[2] == C_0) ? REJ0 : REJ1;  // REJ0 or REJ1
        unsigned char bcc1 = address ^ res_control;

        rej_frame[0] = FLAG;
        rej_frame[1] = address;
        rej_frame[2] = res_control;
        rej_frame[3] = bcc1;
        rej_frame[4] = FLAG;

        if (send_frame(LlRx, rej_frame, 5) != 0) {
            printf("Error sending REJ frame.\n");
            return -1;
        }

        printf("REJ frame sent successfully.\n");
        return -1;
    }

    return -1;
}


////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////

int llclose(int showStatistics) {
    // Define the DISC frame for disconnection
    unsigned char address = A_SENDER;
    unsigned char close_control = DISC;  // DISC control field (00001011)
    unsigned char bcc1 = address ^ close_control;
    const unsigned char disc_frame[5] = {
        FLAG,
        address,
        close_control,
        bcc1,
        FLAG
    };

    const unsigned char ua_frame[5] = {
        FLAG,
        address,
        C_UA,
        address ^ C_UA,
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

        if (check_setup_frame(LlTx, DISC) == 0) {  // DISC received
            printf("Received DISC frame, closing connection.\n");

            if (send_frame(LlTx, ua_frame, sizeof(ua_frame)) != 0) {
                printf("Error sending UA frame.\n");
                return -1;
            }
            alarm(0);  
            closeSerialPort();

            // Always print statistics
            printf("Transmission statistics:\n");
            printf("Total frames sent: %d\n", totalFramesSent);
            printf("Total retransmissions: %d\n", totalRetransmissions);
            printf("Total timeouts: %d\n", totalTimeouts);

            return 0;
        } else {
            printf("Timeout or error receiving DISC, retrying DISC frame...\n");
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
