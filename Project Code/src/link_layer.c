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
int fd;

static unsigned char control = C_0;
static int TIMEOUT;
static int MAX_RETRIES;

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
    INFO_DATA_RCV,
    INFO_BCC2_OK,
    INFO_STOP
} Information_frame_state;

LinkLayerRole ROLE;

unsigned char calculateBCC2(const unsigned char *data, int length) {
    unsigned char bcc2 = 0;
    for (int i = 0; i < length; i++) {
        bcc2 ^= data[i];  // XOR all the bytes
    }
    
    return bcc2;
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


volatile int alarmEnabled = FALSE;
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
    bool buffer_empty = FALSE;

    while (stop_flag == FALSE) {
        if(role == LlTx && !alarmEnabled) {
            printf("BOB\n");
            stop_flag = TRUE;
            break;
        }
        printf("check_setup loop\n");

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
                    if (byte == A_SENDER || byte == A_RECEIVER) {
                        address = byte;
                        currentState = SETUP_A_RCV;
                    } else if (byte != FLAG) {
                        printf("Error in control frame header: Address\n");
                        stop_flag = TRUE;
                    }
                    break;

                case SETUP_A_RCV:
                    // Expecting the control byte
                    if (byte == control_expected) { 
                        control_received = byte;
                        currentState = SETUP_C_RCV;
                    } else { 
                        printf("Error in control frame header: Control\n");
                        stop_flag = TRUE;
                    }
                    break;

                case SETUP_C_RCV:
                    // Expecting BCC (A ^ C)
                    bcc = address ^ control_received;
                    if (byte == bcc) {
                        currentState = SETUP_BCC_OK;
                    } else { 
                        printf("Error in control frame header: BCC1\n");
                        stop_flag = TRUE;
                    }
                    break;

                case SETUP_BCC_OK:
                    // Expecting end flag
                    if (byte == FLAG) {
                        currentState = SETUP_STOP; 
                        stop_flag = TRUE;
                    } else { 
                        printf("Error in control frame header: END_FLAG\n");
                        stop_flag = TRUE;
                    }
                    break;

                default:
                    currentState = SETUP_START;
                    break;
            }
        }
    }

    if (currentState != SETUP_STOP) {
        printf("Error in control frame: Unexpected end of frame1\n");
        while (buffer_empty == FALSE) {
            int bytes = readByteSerialPort(&byte);
            if (bytes <= 0) buffer_empty = TRUE;
        }
        printf("Error in control frame: Unexpected end of frame2\n");
        return -1;
    }
    return 0;

}

int check_information_frame(LinkLayerRole role, unsigned char *data_buffer, int *data_length) {
    Information_frame_state currentState = INFO_START;
    unsigned char byte;
    unsigned char address = 0;
    unsigned char I_control = 0;
    unsigned char bcc1 = 0;
    unsigned char bcc2;
    unsigned char new_bcc2;
    int stuffed_data_index = -1;
    volatile int stop_flag = FALSE;

    unsigned char stuffed_data_frame[MAX_PAYLOAD_SIZE * 2];

    while (!stop_flag) {
        int bytes = readByteSerialPort(&byte);

        if (bytes > 0) {
            // Processa o byte recebido
            switch (currentState) {
                case INFO_START:
                    if (byte == FLAG) {
                        currentState = INFO_FLAG_RCV;
                    }
                    break;

                case INFO_FLAG_RCV:
                    if (byte == A_SENDER || byte == A_RECEIVER) {
                        address = byte;
                        currentState = INFO_A_RCV;
                    } else if (byte != FLAG) {
                        printf("Error in information frame header: Address\n\n");
                        stop_flag = TRUE;
                    }
                    break;

                case INFO_A_RCV:
                    if (byte == C_0 || byte == C_1) { 
                        I_control = byte;
                        currentState = INFO_C_RCV;
                    } else if(byte == DISC) {
                        currentState = INFO_STOP;
                        stop_flag = TRUE;
                    } else {
                        printf("Error in information frame header: Control\n\n");
                        stop_flag = TRUE;
                    }
                    break;

                case INFO_C_RCV:
                    bcc1 = address ^ I_control;
                    if (byte == bcc1) {
                        currentState = INFO_DATA_RCV;
                        stuffed_data_index = 0;
                    } else {
                        printf("Error in information frame header: BCC1\n");
                        stop_flag = TRUE;
                    }
                    break;

                case INFO_DATA_RCV:
                    if (byte == FLAG) {
                        currentState = INFO_STOP;
                        stop_flag = TRUE;
                        stuffed_data_index--;
                        break;
                    }

                    stuffed_data_frame[stuffed_data_index] = byte;
                    stuffed_data_index++;
                    break;

                default:
                    currentState = INFO_START;
                    break;
            }
        }
    }

    if(stuffed_data_index >= 0) {
        *data_length = destuffBytes(stuffed_data_frame, stuffed_data_index, data_buffer);

        if (*data_length == -1) {
            printf("Error in information frame: DATA (BCC2)\n\n");
            return -2;
        }

        bcc2 = stuffed_data_frame[stuffed_data_index];
        new_bcc2 = calculateBCC2(data_buffer, *data_length);

        if (bcc2 != new_bcc2) {
            return -2;
            printf("Error in information frame: DATA (BCC2)\n");
        }
        return 1;
    }

    if (currentState != INFO_STOP) {
        bool buffer_empty = FALSE;
        while (buffer_empty == FALSE) {
            int bytes = readByteSerialPort(&byte);
            if (bytes <= 0) buffer_empty = TRUE;
        }
        return -1;
    }
    
    return 0;
}

int send_frame(LinkLayerRole role, const unsigned char *frame, int frame_size) {

    if (writeBytesSerialPort(frame, frame_size) == -1) {
        perror("Error writing frame to serial port\n");
        return -1;
    }

    return 0;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters) {
    MAX_RETRIES = connectionParameters.nRetransmissions;
    TIMEOUT = connectionParameters.timeout;
    
    fd = openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate);
    if (fd < 0) {
        printf("Error opening serial port\n");
        return -1;
    }

    // Setup connection
    if(connectionParameters.role == LlTx) {
        if (signal(SIGALRM, alarmHandler) == SIG_ERR) {
            perror("Unable to catch SIGALRM\n");
            return -1;
        }
        alarmEnabled = FALSE;
        while(alarmCount < MAX_RETRIES) {
            if (!alarmEnabled) {
                alarm(TIMEOUT);
                alarmEnabled = TRUE;

                char BBC1 = A_SENDER ^ C_SET;
                const unsigned char set_frame[5] = {
                    FLAG,
                    A_SENDER,
                    C_SET,
                    BBC1,
                    FLAG,
                };

                if(send_frame(LlTx, set_frame, 5) == 0) {
                    printf("SET Frame sent\n");
                } else {
                    printf("Error sending SET frame\n");
                    continue;
                }
            }

            if(check_setup_frame(LlTx, C_UA) == 0) {
                alarm(0);
                printf("UA frame read\n");
                return 0;
            } else {
                printf("Error reading UA frame\n");
            }   
        }

        printf("Failed to close connection after %d attempts.\n", MAX_RETRIES);
        closeSerialPort();
        return -1;

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
            printf("Error sending UA frame\n");
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

    if (signal(SIGALRM, alarmHandler) == SIG_ERR) {
        perror("Unable to catch SIGALRM\n");
        return -1;
    }

    alarmCount = 0;
    alarmEnabled = FALSE;
    while(alarmCount < MAX_RETRIES) {
        if (!alarmEnabled) {
            alarm(TIMEOUT);
            alarmEnabled = TRUE;

            int frameSize = bufSize; 
            unsigned char *frame = malloc((frameSize + 2) * sizeof(unsigned char)); 

            if (!frame) {
                printf("Failed to allocate memory for frame.\n");
                return -1;
            }

            free(frame);

            frame[0] = FLAG;
            frame[1] = A_SENDER; 
            frame[2] = control;
            frame[3] = A_SENDER ^ control;

            int stuffedFrameSize;
            unsigned char *stuffedFrame = byteStuffing(buf, bufSize, &stuffedFrameSize);

            memcpy(&frame[4], stuffedFrame, stuffedFrameSize);

            frame[4 + stuffedFrameSize] = calculateBCC2(buf, bufSize);
            frame[5 + stuffedFrameSize] = FLAG;

            if (!stuffedFrame) {
                printf("Failed to allocate memory for stuffed frame.\n");
                return -1;
            }

            if (send_frame(LlTx, frame, stuffedFrameSize+6) != 0) {
                printf("Error sending frame.\n");
                return -1;
            }
            
            // Increment total frames sent
            totalFramesSent++;
        }

        unsigned char res;
        if(control == C_1) {
            res = RR_0;
        } else {
            res = RR_1;
        }

        // Wait for acknowledgment
        printf("before the check_setup_frame\n");
        int result = check_setup_frame(LlTx, res);
        printf("after the check_setup_frame\n");

        if (result == 0) {  // ACK received
            printf("%s answer received.\n", res == RR_0 ? "RR0" : "RR1");
            alarm(0);
            control ^= C_1;
            return bufSize;
        } else if (result == -2) {  // NACK received, retransmit
            printf("Frame rejected, retransmitting...\n");
            totalRetransmissions++;  // Increment retransmission count
            alarmEnabled = FALSE;
        } else {  // Timeout occurred, retransmit
            printf("Timeout, retransmitting...\n");
            totalTimeouts++;  // Increment timeout count
            alarmEnabled = FALSE;
        }
    }

    printf("Failed to close connection after %d attempts.\n", MAX_RETRIES);
    return -1;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet) {
    unsigned char data_buffer[MAX_PAYLOAD_SIZE*2];  // Buffer for the received data
    int data_length = 0;

    int result = check_information_frame(LlRx, data_buffer, &data_length);


    if (result == 1) {  // Successfully received a valid information frame

        // Copy the data from data_buffer to the packet
        memcpy(packet, data_buffer, data_length);

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

        return -1;
    } else if (result == 0) {
        return 0;
    } else if (result == -1) {
        return -1;
    }

    return -1;
}


////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////

int llclose(int showStatistics) {
    // Configura o manipulador de alarme para retransmissões
    if (signal(SIGALRM, alarmHandler) == SIG_ERR) {
        perror("Unable to catch SIGALRM\n");
        return -1;
    }

    int alarmCount = 0;
    alarmEnabled = FALSE;
    while (alarmCount < MAX_RETRIES) {
        if (ROLE == LlTx) {
            if (!alarmEnabled) {
                alarm(TIMEOUT);
                alarmEnabled = TRUE;

                unsigned char disc_frame[5] = {
                    FLAG,
                    A_SENDER,
                    DISC,
                    A_SENDER ^ DISC,
                    FLAG
                };

                if (send_frame(LlTx, disc_frame, sizeof(disc_frame)) != 0) {
                    printf("Error sending DISC frame.\n");
                    return -1;
                }

                printf("DISC frame sent.\n");
            }
            // Espera pelo DISC do receptor
            if (check_setup_frame(LlTx, DISC) == 0) {
                alarm(0);
                alarmEnabled = FALSE;

                if (!alarmEnabled) {
                    alarm(TIMEOUT);
                    alarmEnabled = TRUE;

                    unsigned char ua_frame[5] = {
                        FLAG,
                        A_SENDER,
                        C_UA,
                        A_SENDER ^ C_UA,
                        FLAG
                    };

                    // Envia UA para confirmar o fechamento
                    if (send_frame(LlTx, ua_frame, sizeof(ua_frame)) != 0) {
                        printf("Error sending UA frame.\n");
                    }
                    printf("UA frame sent.\n");

                    alarm(0); // Cancela o alarme após envio do UA
                    closeSerialPort();

                    // Exibe estatísticas, se necessário
                    if (showStatistics == TRUE) {
                        printf("\nTransmission statistics:\n");
                        printf("Total frames sent: %d\n", totalFramesSent);
                        printf("Total retransmissions: %d\n", totalRetransmissions);
                        printf("Total timeouts: %d\n", totalTimeouts);
                    }
                    return 0;
                } else {
                    printf("Timeout or error receiving DISC, retrying DISC frame...\n");
                    alarmEnabled = FALSE;
                }
            } else {  // LlRx: Receptor
                if (!alarmEnabled) {
                    alarm(TIMEOUT);
                    alarmEnabled = TRUE;

                    unsigned char disc_frame[5] = {
                        FLAG,
                        A_RECEIVER,
                        DISC,
                        A_RECEIVER ^ DISC,
                        FLAG
                    };
                    
                    // Envia DISC em resposta
                    if (send_frame(LlRx, disc_frame, sizeof(disc_frame)) != 0) {
                        printf("Error sending DISC frame.\n");
                        return -1;
                    }
                    printf("DISC frame sent.\n");
                }

                if (check_setup_frame(LlRx, C_UA) == 0) {
                    printf("UA frame received\n");

                    alarm(0); // Cancela o alarme após receber o UA
                    closeSerialPort();

                    // // Exibe estatísticas, se necessário
                    // if (showStatistics == TRUE) {
                    //     printf("Reception statistics:\n");
                    //     printf("Total frames sent: %d\n", totalFramesSent);
                    //     printf("Total retransmissions: %d\n", totalRetransmissions);
                    //     printf("Total timeouts: %d\n", totalTimeouts);
                    // }
                    return 0;
                } else {
                    printf("Error receiving UA frame, retrying...\n");
                    alarmEnabled = FALSE;
                }
            }
            printf("Retrying DISC transmission...\n");
        }
    }
    printf("Failed to close connection after %d attempts.\n", MAX_RETRIES);
    return -1;
}
