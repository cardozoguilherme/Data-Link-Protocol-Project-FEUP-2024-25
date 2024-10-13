// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "link_layer.h"
#include "serial_port.h"


void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename) {
    
    LinkLayer connectionParameters;
    strcpy(connectionParameters.serialPort, serialPort);
    connectionParameters.role = strcmp(role, "tx") == 0 ? LlTx : LlRx;
    connectionParameters.baudRate = baudRate;
    connectionParameters.nRetransmissions = nTries;
    connectionParameters.timeout = timeout;

    printf("Attempting to establish connection...\n");
    int fd = llopen(connectionParameters);

    if (fd < 0) {
        printf("Connection failed. Error code: %d\n", fd);
        exit(1);
    }

    printf("Connection established successfully! File descriptor: %d\n", fd);

    // Here you would typically perform file operations or data transfer
    // For now, we'll just close the connection

    printf("Closing connection...\n");
    if (llclose(fd) == 0) {
        printf("Connection closed successfully.\n");
    } else {
        printf("Error closing connection.\n");
    }
}
