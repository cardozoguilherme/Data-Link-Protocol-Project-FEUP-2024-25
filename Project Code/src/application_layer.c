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

    // Configure os parâmetros de conexão
    LinkLayer connectionParameters;
    strcpy(connectionParameters.serialPort, serialPort);
    connectionParameters.role = strcmp(role, "tx") == 0 ? LlTx : LlRx;
    connectionParameters.baudRate = baudRate;
    connectionParameters.nRetransmissions = nTries;
    connectionParameters.timeout = timeout;

    printf("Attempting to establish connection...\n");

    // Passo I: `Rx` invoca `llopen`
    if (connectionParameters.role == LlRx) {
        if (llopen(connectionParameters) < 0) {
            printf("Failed to open connection as receiver.\n");
            exit(1);
        }
        printf("Receiver connection established.\n");
    }
    // Passo II: `Tx` invoca `llopen`
    else if (connectionParameters.role == LlTx) {
        if (llopen(connectionParameters) < 0) {
            printf("Failed to open connection as transmitter.\n");
            exit(1);
        }
        printf("Transmitter connection established.\n");
    }

    // Fase de Transferência de Dados
    FILE *file;
    if (connectionParameters.role == LlTx) {
        file = fopen(filename, "rb");
        if (!file) {
            perror("Error opening file for reading");
            llclose(TRUE);  // Fecha a conexão com exibição de estatísticas
            exit(1);
        }
        
        unsigned char buffer[MAX_PAYLOAD_SIZE];
        int bytesRead;
        while ((bytesRead = fread(buffer, 1, MAX_PAYLOAD_SIZE, file)) > 0) {
            // Transmite pacotes de dados
            if (llwrite(buffer, bytesRead) < 0) {
                printf("Error transmitting data.\n");
                fclose(file);
                llclose(TRUE);
                exit(1);
            }
            printf("Sent %d bytes.\n", bytesRead);
        }
        fclose(file);
        printf("File transmission completed successfully.\n");

    } else if (connectionParameters.role == LlRx) {
        file = fopen(filename, "wb");
        if (!file) {
            perror("Error opening file for writing");
            llclose(TRUE);
            exit(1);
        }

        unsigned char buffer[MAX_PAYLOAD_SIZE];
        int bytesRead;
        
        while (TRUE) {
            bytesRead = llread(buffer);
            // TODO
            if (bytesRead == 968) {  // Fim da transmissão
                printf("Connection closed by remote host.\n");
                break;
            } else if (bytesRead < 0) {  // Erro durante a leitura
                printf("Error reading data. Trying again\n");
            } else if (fwrite(buffer, 1, bytesRead, file) != bytesRead) {
                perror("Error writing to file");
                fclose(file);
                llclose(TRUE);
                exit(1);
            }
            printf("Received %d bytes.\n", bytesRead);
        }

        fclose(file);
        printf("File reception completed successfully.\n");
    }

    // Fase de Encerramento da Conexão
    printf("Closing connection...\n");
    if (llclose(TRUE) == 0) {
        printf("Connection closed successfully.\n");
    } else {
        printf("Error closing connection.\n");
    }
}
