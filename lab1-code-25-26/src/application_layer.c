// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>

// Control packet types
#define CTRL_DATA 0x02
#define CTRL_START 0x01
#define CTRL_END 0x03
// TLV types
#define TLV_FILESIZE 0x00
#define TLV_FILENAME 0x01

// Maximum data per packet (should fit in MAX_PAYLOAD_SIZE)
#define MAX_DATA_SIZE 997


// Build control packet (START or END)
int buildControlPacket(unsigned char *packet, unsigned char ctrl, 
                       const char *filename, long fileSize)
{
    int index = 0;
    
    // Control field
    packet[index++] = ctrl;
    
    // TLV for file size
    packet[index++] = TLV_FILESIZE;
    
    // Calculate size bytes needed
    unsigned char sizeBytes[sizeof(long)];
    int numSizeBytes = 0;
    long tempSize = fileSize;
    
    do {
        sizeBytes[numSizeBytes++] = tempSize & 0xFF;
        tempSize >>= 8;
    } while (tempSize > 0);
    
    packet[index++] = numSizeBytes; // L1
    
    // V1 (size in little-endian)
    for (int i = numSizeBytes - 1; i >= 0; i--)
    {
        packet[index++] = sizeBytes[i];
    }
    
    // TLV for filename
    packet[index++] = TLV_FILENAME;
    
    int filenameLen = strlen(filename);
    packet[index++] = filenameLen; // L2
    
    // V2 (filename)
    memcpy(packet + index, filename, filenameLen);
    index += filenameLen;
    
    return index;
}

// Parse control packet
int parseControlPacket(const unsigned char *packet, int packetSize,
                       unsigned char *ctrl, char *filename, long *fileSize)
{
    int index = 0;
    
    // Control field
    *ctrl = packet[index++];
    
    if (*ctrl != CTRL_START && *ctrl != CTRL_END)
    {
        printf("Invalid control packet type: 0x%02X\n", *ctrl);
        return -1;
    }
    
    // Parse TLVs
    while (index < packetSize)
    {
        unsigned char type = packet[index++];
        unsigned char length = packet[index++];
        
        if (type == TLV_FILESIZE)
        {
            *fileSize = 0;
            for (int i = 0; i < length; i++)
            {
                *fileSize = (*fileSize << 8) | packet[index++];
            }
        }
        else if (type == TLV_FILENAME)
        {
            memcpy(filename, packet + index, length);
            filename[length] = '\0';
            index += length;
        }
        else
        {
            printf("Unknown TLV type: 0x%02X\n", type);
            index += length;
        }
    }
    
    return 0;
}

// Build data packet
int buildDataPacket(unsigned char *packet, const unsigned char *data, int dataSize)
{
    int index = 0;
    
    // Control field
    packet[index++] = CTRL_DATA;
    
    // L2 and L1 (K = 256 * L2 + L1)
    packet[index++] = (dataSize >> 8) & 0xFF; // L2
    packet[index++] = dataSize & 0xFF;         // L1
    
    // Data
    memcpy(packet + index, data, dataSize);
    index += dataSize;
    
    return index;
}

// Parse data packet
int parseDataPacket(const unsigned char *packet, int packetSize,
                    unsigned char *data, int *dataSize)
{
    int index = 0;
    
    // Control field
    unsigned char ctrl = packet[index++];
    if (ctrl != CTRL_DATA)
    {
        printf("Not a data packet: 0x%02X\n", ctrl);
        return -1;
    }
    
    // L2 and L1
    int L2 = packet[index++];
    int L1 = packet[index++];
    
    *dataSize = 256 * L2 + L1;
    
    // Verify size
    if (*dataSize != packetSize - 3)
    {
        printf("Data size mismatch: expected %d, got %d\n", 
               *dataSize, packetSize - 3);
        return -1;
    }
    
    // Copy data
    memcpy(data, packet + index, *dataSize);
    
    return 0;
}

// Get file size
long getFileSize(const char *filename)
{
    struct stat st;
    if (stat(filename, &st) == 0)
    {
        return st.st_size;
    }
    return -1;
}

////////////////////////////////////////////////
// TRANSMITTER
////////////////////////////////////////////////
void transmitFile(const char *filename)
{
    // Open file
    FILE *file = fopen(filename, "rb");
    if (file == NULL)
    {
        perror("Error opening file");
        return;
    }
    
    // Get file size
    long fileSize = getFileSize(filename);
    if (fileSize < 0)
    {
        perror("Error getting file size");
        fclose(file);
        return;
    }
    
    printf("File: %s\n", filename);
    printf("Size: %ld bytes\n", fileSize);
    printf("-----------------------------------\n");
    
    // Build and send START packet
    printf("\n>>> Sending START packet <<<\n");
    unsigned char startPacket[MAX_PAYLOAD_SIZE];
    int startPacketSize = buildControlPacket(startPacket, CTRL_START, filename, fileSize);
    
    if (llwrite(startPacket, startPacketSize) < 0)
    {
        printf("ERROR: Failed to send START packet\n");
        fclose(file);
        return;
    }
    
    printf("START packet sent (%d bytes)\n", startPacketSize);
    
    // Send data packets
    printf("\n>>> Sending file data <<<\n");
    unsigned char buffer[MAX_DATA_SIZE];
    unsigned char dataPacket[MAX_PAYLOAD_SIZE];
    int bytesRead;
    int packetNum = 0;
    long totalBytesSent = 0;
    
    while ((bytesRead = fread(buffer, 1, MAX_DATA_SIZE, file)) > 0)
    {
        int dataPacketSize = buildDataPacket(dataPacket, buffer, bytesRead);
        
        if (llwrite(dataPacket, dataPacketSize) < 0)
        {
            printf("ERROR: Failed to send data packet %d\n", packetNum);
            fclose(file);
            return;
        }
        
        totalBytesSent += bytesRead;
        packetNum++;
        
        printf("Data packet %d sent (%d bytes) - Progress: %.1f%%\n", 
               packetNum, bytesRead, (totalBytesSent * 100.0) / fileSize);
    }
    
    fclose(file);
    
    printf("\nTotal data sent: %ld bytes in %d packets\n", totalBytesSent, packetNum);
    
    // Build and send END packet
    printf("\n>>> Sending END packet <<<\n");
    unsigned char endPacket[MAX_PAYLOAD_SIZE];
    int endPacketSize = buildControlPacket(endPacket, CTRL_END, filename, fileSize);
    
    if (llwrite(endPacket, endPacketSize) < 0)
    {
        printf("ERROR: Failed to send END packet\n");
        return;
    }
    
    printf("END packet sent (%d bytes)\n", endPacketSize);
    printf("\n>>> File transmission complete! <<<\n");
}

////////////////////////////////////////////////
// RECEIVER
////////////////////////////////////////////////
void receiveFile(const char *outputFilename)
{
    printf("Waiting to receive file...\n");
    printf("Output file: %s\n", outputFilename);
    printf("-----------------------------------\n");
    
    // Receive START packet
    printf("\n>>> Waiting for START packet <<<\n");
    unsigned char startPacket[MAX_PAYLOAD_SIZE];
    int startPacketSize = llread(startPacket);
    
    if (startPacketSize < 0)
    {
        printf("ERROR: Failed to receive START packet\n");
        return;
    }
    
    // Parse START packet
    unsigned char ctrl;
    char receivedFilename[256];
    long fileSize;
    
    if (parseControlPacket(startPacket, startPacketSize, &ctrl, 
                          receivedFilename, &fileSize) < 0)
    {
        printf("ERROR: Failed to parse START packet\n");
        return;
    }
    
    if (ctrl != CTRL_START)
    {
        printf("ERROR: Expected START packet, got 0x%02X\n", ctrl);
        return;
    }
    
    printf("START packet received:\n");
    printf("  Filename: %s\n", receivedFilename);
    printf("  Size: %ld bytes\n", fileSize);
    
    // Open output file
    FILE *file = fopen(outputFilename, "wb");
    if (file == NULL)
    {
        perror("Error creating output file");
        return;
    }
    
    // Receive data packets
    printf("\n>>> Receiving file data <<<\n");
    unsigned char packet[MAX_PAYLOAD_SIZE];
    unsigned char data[MAX_DATA_SIZE];
    int dataSize;
    long totalBytesReceived = 0;
    int packetNum = 0;
    
    while (1)
    {
        int packetSize = llread(packet);
        
        if (packetSize < 0)
        {
            // Error receiving packet (BCC2 error, etc.)
            // llread already sent REJ, just wait for retransmission
            printf("Waiting for retransmission...\n");
            continue;  // ← CORREÇÃO CRÍTICA!
        }
        
        if (packetSize == 0)
        {
            // Duplicate frame, continue
            continue;
        }
        
        // Check if it's END packet
        if (packet[0] == CTRL_END)
        {
            printf("\n>>> END packet received <<<\n");
            break;
        }
        
        // Parse data packet
        if (parseDataPacket(packet, packetSize, data, &dataSize) < 0)
        {
            printf("ERROR: Failed to parse data packet\n");
            fclose(file);
            return;
        }
        
        // Write data to file
        if (fwrite(data, 1, dataSize, file) != (size_t)dataSize)
        {
            perror("Error writing to file");
            fclose(file);
            return;
        }
        
        totalBytesReceived += dataSize;
        packetNum++;
        
        printf("Data packet %d received (%d bytes) - Progress: %.1f%%\n",
               packetNum, dataSize, (totalBytesReceived * 100.0) / fileSize);
    }
    
    fclose(file);
    
    printf("\nTotal data received: %ld bytes in %d packets\n", 
           totalBytesReceived, packetNum);
    
    // Verify file size
    if (totalBytesReceived == fileSize)
    {
        printf("\n✓ File size matches! Transfer successful!\n");
    }
    else
    {
        printf("\n✗ WARNING: File size mismatch!\n");
        printf("  Expected: %ld bytes\n", fileSize);
        printf("  Received: %ld bytes\n", totalBytesReceived);
    }
    
    printf("\n>>> File reception complete! <<<\n");
}

////////////////////////////////////////////////
// MAIN APPLICATION LAYER FUNCTION
////////////////////////////////////////////////
void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    // Setup connection parameters
    LinkLayer connectionParams;
    strcpy(connectionParams.serialPort, serialPort);
    connectionParams.baudRate = baudRate;
    connectionParams.nRetransmissions = nTries;
    connectionParams.timeout = timeout;
    
    // Set role
    if (strcmp(role, "tx") == 0)
    {
        connectionParams.role = LlTx;
        printf("\n=================================\n");
        printf("  APPLICATION LAYER: TRANSMITTER\n");
        printf("=================================\n");
    }
    else if (strcmp(role, "rx") == 0)
    {
        connectionParams.role = LlRx;
        printf("\n=================================\n");
        printf("  APPLICATION LAYER: RECEIVER\n");
        printf("=================================\n");
    }
    else
    {
        printf("ERROR: Invalid role '%s'\n", role);
        return;
    }
    
    // Open connection
    printf("\n>>> PHASE 1: Opening connection <<<\n");
    if (llopen(connectionParams) < 0)
    {
        printf("ERROR: Failed to open connection\n");
        return;
    }
    
    printf("\n>>> Connection opened successfully! <<<\n");
    
    // Transfer file
    printf("\n>>> PHASE 2: File transfer <<<\n");
    
    if (connectionParams.role == LlTx)
    {
        transmitFile(filename);
    }
    else
    {
        receiveFile(filename);
    }
    
    // Close connection
    printf("\n>>> PHASE 3: Closing connection <<<\n");
    if (llclose() < 0)
    {
        printf("ERROR: Failed to close connection\n");
        return;
    }
    
    printf("\n>>> Connection closed successfully! <<<\n");
    printf("\n=================================\n");
    printf("  TRANSFER COMPLETE!\n");
    printf("=================================\n\n");
}