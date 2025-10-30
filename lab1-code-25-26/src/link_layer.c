// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

// Frame fields
#define FLAG 0x7E
#define ESCAPE 0x7D
#define ADDR_TX 0x03  // Commands from TX, Replies from RX
#define ADDR_RX 0x01  // Commands from RX, Replies from TX

// Control field values
#define CTRL_SET 0x03
#define CTRL_UA 0x07
#define CTRL_DISC 0x0B
#define CTRL_RR0 0xAA
#define CTRL_RR1 0xAB
#define CTRL_REJ0 0x54
#define CTRL_REJ1 0x55
#define CTRL_I0 0x00
#define CTRL_I1 0x80

// State machine states
typedef enum {
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC1_OK,
    DATA_RCV,
    STOP_STATE
} State;

// Global variables for alarm
static int alarmEnabled = FALSE;
static int alarmCount = 0;
static int timeoutOccurred = FALSE;

// Link layer connection parameters
static LinkLayer connectionParams;

// Frame sequence numbers (Stop-and-Wait)
static int txFrameNumber = 0;  // 0 or 1 - frame being sent
static int rxFrameNumber = 0;  // 0 or 1 - frame expected to receive

// Statistics
static unsigned int framesSent = 0;
static unsigned int framesReceived = 0;
static unsigned int timeouts = 0;
static unsigned int retransmissions = 0;

////////////////////////////////////////////////
// ALARM HANDLER
////////////////////////////////////////////////
void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;
    timeoutOccurred = TRUE;
    timeouts++;
}

////////////////////////////////////////////////
// AUXILIARY FUNCTIONS
////////////////////////////////////////////////

// Calculate BCC1 (XOR of Address and Control fields)
unsigned char calculateBCC1(unsigned char addr, unsigned char ctrl)
{
    return addr ^ ctrl;
}

// Calculate BCC2 (XOR of all data bytes)
unsigned char calculateBCC2(const unsigned char *data, int dataSize)
{
    unsigned char bcc2 = 0;
    for (int i = 0; i < dataSize; i++)
    {
        bcc2 ^= data[i];
    }
    return bcc2;
}

// Byte stuffing: replace FLAG and ESCAPE in data
// Returns the size after stuffing
int byteStuffing(const unsigned char *input, int inputSize, unsigned char *output)
{
    int j = 0;
    for (int i = 0; i < inputSize; i++)
    {
        if (input[i] == FLAG)
        {
            output[j++] = ESCAPE;
            output[j++] = 0x5E;
        }
        else if (input[i] == ESCAPE)
        {
            output[j++] = ESCAPE;
            output[j++] = 0x5D;
        }
        else
        {
            output[j++] = input[i];
        }
    }
    return j;
}

// Byte destuffing: restore original data
// Returns the size after destuffing
int byteDestuffing(const unsigned char *input, int inputSize, unsigned char *output)
{
    int j = 0;
    for (int i = 0; i < inputSize; i++)
    {
        if (input[i] == ESCAPE)
        {
            if (i + 1 < inputSize)
            {
                if (input[i + 1] == 0x5E)
                {
                    output[j++] = FLAG;
                    i++; // Skip next byte
                }
                else if (input[i + 1] == 0x5D)
                {
                    output[j++] = ESCAPE;
                    i++; // Skip next byte
                }
                else
                {
                    // Invalid escape sequence, keep as is
                    output[j++] = input[i];
                }
            }
        }
        else
        {
            output[j++] = input[i];
        }
    }
    return j;
}

// Build supervision frame (SET, UA, DISC, RR, REJ)
int buildSupervisionFrame(unsigned char *frame, unsigned char addr, unsigned char ctrl)
{
    frame[0] = FLAG;
    frame[1] = addr;
    frame[2] = ctrl;
    frame[3] = calculateBCC1(addr, ctrl);
    frame[4] = FLAG;
    return 5;
}

// Send supervision frame
int sendSupervisionFrame(unsigned char addr, unsigned char ctrl)
{
    unsigned char frame[5];
    int frameSize = buildSupervisionFrame(frame, addr, ctrl);
    
    int bytesWritten = writeBytesSerialPort(frame, frameSize);
    if (bytesWritten != frameSize)
    {
        perror("Error writing frame");
        return -1;
    }
    
    framesSent++;
    printf("Sent frame: A=0x%02X C=0x%02X\n", addr, ctrl);
    return 0;
}

// Receive supervision frame using state machine
// Returns 0 on success, -1 on error, 1 on timeout
int receiveSupervisionFrame(unsigned char expectedAddr, unsigned char expectedCtrl)
{
    State state = START;
    unsigned char byte;
    unsigned char addr = 0;
    unsigned char ctrl = 0;
    unsigned char bcc1 = 0;
    
    while (state != STOP_STATE)
    {
        // Check if alarm was triggered
        if (timeoutOccurred)
        {
            return 1; // Timeout
        }
        
        int bytesRead = readByteSerialPort(&byte);
        
        if (bytesRead < 0)
        {
            // Check if interrupted by signal (alarm)
            if (errno == EINTR)
            {
                // Check if it was our timeout alarm
                if (timeoutOccurred)
                {
                    return 1; // Timeout occurred
                }
                // Other interruption, continue reading
                continue;
            }
            perror("Error reading byte");
            return -1;
        }
        
        if (bytesRead == 0)
        {
            continue; // No byte available, continue waiting
        }
        
        switch (state)
        {
            case START:
                if (byte == FLAG)
                {
                    state = FLAG_RCV;
                }
                break;
                
            case FLAG_RCV:
                if (byte == FLAG)
                {
                    state = FLAG_RCV;
                }
                else if (byte == expectedAddr)
                {
                    addr = byte;
                    state = A_RCV;
                }
                else
                {
                    state = START;
                }
                break;
                
            case A_RCV:
                if (byte == expectedCtrl)
                {
                    ctrl = byte;
                    state = C_RCV;
                }
                else if (byte == FLAG)
                {
                    state = FLAG_RCV;
                }
                else
                {
                    state = START;
                }
                break;
                
            case C_RCV:
                bcc1 = byte;
                if (bcc1 == calculateBCC1(addr, ctrl))
                {
                    state = BCC1_OK;
                }
                else
                {
                    printf("BCC1 error! Expected: 0x%02X, Got: 0x%02X\n", 
                           calculateBCC1(addr, ctrl), bcc1);
                    state = START;
                }
                break;
                
            case BCC1_OK:
                if (byte == FLAG)
                {
                    state = STOP_STATE;
                    framesReceived++;
                    printf("Received frame: A=0x%02X C=0x%02X\n", addr, ctrl);
                    return 0;
                }
                else
                {
                    state = START;
                }
                break;
                
            default:
                break;
        }
    }
    
    return 0;
}

// Build information frame (before stuffing)
// Returns frame size
int buildInformationFrame(unsigned char *frame, const unsigned char *data, int dataSize, int frameNum)
{
    unsigned char ctrl = (frameNum == 0) ? CTRL_I0 : CTRL_I1;
    
    frame[0] = FLAG;
    frame[1] = ADDR_TX;
    frame[2] = ctrl;
    frame[3] = calculateBCC1(ADDR_TX, ctrl);
    
    // Copy data
    memcpy(frame + 4, data, dataSize);
    
    // Calculate BCC2
    frame[4 + dataSize] = calculateBCC2(data, dataSize);
    
    frame[5 + dataSize] = FLAG;
    
    return 6 + dataSize;
}

// Send information frame with byte stuffing
int sendInformationFrame(const unsigned char *data, int dataSize, int frameNum)
{
    // Build frame (without stuffing)
    unsigned char frameBeforeStuffing[MAX_PAYLOAD_SIZE * 2 + 10];
    buildInformationFrame(frameBeforeStuffing, data, dataSize, frameNum);
    
    // Apply byte stuffing to data field + BCC2 (not to header or flags)
    unsigned char frameAfterStuffing[MAX_PAYLOAD_SIZE * 2 + 10];
    frameAfterStuffing[0] = FLAG;
    frameAfterStuffing[1] = ADDR_TX;
    frameAfterStuffing[2] = frameBeforeStuffing[2]; // Control
    frameAfterStuffing[3] = frameBeforeStuffing[3]; // BCC1
    
    // Stuff data + BCC2
    int stuffedSize = byteStuffing(frameBeforeStuffing + 4, dataSize + 1, frameAfterStuffing + 4);
    
    frameAfterStuffing[4 + stuffedSize] = FLAG;
    
    int totalSize = 5 + stuffedSize;
    
    // Send frame
    int bytesWritten = writeBytesSerialPort(frameAfterStuffing, totalSize);
    if (bytesWritten != totalSize)
    {
        perror("Error writing I frame");
        return -1;
    }
    
    framesSent++;
    printf("Sent I frame %d (%d bytes of data, %d bytes total)\n", frameNum, dataSize, totalSize);
    return 0;
}

// Receive information frame using state machine
// Returns: 0 on success, -1 on error, 1 on timeout, 2 on duplicate
int receiveInformationFrame(unsigned char *data, int *dataSize, int expectedFrameNum)
{
    State state = START;
    unsigned char byte;
    unsigned char addr = 0;
    unsigned char ctrl = 0;
    unsigned char bcc1 = 0;
    unsigned char expectedCtrl = (expectedFrameNum == 0) ? CTRL_I0 : CTRL_I1;
    
    unsigned char rawData[MAX_PAYLOAD_SIZE * 2];
    int rawDataIndex = 0;
    
    while (state != STOP_STATE)
    {
        if (timeoutOccurred)
        {
            return 1; // Timeout
        }
        
        int bytesRead = readByteSerialPort(&byte);
        
        if (bytesRead < 0)
        {
            // Check if interrupted by signal
            if (errno == EINTR)
            {
                if (timeoutOccurred)
                {
                    return 1;
                }
                continue;
            }
            perror("Error reading byte");
            return -1;
        }
        
        if (bytesRead == 0)
        {
            continue;
        }
        
        switch (state)
        {
            case START:
                if (byte == FLAG)
                {
                    state = FLAG_RCV;
                }
                break;
                
            case FLAG_RCV:
                if (byte == FLAG)
                {
                    state = FLAG_RCV;
                }
                else if (byte == ADDR_TX)
                {
                    addr = byte;
                    state = A_RCV;
                }
                else
                {
                    state = START;
                }
                break;
                
            case A_RCV:
                ctrl = byte;
                if (ctrl == expectedCtrl || ctrl == CTRL_I0 || ctrl == CTRL_I1)
                {
                    state = C_RCV;
                }
                else if (byte == FLAG)
                {
                    state = FLAG_RCV;
                }
                else
                {
                    state = START;
                }
                break;
                
            case C_RCV:
                bcc1 = byte;
                if (bcc1 == calculateBCC1(addr, ctrl))
                {
                    state = BCC1_OK;
                    rawDataIndex = 0;
                }
                else
                {
                    printf("BCC1 error in I frame\n");
                    state = START;
                }
                break;
                
            case BCC1_OK:
                if (byte == FLAG)
                {
                    // Frame complete - process data
                    if (rawDataIndex == 0)
                    {
                        printf("Empty I frame received\n");
                        state = START;
                        break;
                    }
                    
                    // Destuff data
                    unsigned char destuffedData[MAX_PAYLOAD_SIZE * 2];
                    int destuffedSize = byteDestuffing(rawData, rawDataIndex, destuffedData);
                    
                    if (destuffedSize < 1)
                    {
                        printf("Invalid frame: no data after destuffing\n");
                        state = START;
                        break;
                    }
                    
                    // Separate data and BCC2
                    unsigned char receivedBCC2 = destuffedData[destuffedSize - 1];
                    *dataSize = destuffedSize - 1;
                    
                    // Verify BCC2
                    unsigned char calculatedBCC2 = calculateBCC2(destuffedData, *dataSize);
                    
                    if (receivedBCC2 != calculatedBCC2)
                    {
                        printf("BCC2 error! Expected: 0x%02X, Got: 0x%02X\n", 
                               calculatedBCC2, receivedBCC2);
                        
                        // Check if it's the expected frame (new) or duplicate
                        if (ctrl == expectedCtrl)
                        {
                            // New frame with error - send REJ
                            return -1;
                        }
                        else
                        {
                            // Duplicate with error - send RR for next frame
                            return 2;
                        }
                    }
                    
                    // BCC2 correct - copy data
                    memcpy(data, destuffedData, *dataSize);
                    
                    framesReceived++;
                    printf("Received I frame %d (%d bytes of data)\n", 
                           (ctrl == CTRL_I0) ? 0 : 1, *dataSize);
                    
                    // Check if it's the expected frame or duplicate
                    if (ctrl == expectedCtrl)
                    {
                        return 0; // New frame, correct
                    }
                    else
                    {
                        return 2; // Duplicate frame
                    }
                }
                else
                {
                    // Accumulate data
                    if (rawDataIndex < MAX_PAYLOAD_SIZE * 2)
                    {
                        rawData[rawDataIndex++] = byte;
                    }
                    else
                    {
                        printf("Frame too large\n");
                        state = START;
                    }
                }
                break;
                
            default:
                break;
        }
    }
    
    return 0;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    connectionParams = connectionParameters;
    
    if (openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate) < 0)
    {
        perror("Error opening serial port");
        return -1;
    }
    
    printf("Serial port %s opened\n", connectionParameters.serialPort);
    
    // Configure alarm handler
    struct sigaction act = {0};
    act.sa_handler = &alarmHandler;
    if (sigaction(SIGALRM, &act, NULL) == -1)
    {
        perror("sigaction");
        return -1;
    }
    
    // Reset statistics
    framesSent = 0;
    framesReceived = 0;
    timeouts = 0;
    retransmissions = 0;
    txFrameNumber = 0;
    rxFrameNumber = 0;
    
    if (connectionParameters.role == LlTx)
    {
        printf("Opening connection as TRANSMITTER\n");
        
        alarmCount = 0;
        int attempts = 0;
        
        while (attempts < connectionParameters.nRetransmissions)
        {
            if (sendSupervisionFrame(ADDR_TX, CTRL_SET) < 0)
            {
                return -1;
            }
            
            alarmEnabled = TRUE;
            timeoutOccurred = FALSE;
            alarm(connectionParameters.timeout);
            
            int result = receiveSupervisionFrame(ADDR_TX, CTRL_UA);
            
            alarm(0);
            alarmEnabled = FALSE;
            
            if (result == 0)
            {
                printf("Connection established (took %d attempt(s))\n", attempts + 1);
                return 0;
            }
            else if (result == 1)
            {
                printf("Timeout waiting for UA, retrying...\n");
                attempts++;
            }
            else
            {
                return -1;
            }
        }
        
        printf("Failed to establish connection after %d attempts\n", 
               connectionParameters.nRetransmissions);
        return -1;
    }
    else // LlRx
    {
        printf("Opening connection as RECEIVER\n");
        
        if (receiveSupervisionFrame(ADDR_TX, CTRL_SET) < 0)
        {
            return -1;
        }
        
        if (sendSupervisionFrame(ADDR_TX, CTRL_UA) < 0)
        {
            return -1;
        }
        
        printf("Connection established\n");
        return 0;
    }
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    if (connectionParams.role != LlTx)
    {
        printf("ERROR: llwrite can only be called by transmitter\n");
        return -1;
    }
    
    if (bufSize > MAX_PAYLOAD_SIZE)
    {
        printf("ERROR: Data too large (%d bytes, max is %d)\n", bufSize, MAX_PAYLOAD_SIZE);
        return -1;
    }
    
    int attempts = 0;
    
    while (attempts < connectionParams.nRetransmissions)
    {
        // Send I frame
        if (sendInformationFrame(buf, bufSize, txFrameNumber) < 0)
        {
            return -1;
        }
        
        if (attempts > 0)
        {
            retransmissions++;
        }
        
        // Wait for response (RR or REJ)
        alarmEnabled = TRUE;
        timeoutOccurred = FALSE;
        alarm(connectionParams.timeout);
        
        unsigned char expectedRR = (txFrameNumber == 0) ? CTRL_RR1 : CTRL_RR0;
        unsigned char rejCtrl = (txFrameNumber == 0) ? CTRL_REJ0 : CTRL_REJ1;
        
        // Try to receive response
        State state = START;
        unsigned char byte;
        unsigned char addr = 0;
        unsigned char ctrl = 0;
        unsigned char bcc1 = 0;
        int responseReceived = FALSE;
        
        while (!responseReceived && !timeoutOccurred)
        {
            int bytesRead = readByteSerialPort(&byte);
            
            if (bytesRead < 0)
            {
                if (errno == EINTR)
                {
                    if (timeoutOccurred)
                    {
                        break;
                    }
                    continue;
                }
                perror("Error reading response");
                alarm(0);
                return -1;
            }
            
            if (bytesRead == 0)
            {
                continue;
            }
            
            switch (state)
            {
                case START:
                    if (byte == FLAG) state = FLAG_RCV;
                    break;
                    
                case FLAG_RCV:
                    if (byte == FLAG) state = FLAG_RCV;
                    else if (byte == ADDR_TX)
                    {
                        addr = byte;
                        state = A_RCV;
                    }
                    else state = START;
                    break;
                    
                case A_RCV:
                    ctrl = byte;
                    if (ctrl == expectedRR || ctrl == rejCtrl)
                    {
                        state = C_RCV;
                    }
                    else if (byte == FLAG) state = FLAG_RCV;
                    else state = START;
                    break;
                    
                case C_RCV:
                    bcc1 = byte;
                    if (bcc1 == calculateBCC1(addr, ctrl))
                    {
                        state = BCC1_OK;
                    }
                    else
                    {
                        printf("BCC1 error in response\n");
                        state = START;
                    }
                    break;
                    
                case BCC1_OK:
                    if (byte == FLAG)
                    {
                        responseReceived = TRUE;
                        alarm(0);
                        
                        if (ctrl == expectedRR)
                        {
                            printf("Received RR%d - frame accepted\n", (txFrameNumber == 0) ? 1 : 0);
                            txFrameNumber = 1 - txFrameNumber; // Toggle frame number
                            return bufSize;
                        }
                        else if (ctrl == rejCtrl)
                        {
                            printf("Received REJ%d - retransmitting\n", txFrameNumber);
                            attempts++;
                            break;
                        }
                    }
                    else
                    {
                        state = START;
                    }
                    break;
                    
                default:
                    break;
            }
        }
        
        alarm(0);
        
        if (timeoutOccurred)
        {
            printf("Timeout waiting for response, retrying...\n");
            attempts++;
        }
    }
    
    printf("Failed to send frame after %d attempts\n", connectionParams.nRetransmissions);
    return -1;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    if (connectionParams.role != LlRx)
    {
        printf("ERROR: llread can only be called by receiver\n");
        return -1;
    }
    
    int dataSize = 0;
    int result = receiveInformationFrame(packet, &dataSize, rxFrameNumber);
    
    if (result == 0)
    {
        // New frame received correctly
        unsigned char rrCtrl = (rxFrameNumber == 0) ? CTRL_RR1 : CTRL_RR0;
        sendSupervisionFrame(ADDR_TX, rrCtrl);
        rxFrameNumber = 1 - rxFrameNumber; // Toggle frame number
        return dataSize;
    }
    else if (result == 2)
    {
        // Duplicate frame - send RR for next frame
        printf("Duplicate frame detected, sending RR\n");
        unsigned char rrCtrl = (rxFrameNumber == 0) ? CTRL_RR1 : CTRL_RR0;
        sendSupervisionFrame(ADDR_TX, rrCtrl);
        return 0; // Don't process duplicate
    }
    else if (result == -1)
    {
        // Error in frame (BCC2 error) - send REJ
        printf("Frame error, sending REJ\n");
        unsigned char rejCtrl = (rxFrameNumber == 0) ? CTRL_REJ0 : CTRL_REJ1;
        sendSupervisionFrame(ADDR_TX, rejCtrl);
        return -1;
    }
    
    return -1;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose()
{
    if (connectionParams.role == LlTx)
    {
        printf("Closing connection as TRANSMITTER\n");
        
        alarmCount = 0;
        int attempts = 0;
        
        while (attempts < connectionParams.nRetransmissions)
        {
            if (sendSupervisionFrame(ADDR_TX, CTRL_DISC) < 0)
            {
                return -1;
            }
            
            alarmEnabled = TRUE;
            timeoutOccurred = FALSE;
            alarm(connectionParams.timeout);
            
            int result = receiveSupervisionFrame(ADDR_RX, CTRL_DISC);
            
            alarm(0);
            alarmEnabled = FALSE;
            
            if (result == 0)
            {
                if (sendSupervisionFrame(ADDR_RX, CTRL_UA) < 0)
                {
                    return -1;
                }
                
                printf("Connection closed\n");
                break;
            }
            else if (result == 1)
            {
                printf("Timeout waiting for DISC, retrying...\n");
                attempts++;
            }
            else
            {
                return -1;
            }
        }
    }
    else // LlRx
    {
        printf("Closing connection as RECEIVER\n");
        
        if (receiveSupervisionFrame(ADDR_TX, CTRL_DISC) < 0)
        {
            return -1;
        }
        
        if (sendSupervisionFrame(ADDR_RX, CTRL_DISC) < 0)
        {
            return -1;
        }
        
        if (receiveSupervisionFrame(ADDR_RX, CTRL_UA) < 0)
        {
            return -1;
        }
        
        printf("Connection closed\n");
    }
    
    // Print statistics
    printf("\n=== STATISTICS ===\n");
    printf("Frames sent: %u\n", framesSent);
    printf("Frames received: %u\n", framesReceived);
    printf("Timeouts: %u\n", timeouts);
    printf("Retransmissions: %u\n", retransmissions);
    printf("==================\n\n");
    
    if (closeSerialPort() < 0)
    {
        perror("Error closing serial port");
        return -1;
    }
    
    return 0;
}