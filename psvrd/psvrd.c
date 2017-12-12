#include <psvrd.h>
#include "message_queue.h"
#include "MadgwickAHRS.h"
#include "MahonyAHRS.h"
#include "vmath.h"

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libusb.h>

#include <fcntl.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/epoll.h>

#include <assert.h>

#define PSVRD_CALIBRATION_SAMPLES 3000
#define PSVRD_UP_VECTOR gravityVector
/*#define PSVRD_UP_VECTOR psvrd_vector3_new(0, 1, 0)*/

#define PSVRD_COMPLEMENTARY_FILTER_ALPHA 0.001

#define PSVR_VENDOR_ID 0x054c
#define PSVR_PRODUCT_ID 0x09af
#define PSVR_CONTROL_INTERFACE 5
#define PSVR_SENSOR_INTERFACE 4

#define PSVR_SENSOR_READ_ENDPOINT 0x83
#define PSVR_CONTROL_READ_ENDPOINT 0x84
#define PSVR_CONTROL_WRITE_ENDPOINT 0x4

#define PSVR_REGISTER_TRACKING 0x11
#define PSVR_REGISTER_POWEROFF 0x13
#define PSVR_REGISTER_HEADSET_ON 0x17
#define PSVR_REGISTER_VRMODE_ON 0x23

#define MAX_NUMBER_OF_CLIENTS 128

#define ACCELERATION_CONVERTION_FACTOR (2.0/2048.0)
#define GYROSCOPE_CONVERTION_FACTOR ((1998.0/(1<<15)) * (M_PI/180.0))

typedef struct __attribute__((packed)) psvr_sensor_state_s
{
    uint32_t timestamp; /* Most significant byte is not used.*/

    int16_t yaw;
    int16_t pitch;
    int16_t roll;

    int16_t ax;
    int16_t ay;
    int16_t az;
} psvr_sensor_state_t;

/**
 * Message structure obtained from PSVRFramework documentation: https://github.com/gusmanb/PSVRFramework
 */
typedef struct __attribute__((packed)) psvr_sensor_message_s
{
    uint8_t buttons;
    uint8_t unknown_b1;
    uint8_t volume;
    uint8_t unknown_b3;

    uint8_t unknown_b4;
    uint8_t unknown_b5;
    uint8_t unknown_b6;
    uint8_t unknown_b7;

    uint8_t statusMask;
    uint8_t unknown_b9;
    uint8_t unknown_b10;
    uint8_t unknown_b11;
    uint8_t unknown_b12;
    uint8_t unknown_b13;
    uint8_t unknown_b14;
    uint8_t unknown_b15;

    /* Two different timepoints of the sensors.*/
    psvr_sensor_state_t sensorState1;
    psvr_sensor_state_t sensorState2;

    uint8_t calibrationStatus;
    uint8_t sensorReady;
    uint8_t unknown_b50;
    uint8_t unknown_b51;
    uint8_t unknown_b52;
    uint8_t voltageReference;
    uint8_t voltageValue;
    uint16_t infraredSensor;
    uint8_t unknown_b57;
    uint8_t unknown_b58;
    uint8_t unknown_b59;
    uint8_t unknown_b60;
    uint16_t samplingPeriod;
    uint8_t packetSequence;
} psvr_sensor_message_t;

typedef struct psvr_control_message_header_s
{
    uint8_t reg;
    uint8_t status;
    uint8_t magic; /*0xAA*/
    uint8_t length;
} psvr_control_message_header_t;

typedef union psvr_control_message_s
{
    psvr_control_message_header_t header;

    struct {
        psvr_control_message_header_t header;
        uint8_t data[60];
    } generic;

    struct {
        psvr_control_message_header_t header;
        uint32_t value;
    } boolean;

    uint8_t rawData[64];
} psvr_control_message_t;

typedef struct psvrd_client_state_s
{
    int isValid;
    int fd;
    int sensorStreamEnabled;
    int writePolling;
    uint32_t epollID;
    uint32_t sendSequence;
    psvrd_message_queue_t messageSendQueue;
} psvrd_client_state_t;

static int socketHandle;

static libusb_context *context;
static libusb_device_handle *psvrDeviceHandle;

static int quitting = 0;

static struct libusb_transfer *controlReadTransfer;
static unsigned char controlReadBuffer[1024];

static struct libusb_transfer *sensorReadTransfer;
static unsigned char sensorReadBuffer[64];
static psvrd_sensor_state_t currentSensorState;
static psvrd_quaternion_t centerOrientation;
static psvrd_quaternion_t currentOrientation;
static psvrd_quaternion_t currentIntegrationOrientation;

static uint32_t lastIntegrationTimestamp;
static uint32_t integrationSamples;

static psvrd_quaternion_t gyroscopeBias;
static psvrd_vector3_t accelerometerBias;
static psvrd_vector3_t gravityVector;

static int epollHandle;

static psvrd_client_state_t clientStates[MAX_NUMBER_OF_CLIENTS];

static void broadcastNewSensorState(void);

static const psvr_control_message_t PSVREnableVRTrackingMessage = {
    .generic = {
        .header = {
            .reg = 0x11,
            .magic = 0xAA,
            .length = 8
        },
        .data = {0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00}
    }
};

static const psvr_control_message_t PSVRHeadsetOn = {
    .boolean = {
        .header = {
            .reg = PSVR_REGISTER_HEADSET_ON,
            .magic = 0xAA,
            .length = 4
        },
        .value = 1
    }
};

static const psvr_control_message_t PSVRHeadsetOff = {
    .boolean = {
        .header = {
            .reg = PSVR_REGISTER_HEADSET_ON,
            .magic = 0xAA,
            .length = 4
        },
        .value = 0
    }
};

static const psvr_control_message_t PSVREnterVRMode = {
    .boolean = {
        .header = {
            .reg = PSVR_REGISTER_VRMODE_ON,
            .magic = 0xAA,
            .length = 4
        },
        .value = 1
    }
};

static const psvr_control_message_t PSVRLeaveVRMode = {
    .boolean = {
        .header = {
            .reg = PSVR_REGISTER_VRMODE_ON,
            .magic = 0xAA,
            .length = 4
        },
        .value = 0
    }
};

static const psvr_control_message_t PSVRPowerOff = {
    .boolean = {
        .header = {
            .reg = PSVR_REGISTER_POWEROFF,
            .magic = 0xAA,
            .length = 4
        },
        .value = 1
    }
};

/*static void printHexDump(const uint8_t *data, size_t len)
{
    while(len > 0)
    {
        size_t columns = len;
        if(columns > 20)
            columns = 20;
        len -= columns;

        for(size_t i = 0; i < columns; ++i)
            printf("0x%02X ", *data++);
        printf("\n");
    }
}
*/

static void
printSensorState(psvr_sensor_state_t *state)
{
    printf("%08X %04d %04d %04d - %04d %04d %04d\n",
        state->timestamp,
        state->yaw, state->pitch, state->roll, state->ax>>4, state->ay>>4, state->az>>4);
}

static void
printSensorMessage(psvr_sensor_message_t *message)
{
    printf("Sensor message %zu, %zu\n", sizeof(psvr_sensor_message_t), sizeof(psvr_sensor_state_t));
    printSensorState(&message->sensorState1);
    printSensorState(&message->sensorState2);
}

static void
controlReadCallback(struct libusb_transfer *transfer)
{
    if(transfer->status == LIBUSB_TRANSFER_COMPLETED)
    {
        psvr_control_message_t *message = (psvr_control_message_t*)transfer->callback;

        printf("Control message received: reg %02X status %02X length %d\n", message->header.reg, message->header.status, message->header.length);
        //printHexDump(transfer->buffer, transfer->actual_length);
        libusb_submit_transfer(transfer);
    }
    else if(transfer->status == LIBUSB_TRANSFER_NO_DEVICE)
    {
        libusb_free_transfer(transfer);
    }
    else
    {
        fprintf(stderr, "Failed to read usb control data: %s\n", libusb_error_name(transfer->status));
    }
}

static void
controlWriteCallback(struct libusb_transfer *transfer)
{
    free(transfer->buffer);
    libusb_free_transfer(transfer);
}

static psvrd_response_code_t
sendControlMessage(const psvr_control_message_t *message)
{
    if(!psvrDeviceHandle)
        return PSVRD_RESPONSE_ERROR_NO_HEADSET;

    size_t bufferSize = message->generic.header.length + sizeof(psvr_control_message_header_t);
    unsigned char *buffer = malloc(bufferSize);
    memcpy(buffer, message, bufferSize);

    struct libusb_transfer *transfer = libusb_alloc_transfer(0);
    libusb_fill_bulk_transfer(transfer, psvrDeviceHandle, PSVR_CONTROL_WRITE_ENDPOINT, buffer, bufferSize, controlWriteCallback, NULL, 0);
    libusb_submit_transfer(transfer);

    return PSVRD_RESPONSE_OK;
}

static psvrd_vector3_t
rotateSensorCoordinates(psvrd_vector3_t v)
{
    return psvrd_vector3_new(v.y, v.x, -v.z);
}

static psvrd_vector3_t
rotateForMadgwick(psvrd_vector3_t v)
{
    return psvrd_vector3_new(v.x, -v.z, v.y);
}

static psvrd_quaternion_t
rotateInverseForMadgwick(psvrd_quaternion_t q)
{
    return psvrd_quaternion_new(q.x, q.z, -q.y, q.w);
}

static psvrd_vector3_t
convertAccelerometerVector3(psvr_sensor_state_t *state)
{
    return rotateSensorCoordinates(psvrd_vector3_new(
        (state->ax >> 4) * ACCELERATION_CONVERTION_FACTOR,
        (state->ay >> 4) * ACCELERATION_CONVERTION_FACTOR,
        (state->az >> 4) * ACCELERATION_CONVERTION_FACTOR));
}

static psvrd_vector3_t
convertGyroscopeVector3(psvr_sensor_state_t *state)
{
    return rotateSensorCoordinates(psvrd_vector3_new(
        state->yaw * GYROSCOPE_CONVERTION_FACTOR,
        state->pitch * GYROSCOPE_CONVERTION_FACTOR,
        state->roll * GYROSCOPE_CONVERTION_FACTOR));
}

static void
storeRawSensorState(psvr_sensor_state_t *state, psvrd_raw_sensor_state_t *messageState)
{
    messageState->milliseconds = state->timestamp;
    messageState->accelerometer = convertAccelerometerVector3(state);
    messageState->gyroscope = convertGyroscopeVector3(state);
}

static void
resetSensorState(void)
{
    currentOrientation = psvrd_quaternion_unit();
    centerOrientation = psvrd_quaternion_unit();
    integrationSamples = 0;
}

static int
computeSensorDeltaTime(uint32_t newTimeStamp, uint32_t oldTimeStamp)
{
    int result = newTimeStamp - lastIntegrationTimestamp;
    /* Handle the case for overflow*/
    if(result < 0)
        result = newTimeStamp + 0x01000000 - lastIntegrationTimestamp;
    return result;
}

static void
processCalibrationSensorSample(psvr_sensor_state_t *state)
{
    /* For the first sample, only store the timestamp. */
    if(integrationSamples == 0)
    {
        integrationSamples = 1;
        lastIntegrationTimestamp = state->timestamp;
        gyroscopeBias = psvrd_quaternion_new(0, 0, 0, 0);
        accelerometerBias = psvrd_vector3_new(0, 0, 0);
        return;
    }

    psvrd_vector3_t accelerometer = convertAccelerometerVector3(state);
    /*printf("Acceloremeter: %f %f %f\n", accelerometer.x, accelerometer.y, accelerometer.z);*/

    gyroscopeBias = psvrd_quaternion_add(gyroscopeBias, psvrd_quaternion_newv(0, convertGyroscopeVector3(state)));
    accelerometerBias = psvrd_vector3_add(accelerometerBias, accelerometer);
    ++integrationSamples;

    if(integrationSamples <= PSVRD_CALIBRATION_SAMPLES)
        return;

    /* Normalize the result for the bias. */
    gyroscopeBias = psvrd_quaternion_divs(gyroscopeBias, PSVRD_CALIBRATION_SAMPLES);
    accelerometerBias = psvrd_vector3_divs(accelerometerBias, PSVRD_CALIBRATION_SAMPLES);

    gravityVector = psvrd_vector3_normalizeNonZero(accelerometerBias);
    accelerometerBias = psvrd_vector3_sub(accelerometerBias, gravityVector);
    printf("Gravity vector: %f %f %f\n",
        gravityVector.x, gravityVector.y, gravityVector.z);

    currentOrientation = currentIntegrationOrientation = psvrd_quaternion_unit();
    centerOrientation = psvrd_quaternion_inverse(currentOrientation);

    printf("Gyro bias: %f %f %f, Accel bias: %f %f %f \n",
        gyroscopeBias.x, gyroscopeBias.y, gyroscopeBias.z,
        accelerometerBias.x, accelerometerBias.y, accelerometerBias.z
    );
}

static void
integrateSensorState(psvr_sensor_state_t *state)
{
    /* We need a first time stamp*/
    if(integrationSamples <= PSVRD_CALIBRATION_SAMPLES)
    {
        processCalibrationSensorSample(state);
        return;
    }

    /* Compute the delta and handle case of the overflow the overflow */
    int deltaMicroseconds = computeSensorDeltaTime(state->timestamp, lastIntegrationTimestamp);
    lastIntegrationTimestamp = state->timestamp;
    psvrd_scalar_t delta = deltaMicroseconds* ((psvrd_scalar_t)1e-6);

    psvrd_vector3_t gyro = rotateForMadgwick(convertGyroscopeVector3(state));
    psvrd_vector3_t accel = rotateForMadgwick(convertAccelerometerVector3(state));

    //MadgwickAHRSupdateIMU(gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z, delta, MADGWICK_AHRS_DEFAULT_BETA, &currentIntegrationOrientation);
    MahonyAHRSupdateIMU(gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z, delta, MAHONY_TWOKP_DEFAULT, MAHONY_TWOKI_DEFAULT, &currentIntegrationOrientation);

    currentOrientation = rotateInverseForMadgwick(currentIntegrationOrientation);
    currentSensorState.orientation = psvrd_quaternion_mul(centerOrientation, currentOrientation);
}

static void
processSensorData(psvr_sensor_message_t *sensorMessage)
{
    currentSensorState.rawSensorStateCount = 2;
    storeRawSensorState(&sensorMessage->sensorState1, &currentSensorState.rawSensorStates[0]);
    storeRawSensorState(&sensorMessage->sensorState2, &currentSensorState.rawSensorStates[1]);

    integrateSensorState(&sensorMessage->sensorState1);
    integrateSensorState(&sensorMessage->sensorState2);
    //printHexDump(transfer->buffer, transfer->actual_length);
    //printSensorMessage();
    broadcastNewSensorState();
}

static void
sensorReadCallback(struct libusb_transfer *transfer)
{
    if(transfer->status == LIBUSB_TRANSFER_COMPLETED)
    {
        processSensorData((psvr_sensor_message_t*)transfer->buffer);
        libusb_submit_transfer(transfer);
    }
    else if(transfer->status == LIBUSB_TRANSFER_NO_DEVICE)
    {
        libusb_free_transfer(transfer);
    }
    else
    {
        fprintf(stderr, "Failed to read usb sensor data: %s\n", libusb_error_name(transfer->status));
    }
}

static int
openPSVRDevice(struct libusb_device *psvrDevice)
{
    /* Open the PSVR device*/
    int error = libusb_open(psvrDevice, &psvrDeviceHandle);
    if(error)
    {
        fprintf(stderr, "Failed to open the PS VR device: %s\n", libusb_error_name(error));
        return 1;
    }
    libusb_unref_device(psvrDevice);

    /* Claim the interesting interfaces*/
    if(libusb_kernel_driver_active(psvrDeviceHandle, PSVR_CONTROL_INTERFACE))
        libusb_detach_kernel_driver(psvrDeviceHandle, PSVR_CONTROL_INTERFACE);
    if(libusb_kernel_driver_active(psvrDeviceHandle, PSVR_SENSOR_INTERFACE))
        libusb_detach_kernel_driver(psvrDeviceHandle, PSVR_SENSOR_INTERFACE);

    error = libusb_claim_interface(psvrDeviceHandle, PSVR_CONTROL_INTERFACE);
    if(error)
    {
        fprintf(stderr, "Failed to claim the PS VR control interface: %s\n", libusb_error_name(error));
        return 1;
    }

    error = libusb_claim_interface(psvrDeviceHandle, PSVR_SENSOR_INTERFACE);
    if(error)
    {
        fprintf(stderr, "Failed to claim the PS VR sensor interface: %s\n", libusb_error_name(error));
        return 1;
    }

    /* Setup the control read transfer. */
    {
        controlReadTransfer = libusb_alloc_transfer(0);
        libusb_fill_bulk_transfer(controlReadTransfer, psvrDeviceHandle, PSVR_CONTROL_READ_ENDPOINT, controlReadBuffer, sizeof(controlReadBuffer), controlReadCallback, NULL, 0);
        libusb_submit_transfer(controlReadTransfer);
    }

    /* Setup the sensor read transfer. */
    {
        sensorReadTransfer = libusb_alloc_transfer(0);
        libusb_fill_bulk_transfer(sensorReadTransfer, psvrDeviceHandle, PSVR_SENSOR_READ_ENDPOINT, sensorReadBuffer, sizeof(sensorReadBuffer), sensorReadCallback, NULL, 0);
        libusb_submit_transfer(sensorReadTransfer);
    }

    return 0;
}

static int
hotplugCallback(libusb_context *context, libusb_device *device, libusb_hotplug_event event, void *userdata)
{
    struct libusb_device_descriptor descriptor;
    int error = libusb_get_device_descriptor(device, &descriptor);
    if(error)
    {
        return 0;
    }

    switch(event)
    {
    case LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED:
        //printf("Device arrived [%d]%04x:%04x\n", descriptor.iSerialNumber, descriptor.idVendor, descriptor.idProduct);
        openPSVRDevice(device);
        break;
    case LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT:
        //printf("Device left [%d]%04x:%04x\n", descriptor.iSerialNumber, descriptor.idVendor, descriptor.idProduct);
        if(psvrDeviceHandle)
        {
            libusb_close(psvrDeviceHandle);
            psvrDeviceHandle = NULL;
        }
        break;
    default:
        break;
    }
    return 0;
}

static int
initializeLibUSB(void)
{
    /* Initialize libusb. */
    int error = libusb_init(&context);
    if(error < 0)
    {
        fprintf(stderr, "Failed to initialize libusb.\n");
        return 1;
    }

    /* Check for the hotplug capability */
    if(!libusb_has_capability(LIBUSB_CAP_HAS_HOTPLUG))
    {
        fprintf(stderr, "Missing the required hotplug capability in libusb.\n");
        return 1;
    }

    /* Register the hotplug callback. */
    error = libusb_hotplug_register_callback(context, LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED | LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT,
        LIBUSB_HOTPLUG_ENUMERATE, PSVR_VENDOR_ID, PSVR_PRODUCT_ID, LIBUSB_HOTPLUG_MATCH_ANY,
        hotplugCallback, NULL, NULL);
    if(error)
    {
        fprintf(stderr, "Failed to register the hotplug callback: %s\n", libusb_error_name(error));
        return 1;
    }

    return 0;
}

static int
startSocket(void)
{
    /* Create the socket. */
    socketHandle = socket(AF_UNIX, SOCK_SEQPACKET, 0);
    if(socketHandle < 0)
    {
        perror("Failed to create an Unix domain socket.\n");
        return 1;
    }

    /* Make the socket non-blocking. */
    fcntl(socketHandle, F_SETFL, fcntl(socketHandle, F_GETFL) | O_NONBLOCK);

    // Bind the socket
    struct sockaddr_un address;
    memset(&address, 0, sizeof(address));
    address.sun_family = AF_UNIX;
    strcpy(address.sun_path, PSVR_SOCKET_LOCATION);

    /* Unlink any previous version of the socket. */
    unlink(PSVR_SOCKET_LOCATION);
    int error = bind(socketHandle, (const struct sockaddr *)&address, sizeof(address));
    if(error < 0)
    {
        perror("Failed to bind the communication socket.\n");
        return 1;
    }

    chmod(PSVR_SOCKET_LOCATION, 0777);
    listen(socketHandle, 30);

    return 0;
}

static void
handleMainSocketEvents(uint32_t events)
{
    if(events & EPOLLIN)
    {
        struct sockaddr_un clientAddress;
        socklen_t clientAddressLen = sizeof(clientAddress);
        int clientSocket;
        do
        {
            clientSocket = accept(socketHandle, (struct sockaddr *)&clientAddress, &clientAddressLen);
        } while(clientSocket < 0 && errno == EINTR);

        if(clientSocket < 0)
        {
            if(errno != EAGAIN)
                perror("Failed to accept a client.\n");
            return;
        }

        /* Make sure the client socket is non-blocking. */
        fcntl(clientSocket, F_SETFL, fcntl(clientSocket, F_GETFL) | O_NONBLOCK);

        /* Find an invalid client state */
        int clientID;
        for(clientID = 0; clientID < MAX_NUMBER_OF_CLIENTS; ++clientID)
        {
            if(!clientStates[clientID].isValid)
                break;
        }

        /* Cannot accept more clients. */
        if(clientID == MAX_NUMBER_OF_CLIENTS)
        {
            fprintf(stderr, "Cannot accept more clients.\n");
            close(clientSocket);
            return;
        }

        clientStates[clientID].isValid = 1;
        clientStates[clientID].fd = clientSocket;
        clientStates[clientID].epollID = clientID + 2;
        {
            struct epoll_event event;
            event.data.u32 = clientStates[clientID].epollID;
            event.events = EPOLLIN | EPOLLRDHUP | EPOLLERR;
            epoll_ctl(epollHandle, EPOLL_CTL_ADD, clientSocket, &event);
        }
    }
}

static void
activateClientWritePolling(psvrd_client_state_t *clientState)
{
    if(clientState->writePolling)
        return;

    {
        struct epoll_event event;
        event.data.u32 = clientState->epollID;
        event.events = EPOLLIN | EPOLLOUT | EPOLLRDHUP | EPOLLERR;
        epoll_ctl(epollHandle, EPOLL_CTL_MOD, clientState->fd, &event);
    }
    clientState->writePolling = 1;
}

static void
disableClientWritePolling(psvrd_client_state_t *clientState)
{
    if(!clientState->writePolling)
        return;

    {
        struct epoll_event event;
        event.data.u32 = clientState->epollID;
        event.events = EPOLLIN | EPOLLRDHUP | EPOLLERR;
        epoll_ctl(epollHandle, EPOLL_CTL_MOD, clientState->fd, &event);
    }

    clientState->writePolling = 0;
}

static void
sendMessageToClient(psvrd_client_state_t *clientState, psvrd_message_header_t *messageHeader)
{
    assert(clientState->isValid);
    ssize_t n;

    /* If the queue is not empty, send the message through the queue to preserve message ordering. */
    if(clientState->writePolling)
    {
        psvrd_mq_push(&clientState->messageSendQueue, messageHeader);
        return;
    }

    do
    {
        n = send(clientState->fd, messageHeader, messageHeader->length, 0);
    } while(n < 0 && errno == EINTR);

    if(n >= 0)
        return;

    /* Do we need to enqueue the message send? */
    if(errno == EAGAIN)
    {
        psvrd_mq_push(&clientState->messageSendQueue, messageHeader);
        activateClientWritePolling(clientState);
    }
    else
    {
        if(errno != EPIPE)
            perror("Failed to send message to client.");
    }
}

static void
sendMessageResponseCode(psvrd_client_state_t *clientState, uint32_t requestSequence, psvrd_response_code_t responseCode)
{
    psvrd_message_response_t response;
    response.header.type = PSVRD_MESSAGE_RESPONSE_CODE;
    response.header.sequence = clientState->sendSequence++;
    response.header.requestSequence = requestSequence;
    response.header.length = sizeof(response);
    response.header.flags = PSVRD_MESSAGE_FLAG_RESPONSE;
    response.code = responseCode;
    sendMessageToClient(clientState, &response.header);
}

static void
sendSensorDataToClient(psvrd_client_state_t *clientState)
{
    psvrd_sensor_state_t message = currentSensorState;
    message.header.type = PSVRD_MESSAGE_SENSOR_STATE;
    message.header.sequence = clientState->sendSequence++;
    message.header.requestSequence = 0;
    message.header.length = sizeof(message);
    message.header.flags = 0;
    sendMessageToClient(clientState, &message.header);
}

static void
broadcastNewSensorState(void)
{
    for(int i = 0; i < MAX_NUMBER_OF_CLIENTS; ++i)
    {
        psvrd_client_state_t *client = &clientStates[i];
        if(client->isValid && client->sensorStreamEnabled)
            sendSensorDataToClient(client);
    }
}

static void
writePendingClientMessages(psvrd_client_state_t *clientState)
{
    psvrd_generic_message_t *nextMessage;
    ssize_t n;

    while((nextMessage = psvrd_mq_top(&clientState->messageSendQueue)))
    {
        do
        {
            n = send(clientState->fd, nextMessage, nextMessage->header.length, 0);
        } while(n < 0 && errno == EINTR);

        /* check for errors. */
        if(n < 0)
        {
            if(errno != EAGAIN && errno != EPIPE)
                perror("Failed to send to client socket.");
            return;
        }

        /* Pop the message from the queue*/
        psvrd_mq_pop(&clientState->messageSendQueue);
    }

    /* If the queue is now empty, disable the polling from the client state. */
    if(psvrd_mq_isEmpty(&clientState->messageSendQueue))
        disableClientWritePolling(clientState);
}

static int
readAndProcessClientMessage(psvrd_client_state_t *clientState)
{
    psvrd_generic_message_t genericMessage;
    ssize_t n;

    /* Read a single message from the socket*/
    do
    {
        n = recv(clientState->fd, &genericMessage, sizeof(genericMessage), 0);
    } while(n < 0 && errno == EINTR);

    /* Check for errors*/
    if(n < 0)
    {
        if(errno != EAGAIN && errno != EPIPE)
            perror("Failed to read from client socket.");

        return 0;
    }

    /* Check the size of the message */
    if(n < sizeof(psvrd_message_header_t) || n > sizeof(genericMessage) || genericMessage.header.length != n )
        return 0;

    /* Process the message. */
    psvrd_response_code_t responseCode = PSVRD_RESPONSE_ERROR;

    switch(genericMessage.header.type)
    {
    case PSVRD_MESSAGE_HEADSET_ON:
        responseCode = sendControlMessage(&PSVRHeadsetOn);
        sendMessageResponseCode(clientState, genericMessage.header.sequence, responseCode);
        break;
    case PSVRD_MESSAGE_HEADSET_OFF:
        responseCode = sendControlMessage(&PSVRHeadsetOff);
        sendMessageResponseCode(clientState, genericMessage.header.sequence, responseCode);
        break;
    case PSVRD_MESSAGE_POWER_OFF:
        responseCode = sendControlMessage(&PSVRPowerOff);
        sendMessageResponseCode(clientState, genericMessage.header.sequence, responseCode);
        break;
    case PSVRD_MESSAGE_ACTIVATE_VR_MODE:
        responseCode = sendControlMessage(&PSVREnterVRMode);
        sendMessageResponseCode(clientState, genericMessage.header.sequence, responseCode);
        break;
    case PSVRD_MESSAGE_CINEMATIC_MODE:
        responseCode = sendControlMessage(&PSVRLeaveVRMode);
        sendMessageResponseCode(clientState, genericMessage.header.sequence, responseCode);
        break;
    case PSVRD_MESSAGE_REQUEST_SENSOR_STREAM:
        responseCode = PSVRD_RESPONSE_OK;
        sendMessageResponseCode(clientState, genericMessage.header.sequence, responseCode);

        clientState->sensorStreamEnabled = 1;
        sendSensorDataToClient(clientState);
        break;
    case PSVRD_MESSAGE_RECENTER:
        centerOrientation = psvrd_quaternion_inverse(currentOrientation);
        sendMessageResponseCode(clientState, genericMessage.header.sequence, PSVRD_RESPONSE_OK);
        break;
    case PSVRD_MESSAGE_CALIBRATE_SENSORS:
        resetSensorState();
        sendMessageResponseCode(clientState, genericMessage.header.sequence, PSVRD_RESPONSE_OK);
        break;
    default:
        fprintf(stderr, "Received unknown message %08X from client fd number %u.\n", genericMessage.header.type, clientState->fd);
        break;
    }

    return 1;
}

static void
readAndProcessClientMessages(psvrd_client_state_t *clientState)
{
    while(readAndProcessClientMessage(clientState))
        ;
}

static void
handleClientSocketEvents(uint32_t clientID, uint32_t events)
{
    if(clientID >= MAX_NUMBER_OF_CLIENTS)
        return;

    psvrd_client_state_t *clientState = &clientStates[clientID];
    if(!clientState->isValid)
        return;

    /* This events means that the client has disconnected. */
    if(events & EPOLLRDHUP)
    {
        //printf("Client %d disconnected\n", clientID);
        epoll_ctl(epollHandle, EPOLL_CTL_DEL, clientState->fd, NULL);
        shutdown(clientState->fd, SHUT_RDWR);
        close(clientState->fd);
        memset(clientState, 0, sizeof(psvrd_client_state_t));
        return;
    }

    /* Did we get an error? */
    if(events & EPOLLERR)
    {
        /*TODO: Do something*/
    }

    /* Write the pending packets to the socket. */
    if(events & EPOLLOUT)
    {
        writePendingClientMessages(clientState);
    }

    /* Read from the socket. */
    if(events & EPOLLIN)
    {
        readAndProcessClientMessages(clientState);
    }
}

static void
handleSocketEvents(uint32_t socketID, uint32_t events)
{
    /* 0 means this is a libusb fd. */
    if(socketID == 0)
        return;

    if(socketID == 1)
    {
        /* Try to accept a client. */
        handleMainSocketEvents(events);
    }
    else
    {
        /* Read/write data to the socket*/
        handleClientSocketEvents(socketID - 2, events);
    }
}

static void
pollfdAdded(int fd, short events, void *userData)
{
    struct epoll_event event;
    event.data.u32 = 0;
    event.events = events;
    epoll_ctl(epollHandle, EPOLL_CTL_ADD, fd, &event);
}

static void
pollfdRemoved(int fd, void *userData)
{
    epoll_ctl(epollHandle, EPOLL_CTL_DEL, fd, NULL);
}

static void
mainLoop(void)
{
    struct epoll_event pendingEvents[64];

    epollHandle = epoll_create1(EPOLL_CLOEXEC);

    /* Add the socket to epoll. */
    {
        struct epoll_event event;
        event.data.u32 = 1;
        event.events = EPOLLIN | EPOLLERR;
        epoll_ctl(epollHandle, EPOLL_CTL_ADD, socketHandle, &event);
    }

    /* Add the libusb fds to epoll. */
    {
        const struct libusb_pollfd ** pollfds = libusb_get_pollfds(context);
        for(const struct libusb_pollfd **pos = pollfds; *pos; ++pos)
        {
            const struct libusb_pollfd *pollfd = *pos;
            struct epoll_event event;
            event.data.u32 = 0; /*0 For the libusb*/
            event.events = pollfd->events;
            epoll_ctl(epollHandle, EPOLL_CTL_ADD, pollfd->fd, &event);
        }

        libusb_free_pollfds(pollfds);
    }

    libusb_set_pollfd_notifiers(context, pollfdAdded, pollfdRemoved, NULL);

    /* Main event loop*/
    struct timeval zeroTimeout = {0, 0};
    while(!quitting)
    {
        int pendingEventCount = epoll_wait(epollHandle, pendingEvents, 64, -1);
        if(pendingEventCount < 0)
        {
            perror("epoll wait failed");
            break;
        }
        /* printf("pending event count: %d\n", pendingEventCount); */

        /* Check for libUSB events. */
        int haveLibUSBEvents = 0;
        for(int i = 0; i < pendingEventCount; ++i)
        {
            if(pendingEvents[i].data.u32 == 0)
            {
                haveLibUSBEvents = 1;
                break;
            }
        }

        if(haveLibUSBEvents)
            libusb_handle_events_timeout(context, &zeroTimeout);

        /* Handle socket events. */
        for(int i = 0; i < pendingEventCount; ++i)
        {
            if(pendingEvents[i].data.u32 != 0)
                handleSocketEvents(pendingEvents[i].data.fd, pendingEvents[i].events);
        }
    }

    /* Destroy the epoll handle. */
    libusb_set_pollfd_notifiers(context, NULL, NULL, NULL);
    close(epollHandle);
}

static int
serverMain(void)
{
    resetSensorState();

    int error = initializeLibUSB();
    if(error)
        return error;

    error = startSocket();
    if(error)
        return error;

    mainLoop();

    if(psvrDeviceHandle)
        libusb_close(psvrDeviceHandle);
    libusb_exit(context);

    return 0;
}

static void
printHelp(void)
{
    printf(
"psvrd [options]                                                             \n\
-d                                 Start as daemon.\n\
-h                                 Print help message.\n\
-v                                 Print version message\n\
"
    );
}

static void
printVersion(void)
{
}

static int
startDaemon(void)
{
    return 0;
}

int
main(int argc, char* argv[])
{
    int daemon = 0;
    for(int i = 1; i < argc; ++i)
    {
        if(!strcmp(argv[i], "-d"))
        {
            daemon = 1;
        }
        else if(!strcmp(argv[i], "-h"))
        {
            printHelp();
            return 0;
        }
        else if(!strcmp(argv[i], "-version"))
        {
            printVersion();
            return 0;
        }
    }

    if(daemon)
        return startDaemon();

    return serverMain();
}
