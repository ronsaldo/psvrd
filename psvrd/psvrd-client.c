#include <psvrd-client.h>

#include <stdlib.h>
#include <stdio.h>

#include <sys/socket.h>
#include <sys/un.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <poll.h>

struct psvrd_client_connection_s
{
    int fd;
    int sensorStateSharedBufferFD;
    psvrd_sensor_state_shared_buffer_t *sensorStateSharedBuffer;

    psvrd_sequence_t nextSendSequence;
};

psvrd_client_connection_t *
psvrd_client_openConnection(void)
{
    /* Create the socket. */
    int socketHandle = socket(AF_UNIX, SOCK_SEQPACKET, 0);
    if(socketHandle < 0)
       return NULL;

    /* Connect to the server. */
    struct sockaddr_un address;
    memset(&address, 0, sizeof(address));
    address.sun_family = AF_UNIX;
    strcpy(address.sun_path, PSVRD_SOCKET_LOCATION);
    int error = connect(socketHandle, (const struct sockaddr *)&address, sizeof(address));
    if(error)
        return NULL;

    /* Open the shmem file */
    int sensorStateFD = shm_open(PSVRD_SENSOR_STATE_SHMNAME, O_RDONLY, 0);
    if(sensorStateFD < 0)
    {
        close(socketHandle);
        return NULL;
    }

    /* Map the shmem buffer */
    psvrd_sensor_state_shared_buffer_t *sensorStateBuffer = mmap(NULL, sizeof(psvrd_sensor_state_shared_buffer_t), PROT_READ, MAP_SHARED, sensorStateFD, 0);
    if(sensorStateBuffer == MAP_FAILED)
    {
        close(sensorStateFD);
        close(socketHandle);
        return NULL;
    }

    /* Create the connection object. */
    psvrd_client_connection_t *connection = malloc(sizeof(psvrd_client_connection_t));
    memset(connection, 0, sizeof(psvrd_client_connection_t));
    connection->fd = socketHandle;
    connection->sensorStateSharedBufferFD = sensorStateFD;
    connection->sensorStateSharedBuffer = sensorStateBuffer;
    return connection;
}

void
psvrd_client_closeConnection(psvrd_client_connection_t *connection)
{
    if(!connection)
        return;

    munmap(connection->sensorStateSharedBuffer, sizeof(psvrd_client_connection_t));
    close(connection->sensorStateSharedBufferFD);
    close(connection->fd);
    free(connection);
}

uint32_t
psvrd_client_newSendSequence(psvrd_client_connection_t *connection)
{
    if(!connection)
        return PSVRD_CLIENT_ERROR_INVALID_CONNECTION;
    return connection->nextSendSequence++;
}

psvrd_client_error_t
psvrd_client_submitMessage(psvrd_client_connection_t *connection, psvrd_message_header_t *header)
{
    if(!connection)
        return PSVRD_CLIENT_ERROR_INVALID_CONNECTION;

    ssize_t n;
    do
    {
        n = send(connection->fd, header, header->length, 0);
    } while(n < 0 && errno == EINTR);

    if(n < 0)
    {
        return PSVRD_CLIENT_ERROR;
    }

    return PSVRD_CLIENT_OK;
}

psvrd_client_error_t
psvrd_client_waitMessage(psvrd_client_connection_t *connection, psvrd_generic_message_t *buffer)
{
    if(!connection)
        return PSVRD_CLIENT_ERROR_INVALID_CONNECTION;

    ssize_t n;
    do
    {
        n = recv(connection->fd, buffer, sizeof(psvrd_generic_message_t), 0);
    } while(n < 0 && errno == EINTR);

    if(n < 0)
    {
        return PSVRD_CLIENT_ERROR;
    }

    if(buffer->header.length != n)
        return PSVRD_CLIENT_ERROR_INCOMPLETE_MESSAGE;
    return PSVRD_CLIENT_OK;
}

psvrd_client_error_t
psvrd_client_pollMessage(psvrd_client_connection_t *connection, psvrd_generic_message_t *buffer)
{
    if(!connection)
        return PSVRD_CLIENT_ERROR_INVALID_CONNECTION;

    struct pollfd p;
    p.events = POLLIN;
    p.revents = 0;
    p.fd = connection->fd;

    int res = poll(&p, 1, 0);
    if(res > 0)
        return psvrd_client_waitMessage(connection, buffer);
    else if(res < 0)
        perror("Failed to poll a message");
    return PSVRD_CLIENT_ERROR_NO_MESSAGE;
}

psvrd_client_error_t
psvrd_client_waitMessageOfType(psvrd_client_connection_t *connection, psvrd_message_type_t type, psvrd_generic_message_t *buffer)
{
    if(!connection)
        return PSVRD_CLIENT_ERROR_INVALID_CONNECTION;

    psvrd_client_error_t error = PSVRD_CLIENT_OK;
    while((error = psvrd_client_waitMessage(connection, buffer)) == PSVRD_CLIENT_OK)
    {
        if(buffer->header.type == type)
            return PSVRD_CLIENT_OK;
    }

    return error;
}

psvrd_client_error_t
psvrd_client_waitMessageResponseTo(psvrd_client_connection_t *connection, psvrd_sequence_t requestNumber, psvrd_generic_message_t *buffer)
{
    if(!connection)
        return PSVRD_CLIENT_ERROR_INVALID_CONNECTION;

    psvrd_client_error_t error = PSVRD_CLIENT_OK;
    while((error = psvrd_client_waitMessage(connection, buffer)) == PSVRD_CLIENT_OK)
    {
        if((buffer->header.flags & PSVRD_MESSAGE_FLAG_RESPONSE) && buffer->header.requestSequence == requestNumber)
            return PSVRD_CLIENT_OK;
    }

    return error;
}

psvrd_response_code_t
psvrd_client_waitMessageResponseCode(psvrd_client_connection_t *connection, psvrd_sequence_t requestNumber)
{
    if(!connection)
        return PSVRD_RESPONSE_ERROR;

    psvrd_generic_message_t buffer;
    psvrd_client_error_t error = psvrd_client_waitMessageResponseTo(connection, requestNumber, &buffer);
    if(error)
        return PSVRD_RESPONSE_ERROR;

    if(buffer.header.type == PSVRD_MESSAGE_RESPONSE_CODE && buffer.header.length >= sizeof(psvrd_message_response_t))
    {
        psvrd_message_response_t *response = (psvrd_message_response_t *)&buffer;
        return response->code;
    }

    return PSVRD_RESPONSE_ERROR;
}

psvrd_response_code_t
psvrd_client_sendSimpleCommand(psvrd_client_connection_t *connection, psvrd_message_type_t command)
{
    if(!connection)
        return PSVRD_RESPONSE_ERROR;

    psvrd_message_header_t header;
    memset(&header, 0, sizeof(header));
    header.type = command;
    header.length = sizeof(header);
    header.sequence = psvrd_client_newSendSequence(connection);
    psvrd_client_submitMessage(connection, &header);

    return psvrd_client_waitMessageResponseCode(connection, header.sequence);
}

/* Some simple commands. */
psvrd_response_code_t
psvrd_client_recenter(psvrd_client_connection_t *connection)
{
    return psvrd_client_sendSimpleCommand(connection, PSVRD_MESSAGE_RECENTER);
}

psvrd_response_code_t
psvrd_client_calibrateSensors(psvrd_client_connection_t *connection)
{
    return psvrd_client_sendSimpleCommand(connection, PSVRD_MESSAGE_CALIBRATE_SENSORS);
}

psvrd_response_code_t
psvrd_client_headsetOn(psvrd_client_connection_t *connection)
{
    return psvrd_client_sendSimpleCommand(connection, PSVRD_MESSAGE_HEADSET_ON);
}

psvrd_response_code_t
psvrd_client_headsetOff(psvrd_client_connection_t *connection)
{
    return psvrd_client_sendSimpleCommand(connection, PSVRD_MESSAGE_HEADSET_OFF);
}

psvrd_response_code_t
psvrd_client_headsetPower(psvrd_client_connection_t *connection, int value)
{
    if(value)
        return psvrd_client_headsetOn(connection);
    else
        return psvrd_client_headsetOff(connection);
}

psvrd_response_code_t
psvrd_client_enterVRMode(psvrd_client_connection_t *connection)
{
    return psvrd_client_sendSimpleCommand(connection, PSVRD_MESSAGE_ACTIVATE_VR_MODE);
}

psvrd_response_code_t
psvrd_client_enterCinematicMode(psvrd_client_connection_t *connection)
{
    return psvrd_client_sendSimpleCommand(connection, PSVRD_MESSAGE_CINEMATIC_MODE);
}

psvrd_response_code_t
psvrd_client_powerOff(psvrd_client_connection_t *connection)
{
    return psvrd_client_sendSimpleCommand(connection, PSVRD_MESSAGE_POWER_OFF);
}

/* Sensor polling. */
psvrd_client_error_t
psvrd_client_getCurrentSensorState(psvrd_client_connection_t *connection, psvrd_client_sensor_state_t *sensorState)
{
    if(!connection)
        return PSVRD_CLIENT_ERROR_INVALID_CONNECTION;

    volatile uint32_t sensorStateIndex = connection->sensorStateSharedBuffer->lastIntegratedSensorStateIndex.value & PSVRD_INTEGRATED_SENSOR_STATE_INDEX_MASK;
    psvrd_integrated_sensor_state_t state = connection->sensorStateSharedBuffer->integratedSensorStates[sensorStateIndex];
    sensorState->orientation = state.orientation;
    sensorState->translation = state.translation;
    return PSVRD_CLIENT_OK;
}
