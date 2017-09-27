#include <psvrd.h>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libusb.h>

#include <sys/epoll.h>
#include <errno.h>

static int socketHandle;
static uint32_t nextSequenceNumber = 0;

typedef int (*commandHandler_t) (int argc, const char **argv);
typedef struct command_definition_s
{
    const char *name;
    const char *helpString;
    commandHandler_t commandHandler;
} command_definition_t;

static psvrd_response_code_t waitForResponseCode(uint32_t requestSequence)
{
    // Wait for the response
    for(;;)
    {
        psvrd_generic_message_t message;
        ssize_t n = recv(socketHandle, &message, sizeof(message), 0);
        if(n < 0)
        {
            perror("Failed to receive response message");
            return PSVRD_RESPONSE_ERROR;
        }

        if(message.header.flags & PSVRD_MESSAGE_FLAG_RESPONSE &&
            message.header.requestSequence == requestSequence)
        {
            if(message.header.type == PSVRD_MESSAGE_RESPONSE_CODE && message.header.length >= sizeof(psvrd_message_response_t))
            {
                psvrd_message_response_t *response = (psvrd_message_response_t*)&message;
                return response->code;
            }

            break;
        }
    }

    return PSVRD_RESPONSE_ERROR;
}

static int sendSimpleCommand(psvrd_message_type_t type)
{
    uint32_t requestSequence = nextSequenceNumber++;

    {
        psvrd_message_header_t header;
        header.type = type;
        header.length = sizeof(header);
        header.sequence = requestSequence;
        ssize_t n = send(socketHandle, &header, sizeof(header), 0);
        if(n < 0)
        {
            perror("Failed to send message");
            return 1;
        }
    }

    psvrd_response_code_t response = waitForResponseCode(requestSequence);
    if(response != PSVRD_RESPONSE_OK)
    {
        fprintf(stderr, "Got error response code: %d\n", response);
    }

    return response != PSVRD_RESPONSE_OK;
}

static int headsetCommand(int argc, const char *argv[])
{
    int on = 1;
    if(argc >= 1)
    {
        if(!strcmp(argv[1], "on"))
            on = 1;
        else if(!strcmp(argv[1], "off"))
            on = 0;
    }

    return sendSimpleCommand(on ? PSVRD_MESSAGE_HEADSET_ON : PSVRD_MESSAGE_HEADSET_OFF);
}

static int vrCommand(int argc, const char *argv[])
{
    return sendSimpleCommand(PSVRD_MESSAGE_ACTIVATE_VR_MODE);
}

static int cinematicCommand(int argc, const char *argv[])
{
    return sendSimpleCommand(PSVRD_MESSAGE_CINEMATIC_MODE);
}

static int poweroffCommand(int argc, const char *argv[])
{
    return sendSimpleCommand(PSVRD_MESSAGE_POWER_OFF);
}

static int readsensorCommand(int argc, const char *argv[])
{
    int error = sendSimpleCommand(PSVRD_MESSAGE_REQUEST_SENSOR_STREAM);
    if(error)
        return error;

    psvrd_generic_message_t message;

    for(;;)
    {
        ssize_t n = recv(socketHandle, &message, sizeof(message), 0);
        if(n < 0)
        {
            if(errno == EINTR)
                continue;
            break;
        }

        /* Ignore non-sensor messages */
        if(n != message.header.length || message.header.type != PSVRD_MESSAGE_SENSOR_STATE)
            continue;

        psvrd_sensor_state_t *state = (psvrd_sensor_state_t *)&message;
        printf("g: %6.1f %6.1f %6.1f a: %6.3f %6.3f %6.3f q: %0.3f %0.5f %0.5f %0.5f\r",
            state->rawSensorStates[0].gyroscope.x, state->rawSensorStates[0].gyroscope.y, state->rawSensorStates[0].gyroscope.z,
            state->rawSensorStates[0].accelerometer.x, state->rawSensorStates[0].accelerometer.y, state->rawSensorStates[0].accelerometer.z,
            state->orientation.w, state->orientation.x, state->orientation.y, state->orientation.z);
        fflush(stdout);
    }

    return 0;
}

static const command_definition_t commands[] = {
    {"headset", "headset [on*|off]\n\nTurns on or off the PS VR headset.", headsetCommand},
    {"cinematic", "cinematic\n\nEnter into cinematic mode.", cinematicCommand},
    {"vr", "vr\n\nEnter into VR mode.", vrCommand},

    {"poweroff", "poweroff\n\nTurns off the PS VR processing unit box.", poweroffCommand},

    {"readsensor", "readsensor\n\nStarts reading the sensor data.", readsensorCommand},

    {NULL, NULL, NULL},
};

static void printHelp()
{
    printf("psvrd_control [options] command [commmand options]...\n");

    const command_definition_t *pos = commands;
    for(; pos->name; ++pos)
    {
        printf("===============================================================\n%s\n", pos->helpString);
    }

}

static void printVersion()
{
}

int main(int argc, const char *argv[])
{
    int i;

    /* Parse the command line options*/
    int commandArgc = 0;
    const char **commandArgv = NULL;

    for(i = 1; i < argc; ++i)
    {
        if(argv[i][0] == '-')
        {
            if(!strcmp(argv[i], "-h"))
            {
                printHelp();
                return 0;
            }
            else if(!strcmp(argv[i], "-v"))
            {
                printVersion();
                return 0;
            }
        }
        else
        {
            commandArgc = argc - i;
            commandArgv = &argv[i];
            break;
        }
    }

    if(commandArgc == 0)
    {
        printHelp();
        return 0;
    }

    /* Find the command definiton. */
    const char *commandName = commandArgv[0];
    const command_definition_t *command = commands;
    for(; command->name; ++command)
    {
        if(!strcmp(command->name, commandName))
            break;
    }
    if(!command->name)
    {
        printHelp();
        return 0;
    }

    /* Create the socket. */
    socketHandle = socket(AF_UNIX, SOCK_SEQPACKET, 0);
    if(socketHandle < 0)
    {
       perror("Failed to create an Unix domain socket.\n");
       return 1;
    }

    /* Connect to the server. */
    struct sockaddr_un address;
    memset(&address, 0, sizeof(address));
    address.sun_family = AF_UNIX;
    strcpy(address.sun_path, PSVR_SOCKET_LOCATION);
    int error = connect(socketHandle, (const struct sockaddr *)&address, sizeof(address));
    if(error)
    {
        perror("Failed to connect to the psvrd daemon");
        return 1;
    }

    /* Process the command. */
    command->commandHandler(commandArgc, commandArgv);

    /* Close the socket. */
    shutdown(socketHandle, SHUT_RDWR);
    close(socketHandle);
    return 0;
}
