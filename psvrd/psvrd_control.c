#include <psvrd-client.h>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sys/epoll.h>
#include <errno.h>

static psvrd_client_connection_t *connection;

typedef int (*commandHandler_t) (int argc, const char **argv);
typedef struct command_definition_s
{
    const char *name;
    const char *helpString;
    commandHandler_t commandHandler;
} command_definition_t;


static int
headsetCommand(int argc, const char *argv[])
{
    int on = 1;
    if(argc >= 1)
    {
        if(!strcmp(argv[1], "on"))
            on = 1;
        else if(!strcmp(argv[1], "off"))
            on = 0;
    }

    return psvrd_client_headsetPower(connection, on);
}

static int
vrCommand(int argc, const char *argv[])
{
    return psvrd_client_enterVRMode(connection);
}

static int
cinematicCommand(int argc, const char *argv[])
{
    return psvrd_client_enterCinematicMode(connection);
}

static int
poweroffCommand(int argc, const char *argv[])
{
    return psvrd_client_powerOff(connection);
}

static int
recenterCommand(int argc, const char *argv[])
{
    return psvrd_client_recenter(connection);
}

static int
calibrateCommand(int argc, const char *argv[])
{
    return psvrd_client_calibrateSensors(connection);
}

static int
readsensorCommand(int argc, const char *argv[])
{
/*    for(;;)
    {
        psvrd_client_error_t clientError = psvrd_client_waitMessageOfType(connection, PSVRD_MESSAGE_SENSOR_STATE, &message);
        if(clientError)
            break;

        psvrd_sensor_state_t *state = (psvrd_sensor_state_t *)&message;
        printf("g: %6.3f %6.3f %6.3f a: %6.3f %6.3f %6.3f w: %8.5f %8.5f %8.5f %8.5f q: %8.5f %8.5f %8.5f %8.5f\r",
            state->rawSensorStates[0].gyroscope.x, state->rawSensorStates[0].gyroscope.y, state->rawSensorStates[0].gyroscope.z,
            state->rawSensorStates[0].accelerometer.x, state->rawSensorStates[0].accelerometer.y, state->rawSensorStates[0].accelerometer.z,
            state->omega.w, state->omega.x, state->omega.y, state->omega.z,
            state->orientation.w, state->orientation.x, state->orientation.y, state->orientation.z
            );
        fflush(stdout);
    }
*/
    return 0;
}

static const command_definition_t commands[] = {
    {"headset", "headset [on*|off]\n\nTurns on or off the PS VR headset.", headsetCommand},
    {"cinematic", "cinematic\n\nEnter into cinematic mode.", cinematicCommand},
    {"vr", "vr\n\nEnter into VR mode.", vrCommand},

    {"recenter", "recenter\n\nChange the current center.", recenterCommand},
    {"calibrate", "calibrate [sensors]\n\nCalibrate the sensors.", calibrateCommand},

    {"poweroff", "poweroff\n\nTurns off the PS VR processing unit box.", poweroffCommand},

    {"readsensor", "readsensor\n\nStarts reading the sensor data.", readsensorCommand},

    {NULL, NULL, NULL},
};

static void
printHelp()
{
    printf("psvrd_control [options] command [commmand options]...\n");

    const command_definition_t *pos = commands;
    for(; pos->name; ++pos)
    {
        printf("===============================================================\n%s\n", pos->helpString);
    }

}

static void
printVersion()
{
}

int
main(int argc, const char *argv[])
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
    connection = psvrd_client_openConnection();
    if(!connection)
    {
        fprintf(stderr, "Failed to connect to the psvrd daemon.\n");
        return 1;
    }

    /* Process the command. */
    command->commandHandler(commandArgc, commandArgv);

    /* Close the connection. */
    psvrd_client_closeConnection(connection);
    return 0;
}
