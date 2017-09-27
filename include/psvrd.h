#ifndef PSVRD_H
#define PSVRD_H

#include <stdint.h>

#define PSVR_SOCKET_LOCATION "/var/run/psvr.socket"

#define PSVRD_FOURCC(a, b, c, d) ((a) | ((b) << 8) | ((c) << 16) | ((d) << 24))

typedef enum psvrd_message_type_e {
    /* Activation commands. */
    PSVRD_MESSAGE_HEADSET_ON = PSVRD_FOURCC('H', 'D', 'O', 'N'),
    PSVRD_MESSAGE_HEADSET_OFF = PSVRD_FOURCC('H', 'D', 'O', 'F'),
    PSVRD_MESSAGE_SHUTDOWN = PSVRD_FOURCC('S', 'H', 'T', 'D'),
    PSVRD_MESSAGE_ACTIVATE_VR_MODE = PSVRD_FOURCC('V', 'R', 'M', 'D'),
    PSVRD_MESSAGE_CINEMATIC_MODE = PSVRD_FOURCC('C', 'N', 'M', 'D'),

    /* Periodical updates. */
    PSVRD_MESSAGE_SENSOR_STATE =  PSVRD_FOURCC('S', 'N', 'S', 'R')
} psvrd_message_type_t;

/**
 * PSVRD message header.
 */
typedef struct psvrd_message_header_s
{
    psvrd_message_type_t type;
    uint16_t length; /* Includes the size of the header. */
} psvrd_message_header_t;

/**
 * Quaternion.
 */
typedef struct psvrd_quaternion_s
{
    float x, y, z, w;
} psvrd_quaternion_t;

/**
 * Vector3.
 */
typedef struct psvrd_vector3_s
{
    float x, y, z, w;
} psvrd_vector3_t;

/**
 * Raw data from the sensor of the headset.
 */
typedef struct psvrd_raw_sensor_state_s
{
    uint64_t milliseconds;
    psvrd_vector3_t gyroscope;
    psvrd_vector3_t accelerometer;
} psvrd_raw_sensor_state_t;

/**
 * Sensor state
 */
typedef struct psvrd_sensor_state_s
{
    /* The message header. */
    psvrd_message_header_t header;

    /* Integrated sensor data. */
    psvrd_quaternion_t orientation;
    psvrd_vector3_t translation;

    /* Raw sensor data. */
    uint32_t rawSensorStateCount;
    psvrd_raw_sensor_state_t rawSensorStates[4];
} psvrd_sensor_state_t;

#endif /*PSVRD_H*/
