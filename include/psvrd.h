#ifndef PSVRD_H
#define PSVRD_H

#include <stdint.h>

#define PSVRD_SOCKET_LOCATION "/var/run/psvrd.socket"
#define PSVRD_SENSOR_STATE_SHMNAME "/psvrd_sensor_state"

#define PSVRD_FOURCC(a, b, c, d) ((a) | ((b) << 8) | ((c) << 16) | ((d) << 24))
#define PSVRD_MAX_MESSAGE_SIZE 512

#define PSVRD_INTEGRATED_SENSOR_STATE_BUFFER_COUNT (1<<4)
#define PSVRD_INTEGRATED_SENSOR_STATE_INDEX_MASK (PSVRD_INTEGRATED_SENSOR_STATE_BUFFER_COUNT - 1)

typedef double psvrd_scalar_t;
typedef uint32_t psvrd_sequence_t;

/* Atomic uint32 value. For sensor state buffer. */
typedef union __attribute__((aligned(64))) psvrd_atomic_uint_u
{
    volatile uint32_t value;
    uint8_t padding[64];
} psvrd_atomic_uint32_t;

typedef enum psvrd_message_type_e {
    /* Activation commands. */
    PSVRD_MESSAGE_HEADSET_ON = PSVRD_FOURCC('H', 'D', 'O', 'N'),
    PSVRD_MESSAGE_HEADSET_OFF = PSVRD_FOURCC('H', 'D', 'O', 'F'),
    PSVRD_MESSAGE_POWER_OFF = PSVRD_FOURCC('P', 'W', 'R', 'O'),
    PSVRD_MESSAGE_ACTIVATE_VR_MODE = PSVRD_FOURCC('V', 'R', 'M', 'D'),
    PSVRD_MESSAGE_CINEMATIC_MODE = PSVRD_FOURCC('C', 'N', 'M', 'D'),

    /* Calibration sensor and centering. */
    PSVRD_MESSAGE_CALIBRATE_SENSORS = PSVRD_FOURCC('C', 'A', 'L', 'S'),
    PSVRD_MESSAGE_RECENTER = PSVRD_FOURCC('R', 'C', 'N', 'T'),

    /* Message responses*/
    PSVRD_MESSAGE_RESPONSE_CODE = PSVRD_FOURCC('R', 'E', 'S', 'C'),

} psvrd_message_type_t;

typedef enum psvrd_response_code_e {
    PSVRD_RESPONSE_OK = 0,
    PSVRD_RESPONSE_ERROR,
    PSVRD_RESPONSE_ERROR_NO_HEADSET,
} psvrd_response_code_t;

#define PSVRD_MESSAGE_FLAG_RESPONSE (1<<0)

/**
 * PSVRD message header.
 */
typedef struct psvrd_message_header_s
{
    psvrd_message_type_t type;
    uint32_t length; /* Includes the size of the header. */
    psvrd_sequence_t sequence;
    psvrd_sequence_t requestSequence;

    uint32_t flags;
    uint32_t padding;
} psvrd_message_header_t;

/**
 * Quaternion.
 */
typedef struct psvrd_quaternion_s
{
    psvrd_scalar_t x, y, z, w;
} psvrd_quaternion_t;

/**
 * Vector3.
 */
typedef struct psvrd_vector3_s
{
    psvrd_scalar_t x, y, z;
} psvrd_vector3_t;

/**
 * Raw data from the sensor of the headset.
 */
typedef struct psvrd_raw_sensor_state_s
{
    psvrd_vector3_t gyroscope;
    psvrd_vector3_t accelerometer;
    uint32_t milliseconds;
    uint32_t padding;
} psvrd_raw_sensor_state_t;

/**
 * Integrated sensor state
 */
typedef struct psvrd_integrated_sensor_state_s
{
    psvrd_quaternion_t orientation;
    psvrd_quaternion_t omega;
    psvrd_vector3_t translation;
} psvrd_integrated_sensor_state_t;

/**
 * Integrated sensor state buffer.
 */
typedef struct psvrd_sensor_state_shared_buffer_s
{
    psvrd_atomic_uint32_t lastIntegratedSensorStateIndex;
    volatile psvrd_integrated_sensor_state_t integratedSensorStates[PSVRD_INTEGRATED_SENSOR_STATE_BUFFER_COUNT];
} psvrd_sensor_state_shared_buffer_t;

typedef struct psvrd_message_response_s
{
    /* The message header. */
    psvrd_message_header_t header;

    /* Ther response code. */
    psvrd_response_code_t code;
} psvrd_message_response_t;

/**
 * Generic psvrd message
 */
typedef struct psvrd_generic_message_s
{
    psvrd_message_header_t header;
    uint8_t payload[PSVRD_MAX_MESSAGE_SIZE - sizeof(psvrd_message_header_t)];
} psvrd_generic_message_t;

#endif /*PSVRD_H*/
