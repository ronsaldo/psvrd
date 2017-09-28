#ifndef PSVRD_CLIENT_H
#define PSVRD_CLIENT_H

#include <psvrd.h>

typedef enum psvrd_client_error_e {
    PSVRD_CLIENT_OK = 0,
    PSVRD_CLIENT_ERROR,
    PSVRD_CLIENT_ERROR_INVALID_CONNECTION,
    PSVRD_CLIENT_ERROR_NO_MESSAGE,
    PSVRD_CLIENT_ERROR_INCOMPLETE_MESSAGE,
} psvrd_client_error_t;

typedef struct psvrd_client_sensor_state_s
{
    psvrd_quaternion_t orientation;
    psvrd_vector3_t translation;
} psvrd_client_sensor_state_t;

typedef struct psvrd_client_connection_s psvrd_client_connection_t;

psvrd_client_connection_t *psvrd_client_openConnection(void);
void psvrd_client_closeConnection(psvrd_client_connection_t *connection);

/* Message sending. */

psvrd_sequence_t psvrd_client_newSendSequence(psvrd_client_connection_t *connection);

psvrd_client_error_t psvrd_client_submitMessage(psvrd_client_connection_t *connection, psvrd_message_header_t *header);

/* Message reception. */
psvrd_client_error_t psvrd_client_waitMessage(psvrd_client_connection_t *connection, psvrd_generic_message_t *buffer);
psvrd_client_error_t psvrd_client_pollMessage(psvrd_client_connection_t *connection, psvrd_generic_message_t *buffer);

psvrd_client_error_t psvrd_client_waitMessageOfType(psvrd_client_connection_t *connection, psvrd_message_type_t type, psvrd_generic_message_t *buffer);
psvrd_client_error_t psvrd_client_waitMessageResponseTo(psvrd_client_connection_t *connection, psvrd_sequence_t requestNumber, psvrd_generic_message_t *buffer);
psvrd_response_code_t psvrd_client_waitMessageResponseCode(psvrd_client_connection_t *connection, psvrd_sequence_t requestNumber);

/* Simple commands */
psvrd_response_code_t psvrd_client_sendSimpleCommand(psvrd_client_connection_t *connection, psvrd_message_type_t command);

psvrd_response_code_t psvrd_client_recenter(psvrd_client_connection_t *connection);
psvrd_response_code_t psvrd_client_calibrateSensors(psvrd_client_connection_t *connection);
psvrd_response_code_t psvrd_client_headsetOn(psvrd_client_connection_t *connection);
psvrd_response_code_t psvrd_client_headsetOff(psvrd_client_connection_t *connection);
psvrd_response_code_t psvrd_client_headsetPower(psvrd_client_connection_t *connection, int value);
psvrd_response_code_t psvrd_client_enterVRMode(psvrd_client_connection_t *connection);
psvrd_response_code_t psvrd_client_enterCinematicMode(psvrd_client_connection_t *connection);
psvrd_response_code_t psvrd_client_powerOff(psvrd_client_connection_t *connection);

psvrd_response_code_t psvrd_client_requestSensorStream(psvrd_client_connection_t *connection);

psvrd_client_error_t psvrd_client_getCurrentSensorState(psvrd_client_connection_t *connection, psvrd_client_sensor_state_t *sensorState);
#endif /*PSVRD_CLIENT_*/
