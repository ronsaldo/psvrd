// Original Source downloaded from: http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/

//=====================================================================================================
// MahonyAHRS.c
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 06/12/2017   R Salgado       Adapted to PSVR
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files

#include "MahonyAHRS.h"
#include <math.h>

//---------------------------------------------------------------------------------------------------
// Variable definitions

static psvrd_scalar_t integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki

//====================================================================================================
// Functions


//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MahonyAHRSupdateIMU(psvrd_scalar_t gx, psvrd_scalar_t gy, psvrd_scalar_t gz, psvrd_scalar_t ax, psvrd_scalar_t ay, psvrd_scalar_t az,
    psvrd_scalar_t deltaTime, psvrd_scalar_t twoKp, psvrd_scalar_t twoKi, psvrd_quaternion_t *destination)
{
    psvrd_scalar_t q0, q1, q2, q3;

    psvrd_scalar_t recipNorm;
	psvrd_scalar_t halfvx, halfvy, halfvz;
	psvrd_scalar_t halfex, halfey, halfez;
	psvrd_scalar_t qa, qb, qc;

    q0 = destination->w;
    q1 = destination->x;
    q2 = destination->y;
    q3 = destination->z;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = 1.0 / sqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;

		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * deltaTime;	// integral error scaled by Ki
			integralFBy += twoKi * halfey * deltaTime;
			integralFBz += twoKi * halfez * deltaTime;
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= (0.5f * deltaTime);		// pre-multiply common factors
	gy *= (0.5f * deltaTime);
	gz *= (0.5f * deltaTime);
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = 1.0 / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

    destination->w = q0;
    destination->x = q1;
    destination->y = q2;
    destination->z = q3;
}

//====================================================================================================
// END OF CODE
//====================================================================================================
