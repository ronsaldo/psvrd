// Original Source downloaded from: http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/

//=====================================================================================================
// MahonyAHRS.h
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
#ifndef MahonyAHRS_h
#define MahonyAHRS_h

#include "psvrd.h"

//----------------------------------------------------------------------------------------------------
// Variable declaration

#define MAHONY_TWOKP_DEFAULT (2.0f * 0.5f)
#define MAHONY_TWOKI_DEFAULT (2.0f * 0.0f)

//---------------------------------------------------------------------------------------------------
// Function declarations

void MahonyAHRSupdateIMU(psvrd_scalar_t gx, psvrd_scalar_t gy, psvrd_scalar_t gz, psvrd_scalar_t ax, psvrd_scalar_t ay, psvrd_scalar_t az,
    psvrd_scalar_t deltaTime, psvrd_scalar_t twoKp, psvrd_scalar_t twoKi, psvrd_quaternion_t *destination);

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
