// Original Source downloaded from: http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/

//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 06/12/2017   R Salgado       Adapted to PSVR
//
//=====================================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h

#include "psvrd.h"
#define MADGWICK_AHRS_DEFAULT_BETA		0.1		// 2 * proportional gain

//---------------------------------------------------------------------------------------------------
// Function declarations

void MadgwickAHRSupdateIMU(psvrd_scalar_t gx, psvrd_scalar_t gy, psvrd_scalar_t gz, psvrd_scalar_t ax, psvrd_scalar_t ay, psvrd_scalar_t az,
    psvrd_scalar_t deltaTime, psvrd_scalar_t beta, psvrd_quaternion_t *destination);

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
