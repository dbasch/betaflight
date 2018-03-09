/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#include "common/axis.h"

#include "io/gps.h"

#include "fc/runtime_config.h"

#include "flight/altitude.h"
#include "flight/gps_rescue.h"

gpsLocation_t home;
uint16_t      distanceToHome;        // distance to home point in meters                                                                                               
int16_t       directionToHome;
bool          canUseGPSHeading = true; // We will expose this to the IMU so we know when to use gyro only
int16_t       gpsRescueAngle[ANGLE_INDEX_COUNT] = { 0, 0 }; // When we edit this, the PID controller will use these angles as a setpoint

/*
 If we have new GPS data, update home heading
 if possible and applicable.
*/
void rescueNewGpsData(void)
{
    if (FLIGHT_MODE(GPS_RESCUE_MODE)) {
        //really we don't want alt hold, but we have to start somewhere
        applyAltHold();
    }
    else {
	//update state and move on
    }
}

/*
    Use the data we have available to set gpsRescueAngles and update internal state
*/
void updateGPSRescueState(void) 
{
    //TODO:  Make this work
    
    // Just as a test, lets make it pitch forward until our speed is 5m/s
    if(gpsSol.groundSpeed <= 5000) {
        //gpsRescueAngle[AI_PITCH] += 10; // This might be really bad as it will run a bunch of times before 
    }
}