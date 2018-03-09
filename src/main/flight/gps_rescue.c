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

#include <stdint.h>
#include <math.h>

#include "common/axis.h"
#include "common/maths.h"

#include "io/gps.h"

#include "fc/runtime_config.h"
#include "fc/config.h"
#include "fc/rc_controls.h"

#include "flight/altitude.h"
#include "flight/gps_rescue.h"
#include "flight/imu.h"

#include "rx/rx.h"

gpsLocation_t home;
uint16_t      distanceToHome;        // distance to home point in meters                                                                                               
int16_t       directionToHome;
bool          canUseGPSHeading = true; // We will expose this to the IMU so we know when to use gyro only
int16_t       gpsRescueAngle[ANGLE_INDEX_COUNT] = { 0, 0 }; // When we edit this, the PID controller will use these angles as a setpoint

// TEMPORARY SETTINGS UNTIL WE BOTHER ADDING REAL ONES


/*
 If we have new GPS data, update home heading
 if possible and applicable.
*/
void rescueNewGpsData(void)
{
    // Update state
}

/*
    Use the data we have available to set gpsRescueAngles and update internal state
*/
void updateGPSRescueState(void) 

{
    if (!FLIGHT_MODE(GPS_RESCUE_MODE)) {
        // Reset the rescue angles to zero!
        gpsRescueAngle[AI_PITCH] = 0;
        gpsRescueAngle[AI_ROLL] = 0;

        return;
    }

    applyAltHold();

    //TODO:  Make this work
    
    // Just as a test, lets make it pitch forward until our speed is 5m/s
    if(gpsSol.groundSpeed <= 5000) {
        gpsRescueAngle[AI_PITCH] = 250; // This might be really bad as it will run a bunch of times before 
    }
}

// Very similar to maghold function on betaflight/cleanflight
void setBearing(int16_t deg)
{
    int16_t dif = DECIDEGREES_TO_DEGREES(attitude.values.yaw) - deg;
    if (dif <= -180)
        dif += 360;
    if (dif >= +180)
        dif -= 360;
    dif *= -GET_DIRECTION(rcControlsConfig()->yaw_control_reversed);
    if (STATE(SMALL_ANGLE))
        rcCommand[YAW] -= dif * currentPidProfile->pid[PID_MAG].P / 30;    // 18 deg
}
