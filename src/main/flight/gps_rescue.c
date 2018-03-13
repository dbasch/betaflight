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

#include "build/debug.h"
#include "io/gps.h"

#include "fc/runtime_config.h"
#include "fc/config.h"
#include "fc/rc_controls.h"

#include "flight/altitude.h"
#include "flight/gps_rescue.h"
#include "flight/imu.h"
#include "flight/pid.h"

#include "rx/rx.h"

bool          canUseGPSHeading = true; // We will expose this to the IMU so we know when to use gyro only
int16_t       gpsRescueAngle[ANGLE_INDEX_COUNT] = { 0, 0 }; // When we edit this, the PID controller will use these angles as a setpoint

static int32_t       targetBearing; // In centidegrees (100 = 1deg)
static uint32_t      homeDist;
// TEMPORARY SETTINGS UNTIL WE BOTHER ADDING REAL ONES


/*
 If we have new GPS data, update home heading
 if possible and applicable.
*/
void rescueNewGpsData(void)
{
   if (!ARMING_FLAG(ARMED))
    GPS_reset_home_position();

    GPS_distance_cm_bearing(&gpsSol.llh.lat, &gpsSol.llh.lon, &GPS_home[LAT], &GPS_home[LON], homeDist, &targetBearing);
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

    DEBUG_SET(DEBUG_RTH,2, DECIDEGREES_TO_DEGREES(attitude.values.yaw));
    DEBUG_SET(DEBUG_RTH, 3, dif);

   // if (STATE(SMALL_ANGLE)) {
    rcCommand[YAW] -= dif;// * currentPidProfile->pid[PID_MAG].P / 30;    // 18 deg
}   //}
/*
    Use the data we have available to set gpsRescueAngles and update internal state
*/
void updateGPSRescueState(void) 

{
    DEBUG_SET(DEBUG_RTH,1, GPS_directionToHome);
    DEBUG_SET(DEBUG_RTH,0, rcCommand[YAW]);


    if (!FLIGHT_MODE(GPS_RESCUE_MODE)) {
        // Reset the rescue angles to zero!
        gpsRescueAngle[AI_PITCH] = 0;
        gpsRescueAngle[AI_ROLL] = 0;

        return;
    }

    //we are in rescue mode. Here's what we do:
    //1) if we're far from home, make sure we're on the right course
    //2) make sure we're moving at a reasonable speed and angle
    //3) make sure the altitude is reasonable

     if (ABS(rcCommand[YAW]) < 20) {
        setBearing(targetBearing - getHeadingDirection());
     }

     //applyAltHold();


    // Just as a test, lets make it pitch forward until our speed is 5m/s
    if(gpsSol.groundSpeed <= 2000) {
        gpsRescueAngle[AI_PITCH] = 50; // This might be really bad as it will run a bunch of times before
    }
}


