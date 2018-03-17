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

#include "drivers/time.h"

#include "io/gps.h"

#include "fc/runtime_config.h"
#include "fc/config.h"
#include "fc/rc_controls.h"

#include "flight/altitude.h"
#include "flight/gps_rescue.h"
#include "flight/imu.h"
#include "flight/pid.h"

#include "rx/rx.h"

#define FHZ (1000 * 200) // Five Hertz

bool          canUseGPSHeading = true; // We will expose this to the IMU so we know when to use gyro only
int16_t       gpsRescueAngle[ANGLE_INDEX_COUNT] = { 0, 0 }; // When we edit this, the PID controller will use these angles as a setpoint
int32_t       targetAltitude = 0; // Target altitude in cm

/*
 If we have new GPS data, update home heading
 if possible and applicable.
*/
void rescueNewGpsData(void)
{
   if (!ARMING_FLAG(ARMED))
    GPS_reset_home_position();
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

    rcCommand[YAW] -= dif * currentPidProfile->pid[PID_NAVR].P / 20;
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
        canUseGPSHeading = true;

        return;
    }

    //canUseGPSHeading = false; // Stop taking in new GPS heading data when this mode is active.  We're going to rely on gyro only from this point forwards 

    //we are in rescue mode. Here's what we do:
    //1) if we're far from home, make sure we're on the right course
    //2) make sure we're moving at a reasonable speed and angle
    //3) make sure the altitude is reasonable




    if (ABS(rcCommand[YAW]) < 20) {
        setBearing(GPS_directionToHome);
    }

    uint16_t safetyMargin = 1000; // really we want to get this from actual data
    uint16_t targetSpeed = 2500; // cm per second, should be a parameter

    targetAltitude = safetyMargin + 100 * gpsConfig()->gpsRescueInitialAltitude;

     //are we beyond descent_distance? If so, set safe altitude and speed
     if (GPS_distanceToHome < gpsConfig()->gpsRescueDescentDistance) {
          //this is a hack - linear descent and slowdown
          targetAltitude = safetyMargin + gpsConfig()->gpsRescueInitialAltitude * GPS_distanceToHome / gpsConfig()->gpsRescueDescentDistance;
          targetSpeed = constrain(targetSpeed * GPS_distanceToHome / gpsConfig()->gpsRescueDescentDistance, 100, 2500);
     }

     DEBUG_SET(DEBUG_RTH, 0, gpsSol.groundSpeed);
     DEBUG_SET(DEBUG_RTH,1, targetSpeed);
     DEBUG_SET(DEBUG_RTH,2, gpsRescueAngle[AI_PITCH]);
     DEBUG_SET(DEBUG_RTH,3, DECIDEGREES_TO_DEGREES(attitude.values.yaw));
    //this is a hack
    //gpsRescueAngle[AI_PITCH] = gpsConfig()->gpsRescueAngle;

    //this is another hack, version 2
    if (gpsSol.groundSpeed > targetSpeed && (gpsRescueAngle[AI_PITCH] > 0)) {
        gpsRescueAngle[AI_PITCH] --;
    } else if (gpsSol.groundSpeed < targetSpeed && gpsRescueAngle[AI_PITCH] < gpsConfig()->gpsRescueAngle) {
        gpsRescueAngle[AI_PITCH] ++;
    }

    targetAltitude = safetyMargin + gpsConfig()->gpsRescueInitialAltitude * 100 ;
    applyGPSRescueAltitude();
}

void applyGPSRescueAltitude()
{
    static uint32_t previousTimeUs = 0;
    static int32_t previousAltitude = 0; // Altitude in cm
    static int8_t netDirection = 0; // -10 to 10 for direction in the span of a 2s

    const uint32_t currentTimeUs = micros();
    const uint32_t dTime = currentTimeUs - previousTimeUs;

    bool shouldApplyThrottleCorrection = true;

    if (dTime < FHZ) { // Only apply altitude correction at 5hz (lowest common denominator with sensors)
        return;
    }

    const int32_t currentAltitude = gpsSol.llh.alt;

    previousTimeUs = currentTimeUs;

    // Increment or decrement at 5hz, this will function as our integral error over time
    if (currentAltitude > previousAltitude && netDirection < 10) {
        netDirection++;
    } else if(currentAltitude < previousAltitude && netDirection > -10) {
        netDirection--;
    }

    // If we are within 1m of target altitude, KISS
    if(ABS(currentAltitude - targetAltitude) < 500) {
        shouldApplyThrottleCorrection = false;
    }

    // Dont keep changing throttle once it is already moving towards the height we want
    if ((netDirection == 10 && currentAltitude < targetAltitude) || (netDirection == -10 && currentAltitude > targetAltitude)) {
        shouldApplyThrottleCorrection = false;
    }

    int8_t throttleCorrection = 100 - (ABS(netDirection) * 10);

    if (shouldApplyThrottleCorrection) {
        if (currentAltitude < targetAltitude) {
            rcCommand[THROTTLE] = constrain(rcCommand[THROTTLE] + throttleCorrection, PWM_RANGE_MIN, PWM_RANGE_MAX);
        } else if(currentAltitude > targetAltitude) {
            rcCommand[THROTTLE] = constrain(rcCommand[THROTTLE] - throttleCorrection, PWM_RANGE_MIN, PWM_RANGE_MAX);
        }
    }

    DEBUG_SET(DEBUG_ALTITUDE, 0, netDirection);
    DEBUG_SET(DEBUG_ALTITUDE, 1, throttleCorrection);
    DEBUG_SET(DEBUG_ALTITUDE, 2, rcCommand[THROTTLE]);
    DEBUG_SET(DEBUG_ALTITUDE, 3, targetAltitude);

    previousAltitude = currentAltitude;
}
