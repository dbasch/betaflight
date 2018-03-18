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
        rescueThrottle = rcCommand[THROTTLE];

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
          targetAltitude = safetyMargin + 100 * gpsConfig()->gpsRescueInitialAltitude * GPS_distanceToHome / gpsConfig()->gpsRescueDescentDistance;
         // DEBUG_SET(DEBUG_ALTITUDE, 3, targetAltitude);

          targetSpeed = constrain(targetSpeed * GPS_distanceToHome / gpsConfig()->gpsRescueDescentDistance, 100, 2500);
     }

     //DEBUG_SET(DEBUG_RTH, 0, gpsSol.groundSpeed);
     //DEBUG_SET(DEBUG_RTH,1, targetSpeed);
     //DEBUG_SET(DEBUG_RTH,2, gpsRescueAngle[AI_PITCH]);
     //DEBUG_SET(DEBUG_RTH,3, DECIDEGREES_TO_DEGREES(attitude.values.yaw));

    //this is another hack, version 2
    if (gpsSol.groundSpeed > targetSpeed && (gpsRescueAngle[AI_PITCH] > 0)) {
        gpsRescueAngle[AI_PITCH] --;
    } else if (gpsSol.groundSpeed < targetSpeed && gpsRescueAngle[AI_PITCH] < gpsConfig()->gpsRescueAngle) {
        gpsRescueAngle[AI_PITCH] ++;
    }
    applyGPSRescueAltitude();
}

void applyGPSRescueAltitude()
{
    static uint32_t previousTimeUs = 0;
    static int32_t previousAltitude = 0; // Altitude in cm
    static int8_t netDirection = 0; // movement over time
    static int8_t iTermMax = 5; // I term max for netDirection, maybe make this a configuration item later
    static int16_t previousRescueAngle = 0;

    const uint32_t currentTimeUs = micros();
    const uint32_t dTime = currentTimeUs - previousTimeUs;

    if (dTime < FHZ) { // Only apply altitude correction at 5hz (lowest common denominator with sensors)
        return;
    }

    const int32_t currentAltitude = getEstimatedAltitude(); // We can ref this directly later

    previousTimeUs = currentTimeUs;

    // Increment or decrement at 5hz, this will function as our integral error over time (5 samples @ 200ms = 1s)
    netDirection = constrain(netDirection + sign(currentAltitude - previousAltitude), -1 * iTermMax, iTermMax);

    int8_t correctionMagnitude = ABS(netDirection) * gpsConfig()->gpsRescueThrottleGain;

    if (sign(netDirection) == sign(targetAltitude - currentAltitude)) { //moving towards target
        correctionMagnitude = 10 * gpsConfig()->gpsRescueThrottleGain - correctionMagnitude;
    }

    int8_t throttleCorrection = sign(targetAltitude - currentAltitude) * correctionMagnitude;
    float efficiencyGain = 1 + (cos(DECIDEGREES_TO_RADIANS(previousRescueAngle)) - cos(DECIDEGREES_TO_RADIANS(gpsRescueAngle[AI_PITCH])));
    
    rescueThrottle = constrain((rcCommand[THROTTLE] + throttleCorrection) * efficiencyGain, PWM_RANGE_MIN + 200, PWM_RANGE_MAX -200);

    DEBUG_SET(DEBUG_ALTITUDE, 0, netDirection);
    DEBUG_SET(DEBUG_ALTITUDE, 1, throttleCorrection);
    DEBUG_SET(DEBUG_ALTITUDE, 2, rescueThrottle);
    DEBUG_SET(DEBUG_ALTITUDE, 3, targetAltitude);

    previousAltitude = currentAltitude;
    previousRescueAngle = gpsRescueAngle[AI_PITCH];
}



