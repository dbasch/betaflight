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
#include "common/printf.h"

#include "build/debug.h"

#include "drivers/time.h"

#include "io/gps.h"

#include "interface/cli.h"

#include "fc/runtime_config.h"
#include "fc/config.h"
#include "fc/rc_controls.h"

#include "flight/altitude.h"
#include "flight/gps_rescue.h"
#include "flight/imu.h"
#include "flight/pid.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"
#include "sensors/gyro.h"

#define FHZ (1000 * 200) // Five Hertz

static float zGforce = 1; // Z G forces adjusted for tilt angle.

bool          canUseGPSHeading = true; // We will expose this to the IMU so we know when to use gyro only
int16_t       gpsRescueAngle[ANGLE_INDEX_COUNT] = { 0, 0 }; // When we edit this, the PID controller will use these angles as a setpoint
int32_t       targetAltitude = 0; // Target altitude in cm
int32_t       maxAltChangeRate = 400; // 400cm/s max altitude change
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
    calculateAcceleration();

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
    /*
    static uint32_t previousTimeUs = 0;
    //static int32_t previousAltitude = 0; // Altitude in cm
    static int8_t netDirection = 0; // movement over time
    static int8_t iTermMax = 5; // I term max for netDirection, maybe make this a configuration item later

    const uint32_t currentTimeUs = micros();
    const uint32_t dTime = currentTimeUs - previousTimeUs;

    if (dTime < FHZ) { // Only apply altitude correction at 5hz (lowest common denominator with sensors)
        return;
    }

    const int32_t currentAltitude = getEstimatedAltitude(); // We can ref this directly later

    previousTimeUs = currentTimeUs;

    // Increment or decrement at 5hz, this will function as our integral error over time (5 samples @ 200ms = 1s)
    netDirection = constrain(netDirection + sign(zGforce), -1 * iTermMax, iTermMax);

    //int32_t accelerationRate = (sign(zGforce)) ? zGforce * 980 : (zGforce + 1) * 980;

    int8_t correctionMagnitude = ABS(netDirection) * gpsConfig()->gpsRescueThrottleGain;

    if (sign(netDirection) == sign(zGforce)) { //moving towards target
        correctionMagnitude = 10 * gpsConfig()->gpsRescueThrottleGain - correctionMagnitude;
    }

    int8_t throttleCorrection = sign(zGforce) * correctionMagnitude;

    rescueThrottle = constrain((rcCommand[THROTTLE] + throttleCorrection), PWM_RANGE_MIN, PWM_RANGE_MAX);

    DEBUG_SET(DEBUG_ALTITUDE, 0, netDirection);
    DEBUG_SET(DEBUG_ALTITUDE, 1, throttleCorrection);
    DEBUG_SET(DEBUG_ALTITUDE, 2, rescueThrottle);
    DEBUG_SET(DEBUG_ALTITUDE, 3, targetAltitude);

    //previousAltitude = currentAltitude;*/
}

void calculateAcceleration()
{
    zGforce = (acc.accADC[Z] / acc.dev.acc_1G);

    float cosTilt = getCosTiltAngle();
    float gravityScale = (1 + (1 - cosTilt));
    float zgForceAdj = zGforce * gravityScale;

    static float zgForceAvg = 0;

    zgForceAvg = (zgForceAvg * 0.998f) + (zgForceAdj * 0.002f);

    bool withinDeadband = zGforce <= 1.2f && zGforce >= 0.8f;
    int8_t direction = 0;

    if (!withinDeadband) {
        direction = zGforce > 1 ? 1 : -1;
    }

    int32_t zg = zGforce * 100;
    int32_t ct = cosTilt * 100;
    int32_t adj = zgForceAdj * 100;
    int32_t gs = gravityScale * 100;
    int32_t avg = zgForceAvg * 100;
    int32_t dir = direction * 100;

    tfp_sprintf(debugLine, "ZG: %d, SCALE: %d, COSTILT: %d, ADJ: %d, AVG: %d, DIR: %d", zg, gs, ct, adj, avg, dir);

    /*
    float gravityForce = (1 - getCosTiltAngle()); // We need to add this to our Z calculation
    float gravityScale = gravityForce + 1;

    zGforce = (acc.accADC[Z] / acc.dev.acc_1G) - (1);

    static float avgGforce = 0;

    avgGforce = (avgGforce * 0.99f) + (zGforce * 0.01f);

    bool withinDeadband = zGforce <= 1.2f && zGforce >= 0.8f;
    int8_t direction = 0;

    if (!withinDeadband) {
        direction = zGforce > 1 ? 1 : -1;
    }
    

    int32_t zg = zGforce * 100;
    int32_t cmp = ((zGforce + gravityForce) * gravityScale) * 100;
    int32_t avg = (avgGforce * 100);
    int32_t gf = gravityForce * 100;
    int32_t costilt = getCosTiltAngle() * 100;

    tfp_sprintf(debugLine, "ZGforce: %d, AVG: %d,  DIR: %d, REAL: %d, GF: %d, COS: %d", zg, avg, direction, cmp, gf, costilt);*/

    /*
    static uint32_t previousTimeUs = 0;
    static uint32_t velocity = 0;

    const uint32_t currentTimeUs = micros();

    //zGforce = (acc.accADC[Z] / acc.dev.acc_1G) * (1 + (1 - getCosTiltAngle()));

    float[4] q = quaternionUpdate(acc.accADC[X], acc.accADC[Y], acc.accADC[Z], gyro.gyroADCf[X], gyroADCf[Y], gyroADCf[Z]);*

    /*
    float cmPerUs = 980 / 1000 / 1000;

    static float altitude = 0;
    static uint32_t previousTimeUs = 0;

    const uint32_t currentTimeUs = micros();

    zGforce = (acc.accADC[Z] / acc.dev.acc_1G) * (1 + (1 - getCosTiltAngle())) - 1; // Subtract 1 for gravity

    // Acceleration rate when moving up is 
    int32_t accelerationRate = (zGforce >= 0) : zGforce * 980 : 0;

    altitude += accelerationRate * (cmPerUs * (currentTimeUs - previousTimeUs));

    int32_t zg = zGforce * 100;
    int32_t alt = altitude;

    tfp_sprintf(debugLine, "Acc Rate CM: %d ZGforce: %d, ALT: %d", accelerationRate, zg, alt);

    previousTimeUs = currentTimeUs;*/
}
