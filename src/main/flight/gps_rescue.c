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

#include "sensors/acceleration.h"

#define FHZ (1000 * 200) // Five Hertz

static absoluteAccelerationStatus accStatus;


bool          canUseGPSHeading = true; // We will expose this to the IMU so we know when to use gyro only
bool          isDescending = false;
int16_t       gpsRescueAngle[ANGLE_INDEX_COUNT] = { 0, 0 }; // When we edit this, the PID controller will use these angles as a setpoint
int32_t       targetAltitude = 0; // Target altitude in cm
int32_t       highestAltitude;

bool initialized = false;

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
        netThrottle = rescueThrottle - hoverThrottle;


        // Reset accelerometer status
        isDescending = false;

        //DEBUG_SET(DEBUG_ALTITUDE, 1, rcCommand[THROTTLE]);
        //DEBUG_SET(DEBUG_ALTITUDE, 2, attitude.values.pitch);
        //DEBUG_SET(DEBUG_ALTITUDE, 3, attitude.values.roll);
        if (!initialized) {
        //configuration parameters
            highestAltitude = 0;
            hoverThrottle = gpsRescue()->hoverThrottle;
            descentDistance = gpsRescue()->descentDistance;
            rescueAngle = gpsRescue()->angle;
            initialAltitude = gpsRescue()->initialAltitude;
            throttleMax = gpsRescue()-> throttleMax;
            tP = gpsRescue()->tP;
            tI = gpsRescue()->tI;
            tD = gpsRescue()->tD;


            initialized = true;
        }

        if (getEstimatedAltitude() > highestAltitude) {
            highestAltitude = getEstimatedAltitude();
        }

        return;
    }

    //canUseGPSHeading = false; // Stop taking in new GPS heading data when this mode is active.  We're going to rely on gyro only from this point forwards 

    //we are in rescue mode. Here's what we do:
    //1) if we're far from home, make sure we're on the right course
    //2) make sure we're moving at a reasonable speed and angle
    //3) make sure the altitude is reasonable

    setBearing(GPS_directionToHome);

    uint16_t safetyMargin = 1000; // really we want to get this from actual data
    uint16_t targetSpeed = 2500; // cm per second, should be a parameter

    targetAltitude = safetyMargin + 100 * initialAltitude;

    if (targetAltitude < highestAltitude) {
        //targetAltitude = highestAltitude;
    }

     //are we beyond descent_distance? If so, set safe altitude and speed
     if (GPS_distanceToHome < descentDistance) {
          //this is a hack - linear descent and slowdown
          //only reduce altitude from this point on
          int32_t newAlt = safetyMargin + 100 * initialAltitude * GPS_distanceToHome / descentDistance;
          if (newAlt < targetAltitude) {
                targetAltitude = newAlt;
          }
          targetSpeed = constrain(targetSpeed * GPS_distanceToHome / descentDistance, 100, 2500);

          isDescending = true;
     } else { 
        isDescending = false;
     }

    //this is another hack, version 2
    if (gpsSol.groundSpeed > targetSpeed && (gpsRescueAngle[AI_PITCH] > 5)) {
        gpsRescueAngle[AI_PITCH]--;
        canUseGPSHeading = false;
    } else if (gpsSol.groundSpeed < targetSpeed && gpsRescueAngle[AI_PITCH] < rescueAngle) {
        gpsRescueAngle[AI_PITCH]++;
        canUseGPSHeading = true;
    }

    applyGPSRescueAltitude();
}

void applyGPSRescueAltitude()
{
    static uint32_t previousTimeUs = 0;
    static int32_t integral = 0;
    static int32_t previousError = 0;


    const uint32_t currentTimeUs = micros();
    const uint32_t dTime = currentTimeUs - previousTimeUs;

    if (dTime < FHZ) { // Only apply altitude correction at 5hz (lowest common denominator with sensors)
        return;
    }

    const int32_t currentAltitude = getEstimatedAltitude();

    const int32_t error = (targetAltitude - currentAltitude) / 100; // error is in meters
    if (ABS(error) > 10) {// don't dive or climb while moving super fast horizontally
        gpsRescueAngle[AI_PITCH] = 5;
    }
    const int32_t derivative = error - previousError;
    integral = constrain(integral + error, -50, 50);
    //remember state for the next iteration
    previousError = error;
    previousTimeUs = currentTimeUs;

    //apply PID to control variable
    //int32_t ct = 100 * getCosTiltAngle();
    netThrottle = (tP * error + tI * integral + tD * derivative) / (100 * getCosTiltAngle()) ;
    rescueThrottle = constrain(hoverThrottle + netThrottle, hoverThrottle - 30, throttleMax);

    //DEBUG_SET(DEBUG_ALTITUDE, 0, error);
    //DEBUG_SET(DEBUG_ALTITUDE, 1, rescueThrottle);
    //DEBUG_SET(DEBUG_ALTITUDE, 2, netThrottle);
   // DEBUG_SET(DEBUG_ALTITUDE, 3, targetAltitude);


}

void calculateAcceleration()
{
    static float zga = 0;
    static float xga = 0;
    static float yga = 0;

    static float highestZg = 0;
    static float lowestZg = 0;
    static float highestG = 0;


    quaternion q;
    getQuaternion(&q);

    float zg = -1 * ((acc.accADC[Z] / acc.dev.acc_1G) - (q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z));
    float xg = -1 * ((acc.accADC[X] / acc.dev.acc_1G) + (2.0f * (q.x * q.z - q.w * q.y)));
    float yg = -1 * ((acc.accADC[Y] / acc.dev.acc_1G) - (2.0f * (q.w * q.x + q.y * q.z)));

    zga = (zga * 0.998) + (zg * 0.002);
    xga = (xga * 0.998) + (xg * 0.002);
    yga = (yga * 0.998) + (yg * 0.002);

    accStatus.zg = zg;
    accStatus.yg = yg;
    accStatus.xg = xg;

    accStatus.zga = zga;
    accStatus.yga = yga;
    accStatus.xga = xga;

    // If we detect our average spikes, something bad happened

    float bumpMagnitude = (float) sqrt(sq(ABS(xg)) + sq(ABS(yg)) + sq(ABS(zg)));

    if (FLIGHT_MODE(GPS_RESCUE_MODE)) {
        accStatus.crashDetected = (accStatus.crashDetected == true || bumpMagnitude > highestG * 2.0f);

        if(isDescending) {
            accStatus.landingDetected = (accStatus.landingDetected == true
                || ABS(zg) >= constrain(GPS_distanceToHome / descentDistance * ABS(zg), lowestZg * 1.5f, highestZg * 1.5f));
        }
    } else { 
        if (ABS(zg) > highestZg) {
            highestZg = ABS(zg);
        }

        if (ABS(zg) < lowestZg) {
            lowestZg = ABS(zg);
        }

        if (bumpMagnitude > highestG) {
            highestG = bumpMagnitude;
        }

        accStatus.crashDetected = false;
        accStatus.landingDetected = false;
    }

    int8_t direction = 0;

    bool withinDeadband = (zga <= 0.2f && zga >= -0.2f);

    if (!withinDeadband) {
        direction = (zga <= 0.2f) ? 1 : -1;
    }

    accStatus.verticalDirection = direction;

    DEBUG_SET(DEBUG_ACCELEROMETER_STATE, 0, zga * 100);
    DEBUG_SET(DEBUG_ACCELEROMETER_STATE, 1, bumpMagnitude * 100);
    DEBUG_SET(DEBUG_ACCELEROMETER_STATE, 2, (accStatus.landingDetected) ? 1 : 0);
    DEBUG_SET(DEBUG_ACCELEROMETER_STATE, 3, (accStatus.crashDetected) ? 1 : 0);
}
