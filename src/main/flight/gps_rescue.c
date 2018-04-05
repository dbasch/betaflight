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

#include "fc/fc_core.h"
#include "fc/runtime_config.h"
#include "fc/config.h"
#include "fc/rc_controls.h"

#include "flight/altitude.h"
#include "flight/gps_rescue.h"
#include "flight/imu.h"
#include "flight/pid.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"

bool          canUseGPSHeading = true; // We will expose this to the IMU so we know when to use gyro only
int16_t       gpsRescueAngle[ANGLE_INDEX_COUNT] = { 0, 0 };
uint16_t      hoverThrottle = 0;
float         averageThrottle = 0.0;
float         altitudeError = 0.0;
uint32_t      throttleSamples = 0;

static bool newGPSData = false;

rescueState_s rescueState;

/*
 If we have new GPS data, update home heading
 if possible and applicable.
*/
void rescueNewGpsData(void)
{

   if (!ARMING_FLAG(ARMED))
    GPS_reset_home_position();

    newGPSData = true;
}

/*
    Determine what phase we are in, determine if all criteria are met to move to the next phase
*/
void updateGPSRescueState(void)
{
    if (!FLIGHT_MODE(GPS_RESCUE_MODE)) {
        rescueStop();
    } else if(FLIGHT_MODE(GPS_RESCUE_MODE) && rescueState.phase == RESCUE_IDLE){
        rescueStart();
    }

    sensorUpdate();

    switch (rescueState.phase) {
        case RESCUE_IDLE:
            idleTasks();
            break;
        case RESCUE_INITIALIZE:
            if (hoverThrottle == 0) { //no actual throttle data yet, let's use the default.
                hoverThrottle = gpsRescue()->throttleHover;
            }
            // Store whether we are already in angle mode, etc
            if (FLIGHT_MODE(ANGLE_MODE)) {
                rescueState.flags.previouslyAngleMode = true;
            }
            __attribute__ ((fallthrough));
        case RESCUE_ATTAIN_ALT:
            // Get to a safe altitude at a low velocity ASAP
            if (ABS(rescueState.intent.targetAltitude - rescueState.sensor.currentAltitude) < 1000) {
                rescueState.phase = RESCUE_CROSSTRACK;
            }

            rescueState.intent.targetGroundspeed = 500;
            rescueState.intent.targetAltitude = gpsRescue()->initialAltitude * 100;
            rescueState.intent.targetZVelocity = constrain(1000 * ((rescueState.intent.targetAltitude - rescueState.sensor.currentAltitude) / 10000), -300, 800);
            rescueState.intent.minimumAngle = 150;
            rescueState.intent.crosstrack = true;
            break;
        case RESCUE_CROSSTRACK:
            if (rescueState.sensor.distanceToHome < gpsRescue()->descentDistance) {
                rescueState.phase = RESCUE_LANDING_APPROACH;
            }

            // We can assume at this point that we are at or above our RTH height, so we need to try and point to home and tilt while maintaining alt
            // Is our altitude way off?  We should probably kick back to phase RESCUE_ATTAIN_ALT
            rescueState.intent.targetGroundspeed = gpsRescue()->rescueGroundspeed;
            rescueState.intent.targetAltitude = gpsRescue()->initialAltitude * 100;
            rescueState.intent.targetZVelocity = constrain((rescueState.intent.targetAltitude - rescueState.sensor.currentAltitude) / 10, -300, 300);
            rescueState.intent.minimumAngle = 200;
            rescueState.intent.crosstrack = true;

            break;
        case RESCUE_LANDING_APPROACH:
            // We are getting close to home in the XY plane, get Z where it needs to be to move to landing phase
            if (rescueState.sensor.distanceToHome < 10 && rescueState.sensor.currentAltitude <= 1000) {
                rescueState.phase = RESCUE_LANDING;
            }

            int32_t newAlt = gpsRescue()->initialAltitude * 100  * rescueState.sensor.distanceToHome / gpsRescue()->descentDistance;

            if (newAlt < rescueState.intent.targetAltitude) {
                rescueState.intent.targetAltitude = newAlt;
            }

            rescueState.intent.targetGroundspeed = MAX(gpsRescue()->rescueGroundspeed * rescueState.sensor.distanceToHome / gpsRescue()->descentDistance, 100);
            rescueState.intent.targetZVelocity = -200;
            rescueState.intent.minimumAngle = 50;
            rescueState.intent.crosstrack = true;
            break;
        case RESCUE_LANDING:
            // We have reached the XYZ envelope to be considered at "home".  We need to land gently and check our accelerometer for abnormal data.
            // At this point, do not let the target altitude go up anymore, so if we overshoot, we dont' move in a parabolic trajectory

            // If we are over 120% of average magnitude, just disarm since we're pretty much home
            if (rescueState.sensor.accMagnitude > rescueState.sensor.accMagnitudeAvg * 1.2) {
                disarm();
                rescueState.phase = RESCUE_COMPLETE;
            }

            rescueState.intent.targetZVelocity = -100;
            rescueState.intent.targetGroundspeed = 0;
            rescueState.intent.targetAltitude = 0;
            rescueState.intent.minimumAngle = 0;
            rescueState.intent.crosstrack = false;
            break;
        case RESCUE_COMPLETE:
            rescueStop();
            break;
        case RESCUE_ABORT:
            disarm();
            rescueStop();
            break;
    }

    performSanityChecks();

    if (rescueState.phase != RESCUE_IDLE) {
        rescueAttainPosition();
    }

    newGPSData = false;

}

void sensorUpdate()
{
    rescueState.sensor.currentAltitude = getEstimatedAltitude();

    // Calculate altitude velocity
    static uint32_t previousTimeUs;
    static int32_t previousAltitude;

    const uint32_t currentTimeUs = micros();
    const float dTime = currentTimeUs - previousTimeUs;

    if (newGPSData) { // Calculate velocity at lowest common denominator
        rescueState.sensor.distanceToHome = GPS_distanceToHome;
        rescueState.sensor.directionToHome = GPS_directionToHome;
        rescueState.sensor.groundSpeed = gpsSol.groundSpeed;

        rescueState.sensor.zVelocity = (rescueState.sensor.currentAltitude - previousAltitude) * 1000000.0f / dTime;
        rescueState.sensor.zVelocityAvg = 0.8 * rescueState.sensor.zVelocityAvg + rescueState.sensor.zVelocity * 0.2;

        previousAltitude = rescueState.sensor.currentAltitude;
        previousTimeUs = currentTimeUs;
    }

    rescueState.sensor.accMagnitude = (float) sqrt(sq(ABS((acc.accADC[Z] / acc.dev.acc_1G))) + sq(ABS((acc.accADC[X] / acc.dev.acc_1G))) + sq(ABS((acc.accADC[Y] / acc.dev.acc_1G))));
    rescueState.sensor.accMagnitudeAvg = (rescueState.sensor.accMagnitudeAvg * 0.998) + (rescueState.sensor.accMagnitude * 0.002);
}

void performSanityChecks()
{
    if (rescueState.phase == RESCUE_IDLE) {
        return;
    }

    /*
    // If we have a higher accelerometer magnitude than at any previous point in the non RTH flight, we probably crashed!
    if (rescueState.sensor.accMagnitude > rescueState.sensor.accMagnitudeMax * 1.5) {
        rescueState.failure = RESCUE_CRASH_DETECTED;
    }*/


    /*
    // If we get more than 500 meters further than our furthest point prior to RTH, something DEFINITELY went wrong
    if (rescueState.sensor.distanceToHome > rescueState.sensor.maxDistanceToHome + 500) {
        rescueState.failure = RESCUE_TOO_FAR;
    }*/

    /*
    // If we get more than 200 meters above our target altitude, something is very bad
    if (rescueState.sensor.currentAltitude > rescueState.sensor.targetAltitude + 2000) {
        rescueState.failure = RESCUE_TOO_HIGH;
    }*/

    // Just an example, but random sanity checks.  Make sure we haven't landed or crashed, etc
    if (rescueState.failure == RESCUE_CRASH_DETECTED || rescueState.failure == RESCUE_TOO_FAR || rescueState.failure == RESCUE_TOO_HIGH) {
        rescueState.phase = RESCUE_ABORT;
    }
}

void rescueStart()
{
    rescueState.phase = RESCUE_INITIALIZE;
}

void rescueStop()
{
    if (rescueState.flags.previouslyAngleMode) {
        ENABLE_FLIGHT_MODE(ANGLE_MODE);
    }

    rescueState.phase = RESCUE_IDLE;
}

// Things that need to run regardless of GPS rescue mode being enabled or not
void idleTasks()
{
    gpsRescueAngle[AI_PITCH] = 0;
    gpsRescueAngle[AI_ROLL] = 0;

    // Store the max altitude we see not during RTH so we know our fly-back minimum alt
    rescueState.sensor.maxAltitude = MAX(rescueState.sensor.currentAltitude, rescueState.sensor.maxAltitude);
    // Store the max magnitude we see not during RTH so we have a basis for crash detection
    rescueState.sensor.accMagnitudeMax = MAX(rescueState.sensor.accMagnitude, rescueState.sensor.accMagnitudeMax);
    // Store the max distance to home during normal flight so we know if a flyaway is happening
    rescueState.sensor.maxDistanceToHome = MAX(rescueState.sensor.distanceToHome, rescueState.sensor.maxDistanceToHome);

    rescueThrottle = rcCommand[THROTTLE];
    DEBUG_SET(DEBUG_RTH, 2, rescueThrottle);


    //to do: have a default value for hoverThrottle
    float ct = getCosTiltAngle();
    if (ct > 0.5 && ct < 0.96 && throttleSamples < 1E8 && rescueThrottle > 1070) { //5 to 45 degrees tilt
        //TO DO: only sample when acceleration is low
        uint16_t adjustedThrottle = 1000 + (rescueThrottle - 1000) * ct;
        if (throttleSamples == 0) {
            averageThrottle = adjustedThrottle;
        } else {
            averageThrottle += (adjustedThrottle - averageThrottle) / (throttleSamples + 1);
        }
        hoverThrottle = lrintf(averageThrottle);
        DEBUG_SET(DEBUG_RTH, 0, (int32_t)(100 * ct));
        DEBUG_SET(DEBUG_RTH, 1, attitude.values.pitch);
        DEBUG_SET(DEBUG_RTH, 3, hoverThrottle);
        throttleSamples++;
    }

}

void rescueAttainPosition()
{
    static float previousVelocityError = 0;
    static float previousAltitudeError = 0;
    static float velocityIntegral = 0;
    static float altitudeIntegral = 0;

    // Point to home if that is in our intent
    if (rescueState.intent.crosstrack) {
        setBearing(rescueState.sensor.directionToHome);
    }

    /**
        Groundspeed controller
    */
    gpsRescueAngle[AI_ROLL] = 0;
    int16_t speedError = rescueState.intent.targetGroundspeed - rescueState.sensor.groundSpeed;
    int16_t angleGain = constrain(speedError / 500, -5, 5);
    gpsRescueAngle[AI_PITCH] = constrain(gpsRescueAngle[AI_PITCH] + angleGain, 10 * rescueState.intent.minimumAngle, 10 * gpsRescue()->angle);
    canUseGPSHeading = (angleGain >= 0);

    if (ABS(altitudeError) > 15) {// don't dive or climb while moving super fast horizontally
              gpsRescueAngle[AI_PITCH] = constrain(gpsRescueAngle[AI_PITCH], 50, 150);
    }


    if (!newGPSData) {
        return;
    }

    /**
        Vertical velocity PID controller to dampen the changes made by the altitude PID controller
    */

    float velocityError = (rescueState.intent.targetZVelocity - rescueState.sensor.zVelocityAvg) / 100; // Error in meters/s > 0
    altitudeError = (rescueState.intent.targetAltitude - rescueState.sensor.currentAltitude) / 100; // Error in meters

    const int16_t velocityDerivative = velocityError - previousVelocityError;
    velocityIntegral = constrain(velocityIntegral + velocityError, -50, 50);

    previousVelocityError = velocityError;

    int16_t velocityAdjustment = (gpsRescue()->vP * velocityError + gpsRescue()->vI * velocityIntegral + gpsRescue()->vD * velocityDerivative) / (100 * getCosTiltAngle());

    /**
        Altitude PID controller
    */

    const int16_t altitudeDerivative = altitudeError - previousAltitudeError;
    altitudeIntegral = constrain(altitudeIntegral + altitudeError, -50, 50);

    previousAltitudeError = altitudeError;

    int16_t altitudeAdjustment = (gpsRescue()->tP * altitudeError + gpsRescue()->tI * altitudeIntegral + gpsRescue()->tD * altitudeDerivative) / (100 * getCosTiltAngle());

    // Do not let velocity adjustment amplify the gains, only dampen
    if (sign(velocityAdjustment) == sign(altitudeAdjustment)) {
        velocityAdjustment = 0;
    }

    rescueThrottle = constrain(hoverThrottle + altitudeAdjustment + velocityAdjustment, gpsRescue()->throttleMin, gpsRescue()->throttleMax);

    DEBUG_SET(DEBUG_RTH, 0, velocityAdjustment);
    DEBUG_SET(DEBUG_RTH, 1, altitudeAdjustment);
    DEBUG_SET(DEBUG_RTH, 2, rescueThrottle);
    DEBUG_SET(DEBUG_RTH, 3, rescueState.intent.targetZVelocity);
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

    rcCommand[YAW] -= dif * currentPidProfile->pid[PID_NAVR].P / 4;
}

