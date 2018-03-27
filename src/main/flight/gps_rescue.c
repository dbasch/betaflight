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

#define FHZ (1000 * 200) // Five Hertz

bool          canUseGPSHeading = true; // We will expose this to the IMU so we know when to use gyro only
int16_t       gpsRescueAngle[ANGLE_INDEX_COUNT] = { 0, 0 };

static rescueState_s rescueState;

/*
 If we have new GPS data, update home heading
 if possible and applicable.
*/
void rescueNewGpsData(void)
{
   if (!ARMING_FLAG(ARMED))
    GPS_reset_home_position();
}

/*
    Determine what phase we are in, determine if all criteria are met to move to the next phase
*/
void updateGPSRescueState(void) 
{
    if (!FLIGHT_MODE(GPS_RESCUE_MODE)) {
        rescueState.phase = RESCUE_IDLE;
    }

    rescueState.info.currentAltitude = getEstimatedAltitude();
    rescueState.info.currentGroundspeed = gpsSol.groundSpeed;
    rescueState.info.distanceToHome = GPS_distanceToHome;

    switch (rescueState.phase) {
        case RESCUE_IDLE:
            idleTasks();
            break;
        case RESCUE_INITIALIZE:
            // Store whether we are already in angle mode, etc
            if (FLIGHT_MODE(ANGLE_MODE)) {
                rescueState.info.previouslyAngleMode = true;
            }

            rescueState.phase = RESCUE_ATTAIN_ALT;
            break;
        case RESCUE_ATTAIN_ALT:
            if (rescueState.info.currentAltitude > gpsRescue()->initialAltitude) {
                rescueState.phase = RESCUE_CROSSTRACK;
                break;
            }

            rescueState.intent.targetGroundspeed = 0;
            rescueState.intent.destinationAltitude = gpsRescue()->initialAltitude;

            moveTowardsTarget();
            break;
        case RESCUE_CROSSTRACK:
            // We can assume at this point that we are at or above our RTH height, so we need to try and point to home and tilt while maintaining alt
            // Is our altitude way off?  We should probably kick back to phase RESCUE_ATTAIN_ALT
            rescueState.intent.targetGroundspeed = 2500;
            rescueState.intent.destinationAltitude = gpsRescue()->initialAltitude;

            moveTowardsTarget();
            break;
        case RESCUE_LANDING_APPROACH:
            // We have crosstracked our way to our descent radius.  Continue to crosstrack, but change our altitude setpoint to be inversely proportional to delta
            // We need to slow down on our way to our rescue landing envelope
            break;
        case RESCUE_LANDING:
            // We have reached the XYZ envelope to be considered at "home".  We need to land gently and check our accelerometer for abnormal data.
            // At this point, do not let the target altitude go up anymore, so if we overshoot, we dont' move in a parabolic trajectory
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
}

void performSanityChecks()
{
    // Just an example, but random sanity checks.  Make sure we haven't landed or crashed, etc
    if (rescueState.failure == RESCUE_CRASH_DETECTED) {
        rescueState.phase = RESCUE_ABORT;
    }
}

void rescueStart()
{
    rescueState.phase = RESCUE_INITIALIZE;
}

void rescueStop()
{
    if (rescueState.info.previouslyAngleMode) {
        ENABLE_FLIGHT_MODE(ANGLE_MODE);
    }

    rescueState.phase = RESCUE_IDLE;
}

// Things that need to run regardless of GPS rescue mode being enabled or not
void idleTasks()
{
    if (getEstimatedAltitude() > rescueState.info.maxAltitude) {
        rescueState.info.maxAltitude = getEstimatedAltitude();
    }


    resetAngles();

    canUseGPSHeading = true;
    rescueThrottle = rcCommand[THROTTLE];
}


void crossTrack()
{
    setBearing(GPS_directionToHome);
}

void moveTowardsTarget()
{
    // Move towards our designated XYZ envelope
    static uint32_t previousTimeUs = 0;
    const uint32_t currentTimeUs = micros();
    const uint32_t dTime = currentTimeUs - previousTimeUs;

    if (dTime < FHZ) { // Only apply altitude correction at 5hz (lowest common denominator with sensors)
        return;
    }

    applyAngle();
    applyAltitude();
    crossTrack();

    previousTimeUs = currentTimeUs;    
}

void applyAltitude()
{
    // Determine how much vertical thrust (0-1 we need to achieve our desired alt)
    static int32_t integral = 0;
    static int32_t previousError = 0;

    const int32_t error = (rescueState.intent.targetAltitude - rescueState.info.currentAltitude) / 100;
    const int32_t derivative = error - previousError;
    
    integral = constrain(integral + error, -50, 50);

    previousError = error;

    int16_t throttleCorrection = (gpsRescue()->tP * error + gpsRescue()->tI * integral + gpsRescue()->tD * derivative);

    setThrottle(constrain(rescueThrottle + throttleCorrection, PWM_RANGE_MIN, PWM_RANGE_MAX));
}

void applyAngle()
{
    // Determine how much vertical thrust (0-1 we need to achieve our desired alt)
    static int32_t integral = 0;
    static int32_t previousError = 0;

    const int32_t error = (rescueState.intent.targetGroundspeed - rescueState.info.currentGroundspeed) / 100; // How many M/s we are off
    const int32_t derivative = error - previousError;
    
    integral = constrain(integral + error, -50, 50);

    previousError = error;

    int16_t angleCorrection = (gpsRescue()->aP * error + gpsRescue()->aI * integral + gpsRescue()->aD * derivative);

    setPitch(constrain(gpsRescueAngle[AI_PITCH] + angleCorrection, -gpsRescue()->angle, gpsRescue()->angle));
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

void setThrottle(int16_t throttle)
{
    rescueThrottle = throttle;
}

void setPitch(int16_t pitch)
{
    gpsRescueAngle[AI_PITCH] = pitch;
}

void resetAngles()
{
    gpsRescueAngle[AI_PITCH] = 0;
    gpsRescueAngle[AI_ROLL] = 0;
}
