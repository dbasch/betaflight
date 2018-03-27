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
bool          isDescending = false;
int16_t       gpsRescueAngle[ANGLE_INDEX_COUNT] = { 0, 0 };
int32_t       targetAltitude;
int32_t       targetGroundspeed;

static rescueState_s rescueState;
static rescuePidState_s pidState;
static rescueStats_s rescueStats;

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
        rescueState.rescuePhase = RESCUE_IDLE;
    }

    switch (rescueState.rescuePhase) {
        case RESCUE_IDLE:
            idleTasks();
            break;
        case RESCUE_INITIALIZE:
            // Store whether we are already in angle mode, etc
            rescueState.rescuePhase++;
            break;
        case RESCUE_ATTAIN_ALT:
            // Are we below our RTH alt?  if so, get there
            // Are we above our RTH alt?  Skip to the next phase
            rescueState.rescuePhase++;
            break;
        case RESCUE_CROSSTRACK:
            // We can assume at this point that we are at or above our RTH height, so we need to try and point to home and tilt while maintaining alt
            // Is our altitude way off?  We should probably kick back to phase RESCUE_ATTAIN_ALT
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
    moveTowardsTargetEnvelope();
}

void performSanityChecks()
{
    // Just an example, but random sanity checks.  Make sure we haven't landed or crashed, etc
    if (rescueState.rescueFailure == RESCUE_CRASH_DETECTED) {
        rescueState.rescuePhase = RESCUE_ABORT;
    }
}

void rescueStart()
{
    rescueState.rescuePhase = RESCUE_INITIALIZE;
}

void rescueStop()
{
    rescueState.rescuePhase = RESCUE_IDLE;
}

// Things that need to run regardless of GPS rescue mode being enabled or not
void idleTasks()
{
    if (getEstimatedAltitude() > rescueStats.maxAltitude) {
        rescueStats.maxAltitude = getEstimatedAltitude();
    }
}


void crossTrack()
{
    setBearing(GPS_directionToHome);
}

void moveTowardsTargetEnvelope()
{
    // Move towards our designated XYZ envelope
    static uint32_t previousTimeUs = 0;
    const uint32_t currentTimeUs = micros();
    const uint32_t dTime = currentTimeUs - previousTimeUs;

    if (dTime < FHZ) { // Only apply altitude correction at 5hz (lowest common denominator with sensors)
        return;
    }

    updateGroundspeedCalculation();
    updateAltitudeCalculation();
    calculateThrottleAndTilt();

    if (rescueState.rescuePhase == RESCUE_CROSSTRACK || RESCUE_LANDING_APPROACH) {
        crossTrack();
    }

    previousTimeUs = currentTimeUs;    
}

void updateGroundspeedCalculation()
{
    static int32_t integral = 0;
    static int32_t previousError = 0;

    const uint32_t groundSpeed = gpsSol.groundSpeed;

    float sP = 0.05, sI = 0.02, sD = 0.03;

    const int32_t error = (targetGroundspeed - groundSpeed);
    const int32_t derivative = error - previousError;

    integral = constrain(integral + error, -100, 100);

    previousError = error;

    pidState.speedGain = (sP * error + sI * integral + sD * derivative);
}

void updateAltitudeCalculation()
{
    // Determine how much vertical thrust (0-1 we need to achieve our desired alt)
    static int32_t integral = 0;
    static int32_t previousError = 0;

    float tP = 0.05, tI = 0.02, tD = 0.03;

    int32_t currentAltitude = getEstimatedAltitude();

    const int32_t error = (targetAltitude - currentAltitude);
    const int32_t derivative = error - previousError;
    
    integral = constrain(integral + error, -100, 100);

    previousError = error;

    pidState.altGain = (tP * error + tI * integral + tD * derivative);
}

void calculateThrottleAndTilt()
{
    
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
