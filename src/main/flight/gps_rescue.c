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

bool          canUseGPSHeading = true; // We will expose this to the IMU so we know when to use gyro only
bool          isDescending = false;
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
    Determine what phase we are in, determine if all criteria are met to move to the next phase
*/
void updateGPSRescueState(void) 
{
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
            break;
        case RESCUE_LANDING:
            // We have reached the XYZ envelope to be considered at "home".  We need to land gently and check our accelerometer for abnormal data.
            // At this point, do not let the target altitude go up anymore, so if we overshoot, we dont' move in a parabolic trajectory
            break;
        case RESCUE_COMPLETE:
            rescueState.rescuePhase = RESCUE_IDLE;
            break;
    }

    updateGroundspeedCalculation();
    updateAltitudeCalculation();
    calculateThrottleAndTilt();
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
    // Get highest altitude yadda yadda yadda
}

/**
    PID controller for determining 
*/
void updateGroundspeedCalculation()
{

}

void updateAltitudeCalculation()
{

}

void calculateThrottleAndTilt()
{

}
