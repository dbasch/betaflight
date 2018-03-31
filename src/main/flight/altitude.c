/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"
#include "common/filter.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/altitude.h"
#include "flight/imu.h"
#include "flight/pid.h"

#include "io/gps.h"

#include "rx/rx.h"

#include "sensors/sensors.h"
#include "sensors/barometer.h"
#include "sensors/rangefinder.h"
#include "sensors/acceleration.h"


int32_t AltHold;
static int32_t estimatedVario = 0;                      // variometer in cm/s
static int32_t estimatedAltitude = 0;                // in cm

enum {
    DEBUG_ALTITUDE_ACC,
    DEBUG_ALTITUDE_VEL,
    DEBUG_ALTITUDE_HEIGHT
};

// 40hz update rate (20hz LPF on acc)
#define BARO_UPDATE_FREQUENCY_40HZ (1000 * 25)
#define GPS_UPDATE_FREQUENCY_5HZ (1000 * 200)

#if defined(USE_ALT_HOLD)

PG_REGISTER_WITH_RESET_TEMPLATE(airplaneConfig_t, airplaneConfig, PG_AIRPLANE_CONFIG, 0);

PG_RESET_TEMPLATE(airplaneConfig_t, airplaneConfig,
    .fixedwing_althold_reversed = false
);

static int32_t setVelocity = 0;
static uint8_t velocityControl = 0;
static int32_t errorVelocityI = 0;
static int32_t altHoldThrottleAdjustment = 0;
static int16_t initialThrottleHold = 0;


#define DEGREES_80_IN_DECIDEGREES 800

static void applyFixedWingAltHold(void)
{
    // handle fixedwing-related althold. UNTESTED! and probably wrong
    // most likely need to check changes on pitch channel and 'reset' althold similar to
    // how throttle does it on multirotor

    rcCommand[PITCH] += altHoldThrottleAdjustment * GET_DIRECTION(airplaneConfig()->fixedwing_althold_reversed);
}

void applyAltHold(void)
{
    if (STATE(FIXED_WING)) {
        applyFixedWingAltHold();
    } // else multirotoralthold
}

void updateAltHoldState(void)
{

    // Baro alt hold activate
    if (!IS_RC_MODE_ACTIVE(BOXBARO)) {
        DISABLE_FLIGHT_MODE(BARO_MODE);
        return;
    }

    if (!FLIGHT_MODE(BARO_MODE)) {
        ENABLE_FLIGHT_MODE(BARO_MODE);
        AltHold = estimatedAltitude;
        initialThrottleHold = rcData[THROTTLE];
        errorVelocityI = 0;
        altHoldThrottleAdjustment = 0;
    }
}

void updateRangefinderAltHoldState(void)
{
    // Sonar alt hold activate
    if (!IS_RC_MODE_ACTIVE(BOXRANGEFINDER)) {
        DISABLE_FLIGHT_MODE(RANGEFINDER_MODE);
        return;
    }

    if (!FLIGHT_MODE(RANGEFINDER_MODE)) {
        ENABLE_FLIGHT_MODE(RANGEFINDER_MODE);
        AltHold = estimatedAltitude;
        initialThrottleHold = rcData[THROTTLE];
        errorVelocityI = 0;
        altHoldThrottleAdjustment = 0;
    }
}

bool isThrustFacingDownwards(attitudeEulerAngles_t *attitude)
{
    return ABS(attitude->values.roll) < DEGREES_80_IN_DECIDEGREES && ABS(attitude->values.pitch) < DEGREES_80_IN_DECIDEGREES;
}

int32_t calculateAltHoldThrottleAdjustment(int32_t vel_tmp, float accZ_tmp, float accZ_old)
{
    int32_t result = 0;
    int32_t error;
    int32_t setVel;

    if (!isThrustFacingDownwards(&attitude)) {
        return result;
    }

    // Altitude P-Controller

    if (!velocityControl) {
        error = constrain(AltHold - estimatedAltitude, -500, 500);
        error = applyDeadband(error, 10); // remove small P parameter to reduce noise near zero position

        //DEBUG_SET(DEBUG_ALTITUDE, 1, error);

        setVel = constrain((currentPidProfile->pid[PID_ALT].P * error / 128), -300, +300); // limit velocity to +/- 3 m/s
    } else {
        setVel = setVelocity;
    }
    // Velocity PID-Controller

    // P
    error = setVel - vel_tmp;
    result = constrain((currentPidProfile->pid[PID_VEL].P * error / 32), -300, +300);

    // I
    errorVelocityI += (currentPidProfile->pid[PID_VEL].I * error);
    errorVelocityI = constrain(errorVelocityI, -(8192 * 200), (8192 * 200));
    result += errorVelocityI / 8192;     // I in range +/-200

    // D
    result -= constrain(currentPidProfile->pid[PID_VEL].D * (accZ_tmp + accZ_old) / 512, -150, 150);

    //DEBUG_SET(DEBUG_ALTITUDE, 2, result);

    return result;
}
#endif // USE_ALT_HOLD


#if defined(USE_BARO) || defined(USE_GPS)
void calculateEstimatedAltitude(timeUs_t currentTimeUs)
{
    static timeUs_t previousTimeUs = 0;
    const uint32_t dTime = currentTimeUs - previousTimeUs;
    if (dTime < BARO_UPDATE_FREQUENCY_40HZ) {
        return;
    }
    previousTimeUs = currentTimeUs;

    static float vel = 0.0f;
    static float accAlt = 0.0f;

    int32_t baroAlt = 0;
    static int32_t baroAltOffset = 0;
    static float zgOffset = 0;
    int32_t gpsAlt = 0;
    bool haveBaroAlt = false;
    bool haveGPSAlt = false;
#ifdef USE_BARO
    if (sensors(SENSOR_BARO)) {
        if (!isBaroCalibrationComplete()) {
            performBaroCalibrationCycle();
            vel = 0;
            accAlt = 0;
        } else {
            baroAlt = baroCalculateAltitude();


            haveBaroAlt = true;
        }
    }
#endif

#ifdef USE_GPS
    if (sensors(SENSOR_GPS)) {
        gpsAlt = gpsSol.llh.alt;
        haveGPSAlt = true;
    }
#endif

    float accZ_tmp = 0;
#ifdef USE_ACC
    if (sensors(SENSOR_ACC)) {
        static float velocityFromAcc = 0.0;
        quaternion q;
        getQuaternion(&q);

        float zg = (acc.accADC[Z] / acc.dev.acc_1G) - (q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z);
        if (!ARMING_FLAG(ARMED)) {
                zgOffset = zg;
        }
        zg -= zgOffset;

        //integrate acceleration to get velocity
        float prevVel = velocityFromAcc;
        velocityFromAcc += zg * 981 * dTime / 1000000.f;
        //DEBUG_SET(DEBUG_ALTITUDE, 0, (int32_t)velocityFromAcc);
        //DEBUG_SET(DEBUG_ALTITUDE, 1, (int32_t)(zg * 100));


        accAlt += (velocityFromAcc + prevVel) / 2 * dTime / 1000000.f;
    }
#endif




    //DEBUG_SET(DEBUG_ALTITUDE, DEBUG_ALTITUDE_ACC, accSum[2] / accSumCount);
    //DEBUG_SET(DEBUG_ALTITUDE, DEBUG_ALTITUDE_VEL, vel);
    //DEBUG_SET(DEBUG_ALTITUDE, DEBUG_ALTITUDE_HEIGHT, accAlt);

    imuResetAccelerationSum();

    int32_t baroVel = 0;
#ifdef USE_BARO
    if (sensors(SENSOR_BARO)) {
        if (!isBaroCalibrationComplete()) {
            return;
        }

        static int32_t lastBaroAlt = 0;
        baroVel = (baroAlt - lastBaroAlt) * 1000000.0f / dTime;
        lastBaroAlt = baroAlt;

        baroVel = constrain(baroVel, -1500, 1500);  // constrain baro velocity +/- 1500cm/s
        baroVel = applyDeadband(baroVel, 10);       // to reduce noise near zero
    }
#endif // USE_BARO

    // apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity).
    // By using CF it's possible to correct the drift of integrated accZ (velocity) without loosing the phase, i.e without delay
    vel = vel * CONVERT_PARAMETER_TO_FLOAT(barometerConfig()->baro_cf_vel) + baroVel * (1.0f - CONVERT_PARAMETER_TO_FLOAT(barometerConfig()->baro_cf_vel));
    int32_t vel_tmp = lrintf(vel);


      if (!ARMING_FLAG(ARMED)) {
        baroAltOffset = baroAlt;
      }
      baroAlt -= baroAltOffset;

    float gpsTrust = 100.0/gpsSol.hdop;
     if (gpsTrust > 0.9) gpsTrust = 0.9;


    if (haveGPSAlt && haveBaroAlt) {
        //if they are close enough, reset accAlt
        if (ABS(gpsAlt - baroAlt) < 1000) {
             accAlt = (gpsAlt * gpsTrust + baroAlt * (1-gpsTrust)) / 2;
        }
     }


    // to do: weight this by dop: more dop, less weight for gps alt
    estimatedAltitude = (gpsAlt * gpsTrust + baroAlt * (1-gpsTrust) + accAlt) / 3;

     DEBUG_SET(DEBUG_ALTITUDE, 0, 100 * (int32_t)gpsTrust);

     DEBUG_SET(DEBUG_ALTITUDE, 1, accAlt);
     DEBUG_SET(DEBUG_ALTITUDE, 2, baroAlt);
     DEBUG_SET(DEBUG_ALTITUDE, 3, gpsAlt);

    // set vario
    estimatedVario = applyDeadband(vel_tmp, 5);

#ifdef USE_ALT_HOLD
    static float accZ_old = 0.0f;
    altHoldThrottleAdjustment = calculateAltHoldThrottleAdjustment(vel_tmp, accZ_tmp, accZ_old);
    accZ_old = accZ_tmp;
#else
    UNUSED(accZ_tmp);
#endif
}
#endif

int32_t getEstimatedAltitude(void)
{
   //return estimatedAltitude;
   return gpsSol.llh.alt;
}

int32_t getEstimatedVario(void)
{
    return estimatedVario;
}

void setAltitude(uint16_t targetAltitude) {
    AltHold = targetAltitude * 100; //Convert meters to cm
}