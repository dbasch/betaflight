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
#include "common/axis.h"
#include "io/gps.h"

gpsLocation_t home;
uint16_t      distanceToHome;        // distance to home point in meters
int16_t       directionToHome;
uint16_t      rescueThrottle;

extern bool canUseGPSHeading;
extern int16_t gpsRescueAngle[ANGLE_INDEX_COUNT];

typedef enum {
    RESCUE_IDLE,
    RESCUE_INITIALIZE,
    RESCUE_ATTAIN_ALT,
    RESCUE_CROSSTRACK,
    RESCUE_LANDING_APPROACH,
    RESCUE_LANDING,
    RESCUE_ABORT,
    RESCUE_COMPLETE
} rescuePhase_e;

typedef enum {
    RESCUE_HEALTHY,
    RESCUE_TOO_HIGH,
    RESCUE_TOO_FAR,
    RESCUE_CRASH_DETECTED
} rescueFailureState_e;

typedef struct {
    int32_t currentAltitude;
    int32_t maxAltitude;
    int32_t currentGroundspeed;
    uint32_t distanceToHome;
    bool previouslyAngleMode;
} rescueInfo_s;

typedef struct {
    int32_t targetAltitude;
    int32_t destinationAltitude;
    int32_t targetGroundspeed;
} rescueIntent_s;

typedef struct {
    rescuePhase_e phase;
    rescueFailureState_e failure;
    rescueInfo_s info;
    rescueIntent_s intent;
} rescueState_s;

void updateGPSRescueState(void);
void applyAltitude(void);
void applyAngle(void);
void setPitch(int16_t pitch);
void setThrottle(int16_t throttle);
void moveTowardsTarget(void);
void idleTasks(void);
void moveTowardsTargetEnvelope(void);
void rescueStop(void);
void rescueStart(void);
void setBearing(int16_t deg);
void performSanityChecks(void);
void resetAngles(void);
