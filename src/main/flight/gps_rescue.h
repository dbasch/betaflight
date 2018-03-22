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

//configuration parameters
uint16_t hoverThrottle;
uint16_t throttleMax;
uint16_t descentDistance; //XXX TODO: Document units for these
int16_t rescueAngle;
uint16_t initialAltitude;
int32_t netThrottle;
uint16_t tP, tI, tD;


extern bool canUseGPSHeading;
extern int16_t gpsRescueAngle[ANGLE_INDEX_COUNT];

enum {
    NAV_RTH_NO_ALT          = 0,            // Maintain current altitude
    NAV_RTH_EXTRA_ALT       = 1,            // Maintain current altitude + predefined safety margin
    NAV_RTH_CONST_ALT       = 2,            // Climb/descend to predefined altitude
    NAV_RTH_MAX_ALT         = 3,            // Track maximum altitude and climb to it when RTH
    NAV_RTH_AT_LEAST_ALT    = 4,            // Climb to predefined altitude if below it
 };

 uint16_t final_altitude;

void updateGPSRescueState(void);
void applyGPSRescueAltitude(void);