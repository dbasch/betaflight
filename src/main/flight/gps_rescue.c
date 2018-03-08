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
#include "io/gps.h"
#include "fc/runtime_config.h"

gpsLocation_t home;
uint16_t      distanceToHome;        // distance to home point in meters                                                                                               
int16_t       directionToHome;
bool        canUseGPSHeading = true; // We will expose this to the IMU so we know when to use gyro only

#include "flight/altitude.h"
#include "flight/gps_rescue.h"

/*
 If we have new GPS data, update home heading
 if possible and applicable.
*/
void rescueNewGpsData(void) {
    if (FLIGHT_MODE(GPS_RESCUE_MODE)) {
        //really we don't want alt hold, but we have to start somewhere
        applyAltHold();
    }
    else {
	//update state and move on
    }
}

void setDesiredAltitude(void) {

}