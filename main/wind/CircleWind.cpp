/***********************************************************************
 **
 **   CircleWind.cpp
 **
 **   This file is part of Cumulus
 **
 ************************************************************************
 **
 **   Copyright (c):  2002      by Andre Somers
 **                   2009-2015 by Axel Pauli
 **
 **   This file is distributed under the terms of the General Public
 **   License. See the file COPYING for more information.
 **
 ***********************************************************************/

#include "CircleWind.h"
#include "Units.h"
#include "wind/StraightWind.h"
#include "setup/SetupNG.h"
#include "math/Floats.h"
#include "math/Trigonometry.h"
#include "logdef.h"

#include <cmath>
#include <math.h>


/*
  About Wind analysis

  Currently, the wind is analyzed by finding the minimum and the maximum
  ground speeds measured while flying a circle. The direction of the wind is taken
  to be the direction in which the speed reaches it's maximum value, the speed
  is half the difference between the maximum and the minimum speeds measured.
  A jitter parameter, based on the number of circles already flown (the first
  circles are taken to be less accurate) and the angle between the headings at
  minimum and maximum speeds, is calculated in order to be able to weigh the
  resulting measurement. This method do not more work for wind speeds < 5Km/h.

  There are other options for determining the wind speed. You could for instance
  add all the vectors in a circle, and take the resulting vector as the wind speed.
  This is a more complex method, but because it is based on more heading/speed
  measurements by the GPS, it is probably more accurate. If equipped with
  instruments that pass along airspeed, the calculations can be compensated for
  changes in airspeed, resulting in better measurements. We are now assuming
  the pilot flies in perfect circles with constant airspeed, which is of course
  not a safe assumption.
  The jitter indication we are calculation can also be approached differently,
  by calculating how constant the speed in the circle would be if corrected for
  the wind speed we just derived. The more constant, the better. This is again
  more CPU intensive, but may produce better results.

  Some of the errors made here will be averaged-out by the WindStore, which keeps
  a number of wind measurements and calculates a weighted average based on jitter.

 */



constexpr const rad_t MINTURNINGDIFF = Units::deg_to_rad(4);

CircleWind *circleWind = nullptr;

int16_t CircleWind::_age = 9000;

CircleWind::CircleWind() :
    _lp_headdiff(0.3)
{
	// Initialization
    status = "idle";
    restartCycle(true);
}

CircleWind::~CircleWind()
{}

/** Called if a new sample is available in the sample list. */
void CircleWind::newSample() {
    // circle detection
    rad_t headingDiff = _lp_headdiff.filter(Vector::angleDiff(flarmVec.getAngleRad(), lastHeading));
    calcFlightMode(headingDiff, flarmVec.getSpeed());
    if (flightMode == circling_t::circlingL || flightMode == circling_t::circlingR) {
        circleArc += fabs(headingDiff);
    }

    lastHeading = flarmVec.getAngleRad();
    if (flightMode != circling_t::circlingL && flightMode != circling_t::circlingR) {
        // ESP_LOGI(FNAME,"FlightMode not circling %d", flightMode );
        status = "Not Circling";
        circleArc = 0;
        return;
    }
    status = "Sampling";
    // ESP_LOGI(FNAME,"GPS Sample, dir:%3.2f° speed:%3.2f", flarmVec.getAngleDeg(), flarmVec.getSpeed() );

    if (flarmVec.getSpeed() < minVector.getSpeed()) {
        // New minimum speed detected
        minVector = flarmVec;
        minVecTas = tas.get();
        // ESP_LOGI(FNAME,"new minimum detected:  %3.2f°/%3.2f mps", minVector.getAngleDeg(), minVector.getSpeed() );
    }

    if (flarmVec.getSpeed() > maxVector.getSpeed()) {
        // New maximum speed detected
        maxVector = flarmVec;
        maxVecTas = tas.get();
        // ESP_LOGI(FNAME,"new maximum detected: %3.2f°/%3.2f mps",  maxVector.getAngleDeg(), maxVector.getSpeed() );
    }

    if (circleArc > deg2rad(361.f))  // a bit more than one circle to ensure both ends are in
    {
        status = "Calculating";
        circleCount++;  // increase the number of circles flown (used to determine the jitter)
        ESP_LOGI(FNAME, "full circle made, circles %d last circle had %f °", circleCount, Units::rad_to_deg(circleArc));
        calcWind();  // calculate the wind for this circle
        restartCycle(false);
    }
}

void CircleWind::calcFlightMode(rad_t headingDiff, mps_t speed) {
    // ESP_LOGI(FNAME,"calcFlightMode head diff:%3.2f gs:%3.2f", headingDiff, speed  );
    if (speed < Units::kmh_to_mps(25)) {
        if (flightMode != circling_t::undefined) {
            newFlightMode(circling_t::undefined);
            flightMode = circling_t::undefined;
            ESP_LOGI(FNAME, "New flightmode: undefined, no GS");
        }
    } else {
        if (headingDiff > MINTURNINGDIFF) {
            if (turn_right < 4)  // hold down
                turn_right++;
            fly_straight = 0;
            if (flightMode != circling_t::circlingR && turn_right > 2) {
                newFlightMode(circling_t::circlingR);
                flightMode = circling_t::circlingR;
                // ESP_LOGI(FNAME,"New flightmode: circle right");
            }
        } else if (headingDiff < -MINTURNINGDIFF) {
            if (turn_left < 4)
                turn_left++;
            fly_straight = 0;
            if (flightMode != circling_t::circlingL && turn_left > 2) {
                newFlightMode(circling_t::circlingL);
                flightMode = circling_t::circlingL;
                // ESP_LOGI(FNAME,"New flightmode: circle left");
            }
        } else {
            turn_left = turn_right = 0;
            if (fly_straight < 4)
                fly_straight++;
            if (flightMode != circling_t::straight && fly_straight > 2) {
                newFlightMode(circling_t::straight);
                flightMode = circling_t::straight;
                // ESP_LOGI(FNAME,"New flightmode: straight");
            }
        }
    }
}

/** Called if the flight mode changes */
void CircleWind::newFlightMode(circling_t newFlightMode) {
    // Reset the circle counter for each flight mode change. The important thing
    // to measure is the number of turns in a thermal per turn direction.
    ESP_LOGI(FNAME, "newFlightMode %d", (uint8_t)newFlightMode);
    if (newFlightMode == circling_t::circlingL || newFlightMode == circling_t::circlingR) {
        if (circlingMode != newFlightMode) {
            circlingMode = newFlightMode;
            restartCycle(true);
        }
    }
}

const char* CircleWind::getFlightModeStr() const {
    if (flightMode == circling_t::straight)
        return "straight";
    else if (flightMode == circling_t::circlingL)
        return "circle left";
    else if (flightMode == circling_t::circlingR)
        return "circle right";
    else
        return "undefined";
}

bool CircleWind::getWind(int16_t* dir, mps_t* speed) {
    *dir = fast_iroundf(Units::rad_to_deg(cwind_dir.get()));
    *speed = cwind_speed.get();
    if (_age < 7200) {
        return true;
    } else {
        return false;
    }
}

void CircleWind::resetAge() {
    // ESP_LOGI(FNAME,"resetAge");
    _age = 0;
}

void CircleWind::calcWind() {
    // the angle difference between the minimum and maximum speed vectors should be 180 degree, so we add 180 degree to the max vector
    rad_t aDiff = Vector::angleDiff(minVector.getAngleRad() + My_PIf, maxVector.getAngleRad());

    Vector maxVec = maxVector;
    maxVec.setAngleRad(maxVector.getAngleRad() + My_PIf);
    // rad_t aDiff2 = std::acos(minVector.dot(maxVec) / (minVector.getSpeed() * maxVec.getSpeed())); just to check
    ESP_LOGI(FNAME, "calcWind, min/max diff %3.2f", Units::rad_to_deg(aDiff));

    rad_t delta = abs(aDiff);
    if (delta > (Units::deg_to_rad(max_circle_wind_diff.get()))) {
        ESP_LOGI(FNAME, "true course jitter bad %3.1f > %3.1f: Drop Sample ", Units::rad_to_deg(delta), max_circle_wind_diff.get());
        status = "Too Low Qual";
        return;  // Measurement jitter too low
    }

    // take both directions for min and max vector into account
    ESP_LOGI(FNAME, "maxGndV=%3.1f°/%.0f   minGndV=%3.1f°/%.0f Jitter=%2.1f°", maxVector.getAngleDeg(), maxVector.getSpeed(),
             minVector.getAngleDeg(), minVector.getSpeed(), Units::rad_to_deg(aDiff));

    // the direction of the wind is the direction where the greatest speed occurred
    result = maxVec + minVector; // Pointing into the wind

    // The speed of the wind is half the difference between the minimum and the maximum speeds.
    // compensate for variable TAS
    result.setSpeed((maxVector.getSpeed() - maxVecTas - minVector.getSpeed() + minVecTas) / 2.0);

    // Let the world know about our measurement!
    // float dir = Vector::normalizePI2((Vector::normalizePI(maxVec.getAngleRad()) + Vector::normalizePI(minVector.getAngleRad())) / 2.0);
    // float speed = (maxVector.getSpeed() - minVector.getSpeed()) / 2.0;
    // ESP_LOGI(FNAME, "### RAW CircleWind: %3.1f°/%.1fmps vs. %3.1f°/%.1fmps", result.getAngleDeg(), result.getSpeed(), Units::rad_to_deg(dir), speed);
    newWind(result.getAngleRad(), result.getSpeed());
}

// Jitter in the signal leads to higher windspeed measures as delta's grow
// The new algorithm considers jitter and corrects windspeed according to measured jitter value
void CircleWind::newWind( rad_t angle, mps_t speed ){
	ESP_LOGI(FNAME,"New Wind Vector angle %.1f speed %.1fmps", Units::rad_to_deg(angle), speed );

	windVectors.push_back( Vector( angle, speed ) );
	while( windVectors.size() > (int)circle_wind_lowpass.get() ){
		windVectors.pop_front();
	}
	result = Vector( 0.0, 0.0 );
	for( auto it=windVectors.begin(); it != windVectors.end(); it++ ){
		result.add( *it );
		ESP_LOGI(FNAME,"angle %.1f speed %.1f", it->getAngleDeg(), it->getSpeed() );
	}
	rad_t direction = result.getAngleRad();
	mps_t windspeed = result.normBy((float)windVectors.size());

	ESP_LOGI(FNAME,"### NEW AVG CircleWind: %.1f°/%.1fKmh", Units::rad_to_deg(direction), Units::mps_to_kmh(windspeed));

	cwind_dir.set(direction);
    cwind_speed.set(windspeed, true, false);

    rad_t deltaDir = abs( Vector::angleDiff( lastWindDir, angle ) );
	mps_t deltaSpeed = abs( lastWindSpeed - speed );
	lastWindDir = angle;
	lastWindSpeed = speed;

	if( deltaDir < Units::deg_to_rad(max_circle_wind_delta_deg.get()) && deltaSpeed < Units::kmh_to_mps(max_circle_wind_delta_speed.get()) ){
		if ( straightWind ) {
			straightWind->newCirclingWind( direction, windspeed );
		}
	}else{
		status = "Delta too big";
	}
}

void CircleWind::tick(){
	_age++;
	// ESP_LOGI(FNAME,"age: %d CWD: %.1f CWS %.1f", _age, cwind_dir.get(), cwind_speed.get() );
}


void CircleWind::restartCycle( bool clean ){
	// ESP_LOGI(FNAME,"restartCycle( clean=%d)", clean  );
	if( clean ) {
		circleCount = 0;
	}
	circleArc = 0;
    _lp_headdiff.reset(0.f);
	minVector.setSpeedKmh( 370.0 );
	maxVector.setSpeed( 0.0 );
}

void CircleWind::newConstellation( int numSat )
{
	ESP_LOGI(FNAME,"newConstellation num sat:%d", numSat );
	satCnt = numSat;

	if( satCnt < minSatCnt )
	{
		ESP_LOGW(FNAME,"Sat Constellation below minimum");
		// we are active, but the satellite count drops below minimum
		status = "< 5 Sat";
		restartCycle( true );
	}
}

void CircleWind::setGpsStatus( bool newStatus )
{
	if( gpsStatus != newStatus && newStatus )
	{
		ESP_LOGI(FNAME,"gpsStatusChange status:%d", newStatus );
		// we are not active because we had no GPS fix but that has been
		// changed now. So we become active.
		// Initialize analyzer-parameters
		gpsStatus = newStatus;
		if( ! gpsStatus ) {
			status = "Bad GPS";
        }
		restartCycle( true );
	}
}
