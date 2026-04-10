/*
 * Wind.cpp
 *
 *  Module to calculate straight wind
 *
 *  Created on: Mar 21, 2021
 *
 *  Author: Eckhard Völlm, Axel Pauli
 *
 *
       Calculate wind continuously by using wind triangle, see more here:
       http://klspublishing.de/downloads/KLSP%20061%20Allgemeine%20Navigation%20DREHMEIER.pdf

       The Wind Correction Angle is the angle between the Heading and the
       Desired Course:

       WCA = Heading - DesiredCourse
 *
 *
 *
 *  Last update: 2021-04-21
 */
#include "StraightWind.h"

#include "math/Units.h"
#include "protocol/ProtocolItf.h"
#include "protocol/NMEA.h"
#include "sensor/mag/Compass.h"
#include "Flarm.h"
#include "setup/SetupNG.h"
#include "sensor.h"
#include "sensor/imu/AccMPU6050.h"
#include "sensor/imu/GyroMPU6050.h"
#include "math/Trigonometry.h"
#include "math/Floats.h"
#include "comm/DeviceMgr.h"
#include "math/vector_3d_fwd.h"
#include "wind/CircleWind.h"
#include "logdef.h"

#include <esp_system.h>

#include <algorithm>
#include <cmath>

int16_t StraightWind::_age = 10000;


StraightWind::StraightWind() :
	averageTH( 0.0 ),
	averageTC( 0.0 ),
	averageGS(0.0),
	windDir( -1.0 ),
	windSpeed( -1.0 ),
	noWindMeasuring( false ),
	circlingWindDir( -1.0 ),
	circlingWVecDir( -1.0 ),
	circlingWindSpeed( -1.0 ),
	circlingWindAge( 0 ),
	airspeedCorrection( 1.0 ),
	_tick(0),
	gpsStatus(false),
	deviation_cur(0),
	magneticHeading(0),
	status( "Initial" ),
	jitter(0),
	newWindSpeed(0),
	newWindDir(0),
	slipAverage(0),
	lastHeading(0),
	lastGroundCourse(0)
{
}

void StraightWind::begin(){
	// if( compass_dev_auto.get() )
	// 	airspeedCorrection = wind_as_calibration.get();
}

void StraightWind::tick(){
	_age++;
	circlingWindAge++;
	_tick++;
}

bool StraightWind::getWind(int16_t *direction, mps_t *speed)
{
	*direction = fast_iroundf(Units::rad_to_deg(swind_dir.get()));
	*speed = swind_speed.get();
	if( _age < 7200 )
		return true;
	else
		return false;
}

/**
 * Measurement cycle for wind calculation in straight flight. Should be
 * triggered periodically, maybe once per second.
 *
 * Returns true, if a new wind was calculated.
 */
bool StraightWind::calculateWind()
{
	// ESP_LOGI(FNAME,"Straight wind, calculateWind()");
	if( Flarm::gpsStatus() == false ) {
		// GPS status not valid
		ESP_LOGI(FNAME,"Restart Cycle: GPS Status invalid");
		status="Bad GPS";
		gpsStatus = false;
	} else {
		gpsStatus = true;
	}

	// ESP_LOGI(FNAME,"calculateWind flightMode: %d", CircleStraightWind::getFlightMode() );
	// Check if straight wind requirements are fulfilled 
	// fixme
	return false; // currently no compass

	// if( ! theCompass || ! theCompass->isCalibrated() ) {
	// 	if( ! theCompass ) {
	// 		status="Compass not available";
	// 	}
	// 	else if( ! theCompass->isCalibrated() ) {
	// 		status="Compass not calibrated";
	// 	}
	// 	return false;
	// }

	// Get current ground speed in km/h
	mps_t cgs = Flarm::getGndSpeed();

	if( ! airborne.get() )
	{
		// We start a new measurement cycle.
		if( !noWindMeasuring ) {
			ESP_LOGI(FNAME,"Low Airspeed, stop wind calculation");
			noWindMeasuring = true;
		}
		status="Low AS";
	}else{
		if( noWindMeasuring ) {
			ESP_LOGI(FNAME,"Airspeed OK, start wind calculation");
			noWindMeasuring = false;
		}
	}
	// Get current true heading from compass.
	bool THok = false;
	// if( theCompass ) {
	// 	averageTH = theCompass->filteredTrueHeading( &THok, false ); // no deviation considered here (we add ourselfs as for reverse calculation we need also the pure heading)
	// }
	if( THok == false ) {
		// No valid heading available
		status="No Compass";
		if( wind_enable.get() & WA_STRAIGHT ){
			ESP_LOGI(FNAME,"Restart Cycle: No magnetic heading");
		}
	}

	// Get current true course from GPS .. Calculate average true course TC
	averageTC = Flarm::getGndCourse();
	// averageTC = Vector::normalize( averageTC + (ctc - averageTC) * 1/wind_gps_lowpass.get());
	averageGS += (cgs -averageGS) * 1/wind_gps_lowpass.get();

	// WCA in radians
	magneticHeading = averageTH;

	float deviation = 0.f; // theCompass->getDeviation( averageTH );

	if( false ) { // logging.get() != LOGG_DISABLE) && theCompass ){
		if( logging.get() & LOGG_WIND ){
			char log[ProtocolItf::MAX_LEN];
			sprintf( log, "$WIND;");
			int pos = strlen(log);
			sprintf( log+pos, "%d;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%d;%d;%.1f", _tick, Units::rad_to_deg(averageTC), 
					cgs, Units::rad_to_deg(averageTH), tas.get(), Units::rad_to_deg(newWindDir), Units::mps_to_kmh(newWindSpeed), 
					Units::rad_to_deg(windDir), Units::mps_to_kmh(windSpeed), Units::rad_to_deg(circlingWindDir), Units::mps_to_kmh(circlingWindSpeed), 
					(airspeedCorrection-1)*100, static_cast<uint8_t>(circleWind->getFlightMode()), gpsStatus, deviation );
			pos=strlen(log);
			sprintf( log+pos, "\n");
			const NmeaPrtcl *prtcl = DEVMAN->getNMEA(NAVI_DEV); // Todo preliminary solution ..
			if ( prtcl ) {
				prtcl->sendXCV(log);
			}
			ESP_LOGI( FNAME,"%s", log );
		}

		// if( logging.get() & LOGG_GYRO_MAG ){
		// 	char log2[ProtocolItf::MAX_LEN];
		// 	sprintf( log2, "$IMU;");
		// 	int pos = strlen(log2);
		// 	vector_f acc = accSensor->getHead();
		// 	vector_f gyrodeg = gyroSensor->getHead() * rad2deg(1.f);
		// 	sprintf( log2+pos, ";%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f",
		// 			theCompass->rawX()/16384.0,theCompass->rawY()/16384.0,theCompass->rawZ()/16384.0,
		// 			acc.x, acc.y, acc.z, gyrodeg.x, gyrodeg.y, gyrodeg.z );
		// 	pos = strlen(log2);
		// 	sprintf(log2+pos, "\n");
		// 	const NmeaPrtcl *prtcl = DEVMAN->getNMEA(NAVI_DEV); // Todo preliminary solution ..
		// 	if ( prtcl ) {
		// 		prtcl->sendXCV(log2);
		// 	}
		// 	ESP_LOGI( FNAME,"%s", log2 );
		// }
	}

	if( (circleWind->getFlightMode() != circling_t::straight) || noWindMeasuring || !THok || !gpsStatus ){
		// ESP_LOGI(FNAME,"In Circling, stop ");
		return false;
	}

	status="Calculating";
	// ESP_LOGI(FNAME,"%d TC: %3.1f GS:%3.1f TH: %3.1f (avg:%3.1f) TAS: %3.1f", nunberOfSamples, averageTC, cgs, cth, averageTH, tas.get() );
	calculateWind( averageTC, averageGS, averageTH, deviation );
	return true;
}

// length (or speed) of third vector in windtriangle
// and calculate WA (wind angle) in degree
// wind direction calculation taken from here:
// view-source:http://www.owoba.de/fliegerei/flugrechner.html
// direction in degrees of third vector in windtriangle
void StraightWind::calculateSpeedAndAngle( float angle1, mps_t speed1, float angle2, mps_t speed2, mps_t& speed, float& angle ){
	float tcrad = deg2rad( angle1 );
	float thrad = deg2rad( angle2 );
	float wca = Vector::angleDiff( thrad, tcrad );
	float s2wca = speed2 * cos( wca );
	float ang = tcrad + atan2( speed2 * sin( wca ), s2wca - speed1 );
	// Cosinus sentence: c^2 = a^2 + b^2 − 2 * a * b * cos( α ) for wind speed in km/h
	speed = sqrt( (speed2 * speed2) + (speed1 * speed1 ) - ( 2 * s2wca * speed1  ) );
	angle = Vector::normalizeDeg( rad2deg( ang ) );  // convert radian to degree
	// ESP_LOGI(FNAME,"calcAngleSpeed( A1/S1=%3.1f°/%3.1f km/h  A2/S2=%3.1f°/%3.1f km/h): A/S: %3.2f°/%3.2f km/h", angle1, speed1, angle2, speed2, angle, speed  );
}

float StraightWind::getAngle() { return swind_dir.get(); };
float StraightWind::getSpeed() { return swind_speed.get(); };

void StraightWind::calculateWind( float tc, mps_t gs, float th, float deviation  ){

	mps_t mytas = tas.get();
	// ESP_LOGI(FNAME,"calculateWind: TC:%3.1f GS:%3.1f TH:%3.1f TAS:%3.1f Dev:%2.2f", tc, gs, th, mytas, deviation );
	// Wind correction angle WCA
	if( gs < Units::kmh_to_mps(5.f) ){
		tc = th;   // what will deliver heading and airspeed for wind
	}
	// Wind speed
	// Reverse calculate windtriangle for deviation and airspeed calibration
	bool devOK = true;

	// Gating for proper heading alignment
	slipAverage += (slip_angle.get() - slipAverage)*0.0005;
	if( abs( slip_angle.get() - slipAverage) > swind_sideslip_lim.get() ){
		status = "Side Slip";
		ESP_LOGI( FNAME, "Slip overrun %.2f, average %.2f", slip_angle.get(), slipAverage );
		return;
	}
	float headingDelta = Vector::angleDiffDeg( th , lastHeading );
	lastHeading = th;
	if(  abs(headingDelta) > wind_straight_course_tolerance.get() ){
		ESP_LOGI(FNAME,"Not really straight flight, heading delta: %f", headingDelta );
		return;
	}
	float groundCourseDelta = Vector::angleDiffDeg( tc , lastGroundCourse );
	lastGroundCourse = tc;
	if(  abs(groundCourseDelta) > 7.5  ){
			ESP_LOGI(FNAME,"Not really straight flight, GND course delta: %f", groundCourseDelta );
			return;
	}

	if( circlingWindSpeed > 0 ) { // && compass_dev_auto.get() ){
		if( circlingWindAge > 1200 ){
			status = "OLD CIRC WIND";
			ESP_LOGI(FNAME,"Circling Wind exired");
		}else{
			mps_t airspeed = 0;
			float heading = 0;
			Vector wind( circlingWindDir, circlingWindSpeed );
			Vector groundTrack( tc, gs );
			groundTrack.add( wind );
			airspeed = groundTrack.getSpeed();
			heading = groundTrack.getAngleDeg();
//#ifdef VERBOSE_LOG
			ESP_LOGI(FNAME,"Using CWind: %.2f°/%.2f, TC/GS: %.1f°/%.1f, HD/AS: %.2f°/%.2f, tas=%.2f, ASdelta %.3f", circlingWindDir, circlingWindSpeed, tc, gs, heading, airspeed, mytas, airspeed-mytas );
// #endif
			if( abs( airspeed - mytas ) > wind_straight_speed_tolerance.get() ){  // 30 percent max deviation
				status = "AS OOB";
				ESP_LOGI(FNAME,"Estimated Airspeed/Groundspeed OOB, max delta: %f km/h, delta: %f km/h", wind_straight_speed_tolerance.get(), abs( airspeed - mytas ) );
				return;
			}
			airspeedCorrection +=  (airspeed/mytas - airspeedCorrection) * wind_as_filter.get();
			if( airspeedCorrection > 1.01 ) // we consider 1% as maximum needed correction
				airspeedCorrection = 1.01;
			else if( airspeedCorrection < 0.99 )
				airspeedCorrection = 0.99;
			if( abs( wind_as_calibration.get() - airspeedCorrection )*100 > 0.5 )
					wind_as_calibration.set( airspeedCorrection );
			// if( theCompass )
			// 	devOK = theCompass->newDeviation( th, heading );
			else{
				status = "No Compass";
				return;
			}
			// ESP_LOGI(FNAME,"Calculated TH/TAS: %3.1f°/%3.1f km/h  Measured TH/TAS: %3.1f°/%3.1f, asCorr:%2.3f, deltaAS:%3.2f, Age:%d", tH, airspeed, averageTH, mytas, airspeedCorrection , airspeed-mytas, circlingWindAge );
		}
	}else{
		status = "No Circ Wind";
		// float airspeed = calculateSpeed( windDir, windSpeed, tc, gs );
		// airspeedCorrection +=  (airspeed/mytas - airspeedCorrection) * wind_as_filter.get();
	}
	if( !devOK ){ // data is not plausible/useful
			ESP_LOGI( FNAME, "Calculated deviation out of bounds: Drop also this wind calculation");
			status = "Deviation OOB";
			return;
	}

    // wind speed and direction
	calculateSpeedAndAngle( tc, gs, th+deviation, mytas*airspeedCorrection, newWindSpeed, newWindDir );
	// ESP_LOGI( FNAME, "Calculated raw windspeed %.1f jitter:%.1f", newWindSpeed, jitter );

	Vector v;
	v.setAngleRad( newWindDir );
	v.setSpeed( newWindSpeed );

	windVectors.push_back( v );
	while( windVectors.size() > wind_filter_lowpass.get() ){
		windVectors.pop_front();
	}

	Vector result;
	for( auto it=windVectors.begin(); it != windVectors.end(); it++ ){
		result.add( *it );
		// ESP_LOGI(FNAME,"angle %.1f speed %.1f", it->getAngleDeg(), it->getSpeed() );
	}

	windDir   = result.getAngleRad(); // Vector::normalizeDeg( result.getAngleDeg()/circle_wind_lowpass.get() );
	windSpeed = result.normBy(windVectors.size());

	// ESP_LOGI(FNAME,"New AVG WindDirection: %3.1f deg,  Strength: %3.1f km/h JI:%2.1f", windDir, windSpeed, jitter );
	_age = 0;
	if( (int)windDir != (int)swind_dir.get()  ){
		swind_dir.set( windDir );
	}
	if( (int)windSpeed != (int)swind_speed.get() ){
		swind_speed.set( windSpeed );
	}
}


void StraightWind::newCirclingWind( rad_t angle, mps_t speed ){
	ESP_LOGI(FNAME,"New good circling wind %3.2f°/%3.2f", angle, speed );
	circlingWindDir = angle;
	circlingWVecDir = Vector::reverseBearingRad(angle); // windvector
	ESP_LOGI(FNAME,"Wind dir %3.2f, reverse circling wind dir %3.2f", angle, circlingWVecDir );
	circlingWindSpeed = speed;
	circlingWindAge = 0;
}

#ifdef Wind_Test

void StraightWind::test()
{    // Class Test, check here the results: http://www.owoba.de/fliegerei/flugrechner.html
	calculateWind( 90, Units::kmh_to_mps(100), 0, Units::kmh_to_mps(100), 0 );
	if( int( windSpeed ) != 141/3.6 || int(windDir +0.5) != 135/3.6 ) {
		ESP_LOGI(FNAME,"Failed");
	}
	calculateWind( 270, Units::kmh_to_mps(100), 0, Units::kmh_to_mps(100), 0 );
	if( int( windSpeed ) != 141/3.6 || int(windDir +0.5) != 225/3.6 ) {
		ESP_LOGI(FNAME,"Failed");
	}
	calculateWind( 0, Units::kmh_to_mps(100), 90, Units::kmh_to_mps(100), 0 );
	if( int( windSpeed ) != 141/3.6 || int(windDir +0.5) != 315/3.6 ) {
		ESP_LOGI(FNAME,"Failed");
	}
	calculateWind( 0, Units::kmh_to_mps(100), 270, Units::kmh_to_mps(100), 0 );
	if( int( windSpeed ) != 141/3.6 || int(windDir +0.5) != 45/3.6 ) {
		ESP_LOGI(FNAME,"Failed");
	}

	calculateWind( 90, Units::kmh_to_mps(100), 180, Units::kmh_to_mps(100), 0 );
	if( int( windSpeed ) != 141/3.6 || int(windDir +0.5) != 45/3.6 ) {
		ESP_LOGI(FNAME,"Failed");
	}

	calculateWind( 180, Units::kmh_to_mps(100), 270, Units::kmh_to_mps(100), 0 );
	if( int( windSpeed ) != 141/3.6 || int(windDir +0.5) != 135/3.6  ) {
		ESP_LOGI(FNAME,"Failed");
	}
}
#endif