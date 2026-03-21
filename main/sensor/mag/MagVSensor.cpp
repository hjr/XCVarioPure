/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "MagVSensor.h"

#include "Units.h"
#include "sensor/SensorMgr.h"
#include "vector.h"
#include "math/vector_3d.h"
#include "protocol/Clock.h"
#include "sensor/imu/AccMPU6050.h"
#include "driver/gpio/ESPRotary.h"
#include "logdefnone.h"


constexpr int DUTY_CYCLE_MS = 500; // 2Hz
// provide a 10Hz sized buffer, but only process it with 2Hz
static vector_f mag_buffer[ (SENSOR_HISTORY_DURATION_MS / 100) + 1 ];

MagVSensor* magSensor = nullptr;

MagVSensor::MagVSensor() : SensorTP<vector_f>(mag_buffer, DUTY_CYCLE_MS),
    _lpf_heading(LowPassFilterT<float>::alphaFromTau(1.f, DUTY_CYCLE_MS / 1000.f))
{
    _id = SensorId::MAGNETO;
    _latency_ms = 20; // estimated latency of CAN sensor
}

MagVSensor* MagVSensor::createMagVSensor() {
    if ( !magSensor ) {
        magSensor = new MagVSensor();
        magSensor->setup();
    }
    return magSensor;
}

bool MagVSensor::setup() {
    loadCalibration();

    mag_hdm.setInvalid();
    mag_hdt.setInvalid();

    return true;
}

void MagVSensor::postProcess()
{
    if ( ! getHeadValid() ) {
        mag_hdm.setInvalid();
        mag_hdt.setInvalid();
        ESP_LOGI(FNAME, "mag reading not valid");
        return;
    }
    const vector_f mv = getAVG(800); // avg over the last ca. 0.5sec
    ESP_LOGI(FNAME, "mag reading AVG(0.5s): (%.2f,%.2f,%.2f)", mv.x, mv.y, mv.z);
    assert(accSensor);
    vector_f d = accSensor->getAttVector().get_normalized();  // a down vector, relative to glider
    _processed = mv - d * mv.dot(d); // a north vector
    // vector_f north = accSensor->getAHRSQuaternion().conjugate().rotate(mv); // works but is more expensive than the above calculation
    rad_t heading = Vector::polar(-_processed.y, _processed.x);  // the "-" translates into a ENU nav frame
    // ESP_LOGI(FNAME, "mag (%.2f,%.2f,%.2f) heading %.2f", _processed.x, _processed.y, _processed.z, Units::rad_to_deg(heading));

    // publish the heading
    mag_hdm.set(_lpf_heading.filter(heading));
    mag_hdt.set(_lpf_heading.get() - Units::deg_to_rad(compass_declination.get()));
}

// this is called from the protocol handler when a new magnetometer reading is received from CAN
void MagVSensor::inject(int16_t x, int16_t y, int16_t z)
{
    vector_f mv;
    // rotate -90° around Z (the data sheet of the used QMC5883L sensor shows a wrong axes orientation)
    // the move to NED reference system does not need any other rotation anymore
    mv.x = -((float)y - _bias.y) * _scale.y;
    mv.y =  ((float)x - _bias.x) * _scale.x;
    mv.z =  ((float)z - _bias.z) * _scale.z;

    // backdate the timestamp to approximate the actual measurement time
    int time = Clock::getMillis();
    pushAndPublish(mv, time);
}


//////////////////////////////
// old calibration code, todo replace with proper auto-calibration
//////////////////////////////


/**
 * Calibrate compass by using the read x, y, z raw values. The calibration is
 * stopped by the reporter function which displays intermediate results of the
 * calibration action.
 */
bool MagVSensor::calibrate( void (*reporter)(const CompassCalibrationData &data, bool print), bool only_show  )
{
	bool ret = true;
	ESP_LOGI( FNAME, "calibrate/show magnetic sensor, only_show=%d ", only_show );
	if( !only_show ) {
		// reset all old calibration data
        _bias = {};
        _scale = { 1.f,1.f,1.f };
        CompassCalibrationData *data = new CompassCalibrationData;
		while( true )
		{
            // Grab a sample
            if ( magSensor->getHeadValid() && magSensor->getLevel() > 10) {
                data->sample = magSensor->getAVG(1300); // avg over the last ca. 1sec
                data->var = magSensor->getVariance(1300);

                // Evaluate the sample for calibration and update the calibration data
                calcCalibration(*data);

                // Send a calibration report to the subscriber
                reporter(*data, false);
            } else {
                ESP_LOGI(FNAME, "Compass reading not valid during calibration");
            }
			if( Rotary->readSwitch(100) )  // more responsive to query every loop
				break;
		}
		ESP_LOGI( FNAME, "Read Cal-Samples=%d", data->nrsamples );
		if( ! data->bits.allAxesGood() || data->nrsamples < 2 ) {
			ESP_LOGI( FNAME, "calibrate min-max xyz not enough samples");
			ret = false;
		}
        else {
            // save calibration
            saveCalibration(data->bias, data->scale);
            calibration_bits.set( data->bits );

            ESP_LOGI( FNAME, "Compass hard-iron: bias.x=%.3f, bias.y=%.3f, bias.z=%.3f", _bias.x, _bias.y, _bias.z );
            ESP_LOGI( FNAME, "Compass soft-iron: scale.x=%.3f, scale.y=%.3f, scale.z=%.3f",	_scale.x, _scale.y, _scale.z );
            ESP_LOGI( FNAME, "calibration end" );
        }

        delete data;
	}
    else {
		ESP_LOGI( FNAME, "Show Calibration");
        CompassCalibrationData *data = new CompassCalibrationData;
        data->bits = calibration_bits.get();
        data->bias = _bias;
        data->scale = _scale;
		reporter( *data, true );
        delete data;
		while( ! Rotary->readSwitch(100) ) ; // wait for button
	}

    return ret;
}

// calibration calculation in sync with data received in compass task
void MagVSensor::calcCalibration(CompassCalibrationData &data) {
	data.nrsamples++;
	// Variance low pass filtered
	// data.variance += (data.var.get_norm2() - data.variance) / 50.f;
    data.variance += (data.var.get_norm() - data.variance) / 5.f;
	ESP_LOGI( FNAME, "Mag Var: %7.3f", data.variance );

	// ESP_LOGI( FNAME, "Mag Var X:%.2f Y:%.2f Z:%.2f", data.var.x, data.var.y, data.var.z  );
	/* Find max/min peak values */
	data.min.x = (data.sample.x < data.min.x ) ? data.sample.x : data.min.x;
	data.min.y = (data.sample.y < data.min.y ) ? data.sample.y : data.min.y;
	data.min.z = (data.sample.z < data.min.z ) ? data.sample.z : data.min.z;
	data.max.x = (data.sample.x > data.max.x ) ? data.sample.x : data.max.x;
	data.max.y = (data.sample.y > data.max.y ) ? data.sample.y : data.max.y;
	data.max.z = (data.sample.z > data.max.z ) ? data.sample.z : data.max.z;

	constexpr float minval = (32768/100)*1.2; // 1.2% full scale
    constexpr float maxvar = 1000.f; // empirically determined maximum variance for a valid sample
    ESP_LOGI( FNAME, "Mag Sample: x=%.2f y=%.2f z=%.2f, < %.2f, var=%.2f < %.2f", data.sample.x, data.sample.y, data.sample.z, minval, data.variance, maxvar );
	if( abs(data.sample.x) < minval && abs(data.sample.y) < minval && data.variance < maxvar && data.sample.z > 2*minval  )
		data.bits.zmax_green = true;
	if( abs(data.sample.x) < minval && abs(data.sample.y) < minval && data.variance < maxvar && data.sample.z < -2*minval  )
		data.bits.zmin_green = true;
	if( abs(data.sample.x) < minval && abs(data.sample.z) < minval && data.variance < maxvar && data.sample.y > 2*minval  )
		data.bits.ymax_green = true;
	if( abs(data.sample.x) < minval && abs(data.sample.z) < minval && data.variance < maxvar && data.sample.y < -2*minval  )
		data.bits.ymin_green = true;
	if( abs(data.sample.y) < minval && abs(data.sample.z) < minval && data.variance < maxvar && data.sample.x > 2*minval  )
		data.bits.xmax_green = true;
	if( abs(data.sample.y) < minval && abs(data.sample.z) < minval && data.variance < maxvar && data.sample.x < -2*minval  )
		data.bits.xmin_green = true;

	if( data.nrsamples < 2 )
		return;

	// Calculate hard iron correction
	// calculate average x, y, z magnetic bias.x in counts
	data.bias.x = (data.max.x + data.min.x) / 2;
	data.bias.y = (data.max.y + data.min.y) / 2;
	data.bias.z = (data.max.z + data.min.z) / 2;

	// Calculate soft-iron scale factors
	// calculate average x, y, z axis max chord length in counts
    vector_f chord = data.max - data.min;
	float cord_avgerage = ( chord.x + chord.y + chord.z ) / 3.;
	data.scale.x = cord_avgerage / chord.x;
	data.scale.y = cord_avgerage / chord.y;
	data.scale.z = cord_avgerage / chord.z;
}


//////////////////////////////
// helpers
//////////////////////////////


// Loads a stored compass calibration. Returns true, if valid calibration
// data could be loaded, otherwise false.
bool MagVSensor::loadCalibration() {
    if (compass_calibrated.get() == 0) {
        ESP_LOGI(FNAME, "Compass calibration is not valid");
        // no stored calibration available
        return false;
    }

    _bias.x = compass_x_bias.get();
    _bias.y = compass_y_bias.get();
    _bias.z = compass_z_bias.get();
    _scale.x = compass_x_scale.get();
    _scale.y = compass_y_scale.get();
    _scale.z = compass_z_scale.get();

    ESP_LOGI(FNAME, "Read calibration: %f, %f, %f, %f, %f, %f, valid=%d", 
        _bias.x, _bias.y, _bias.z, _scale.x, _scale.y, _scale.z, compass_calibrated.get());

    return true;
}

/**
 * Resets the whole compass calibration, also the saved configuration.
 */
void MagVSensor::resetCalibration()
{
	_bias = {};
	_scale = { 1.f,1.f,1.f };

	// reset nonvolatile configuration data
	compass_x_bias.set( 0.f );
	compass_y_bias.set( 0.f );
	compass_z_bias.set( 0.f );
	compass_x_scale.set( 1.f );
	compass_y_scale.set( 1.f );
	compass_z_scale.set( 1.f );
	compass_calibrated.set( 0 );
}

/**
 * Saves a done compass calibration.
 */
void MagVSensor::saveCalibration(const vector_f &bias, const vector_f &scale)
{
    _bias = bias;
    _scale = scale;

	compass_x_bias.set( bias.x );
	compass_y_bias.set( bias.y );
	compass_z_bias.set( bias.z );
	compass_x_scale.set( scale.x );
	compass_y_scale.set( scale.y );
	compass_z_scale.set( scale.z );
	compass_calibrated.set( 1 );
}

