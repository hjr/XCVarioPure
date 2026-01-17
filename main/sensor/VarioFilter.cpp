#include "VarioFilter.h"

#include "pressure/PressureSensor.h"
#include "press_diff/AirspeedSensor.h"
#include "Atmosphere.h"
#include "S2F.h"
#include "AverageVario.h"
#include "setup/SetupNG.h"
#include "logdefnone.h"


#include <cmath>

const double sigmaAdjust = 255 * 2.0/33;  // 2 Vss

void VarioFilter::begin( PressureSensor *te, PressureSensor *baro, S2F *aS2F  ) {
	_sensorTE = te;
	_sensorBARO = baro;
	myS2F = aS2F;
	avgTE.setLength(vario_av_delay.get());
}

double VarioFilter::readAVGTE() {
	return _avgTE;
}

// float VarioFilter::readS2FTE() {
// 	return _TEF;
// }


static int lastrts = 0;

void VarioFilter::configChange(){
	float damping = vario_delay.get();
	_damping_factor = (1.0/(damping));
	int filter_len = rint(damping*(10.0/3));
	TEavg.setLength( filter_len );
	ESP_LOGI(FNAME, "configChange damping:%f filter_len:%d", damping, filter_len );
}

void VarioFilter::setup() {
	_qnh = QNH.get();
	configChange();
	lastrts = Clock::getMillis();
	vTaskDelay(pdMS_TO_TICKS(100));
	bool success;
	float curr_altitude = _sensorTE->readAltitude(_qnh, success ) * 1.03; // we want have some beep when powerd on
	if( success ){
		lastAltitude = curr_altitude;
		predictAlt = curr_altitude;
		Altitude = curr_altitude;
		averageAlt = curr_altitude;
		ESP_LOGI(FNAME, "Initial Alt=%0.1f",Altitude );
	}else{
		ESP_LOGE(FNAME, "Initial Alt read error Alt=%0.1f",Altitude );
	}
}


double VarioFilter::readTE( float tas, float tep ) {
	bool success;
	N++;
	// Latency supervision and correction
	int rts = Clock::getMillis();
	float time_delta = (float)(rts - lastrts)/1000.0f;   // in seconds
	if( time_delta < 0.095 ) {   // ensure time_delta is 100 mS at least
		int addwait = (int)(100.0-time_delta*1000);
		// ESP_LOGW(FNAME, "Too short TE time time_delta <95 mS: %4.1f, add wait %d ", time_delta*1000, addwait );
		vTaskDelay(pdMS_TO_TICKS( addwait ));
		rts = Clock::getMillis();
		time_delta = (rts - lastrts)/1000.0f;
	}
	lastrts = rts;
	if( time_delta < 0.090 || time_delta > 0.2 ) {
		ESP_LOGW(FNAME,"Vario time_delta=%2.3f sec", time_delta );
	}

	float curr_altitude;
	float dynP = asSensor->getHead();
	if( te_comp_enable.get() == TE_TEK_EPOT ) {
		curr_altitude = altitude.get(); // already read
		if( !std::isnan(curr_altitude) )
			curr_altitude = lastAltitude;  // ignore readout when failed
		float mps = tas / 3.6;  // m/s
		float cw  = myS2F->cw( mps );
		float ealt = (((  (mps*mps)/19.62) * (1+(te_comp_adjust.get()/100.0) ))) * ( 1 - cw );  // Ekin ~ h = v²/2g  * adjust * (1-cw)
		curr_altitude += ealt;
		ESP_LOGD(FNAME,"Energiehöhe @%0.1f km/h: %0.1f cw: %f", tas, ealt, cw );
	}
	else if( te_comp_enable.get() == TE_TEK_PRESSURE ){
		float barP = _sensorBARO->getHead();
		curr_altitude = Atmosphere::calcAltitude(_qnh, barP-(dynP/100.0)*(1+(te_comp_adjust.get()/100.0) ));  // subtract PI pressure like TEK probe does
	}
	else { // TE_TEK_PROBE
		_sensorTE->readAltitude( _qnh, success );
		curr_altitude = Atmosphere::calcAltitude(_qnh, tep );
	}
	// ESP_LOGI(FNAME,"TE alt: %4.3f m, ST: %.1f PI: %.1f", _currentAlt, barP, (dynP*100) );
	averageAlt += (curr_altitude - averageAlt) * 0.1;
	float adiff = curr_altitude - Altitude;
	// ESP_LOGI(FNAME,"VarioFilter new alt %0.1f err %0.1f", _currentAlt, err);
	float diff = (abs(adiff) * 1000) + 1;
	if(diff > 1000000){  // more than 100 m altitude diff in 0.1 second not plausible ( > 400 km/h vertical ) -> handled by Kalman filter
		 ESP_LOGW(FNAME,"TE sensor delta OOB: %f m", diff/10000 );
	}
	float err = (abs(curr_altitude - predictAlt) * 1000) + 1;
	averageAlt += (curr_altitude - averageAlt) * 0.1;
	float kg = (diff / (err*_errorval + diff)) * _alpha;
	Altitude += (adiff) * kg;
	float altDiff = Altitude - lastAltitude;
	// ESP_LOGI(FNAME," altDiff %0.1f diff %0.1f", TE, diff);
	lastAltitude = Altitude;
	float TEAVG = TEavg( altDiff / time_delta );
	predictAlt = Altitude + (TEAVG * time_delta);
	_TEF += ((TEAVG - _TEF)) * _damping_factor;
	if( !(N%10) ){ // every second one sample
		_avgTE = avgTE( _TEF );
		// ESP_LOGI(FNAME," _avgTE: %f ", _avgTE);
	}
	AverageVario::newSample( _TEF );
	// Bird catcher
	if( (altDiff > 0.2) || (altDiff < -0.2) ){
		ESP_LOGI(FNAME,"Vario alt: %f, Vario: %f, Vario-AVG: %f, t-delta=%2.3f sec \b\b\b", curr_altitude, _TEF, TEAVG, time_delta );
	}
	return _TEF;
}


VarioFilter bmpVario; // fixme create only if needed.
