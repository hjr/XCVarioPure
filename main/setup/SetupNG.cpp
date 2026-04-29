/*
 * SetupNG.cpp
 *
 *  Created on: Dec 23, 2017
 *      Author: iltis
 */

#include "setup/SetupNG.h"

#include "IpsDisplay.h"
#include "math/Units.h"
#include "setup/CruiseMode.h"
#include "math/Quaternion.h"
#include "driver/audio/ESPAudio.h"
#include "glider/Polars.h"
#include "sensor.h"
#include "wind/StraightWind.h"
#include "wind/CircleWind.h"
#include "driver/audio/ESPAudio.h"
#include "Flap.h"
#include "comm/DeviceMgr.h"
#include "comm/CanBus.h"
#include "comm/Configuration.h"
#include "comm/SerialLine.h"
#include "protocol/NMEA.h"
#include "driver/time/AliveMonitor.h"
#include "protocol/nmea/SeeYouMsg.h"
#include "protocol/nmea/XCVSyncMsg.h"
#include "screen/element/Battery.h"
#include "screen/element/Altimeter.h"
#include "screen/element/PolarGauge.h"
#include "screen/element/MultiGauge.h"
#include "sensor/press_diff/AirspeedSensor.h"
#include "Atmosphere.h"
#include "sensor/VarioFilter.h"
#include "sensor/imu/AccMPU6050.h"
#include "logdefnone.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp32/rom/uart.h>
#include <esp_system.h>
#include <esp_http_server.h>
#include <string>


template <typename T>
SetupNG<T>::SetupNG( const char *akey, T adefault, bool reset, e_sync_t sync, e_volatility vol,
        void (* action)(), quantity_t quant, const limits_t *l) :
    SetupCommon(akey),
    _default(adefault),
    _limt(l)
{
    // ESP_LOGI(FNAME,"SetupNG(%s)", akey );
    // if( strlen( akey ) > 15 ) {
    // 	ESP_LOGE(FNAME,"SetupNG(%s) key > 15 char !", akey );
    // }
    flags._reset = reset;
    flags._sync = sync;
    flags._volatile = vol;
    flags._quant = (uint8_t)quant;
    _action = action;
}

template <typename T>
char SetupNG<T>::typeName(void) const {
    if constexpr (std::is_same_v<T, float>)
        return 'F';
    else if constexpr (std::is_same_v<T, int>)
        return 'I';
    else if constexpr (std::is_same_v<T, Quaternion>)
        return 'Q';
    else if constexpr (std::is_same_v<T, bitfield_compass>)
        return 'B';
    else if constexpr (std::is_same_v<T, axes_i16_abi>)
        return 'A';
    else if constexpr (std::is_same_v<T, DeviceNVS>)
        return 'D';
    else
        return 'U';
}

template <typename T>
std::string SetupNG<T>::getValueAsStr() const {
    std::string str;
    if( flags._volatile != VOLATILE ){
        if constexpr (std::is_same_v<T, float>) {
            str = std::to_string(_value);
        }
        else if constexpr (std::is_same_v<T, int>) {
            str = std::to_string(_value);
        }
        else if constexpr (std::is_same_v<T, Quaternion>) {
            str = std::to_string(_value._w) + '/' + std::to_string(_value._x) + '/' + std::to_string(_value._y) + '/' + std::to_string(_value._z);
        }
        else if constexpr (std::is_same_v<T, bitfield_compass>) {
            uint8_t as_byte = *reinterpret_cast<const uint8_t*>(&_value);
            str = std::to_string(as_byte);
        }
        else if constexpr (std::is_same_v<T, axes_i16_abi>) {
            str = std::to_string(_value.x) + '/' +std::to_string(_value.y) + '/' +std::to_string(_value.z);
        }
        else if constexpr (std::is_same_v<T, DeviceNVS>) {
            str = std::to_string(_value.target.raw) + '/' +std::to_string(_value.setup.data) +
                '/' +std::to_string(_value.bin_sp) + '/' +std::to_string(_value.nmea_sp);
        }
    }
    return str;
}

template <typename T>
void SetupNG<T>::setValueFromStr( const char * str ) {
    if( flags._volatile != VOLATILE ){
        if constexpr (std::is_same_v<T, float>) {
            sscanf(str, "%f", &_value);
        }
        else if constexpr (std::is_same_v<T, int>) {
            sscanf(str, "%d", &_value);
        }
        else if constexpr (std::is_same_v<T, Quaternion>) {
            sscanf( str,"%f/%f/%f/%f", &_value._w, &_value._x, &_value._y, &_value._z );
        }
        else if constexpr (std::is_same_v<T, bitfield_compass>) {
            unsigned temp;
            if (sscanf(str ,"%u", &temp) == 1 && temp < 256) {
                _value = bitfield_compass(temp);
            }
        }
        else if constexpr (std::is_same_v<T, axes_i16_abi>) {
            sscanf( str,"%hd/%hd/%hd", &_value.x, &_value.y, &_value.z );
        }
        else if constexpr (std::is_same_v<T, DeviceNVS>) {
            uint32_t t, s;
            int16_t bp, np;
            if (sscanf(str ,"%d/%d/%hd/%hd", (unsigned*)&t, (unsigned*)&s, &bp, &np) == 4) {
                _value = DeviceNVS(t, s, bp, np);
            }
        }
        setDirty();
    }
}

// Specialization for float
template <typename T>
bool SetupNG<T>::inLimits() const {
    return true;
}
template <>
bool SetupNG<float>::inLimits() const {
    if (std::isnan(_value)) {
        return false;
    }
    if (_limt) {
        // check min/max limit
        if (_value < _limt->_min) {
            return false;
        }
        if (_value > _limt->_max) {
            return false;
        }
    }
    return true;
}
template <>
bool SetupNG<int>::inLimits() const {
    if (_limt) {
        // check min/max limit
        if (_value < (int)_limt->_min) {
            return false;
        }
        if (_value > (int)_limt->_max) {
            return false;
        }
    }
    return true;
}

template <typename T>
bool SetupNG<T>::set( T aval, bool dosync, bool doAct ) {
    if constexpr (std::is_same_v<T, float>) {
        flags._valid = !std::isnan(aval);
    } else {
        flags._valid = true;
    }

    if( _value == aval && !dosync && !doAct) {
        // ESP_LOGI(FNAME,"Value already in config: %s(%s)", _key.data(), getValueAsStr().c_str() );
        return( true );
    }

    _value = aval;
    if ( dosync ) {
        // ESP_LOGI( FNAME,"Syncing %s after set", _key.data());
        sync();
    }
    if( doAct ){
        if( _action != 0 ) {
            (*_action)();
        }
    }
    if( flags._volatile == VOLATILE ){
        return true;
    }
    setDirty();
    // ESP_LOGI(FNAME,"set_%s(%s)", _key.data(), getValueAsStr().c_str() );
    return true;
}

template <typename T>
bool SetupNG<T>::setCheckRange( T aval, bool dosync, bool doAct ) {
    if constexpr (std::is_same_v<T, float> || std::is_same_v<T, int>) {
        if( hasLimits() ){
            if( aval < getMin() ) {
                return set( getMin(), dosync, doAct );
            }
            if ( aval > getMax() ){
                return set( getMax(), dosync, doAct );
            }
        }
    }
    return set( aval, dosync, doAct );
}

// All the instantiations we need
template class SetupNG<float>;
template class SetupNG<int>;
template class SetupNG<Quaternion>;
template class SetupNG<t_tenchar_id>;
template class SetupNG<axes_i16_abi>;
template class SetupNG<bitfield_compass>;
template class SetupNG<DeviceNVS>;


////////////////////////////////////////////////////////////////////////////
// static callbacks
////////////////////////////////////////////////////////////////////////////
void change_mc()
{
    Speed2Fly.changeMc();
    ProtocolItf *prtcl = DEVMAN->getProtocol(NAVI_DEV, SEEYOU_P);
    if (prtcl)
    {
        (static_cast<NmeaPrtcl *>(prtcl))->sendSeeYouVal(MC.get(), SeeYouMsg::MC_VAL);
    }
}

static void changeQnh() {
    ProtocolItf *prtcl = DEVMAN->getProtocol(NAVI_DEV, SEEYOU_P);
    if ( prtcl ) {
        (static_cast<NmeaPrtcl*>(prtcl))->sendSeeYouVal(QNH.get(), SeeYouMsg::QNH_VAL);
    }
    Display->setQnhChanged();
}

void change_ballast() {
	Speed2Fly.changeBallast();
}

void change_crew_weight() {
	ProtocolItf *prtcl = DEVMAN->getProtocol(NAVI_DEV, XCVARIO_P);
	if ( prtcl ) {
		(static_cast<NmeaPrtcl*>(prtcl))->sendXCVCrewWeight(crew_weight.get());
	}
	change_ballast();
}

void change_empty_weight(){
	ProtocolItf *prtcl = DEVMAN->getProtocol(NAVI_DEV, XCVARIO_P);
	if ( prtcl ) {
		(static_cast<NmeaPrtcl*>(prtcl))->sendXCVEmptyWeight(empty_weight.get());
	}
	change_ballast();
}

void change_bal_water(){
	ProtocolItf *prtcl = DEVMAN->getProtocol(NAVI_DEV, XCVARIO_P);
	if ( prtcl ) {
		(static_cast<NmeaPrtcl*>(prtcl))->sendXCVWaterWeight(ballast_kg.get());
	}
    else if ( (prtcl = DEVMAN->getProtocol(NAVI_DEV, SEEYOU_P)) ) {
        (static_cast<NmeaPrtcl *>(prtcl))->sendSeeYouVal(ballast_kg.get(), SeeYouMsg::BAL_VAL);
    }
	change_ballast();
}

void polar_update(){
	MyGliderPolarIndex = Polars::findMyGlider(glider_type.get());
	Speed2Fly.setPolar();
}

static void change_polar() {
    Speed2Fly.changePolar();
}

static void change_bugs() {
    change_polar();
    ProtocolItf *prtcl = DEVMAN->getProtocol(NAVI_DEV, SEEYOU_P);
    if ( prtcl ) {
        (static_cast<NmeaPrtcl*>(prtcl))->sendSeeYouVal(bugs.get(), SeeYouMsg::BUGS_VAL);
    }
}

void change_cruise() {
    CRMOD.updateCache();
    Display->setCruiseChanged();
    AUDIO->updateAudioMode();
}

static void calc_altis() {
	altitude.set( Units::calcAltitude(QNH.get(), statp.get()) );
	altitude_isa.set( Units::calcAltitudeISA(statp.get()) );
}
static void calc_speeds() {
    if ( dynp.getValid() ) {
        ias.set(Units::pascal_to_mps(dynp.get())); // already gated in AirspeedSensor
    }
    else {
        ias.setInvalid();
    }
    
    // IAS to TAS conversion
    kelvin_t temp = OAT.get();
    if (!OAT.getValid()) {
        temp = Units::isa_temperature(altitude.get());
        // ESP_LOGW(FNAME,"T invalid, using 15 deg");
    }
    if (statp.getValid() && ias.getValid()) {
        tas.set(Atmosphere::TAS(ias.get(), statp.get(), temp));
        ESP_LOGI(FNAME, "calc_speeds: IAS=%.2f, statp=%.2f, OAT=%.2f -> TAS=%.2f", ias.get(), statp.get(), temp, tas.get());
    }
    else {
        tas.setInvalid();
    }
}

static void feed_te_alt() {
    bmpVario.pushToHistory(te_alt.get(), Clock::getMillis());
}

void change_volume() {
	float vol = audio_volume.get();
	AUDIO->setVolume(vol, false);
	ESP_LOGI(FNAME,"change_volume -> %f", vol );
}

void flap_act() {
    if ( flapbox_enable.get() || Flap::sensAvailable() ) {
        FLAP = Flap::theFlap(); // check on FLAP pointer further on
    }
    else if ( FLAP ) {
        delete FLAP;
        FLAP = nullptr;
    }
}

static void flap_update_act() {
	if( FLAP ) {
		FLAP->reLoadLevels();
	}
}

void send_config( httpd_req *req ){
	SetupCommon::giveConfigChanges( req );
}

int restore_config(int len, char *data){
	return( SetupCommon::restoreConfigChanges( len, data ) );
}

static void propagate_caps()
{
    // when my_caps changes, propagate to peer side
    XCVSyncMsg* proto = SetupCommon::getSyncProto();
    if ( proto ) {
        proto->sendCAPs( my_caps.get() );
    }
}

static void set_imu_leverarm() {
    if (accSensor) {
        accSensor->getMpu().setLeverArm(imu_leverarm.get());
    }
}

void chg_display_orientation(){
	ESP_LOGI(FNAME, "display changed");
	imu_reference.set(imu_reference.getDefault());
}

static void ch_airborne_state() {
    ESP_LOGI(FNAME, "airborne state changed");
    if (airborne.get() && logged_tests.find("FAILED") == std::string::npos) {
        logged_tests.clear();
        logged_tests.shrink_to_fit();
    }
}

//////////////////////////
// configuration variables
SetupNG<mps_t>          MC(  "MacCready", 0.5, true, SYNC_BIDIR, PERSISTENT, change_mc, quantity_t::QUANT_VSPEED, LIMITS(0.0, 9.9, 0.1) );
SetupNG<pascal_t>  		QNH( "QNHpa", 101325., true, SYNC_BIDIR, PERSISTENT, changeQnh, quantity_t::QUANT_QNH, LIMITS(90000., 110000., 6.0) );
SetupNG<float> 			polar_wingload( "POLAR_WINGLOAD", 34.40, true, SYNC_BIDIR, PERSISTENT, change_ballast, quantity_t::QUANT_NONE, LIMITS(10.0, 100.0, 0.1) );
const limits_t polar_speed_limits = {0.0, 450.0, 1};
SetupNG<kmh_t> 			polar_speed1( "POLAR_SPEED1",   80, true, SYNC_BIDIR, PERSISTENT, change_polar, quantity_t::QUANT_NONE, &polar_speed_limits);
const limits_t polar_sink_limits = {-6.0, 0.0, 0.01};
SetupNG<mps_t> 			polar_sink1( "POLAR_SINK1",    -0.66, true, SYNC_BIDIR, PERSISTENT, change_polar, quantity_t::QUANT_NONE, &polar_sink_limits);
SetupNG<kmh_t> 			polar_speed2( "POLAR_SPEED2",   125, true, SYNC_BIDIR, PERSISTENT, change_polar, quantity_t::QUANT_NONE, &polar_speed_limits);
SetupNG<mps_t> 			polar_sink2( "POLAR_SINK2",    -0.97, true, SYNC_BIDIR, PERSISTENT, change_polar, quantity_t::QUANT_NONE, &polar_sink_limits);
SetupNG<kmh_t> 			polar_speed3( "POLAR_SPEED3",   175, true, SYNC_BIDIR, PERSISTENT, change_polar, quantity_t::QUANT_NONE, &polar_speed_limits);
SetupNG<mps_t> 			polar_sink3( "POLAR_SINK3",    -2.24, true, SYNC_BIDIR, PERSISTENT, change_polar, quantity_t::QUANT_NONE, &polar_sink_limits);
SetupNG<kmh_t>			polar_stall_speed( "STALL_SPEED", 0, true, SYNC_BIDIR, PERSISTENT, change_polar, quantity_t::QUANT_HSLEGACY, &polar_speed_limits);
SetupNG<kilogram_t> 	polar_max_ballast( "POLAR_MAX_BAL",  80, true, SYNC_BIDIR, PERSISTENT, change_ballast, quantity_t::QUANT_MASS, LIMITS(0, 500, 1));
SetupNG<float> 			polar_wingarea( "POLAR_WINGAREA", 10.5, true, SYNC_BIDIR, PERSISTENT, change_ballast, quantity_t::QUANT_NONE, LIMITS(0, 50, 0.1));

SetupNG<float>  		speedcal( "SPEEDCAL", 0.0, true, SYNC_BIDIR, PERSISTENT, nullptr, quantity_t::QUANT_NONE, LIMITS(-100, 100, 1));
SetupNG<second_t>  		vario_delay( "VARIO_DELAY", 3.0, true, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, LIMITS(2.0, 10.0, 0.1));
SetupNG<second_t>  		vario_av_delay( "VARIO_AV_DELAY", 20.0, true, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, LIMITS(7.0, 50.0, 1)); // changed to 20 seconds (quasi standard) what equals to a half circle
SetupNG<mps_t>  		scale_range( "VARIO_RANGE", 5.0, true, SYNC_NONE, PERSISTENT, 0, quantity_t::QUANT_VSPEED, LIMITS(3.0, 7.0, 1));
SetupNG<int>			log_scale( "LOG_SCALE", 0 );
SetupNG<float>  		ballast( "BALLAST" , 0.0, true, SYNC_NONE, VOLATILE, 0 );  // ballast increase from reference weight in %
SetupNG<kilogram_t>  	ballast_kg( "BAL_KG" , 0.0, true, SYNC_BIDIR, PERSISTENT, change_bal_water, quantity_t::QUANT_MASS, LIMITS(0.0, 500, 1));
SetupNG<kilogram_t>		empty_weight( "EMPTY_WGT", 361.2, true, SYNC_BIDIR, PERSISTENT, change_empty_weight, quantity_t::QUANT_MASS, LIMITS(0, 1000, 1));
SetupNG<kilogram_t>		crew_weight( "CREW_WGT", 80, true, SYNC_BIDIR, PERSISTENT, change_crew_weight, quantity_t::QUANT_MASS, LIMITS(0, 300, 1));
SetupNG<kilogram_t>		gross_weight( "GROSS_WGT", 350, true, SYNC_NONE, VOLATILE ); // derived from above
SetupNG<float>  		bugs( "BUGS", 0.0, true, SYNC_BIDIR, VOLATILE, change_bugs, quantity_t::QUANT_NONE, LIMITS(0.0, 50, 1));

SetupNG<int>  			cruise_mode( "CRUISE", 0, false, SYNC_BIDIR, VOLATILE, change_cruise ); // use the CruiseMode wrapper to access and modify
SetupNG<kelvin_t>  		OAT( "OAT", NAN, false, SYNC_BIDIR, VOLATILE );   // outside air temperature, sensor on any side
SetupNG<int>  			synoptic_wind( "SYNWND", 0.0, false, SYNC_FROM_MASTER, VOLATILE );
SetupNG<float>  		swind_sideslip_lim( "SWSL", 2.0, false, SYNC_FROM_MASTER, PERSISTENT, nullptr, quantity_t::QUANT_NONE, LIMITS(0, 45.0, 0.1));
SetupNG<int>  			ext_syn_wind( "EXSYNWND", 0.0, false, SYNC_BIDIR, VOLATILE ); // synoptic and
SetupNG<int>  			ext_inst_wind( "EXINSWND", 0.0, false, SYNC_BIDIR, VOLATILE ); // instant external wind
SetupNG<rad_t>  		mag_hdm( "HDM", -1.0, false, SYNC_FROM_MASTER, VOLATILE );
SetupNG<rad_t>  		mag_hdt( "HDT", -1.0, false, SYNC_FROM_MASTER, VOLATILE );
SetupNG<mps_t>  		average_climb( "AVCL", 0.0, false, SYNC_NONE, VOLATILE );
SetupNG<float>  		flap_pos( "FLPS", 0.0, false, SYNC_BIDIR, VOLATILE );
SetupNG<pascal_t>  		statp( "STAT", 0.0, false, SYNC_FROM_MASTER, VOLATILE, calc_altis );
SetupNG<pascal_t>  		dynp( "DYNP", 0.0, false, SYNC_FROM_MASTER, VOLATILE, calc_speeds );
SetupNG<meter_t>  		altitude( "ALTI", 0.0, false, SYNC_NONE, VOLATILE ); // derived from statp
SetupNG<meter_t>  		altitude_isa( "ALT_ISA", 0.0, false, SYNC_NONE, VOLATILE ); // derived from statp
SetupNG<mps_t>  		ias( "IASV", 0.0, false, SYNC_NONE, VOLATILE); // derived from dynp in calc_speeds()
SetupNG<mps_t>  		tas( "TASV", 0.0, false, SYNC_NONE, VOLATILE ); // derived from ias + OAT + altitude in calc_speeds()
SetupNG<mps_t>  		gnd_speed( "GNDV", -1.0, false, SYNC_NONE, VOLATILE );
SetupNG<meter_t>  		te_alt( "TEALT", 0.0, false, SYNC_FROM_MASTER, VOLATILE, feed_te_alt );
SetupNG<mps_t>  		te_vario( "TEVA", 0.0, false, SYNC_NONE, VOLATILE ); // derived from te_alt in VarioFilter
SetupNG<mps_t>  		te_netto( "TENET", 0.0, false, SYNC_NONE, VOLATILE ); // derived from te_alt in VarioFilter
SetupNG<rad_t>  		slip_angle( "SLANGLE", 0.0, false, SYNC_FROM_MASTER, VOLATILE );
SetupNG<float>  		battery_voltage( "BATV", 0.0, false, SYNC_FROM_MASTER, VOLATILE );

SetupNG<int>  			xcv_alive( "AL_XCV", ALIVE_NONE, false, SYNC_NONE, VOLATILE );
SetupNG<int>  			mags_alive( "AL_MAGS", ALIVE_NONE, false, SYNC_NONE, VOLATILE );
SetupNG<int>  			flarm_alive( "AL_FLARM", ALIVE_NONE, false, SYNC_NONE, VOLATILE );
SetupNG<int>  			airborne("AIRBORNE", 0, false, SYNC_FROM_MASTER, VOLATILE, &ch_airborne_state);

SetupNG<mps_t>  		s2f_ideal( "S2F_IDEAL", 27.778, false, SYNC_NONE, VOLATILE);
SetupNG<int>  			s2f_switch_mode( "AUDIO_MODE", AM_MANUALLY, false, SYNC_BIDIR, PERSISTENT );
SetupNG<kmh_t>  		s2f_threshold( "S2F_SPEED", 120.0, true, SYNC_BIDIR, PERSISTENT, nullptr, quantity_t::QUANT_HSLEGACY, LIMITS(20.0, 250.0, 1.0));
SetupNG<float>  		s2f_flap_pos( "S2F_FLAP", 1, true, SYNC_BIDIR, PERSISTENT, nullptr, quantity_t::QUANT_NONE, LIMITS(0, 6, 0.1));
static const limits_t percentage_limits = {0, 100, 1.0};
SetupNG<float>  		s2f_gyro_deg( "S2F_GYRO", 10, true, SYNC_BIDIR, PERSISTENT, nullptr, quantity_t::QUANT_NONE, &percentage_limits);
SetupNG<float>  		s2f_auto_lag( "S2F_HYST", 5, true, SYNC_BIDIR, PERSISTENT, nullptr, quantity_t::QUANT_NONE, LIMITS(2, 20, 1));

						// set audio volume exclusively through the Audio class
SetupNG<float> 			audio_volume("AUD_VOL", 10, true, SYNC_BIDIR, VOLATILE, change_volume, quantity_t::QUANT_NONE, &percentage_limits);
SetupNG<float>  		default_volume( "DEFAULT_VOL", 25.0, true, SYNC_BIDIR, PERSISTENT, nullptr, quantity_t::QUANT_NONE, &percentage_limits);
SetupNG<float>          alarm_volraise( "FLARM_VOL", 40, true, SYNC_BIDIR, PERSISTENT, nullptr, quantity_t::QUANT_NONE, &percentage_limits);
SetupNG<int>  			audio_split_vol( "AUD_SPLIT", 0, true, SYNC_BIDIR );
SetupNG<int>  			audio_range( "AUDIO_RANGE" , AUDIO_RANGE_5_MS, true, SYNC_BIDIR );
SetupNG<float>  		center_freq( "AUDIO_CENTER_F", 500.0, true, SYNC_BIDIR, PERSISTENT, nullptr, quantity_t::QUANT_NONE, LIMITS(200.0, 2000.0, 10.0));
SetupNG<float>  		tone_var( "OCTAVES", 2.0, true, SYNC_BIDIR, PERSISTENT, nullptr, quantity_t::QUANT_NONE, LIMITS(1.2, 4, 0.05));
SetupNG<int>  			dual_tone( "DUAL_TONE", 0, true, SYNC_BIDIR );
SetupNG<int>  			chopping_mode( "CHOPPING_MODE", BOTH_CHOP, true, SYNC_BIDIR );
SetupNG<int>  			audio_harmonics( "AUD_HARMONICS" , AUD_HARM_HIGH, true, SYNC_BIDIR );
SetupNG<float>		    audio_factor( "AUDIO_FACTOR", 1, true, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, LIMITS(0.1, 2, 0.025));
SetupNG<float>  		deadband( "DEADBAND", 0.3, true, SYNC_BIDIR, PERSISTENT, nullptr, quantity_t::QUANT_VSPEED, LIMITS(.0, 5.0, 0.1));
SetupNG<float>  		deadband_neg("DEADBAND_NEG" , -0.3, true, SYNC_BIDIR, PERSISTENT, nullptr, quantity_t::QUANT_VSPEED, LIMITS(-5.0, .0, 0.1));

SetupNG<float>  		wifi_max_power( "WIFI_MP" , 50, true, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, LIMITS(10.0, 100.0, 5.0));
SetupNG<int>  			factory_flag( "FACTORY_RES" , 0, false ); // bit 0: factory reset flag, bit 1: normal/factory menu flag
SetupNG<int>  			alt_select( "ALT_SELECT" , ALT_BARO_SENSOR );
SetupNG<int>  			fl_auto_transition( "FL_AUTO" , 0 );
SetupNG<int>  			alt_display_mode( "ALT_DISP_MODE" , Altimeter::MODE_QNH );
SetupNG<float>  		transition_alt( "TRANS_ALT", 50, true, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, LIMITS(0, 400, 10)); // Transition Altitude
SetupNG<int>  			glider_type( "GLIDER_TYPE_IDX", 1000, true, SYNC_BIDIR, PERSISTENT, polar_update );

SetupNG<float>  		as_offset( "AS_OFFSET" , -1.f ); // enforce an air speed sensor zero calibration at first start and after a factory reset
static const limits_t bat_limits = {0.0, 28.0, 0.1};
SetupNG<float>  		bat_low_volt( "BAT_LOW_VOLT" , 11.5, true, SYNC_BIDIR, PERSISTENT, nullptr, quantity_t::QUANT_NONE, &bat_limits);
SetupNG<float>  		bat_red_volt( "BAT_RED_VOLT", 11.75, true, SYNC_BIDIR, PERSISTENT, nullptr, quantity_t::QUANT_NONE, &bat_limits);
SetupNG<float>  		bat_yellow_volt( "BAT_YELLOW_VOLT" , 12.0, true, SYNC_BIDIR, PERSISTENT, nullptr, quantity_t::QUANT_NONE, &bat_limits);
SetupNG<float>  		bat_full_volt( "BAT_FULL_VOLT", 12.8, true, SYNC_BIDIR, PERSISTENT, nullptr, quantity_t::QUANT_NONE, &bat_limits);
SetupNG<second_t>  		core_climb_period( "CORE_CLIMB_P" , 60, true, SYNC_BIDIR, PERSISTENT, nullptr, quantity_t::QUANT_NONE, LIMITS(60, 300, 1));
SetupNG<mps_t>  		core_climb_min( "CORE_CLIMB_MIN" , 0.5, true, SYNC_BIDIR, PERSISTENT, nullptr, quantity_t::QUANT_NONE, LIMITS(0.0, 2.0, 0.1));
SetupNG<minute_t>  		core_climb_history( "CORE_CLIMB_HIST" , 45, true, SYNC_BIDIR, PERSISTENT, nullptr, quantity_t::QUANT_NONE, LIMITS(1, 300, 1));
SetupNG<mps_t>  		mean_climb_major_change( "MEAN_CLMC", 0.5, true, SYNC_BIDIR, PERSISTENT, nullptr, quantity_t::QUANT_NONE, LIMITS(0.1, 5.0, 0.1));
SetupNG<meter_t>  		airfield_elevation( "ELEVATION", NO_ELEVATION, true, SYNC_BIDIR, PERSISTENT, nullptr, quantity_t::QUANT_ALT, LIMITS(NO_ELEVATION, 3000, 1));
SetupNG<float>  		s2f_deadband( "DEADBAND_S2F", 10.0, true, SYNC_BIDIR, PERSISTENT, nullptr, quantity_t::QUANT_HSLEGACY, LIMITS(.0, 25.0, 1));
SetupNG<float>  		s2f_deadband_neg( "DB_S2F_NEG", -10.0, true, SYNC_BIDIR, PERSISTENT, nullptr, quantity_t::QUANT_HSLEGACY, LIMITS(-25.0, .0, 1));
SetupNG<float>  		s2f_delay( "S2F_DELAY", 5.0, true, SYNC_BIDIR, PERSISTENT, nullptr, quantity_t::QUANT_TIME, LIMITS(2.0, 12.0, 0.5));
SetupNG<float>  		factory_volt_adjust("FACT_VOLT_ADJ", 0.00815, false, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, LIMITS(-25.0, 25.0, 0.01));

SetupNG<int>  			display_type( "DISPLAY_TYPE",  UNIVERSAL );
SetupNG<int>  			display_test( "DISPLAY_TEST", 0, false, SYNC_NONE, VOLATILE ); // dummy variable to trigger display test patterns
SetupNG<int>  			display_orientation("DISPLAY_ORIENT" , DISPLAY_NORMAL, true, SYNC_NONE, PERSISTENT, chg_display_orientation );
SetupNG<int>  			flapbox_enable( "FLAP_ENABLE", 0, true, SYNC_NONE, PERSISTENT, flap_act);
SetupNG<kmh_t>  		wk_speed_0( "WK_SPD_0", 0, false, SYNC_BIDIR, PERSISTENT, flap_update_act, quantity_t::QUANT_HSLEGACY, &polar_speed_limits);
SetupNG<kmh_t>  		wk_speed_1( "WK_SPD_1", 0, false, SYNC_BIDIR, PERSISTENT, flap_update_act, quantity_t::QUANT_HSLEGACY, &polar_speed_limits);
SetupNG<kmh_t>  		wk_speed_2( "WK_SPD_2", 0, false, SYNC_BIDIR, PERSISTENT, flap_update_act, quantity_t::QUANT_HSLEGACY, &polar_speed_limits);
SetupNG<kmh_t>  		wk_speed_3( "WK_SPD_3", 0, false, SYNC_BIDIR, PERSISTENT, flap_update_act, quantity_t::QUANT_HSLEGACY, &polar_speed_limits);
SetupNG<kmh_t>  		wk_speed_4( "WK_SPD_4", 0, false, SYNC_BIDIR, PERSISTENT, flap_update_act, quantity_t::QUANT_HSLEGACY, &polar_speed_limits);
SetupNG<kmh_t>  		wk_speed_5( "WK_SPD_5", 0, false, SYNC_BIDIR, PERSISTENT, flap_update_act, quantity_t::QUANT_HSLEGACY, &polar_speed_limits);
SetupNG<kmh_t>  		wk_speed_6( "WK_SPD_6", 0, false, SYNC_BIDIR, PERSISTENT, flap_update_act, quantity_t::QUANT_HSLEGACY, &polar_speed_limits);
SetupNG<int>  			alt_unit( "ALT_UNIT", (int)alt_unit_t::ALT_UNIT_METER );
SetupNG<int>  			alt_quantization( "ALT_QUANT", Altimeter::ALT_QUANT_10 );
SetupNG<int>  			ias_unit( "IAS_UNIT", SPEED_UNIT_KMH );
SetupNG<int>  			vario_unit( "VARIO_UNIT", VARIO_UNIT_MS );
SetupNG<int>  			temperature_unit( "TEMP_UNIT", T_CELCIUS );
SetupNG<int>  			dst_unit( "DST_UNIT", DST_UNIT_M );
SetupNG<int>  			qnh_unit("QNH_UNIT", QNH_HPA );
SetupNG<int>  			rot_default( "ROTARY_DEFAULT", 0 );
SetupNG<int>  			serial1_speed( "SER1_SPEED", BAUD_19200, true, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, LIMITS(BAUD_2400, BAUD_115200, 1));
SetupNG<int>  			serial1_pin_swap( "SER1_PINS", 0 );
SetupNG<int>  			serial1_ttl_signals( "SER1_TTL", RS232_TTL );
SetupNG<int>  			serial1_tx_enable( "SER1_TX_ENA", 1 );
SetupNG<int>  			serial2_speed( "SER2_SPEED", BAUD_38400, true, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, LIMITS(BAUD_2400, BAUD_115200, 1));
SetupNG<int>  			serial2_pin_swap( "SER2_PINS", 0 );
SetupNG<int>  			serial2_ttl_signals( "SER2_TTL", RS232_NORMAL );
SetupNG<int>  			serial2_tx_enable( "SER2_TX_ENA", 1 );
SetupNG<int>  			software_update( "SOFTWARE_UPDATE", 0 );
SetupNG<int>  			battery_display( "BAT_DISPLAY", Battery::BAT_NONE, true );
SetupNG<int>		    log_level( "LOG_LEVEL", 3 );
SetupNG<float>		    te_comp_adjust ( "TECOMP_ADJ", 0, true, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, LIMITS(-100, 100, 0.1));
SetupNG<int>		    te_comp_enable( "TECOMP_ENA", VarioFilter::TE_TEK_PROBE );
SetupNG<int>		    rotary_dir( "ROTARY_DIR", 0 );
SetupNG<int>		    rotary_inc( "ROTARY_INC", 1 );
SetupNG<int>		    student_mode( "STUD_MOD", 0 );
SetupNG<float>		    password( "PASSWORD", 0, true, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, LIMITS(0, 1000, 1));
SetupNG<int>		    ahrs_rpyl_dataset("RPYL", 0 );
SetupNG<int>		    ahrs_autozero("AHRSAZ", 0 );
SetupNG<float>		    ahrs_gyro_factor("AHRSMGYF", 100, true, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, &percentage_limits);
SetupNG<float>		    ahrs_min_gyro_factor("AHRSLGYF", 20, true, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, &percentage_limits);
SetupNG<float>		    ahrs_dynamic_factor("AHRSGDYN", 5, true, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, LIMITS(0.5, 10, 0.1));
SetupNG<int>		    ahrs_roll_check("AHRSRCHECK", 0 );
SetupNG<float>       	gyro_gating("GYRO_GAT", 1.0, true, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, LIMITS(0, 10, 0.1));
SetupNG<int>		    s2f_switch_type("S2FHWSW", S2F_HW_SWITCH );
SetupNG<int>		    hardwareRevision("HWREV", HW_UNKNOWN, false);
SetupNG<t_tenchar_id>	ahrs_licence("AHRS_LIC", t_tenchar_id(""), false );
// SetupNG<int>		    dummy("DUMMY", 0, false, SYNC_NONE, VOLATILE );
SetupNG<int>		    wk_sens_pos_0("WK_SP_0", 0, false );
SetupNG<int>		    wk_sens_pos_1("WK_SP_1", 0, false );
SetupNG<int>		    wk_sens_pos_2("WK_SP_2", 0, false );
SetupNG<int>		    wk_sens_pos_3("WK_SP_3", 0, false );
SetupNG<int>		    wk_sens_pos_4("WK_SP_4", 0, false );
SetupNG<int>		    wk_sens_pos_5("WK_SP_5", 0, false );
SetupNG<int>		    wk_sens_pos_6("WK_SP_6", 0, false );
SetupNG<int>            stall_warning( "STALL_WARN", 0, true, SYNC_BIDIR, PERSISTENT );
SetupNG<int>            flarm_warning( "FLARM_LEVEL", 1, true, SYNC_BIDIR, PERSISTENT );
SetupNG<second_t>       flarm_alarm_time( "FLARM_ALM", 5, true, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, LIMITS(1, 15, 1));
SetupNG<int>            flap_sensor( "FLAP_SENS", FLAP_SENSOR_DISABLE, false, SYNC_NONE, PERSISTENT, flap_act);
// SetupNG<float>          compass_dev_0( "CP_DEV_0", 0 );
// SetupNG<float>          compass_dev_45( "CP_DEV_45", 0 );
// SetupNG<float>          compass_dev_90( "CP_DEV_90", 0 );
// SetupNG<float>          compass_dev_135( "CP_DEV_135", 0 );
// SetupNG<float>          compass_dev_180( "CP_DEV_180", 0 );
// SetupNG<float>          compass_dev_225( "CP_DEV_225", 0 );
// SetupNG<float>          compass_dev_270( "CP_DEV_279", 0 );
// SetupNG<float>          compass_dev_315( "CP_DEV_315", 0 );
SetupNG<float>          compass_x_bias( "CP_X_BIAS", 0 );
SetupNG<float>          compass_y_bias( "CP_Y_BIAS", 0 );
SetupNG<float>          compass_z_bias( "CP_Z_BIAS", 0 );
SetupNG<float>          compass_x_scale( "CP_X_SCALE", 1.0 );
SetupNG<float>          compass_y_scale( "CP_Y_SCALE", 1.0 );
SetupNG<float>          compass_z_scale( "CP_Z_SCALE", 1.0 );
SetupNG<int>            compass_calibrated( "CP_CALIBRATED", 0 );
SetupNG<float>          compass_declination( "CP_DECL", 0, true, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, LIMITS(-180, 180, 1.0));
// SetupNG<int>            compass_declination_valid( "CP_DECL_VALID", 0 );
SetupNG<int>            compass_nmea_hdm( "CP_NMEA_HDM", 0 );
SetupNG<int>            compass_nmea_hdt( "CP_NMEA_HDT", 0 );
SetupNG<float>          wind_as_filter( "WINDASF", 0.02, true, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, LIMITS(0, 0.05, 0.001));
SetupNG<float>          wind_gps_lowpass( "WINDGPSLP", 1.00, true, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, LIMITS(0.1, 10.0, 0.1));
SetupNG<float>          wind_dev_filter( "WINDDEVF", 0.010, true, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, LIMITS(0, 0.05, 0.001));
SetupNG<int> 			wind_enable( "WIND_ENA", WA_OFF );
SetupNG<float> 			wind_as_calibration("WIND_AS_CAL", 1.0 );
SetupNG<float> 			wind_filter_lowpass("SWINDAVER", 60, true, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, LIMITS(5, 120, 1));
SetupNG<float> 			wind_straight_course_tolerance("WINDSTOL", 7.5, true, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, LIMITS(2.0, 30.0, 0.1));
SetupNG<float> 			wind_straight_speed_tolerance("WINDSSTOL", 15, true, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, LIMITS(1.0, 30.0, 1));
SetupNG<int> 			wind_reference( "WIND_REF", static_cast<int>(WindReference::WR_HEADING) );
SetupNG<float> 			wind_max_deviation("WIND_MDEV", 30.0, true, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, LIMITS(0.0, 180.0, 1.0));
SetupNG<int> 			s2f_blockspeed( "S2G_BLOCKSPEED", 0, false, SYNC_BIDIR );  // considering netto vario and g load for S2F or not
SetupNG<int> 			needle_color("NEEDLE_COLOR", VN_COLOR_ORANGE );
SetupNG<int> 			wk_label_0( "WK_LBL_0", 0, false, SYNC_BIDIR, PERSISTENT, flap_update_act );
SetupNG<int> 			wk_label_1( "WK_LBL_1", 0, false, SYNC_BIDIR, PERSISTENT, flap_update_act );
SetupNG<int> 			wk_label_2( "WK_LBL_2", 0, false, SYNC_BIDIR, PERSISTENT, flap_update_act );
SetupNG<int> 			wk_label_3( "WK_LBL_3", 0, false, SYNC_BIDIR, PERSISTENT, flap_update_act );
SetupNG<int> 			wk_label_4( "WK_LBL_4", 0, false, SYNC_BIDIR, PERSISTENT, flap_update_act );
SetupNG<int> 			wk_label_5( "WK_LBL_5", 0, false, SYNC_BIDIR, PERSISTENT, flap_update_act );
SetupNG<int> 			wk_label_6( "WK_LBL_6", 0, false, SYNC_BIDIR, PERSISTENT, flap_update_act );
SetupNG<float> 			flap_takeoff("FLAPTOp", 0,  false, SYNC_BIDIR, PERSISTENT, nullptr, quantity_t::QUANT_NONE, LIMITS(0, 6, 1));
SetupNG<int> 			audio_mute_sink( "AUDISS", 0 );
SetupNG<int> 			audio_mute_gen( "AUDISG", AUDIO_ON );
SetupNG<int>			vario_mode("VAMOD", CRUISE_ONLY_NETTO, true, SYNC_NONE, PERSISTENT, change_cruise);  // switch to netto mode when cruising
SetupNG<int>			airspeed_sensor("PTYPE", AirspeedSensor::NONE, false);
SetupNG<int>			cruise_audio_mode("CAUDIO", 0 );
SetupNG<int>			netto_mode("NETMOD", NETTO_RELATIVE, true, SYNC_NONE, PERSISTENT, change_cruise);  // regard polar sink
SetupNG<kmh_t>			v_max("VMAX", 270, true, SYNC_BIDIR, PERSISTENT, nullptr, quantity_t::QUANT_HSLEGACY, &polar_speed_limits);
static const limits_t pos_g_limits = {1.0, 8.0, 0.1};
static const limits_t neg_g_limits = {-8.0, -1.0, 0.1};
SetupNG<float>			gload_pos_limit_low("GLOADPLL", 3, true, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, &pos_g_limits);
SetupNG<float>			gload_neg_limit_low("GLOADNLL", -2, true, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, &neg_g_limits);
SetupNG<float>			gload_pos_limit("GLOADPL", 5, true, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, &pos_g_limits);
SetupNG<float>			gload_neg_limit("GLOADNL", -3, true, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, &neg_g_limits);
SetupNG<float>			gload_pos_max("GLOADPM", 1);
SetupNG<float>			gload_neg_max("GLOADNM", 0);
SetupNG<float>			airspeed_max("ASMAX", 0 );
SetupNG<int>        	display_variant("DISPLAY_VARIANT", 0 );
// SetupNG<int>        	compass_dev_auto("COMPASS_DEV", 0 );
SetupNG<degree_t>    	max_circle_wind_diff("CI_WINDDM", 60.0, true, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, LIMITS(0, 90.0, 1.0));
SetupNG<degree_t>    	max_circle_wind_delta_deg("CIMDELD", 20.0, true, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, LIMITS(0.0, 60.0, 0.1));
SetupNG<kmh_t>       	max_circle_wind_delta_speed("CIMDELS", 5.0, true, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, LIMITS(0.0, 20.0, 0.1));
SetupNG<float>       	circle_wind_lowpass("CI_WINDLOW", 5, true, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, LIMITS(1, 10, 1));
SetupNG<int> 			can_speed( "CANSPEED", CAN_SPEED_1MBIT, true, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, LIMITS(CAN_SPEED_250KBIT, CAN_SPEED_1MBIT, 1));
SetupNG<float> 			master_xcvario( "MSXCV", 0, false, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, LIMITS(1000, 9999, 1));
SetupNG<int> 			menu_long_press("MENU_LONG", 0 );
SetupNG<int> 			screen_gmeter("SCR_GMET", SCREEN_OFF, false);
SetupNG<int> 			screen_horizon("SCR_HORIZ", SCREEN_OFF);
SetupNG<int> 			vario_centeraid("SCR_CA", 1, false); // 0: off, 1: top-ref, 2: side-ref
SetupNG<int> 			vario_upper_gauge("SCR_GT", MultiGauge::GAUGE_NONE, false);
SetupNG<int> 			vario_lower_gauge("SCR_GB", MultiGauge::GAUGE_NONE, false);
SetupNG<int> 			vario_mc_gauge("SCR_GMC", 1, false);
// SetupNG<bitfield_compass>  calibration_bits("CALBIT", { 0,0,0,0,0,0 } );
SetupNG<int> 			gear_warning("GEARWA", 0 );
SetupNG<t_tenchar_id>	custom_wireless_id("WLID", t_tenchar_id("") );
SetupNG<int> 			logging("LOGGING", 0 );
SetupNG<float>      	display_clock_adj("DSCLADHJ", 0, true, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, LIMITS(-2, 2, 0.1));
SetupNG<int> 			ahrs_raw_data("AHRSRAW", 0 );

SetupNG<float>				glider_ground_aa("GLD_GND_AA", 12.0, true, SYNC_FROM_MASTER, PERSISTENT, nullptr, quantity_t::QUANT_NONE, LIMITS(-5, 20, 1));
SetupNG<Quaternion>			imu_reference("IMU_REFERENCE", Quaternion(), false);
SetupNG<axes_i16_abi>		gyro_bias("GYRO_BIAS", {0, 0, 0} );
SetupNG<axes_i16_abi>		accl_bias("ACCL_BIAS", {0, 0, 0}, false ); // never reset this factoy calibration
SetupNG<celsius_t> 			mpu_temperature("MPUTEMP", 45.0, true, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, LIMITS(30, 60, 1)); // default for AHRS chip temperature (XCV 2023)
SetupNG<meter_t> 			imu_leverarm("IMU_LEVER", 0.f, true, SYNC_NONE, PERSISTENT, set_imu_leverarm, quantity_t::QUANT_NONE, LIMITS(0, 3, .1));
// Master or Second device role
SetupNG<int> 			xcv_role("XCVROLE", MASTER_ROLE, false, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, LIMITS(MASTER_ROLE, SECOND_ROLE, 1));
// Bitfield to exchange status on connected devices between master and second
SetupNG<int> 			my_caps("MACAPS", 0, false, SYNC_NONE, VOLATILE, propagate_caps );
SetupNG<int>			peer_caps("SECAPS", 0, false, SYNC_NONE, VOLATILE );
// Connected device entries persistance
SetupNG<DeviceNVS>		anemoi_devsetup("ANEMOI", DeviceNVS() );
SetupNG<DeviceNVS>		flarm_devsetup("FLARM", DeviceNVS() );
SetupNG<DeviceNVS>		master_devsetup("MASTER", DeviceNVS() );
SetupNG<DeviceNVS>		second_devsetup("SECOND", DeviceNVS() );
SetupNG<DeviceNVS>		magleg_devsetup("MAGLEG", DeviceNVS() );
SetupNG<DeviceNVS>		navi_devsetup("NAVI", DeviceNVS() );
SetupNG<DeviceNVS>		flarm_host_setup("NAVIFLARM", DeviceNVS() );
SetupNG<DeviceNVS>		flarm_host2_setup("NAVIFLDOWN", DeviceNVS() );
SetupNG<DeviceNVS>		flarm_host3_setup("FLARMDISP", DeviceNVS() );
SetupNG<DeviceNVS>		radio_host_setup("NAVIRADIO", DeviceNVS() );
SetupNG<DeviceNVS>		krt_devsetup("KRTRADIO", DeviceNVS() );
SetupNG<DeviceNVS>		atr_devsetup("ATRIRADIO", DeviceNVS() );
SetupNG<DeviceNVS>		temp_devsetup("TEMPDEV", DeviceNVS() );
