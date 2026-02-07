/*
 * SetupNG.h
 *
 *  Created on: August 23, 2020
 *      Author: iltis
 */

#pragma once

#include "Compass.h"
#include "comm/Configuration.h"
#include "math/Units.h"
#include "setup/SetupCommon.h"
// #include "logdef.h" // do not include this in a header file


#include <esp_err.h>
#include <nvs.h>
#include <nvs_flash.h>

#include <cstring>
#include <string>

// forwards
class Quaternion;

// abi robust int16 vector, to avoid issues with padding and alignment of the original types
struct axes_i16_abi {
    int16_t x, y, z;
    constexpr axes_i16_abi() = default;
    constexpr axes_i16_abi(int16_t a, int16_t b, int16_t c) : x(a), y(b), z(c) {}
    constexpr bool operator==(const axes_i16_abi& right) const { return x == right.x && y == right.y && z == right.z; }
};


/*
 *
 * NEW Simplified and distributed non volatile config data API with template classes:
 * SetupNG<int>  MC( "MacCready", 0.5 );
 *
 *  int as = MC.get();
 *  MC.set( 1.5 );
 *
 */

enum e_display_type { UNIVERSAL, RAYSTAR_RFJ240L_40P, ST7789_2INCH_12P, ILI9341_TFT_18P };
typedef enum chopping_mode { NO_CHOP=0, VARIO_CHOP=1, S2F_CHOP=2, BOTH_CHOP=3 } chopping_mode_t; // bit masked
typedef enum ext_device_protocol  { DEV_DISABLE, DEV_FLARM, DEV_KRT2_RADIO, DEV_BECKER_RADIO, DEV_GNSS_UBX, DEV_ANEMOI } ext_device_proto_t;
typedef enum e_display_style  { DISPLAY_AIRLINER, DISPLAY_RETRO } display_style_t;
typedef enum e_display_variant { DISPLAY_WHITE_ON_BLACK, DISPLAY_BLACK_ON_WHITE } display_variant_t;
typedef enum e_s2f_type  { S2F_HW_SWITCH, S2F_HW_PUSH_BUTTON, S2F_HW_SWITCH_INVERTED, S2F_SWITCH_DISABLE } e_s2f_type;
typedef enum e_wireless_type { WL_DISABLE, WL_BLUETOOTH, WL_WLAN_MASTER, WL_WLAN_CLIENT, WL_WLAN_STANDALONE, WL_BLUETOOTH_LE } e_wireless_t;
enum e_xcvrole { NO_ROLE, MASTER_ROLE, SECOND_ROLE };
enum e_audiomode_type { AM_VARIO, AM_S2F, AM_MANUALLY, AM_AUTOSPEED, AM_FLAP, AM_AHRS };
enum e_audio_tone_mode { ATM_SINGLE_TONE, ATM_DUAL_TONE };
enum e_audio_chopping_style { AUDIO_CHOP_SOFT, AUDIO_CHOP_HARD };
enum e_audio_range { AUDIO_RANGE_5_MS, AUDIO_RANGE_10_MS, AUDIO_RANGE_VARIABLE };
enum e_audio_harmonics { AUD_HARM_OFF=0, AUD_HARM_LOW=7, AUD_HARM_MED=14, AUD_HARM_HIGH=21 };
enum e_audio_mute_gen { AUDIO_ON, AUDIO_ALARMS_ONLY, AUDIO_OFF };
typedef enum e_amp_shutdown { AMP_STAY_ON, AMP_SHUTDOWN, AMP_SHUTDOWN_5S } e_amp_shutdown_t;
enum e_flap_sensor { FLAP_SENSOR_DISABLE, FLAP_SENSOR_ENABLE };
typedef enum e_cruise_audio { AUDIO_S2F, AUDIO_VARIO } e_cruise_audio_2;
typedef enum e_vario_mode { VARIO_BRUTTO, VARIO_NETTO, CRUISE_ONLY_NETTO } e_vario_mode_t;
typedef enum e_netto_mode { NETTO_NORMAL, NETTO_RELATIVE } e_netto_mode_t;
typedef enum e_screen_mode { SCREEN_OFF, SCREEN_DYNAMIC, SCREEN_ON, SCREEN_PRIMARY } e_screen_mode_t;
enum e_windanalyser_mode { WA_OFF=0, WA_STRAIGHT=1, WA_CIRCLING=2, WA_BOTH=3, WA_EXTERNAL=4 }; // do nto change (bit-field)
enum e_logging { LOGG_DISABLE, LOGG_WIND, LOGG_GYRO_MAG, LOGG_BOTH, LOGG_RAW_SENSOR_DATA }; // bit field (!)
enum class quantity_t : uint8_t { QUANT_NONE, QUANT_TEMPERATURE, QUANT_ALT, QUANT_HSPEED, QUANT_HSLEGACY, QUANT_VSPEED, QUANT_QNH, QUANT_MASS, QUANT_TIME };
enum temperature_unit_t { T_CELCIUS, T_FAHRENHEIT, T_KELVIN };
enum class alt_unit_t : uint8_t { ALT_UNIT_METER, ALT_UNIT_FT, ALT_UNIT_FL };
enum dst_unit_t { DST_UNIT_M, DST_UNIT_FT, DST_UNIT_MILES, DST_UNIT_NAUTICAL_MILES };
enum speed_unit_t { SPEED_UNIT_KMH, SPEED_UNIT_MPH, SPEED_UNIT_KNOTS };
enum vario_unit_t { VARIO_UNIT_MS, VARIO_UNIT_FPM, VARIO_UNIT_KNOTS };
enum qnh_unit_t { QNH_HPA, QNH_INHG };
typedef enum e_compasss_sensor_type { CS_DISABLE=0, CS_I2C=1, CS_CAN=3 } e_compasss_sensor_type_t;
typedef enum e_sync { SYNC_NONE, SYNC_FROM_MASTER, SYNC_FROM_CLIENT, SYNC_BIDIR } e_sync_t;       // determines if data is synched from/to client. BIDIR means sync at commit from both sides
typedef enum e_reset { RESET_NO, RESET_YES } e_reset_t;   // determines if data is reset to defaults on factory reset
enum e_volatility { VOLATILE, PERSISTENT };  // stored in RAM only, or additionally in FLASH
typedef enum e_can_mode { CAN_MODE_MASTER, CAN_MODE_CLIENT, CAN_MODE_STANDALONE } e_can_mode_t;
typedef enum e_altimeter_select { AS_TE_SENSOR, AS_BARO_SENSOR, AS_EXTERNAL } e_altimeter_select_t;
typedef enum e_s2f_arrow_color { AC_WHITE_WHITE, AC_BLUE_BLUE, AC_GREEN_RED } e_s2f_arrow_color_t;
typedef enum e_vario_needle_color { VN_COLOR_WHITE, VN_COLOR_ORANGE, VN_COLOR_RED }  e_vario_needle_color_t;
typedef enum e_display_orientation { DISPLAY_NORMAL, DISPLAY_TOPDOWN, DISPLAY_NINETY } e_display_orientation_t;
typedef enum e_gear_warning_io { GW_OFF, GW_FLAP_SENSOR, GW_S2_RS232_RX, GW_FLAP_SENSOR_INV, GW_S2_RS232_RX_INV, GW_EXTERNAL }  e_gear_warning_io_t;
typedef enum e_data_mon_mode { MON_MOD_ASCII, MON_MOD_BINARY } e_data_mon_mode_t;
typedef enum e_hardware_rev {
    HW_UNKNOWN = 0,
    HW_LONG_VARIO = 1,
    XCVARIO_20 = 2,  // 1 x RS232
    XCVARIO_21 = 3,  // 2 x RS232, AHRS MPU6500
    XCVARIO_22 = 4,  // 2 x RS232, AHRS MPU6500, CAN Bus,
    XCVARIO_23 = 5,  // 2 x RS232, AHRS MPU6500, CAN Bus, AHRS temperature control
    XCVARIO_25 = 7   // 2 x RS232, AHRS ICM-20602, CAN Bus, AHRS temperature control
} e_hardware_rev_t;  // XCVario-Num = hardware revision + 18
enum e_battery_type { BATTERY_CANCEL, BATTERY_LEADACID, BATTERY_LIFEPO4 };

constexpr int NO_ELEVATION = -1;

struct t_tenchar_id {
    char id[10];
    t_tenchar_id() {}
    t_tenchar_id(const char* val) { strncpy(id, val, 10); }
    t_tenchar_id(const t_tenchar_id& val) { strncpy(id, val.id, 10); }
    bool operator==(const t_tenchar_id& other) const { return (strcmp(id, other.id) == 0); }
    t_tenchar_id operator=(const t_tenchar_id& other) {
        strncpy(id, other.id, 10);
        return *this;
    }
    t_tenchar_id operator=(const char* other) {
        strncpy(id, other, 10);
        id[9] = '\0';
        return *this;
    }
};

struct limits_t {
    float _min;
    float _max;
    float _step;
    constexpr limits_t(float min, float max, float step) : _min(min), _max(max), _step(step) {}
};
// create static pointers to limits that live only in flash
#define LIMITS(min, max, step)                                   \
    ([]() -> const limits_t* {                                   \
        static constexpr limits_t _lim = {(min), (max), (step)}; \
        return &_lim;                                            \
    }())
extern const limits_t polar_speed_limits;

template <typename T>
class SetupNG : public SetupCommon {
   public:
    SetupNG(const char* akey,
            T adefault,
            bool reset = true,
            e_sync_t sync = SYNC_NONE,
            e_volatility vol = PERSISTENT,
            void (*action)() = nullptr,
            quantity_t quant = quantity_t::QUANT_NONE,
            const limits_t* l = nullptr);
    virtual ~SetupNG() = default;
    char typeName(void) const override;
    std::string getValueAsStr() const override;
    void setValueFromStr(const char* str) override;

    void* getPtr() override { return &_value; }
    int getSize() override { return sizeof(_value); }
    bool isDefault() override { return _default == _value; }
    bool inLimits() const override;  // check on nan for <float>

    // virtual T getGui() const { return get(); } // tb. overloaded for blackboard fixme
    // virtual const char* unit() const { return ""; } // tb. overloaded for blackboard
    T get() const { return _value; }
    bool set(T aval, bool dosync = true, bool doAct = true);
    bool setCheckRange(T aval, bool dosync = true, bool doAct = true);

    quantity_t quantityType() { return (quantity_t)flags._quant; }
    T getDefault() const { return _default; }

    bool hasLimits() const { return _limt != nullptr; }
    float getMin() const { return _limt->_min; }
    float getMax() const { return _limt->_max; }
    float getStep() const { return _limt->_step; }

   protected:
    void setDefault() override {
        // do not do the set procedure here (only for intialization or factory reset)
        if (_value == _default) {
            return;
        }
        _value = _default;
        if (flags._volatile == PERSISTENT) {
            setDirty();
        }
    }

   private:
    T       _value;    // the value
    const T _default;  // value applied with a factory reset
    const limits_t* _limt = nullptr;
};


//////////////////////////
// configuration variables
extern SetupNG<mps_t> 		MC;
extern SetupNG<pascal_t> 	QNH;
extern SetupNG<float> 		polar_wingload;

extern SetupNG<kmh_t> 		polar_speed1;

extern SetupNG<mps_t> 		polar_sink1;
extern SetupNG<kmh_t> 		polar_speed2;
extern SetupNG<mps_t> 		polar_sink2;
extern SetupNG<kmh_t> 		polar_speed3;
extern SetupNG<mps_t> 		polar_sink3;
extern SetupNG<kmh_t>		polar_stall_speed;
extern SetupNG<kilogram_t> 	polar_max_ballast;
extern SetupNG<float> 		polar_wingarea;

extern SetupNG<float>  		speedcal;
extern SetupNG<second_t>  	vario_delay;
extern SetupNG<second_t>  	vario_av_delay;
extern SetupNG<mps_t>  		scale_range;
extern SetupNG<int>			log_scale;
extern SetupNG<float>  		ballast;
extern SetupNG<kilogram_t>  ballast_kg;
extern SetupNG<kilogram_t>	empty_weight;
extern SetupNG<kilogram_t>	crew_weight;
extern SetupNG<kilogram_t>	gross_weight;
extern SetupNG<float>  		bugs;

extern SetupNG<int>  		cruise_mode;
extern SetupNG<kelvin_t>    OAT;   // outside temperature
extern SetupNG<float>  		swind_dir;   // straight wind direction
extern SetupNG<mps_t>  		swind_speed;
extern SetupNG<float>  		swind_sideslip_lim;
extern SetupNG<float>  		cwind_dir;   // cirling wind direction
extern SetupNG<mps_t>  		cwind_speed;
extern SetupNG<int>  		extwind_sptc_dir; // synoptic and
extern SetupNG<mps_t>  		extwind_sptc_speed;
extern SetupNG<int>  		extwind_inst_dir; // instant external wind
extern SetupNG<mps_t>  		extwind_inst_speed;
extern SetupNG<int>  		extwind_status;
extern SetupNG<float>  		mag_hdm;
extern SetupNG<float>  		mag_hdt;
extern SetupNG<float>  		average_climb;
extern SetupNG<float>  		flap_pos;
extern SetupNG<pascal_t>  	statp;
extern SetupNG<pascal_t>  	dynp;
extern SetupNG<meter_t>  	altitude;
extern SetupNG<meter_t>  	altitude_isa;
extern SetupNG<mps_t>       ias;
extern SetupNG<mps_t>  		tas;
extern SetupNG<mps_t>  		gnd_speed;
extern SetupNG<meter_t>  		te_alt;
extern SetupNG<mps_t>  		te_vario;
extern SetupNG<mps_t>  		te_netto;
extern SetupNG<float>  		slip_angle;
extern SetupNG<float>  		battery_voltage;

extern SetupNG<int>  		xcv_alive;
extern SetupNG<int>  		mags_alive;
extern SetupNG<int>  		flarm_alive;
extern SetupNG<int>  		airborne;

extern SetupNG<mps_t>  		s2f_ideal;
extern SetupNG<int>  		s2f_switch_mode;
extern SetupNG<kmh_t>  		s2f_threshold;
extern SetupNG<float>  		s2f_flap_pos;

extern SetupNG<float>  		s2f_gyro_deg;
extern SetupNG<float>  		s2f_auto_lag;


extern SetupNG<float>  		audio_volume;
extern SetupNG<float>  		default_volume;
extern SetupNG<float>     	alarm_volraise;
extern SetupNG<int>  		audio_split_vol;
extern SetupNG<int>  		audio_range;
extern SetupNG<float>  		center_freq;
extern SetupNG<float>  		tone_var;
extern SetupNG<int>  		dual_tone;
extern SetupNG<int>  		chopping_mode;
extern SetupNG<int>  		audio_harmonics;
extern SetupNG<float>		audio_factor;
extern SetupNG<float>  		deadband;
extern SetupNG<float>  		deadband_neg;

extern SetupNG<float>  		wifi_max_power;
extern SetupNG<int>  		factory_reset;
extern SetupNG<int>  		alt_select;
extern SetupNG<int>  		fl_auto_transition;
extern SetupNG<int>  		alt_display_mode;
extern SetupNG<float>  		transition_alt;
extern SetupNG<int>  		glider_type;

extern SetupNG<float>  		as_offset;

extern SetupNG<float>  		bat_low_volt;
extern SetupNG<float>  		bat_red_volt;
extern SetupNG<float>  		bat_yellow_volt;
extern SetupNG<float>  		bat_full_volt;
extern SetupNG<float>  		core_climb_period;
extern SetupNG<float>  		core_climb_min;
extern SetupNG<float>  		core_climb_history;
extern SetupNG<float>  		mean_climb_major_change;
extern SetupNG<meter_t>  	airfield_elevation;
extern SetupNG<float>  		s2f_deadband;
extern SetupNG<float>  		s2f_deadband_neg;
extern SetupNG<float>  		s2f_delay;
extern SetupNG<float>  		factory_volt_adjust;

extern SetupNG<int>  		display_type;
extern SetupNG<int>  		display_test;
extern SetupNG<int>  		display_orientation;
extern SetupNG<int>  		flapbox_enable;
extern SetupNG<kmh_t>  		wk_speed_0;
extern SetupNG<kmh_t>  		wk_speed_1;
extern SetupNG<kmh_t>  		wk_speed_2;
extern SetupNG<kmh_t>  		wk_speed_3;
extern SetupNG<kmh_t>  		wk_speed_4;
extern SetupNG<kmh_t>  		wk_speed_5;
extern SetupNG<kmh_t>  		wk_speed_6;
extern SetupNG<int>  		alt_unit;
extern SetupNG<int>  		alt_quantization;
extern SetupNG<int>  		ias_unit;
extern SetupNG<int>  		vario_unit;
extern SetupNG<int>  		temperature_unit;
extern SetupNG<int>  		dst_unit;
extern SetupNG<int>  		qnh_unit;
extern SetupNG<int>  		rot_default;
extern SetupNG<int>  		serial1_speed;
extern SetupNG<int>  		serial1_pin_swap;
extern SetupNG<int>  		serial1_ttl_signals;
extern SetupNG<int>  		serial1_tx_enable;
extern SetupNG<int>  		serial2_speed;
extern SetupNG<int>  		serial2_pin_swap;
extern SetupNG<int>  		serial2_ttl_signals;
extern SetupNG<int>  		serial2_tx_enable;
extern SetupNG<int>  		software_update;
extern SetupNG<int>  		battery_display;
extern SetupNG<int>		    log_level;
extern SetupNG<float>		te_comp_adjust;
extern SetupNG<int>		    te_comp_enable;
extern SetupNG<int>		    rotary_dir;
extern SetupNG<int>		    rotary_inc;
extern SetupNG<int>		    student_mode;
extern SetupNG<float>		password;
extern SetupNG<int>		    ahrs_rpyl_dataset;
extern SetupNG<int>		    ahrs_autozero;
extern SetupNG<float>		ahrs_gyro_factor;
extern SetupNG<float>		ahrs_min_gyro_factor;
extern SetupNG<float>		ahrs_dynamic_factor;
extern SetupNG<int>		    ahrs_roll_check;
extern SetupNG<float>       gyro_gating;
extern SetupNG<int>		    s2f_switch_type;
extern SetupNG<int>		    hardwareRevision;
extern SetupNG<t_tenchar_id> ahrs_licence;
// extern SetupNG<int>		    dummy;
extern SetupNG<int>		    wk_sens_pos_0;
extern SetupNG<int>		    wk_sens_pos_1;
extern SetupNG<int>		    wk_sens_pos_2;
extern SetupNG<int>		    wk_sens_pos_3;
extern SetupNG<int>		    wk_sens_pos_4;
extern SetupNG<int>		    wk_sens_pos_5;
extern SetupNG<int>		    wk_sens_pos_6;
extern SetupNG<int>       	stall_warning;
extern SetupNG<int>       	flarm_warning;
extern SetupNG<float>       flarm_alarm_time;
extern SetupNG<int>       	flap_sensor;
extern SetupNG<float>       compass_dev_0;
extern SetupNG<float>       compass_dev_45;
extern SetupNG<float>       compass_dev_90;
extern SetupNG<float>       compass_dev_135;
extern SetupNG<float>       compass_dev_180;
extern SetupNG<float>       compass_dev_225;
extern SetupNG<float>       compass_dev_270;
extern SetupNG<float>       compass_dev_315;
extern SetupNG<float>       compass_x_bias;
extern SetupNG<float>       compass_y_bias;
extern SetupNG<float>       compass_z_bias;
extern SetupNG<float>       compass_x_scale;
extern SetupNG<float>       compass_y_scale;
extern SetupNG<float>       compass_z_scale;
extern SetupNG<int>         compass_calibrated;
extern SetupNG<float>       compass_declination;
extern SetupNG<int>         compass_declination_valid;
extern SetupNG<float>		compass_damping;
extern SetupNG<int>         compass_nmea_hdm;
extern SetupNG<int>         compass_nmea_hdt;
extern SetupNG<float>       wind_as_filter;
extern SetupNG<float>       wind_gps_lowpass;
extern SetupNG<float>       wind_dev_filter;
extern SetupNG<int> 		wind_enable;
extern SetupNG<float> 		wind_as_calibration;
extern SetupNG<float> 		wind_filter_lowpass;
extern SetupNG<float>		wind_straight_course_tolerance;
extern SetupNG<float> 		wind_straight_speed_tolerance;
extern SetupNG<int> 		wind_reference;
extern SetupNG<float> 		wind_max_deviation;
extern SetupNG<int> 		s2f_blockspeed;
extern SetupNG<int>			needle_color;
extern SetupNG<int> 		wk_label_0;
extern SetupNG<int> 		wk_label_1;
extern SetupNG<int> 		wk_label_2;
extern SetupNG<int> 		wk_label_3;
extern SetupNG<int> 		wk_label_4;
extern SetupNG<int> 		wk_label_5;
extern SetupNG<int> 		wk_label_6;
extern SetupNG<float> 		flap_takeoff;
extern SetupNG<int> 		audio_mute_sink;
extern SetupNG<int> 		audio_mute_gen;
extern SetupNG<int>			vario_mode;
extern SetupNG<int>			airspeed_sensor;
extern SetupNG<int>			cruise_audio_mode;
extern SetupNG<int>			netto_mode;
extern SetupNG<kmh_t>		v_max;

extern SetupNG<float>		gload_pos_limit_low;
extern SetupNG<float>		gload_neg_limit_low;
extern SetupNG<float>		gload_pos_limit;
extern SetupNG<float>		gload_neg_limit;
extern SetupNG<float>		gload_pos_max;
extern SetupNG<float>		gload_neg_max;
extern SetupNG<float>		airspeed_max;
// extern SetupNG<float>		gload_alarm_volume; fixme no use
extern SetupNG<int>		    display_variant;
extern SetupNG<int>       	compass_dev_auto;
extern SetupNG<float>       max_circle_wind_diff;
extern SetupNG<float>      	max_circle_wind_delta_deg;
extern SetupNG<float>      	max_circle_wind_delta_speed;
extern SetupNG<float>       circle_wind_lowpass;
extern SetupNG<int> 		can_speed;
extern SetupNG<float> 		master_xcvario;
extern SetupNG<int> 		menu_long_press;
extern SetupNG<int> 		screen_gmeter;
extern SetupNG<int> 		screen_horizon;
extern SetupNG<int> 		vario_centeraid;
extern SetupNG<int> 		vario_upper_gauge;
extern SetupNG<int> 		vario_lower_gauge;
extern SetupNG<int> 		vario_mc_gauge;
extern SetupNG<bitfield_compass> 	calibration_bits;
extern SetupNG<int> 		gear_warning;
extern SetupNG<t_tenchar_id>  custom_wireless_id;
extern SetupNG<int> 		logging;
extern SetupNG<float>      	display_clock_adj;

extern SetupNG<float> 		glider_ground_aa;
extern SetupNG<Quaternion> 	imu_reference;
extern SetupNG<axes_i16_abi> gyro_bias;
extern SetupNG<axes_i16_abi> accl_bias;
extern SetupNG<float> 		mpu_temperature;
extern SetupNG<meter_t> 	imu_leverarm;
extern SetupNG<int> 		xcv_role;
extern SetupNG<int> 		my_caps;
extern SetupNG<int> 		peer_caps;

extern SetupNG<DeviceNVS>	anemoi_devsetup;
extern SetupNG<DeviceNVS>	flarm_devsetup;
extern SetupNG<DeviceNVS>	master_devsetup;
extern SetupNG<DeviceNVS>	second_devsetup;
extern SetupNG<DeviceNVS>	magleg_devsetup;
extern SetupNG<DeviceNVS>	magsens_devsetup;
extern SetupNG<DeviceNVS>	navi_devsetup;
extern SetupNG<DeviceNVS>	flarm_host_setup;
extern SetupNG<DeviceNVS>	flarm_host2_setup;
extern SetupNG<DeviceNVS>	radio_host_setup;
extern SetupNG<DeviceNVS>	krt_devsetup;
extern SetupNG<DeviceNVS>	atr_devsetup;

