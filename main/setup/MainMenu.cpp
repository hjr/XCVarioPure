/*
 * SetupMenu.cpp
 *
 *  Created on: Feb 4, 2018
 *      Author: iltis
 */

#include "SetupMenu.h"

#include "CenterAid.h"
#include "SetupCommon.h"
#include "setup/SubMenuAudio.h"
#include "setup/SubMenuDevices.h"
#include "setup/SubMenuCompassWind.h"
#include "setup/SubMenuGlider.h"
#include "setup/SubMenuFlap.h"
#include "setup/SubMenuImu.h"
#include "setup/ShowBootMsg.h"
#include "setup/ShowFlightInfo.h"
#include "screen/element/MultiGauge.h"
#include "sensor/VarioFilter.h"
#include "S2F.h"
#include "Flarm.h"
#include "version.h"
#include "glider/Polars.h"
#include "math/Floats.h"
#include "math/Units.h"
#include "Atmosphere.h"
#include "driver/gpio/S2fSwitch.h"
#include "Flap.h"
#include "setup/SetupMenuSelect.h"
#include "setup/SetupMenuValFloat.h"
#include "setup/SetupMenuChar.h"
#include "setup/SetupMenuDisplay.h"
#include "setup/SetupAction.h"
#include "setup/SetupNG.h"
#include "sensor/imu/AccMPU6050.h"
#include "sensor/pressure/PressureSensor.h"
#include "sensor/press_diff/AirspeedSensor.h"
// #include "driver/audio/ESPAudio.h"
#include "driver/gpio/AnalogInput.h"
#include "protocol/CANPeerCaps.h"
#include "Colors.h"
#include "AdaptUGC.h"
#include "sensor.h"
#include "test/LeakTest.h"
#include "logdef.h"

#include "comm/DeviceMgr.h"
#include "protocol/NMEA.h"
#include "protocol/nmea/JumboCmdMsg.h"
#include "protocol/FlarmSim.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <string_view>

extern AdaptUGC *MYUCG;

static void wiper_menu_create(SetupMenu *top);
static void bugs_item_create(SetupMenu *top);
static void vario_menu_create(SetupMenu *top);
static void vario_menu_create_damping(SetupMenu *top);
static void vario_menu_create_meanclimb(SetupMenu *top);
static void vario_menu_create_s2f(SetupMenu *top);
static void vario_menu_create_ec(SetupMenu *top);

static void options_menu_create(SetupMenu *top);
static void options_menu_create_units(SetupMenu *top);
static void options_menu_create_flarm(SetupMenu *top);

static void system_menu_create(SetupMenu *top);
static void system_menu_create_software(SetupMenu *top);
static void system_menu_create_hardware_type(SetupMenu *top);
static void system_menu_create_hardware_rotary(SetupMenu *top);


static int caid_reference(SetupMenuSelect* p) {
    if ( theCenteraid ) {
        theCenteraid->setGliderOnTop(p->getSelect() != 2);
    }
    return 0;
}

static int factory_nvs_action(SetupMenuSelect* p) {
    if (p->getValue() == 1) {
        factory_reset.set( 1);
    }
    else if (p->getValue() == 2) {
        ESP_LOGI(FNAME, "Clearing NVS...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_flash_init();
        p->reBoot();
    }
    else if (p->getValue() == 3) {
        factory_menu.set(0);
    }
    return 0;
}

static int expert_menu_action(SetupMenuChar *p) {
    ESP_LOGI(FNAME,"Code %s", p->value());
    const char *code = p->value();
    if ( strcmp(code, "0007") == 0 ) {
        gflags.expert = 1;
        p->reset(); // hide the once entered code
        p->setHelp("Expert menus are unlocked now");
    }
    p->showhelp();
    vTaskDelay(pdMS_TO_TICKS(1000));
    p->setTerminateSetup();
    return 0;
}

int compass_ena(SetupMenuSelect *p) {
	return 0;
}

static int vario_setup(SetupMenuValFloat* p) {
    bmpVario.configChange();
    return 0;
}

static int s2f_change_action(SetupMenuValFloat *p) {
    Speed2Fly.changeDamping();
    return 0;
}

static int set_rotary_increment(SetupMenuSelect *p) {
	Rotary->updateIncrement(p->getSelect());
	return 0;
}

static int speedcal_change(SetupMenuValFloat* p) {
    if (asSensor) {
        asSensor->changeConfig();
    }
    return 0;
}

static int airspeed_zero(SetupMenuSelect *p) {
    if (asSensor) {
        as_offset.set(-1); // invalidate offset to force recalibration with new speedcal
        asSensor->setup();
    }
    return 0;
}

static int tube_swap(SetupMenuSelect* p) {
    speedcal_change(nullptr);
    airspeed_zero(nullptr);
    return 0;
}

int config_gear_warning(SetupMenuSelect *p) {
	SetupMenu::initGearWarning();
	return 0;
}

int do_display_test(SetupAction* p) {
    MYUCG->setColor(0, 0, 0);
    MYUCG->drawBox(0, 0, DISPLAY_W, DISPLAY_H);
    while (!Rotary->readSwitch(300)) {
        ESP_LOGI(FNAME, "Wait for key press");
    }
    MYUCG->setColor(255, 255, 255);
    MYUCG->drawBox(0, 0, DISPLAY_W, DISPLAY_H);
    while (!Rotary->readSwitch(300)) {
        ESP_LOGI(FNAME, "Wait for key press");
    }
    return 0;
}

int select_battery_type(SetupMenuSelect *p) {
	ESP_LOGI(FNAME, "select_battery_type %d", p->getSelect());
	switch ( p->getSelect() ){
		case BATTERY_CANCEL:
			break;
		case BATTERY_LEADACID:
			bat_low_volt.set(11.5);  // its about 20% but LeadAcid already deep discharged -> 0%
			bat_red_volt.set(11.75); // 30% of charge  -> 20%
			bat_yellow_volt.set(12.0); // 50% of charge -> 40%
			bat_full_volt.set(12.8);   // 100% 
			break;
		case BATTERY_LIFEPO4:
			bat_low_volt.set(12.8);  // is about 20% of charge, recommended minimum for LiFePo4  -> display 0%
			bat_red_volt.set(12.9);    // 20% charge 
			bat_yellow_volt.set(13.0); // 25% charge
			bat_full_volt.set(13.6); // 100% charge
		break;
	}
	set_parent_dirty(p);
	return 0;
}

const std::array<std::string_view, 2> alti_mode = {"QNH", "QFE"};
static int update_alti_buzz(SetupMenuSelect *p) {
	SetupMenu *alti = static_cast<SetupMenu*>(p->getParent()->getParent()->getEntry(4));
	alti->setBuzzword(alti_mode[alt_display_mode.get()].data());
	return 0;
}

int qnh_adj(SetupMenuValFloat* p) {
    ESP_LOGI(FNAME, "qnh_adj %f", p->get());
    float alt = 0;
    // catch the 10 samples from the pressure sensor self test
    alt = (baroSensor) ? Units::calcAltitude(p->get(), baroSensor->getAVG(5000)) : altitude.get();
    ESP_LOGI(FNAME, "Setup BA alt=%f QNH=%f Pa", alt, p->get());
    MYUCG->setFont(ucg_font_fub25_hf, true);
    alt = AltUnit->apply(alt);

    MYUCG->setPrintPos(1, 120);
    MYUCG->setColor(COLOR_WHITE);
    MYUCG->printf("% 5d   %s ", fast_iroundf(alt), AltUnit->getName());

    MYUCG->setFont(ucg_font_ncenR14_hr);
    return 0;
}

// Battery Voltage Meter Calibration
int factv_adj(SetupMenuValFloat *p) {
	ESP_LOGI(FNAME,"factv_adj");
	BatVoltage->setAdjust(factory_volt_adjust.get());
	float bat = BatVoltage->get();
	MYUCG->setFont(ucg_font_ncenR14_hr, true);
	MYUCG->setPrintPos(1, 100);
	MYUCG->setColor( COLOR_WHITE );
	MYUCG->printf("%0.2f Volt", bat);
	return 0;
}

static int water_adj(SetupMenuValFloat* p) {
    p->set(std::clamp(p->get(), 0.0f, polar_max_ballast.get()));
    start_weight_adj(p);
    return 0;
}

static int wiper_button(SetupAction *p) {
	NmeaPrtcl *jumbo = static_cast<NmeaPrtcl*>(DEVMAN->getProtocol(DeviceId::JUMBO_DEV, ProtocolType::JUMBOCMD_P));

	ESP_LOGI(FNAME, "wipe %d", p->getCode());
	jumbo->sendJPShortPress(p->getCode());
	return 0;
}

static int startFlarmSimulation(SetupMenuSelect *p) {
	if ( p->getSelect() > 0 ) {
		FlarmSim::StartSim(p->getSelect()-1);
	}
	return 0;
}

static int student_mode_action(SetupMenuSelect* p) {
    if (p->getSelect() == 1) {
        p->setTerminateSetup();
    }
    return 0;
}


int deviceDumpAction(SetupAction *p)
{
	// dump devices
	DEVMAN->dumpMap();
	return 0;
}

static int s2fModeChange(SetupMenuSelect* p) {
    if (S2FSWITCH) {
        S2FSWITCH->updateSwitchSetup();
    }
    return 0;
}
static int s2fModeChangeF(SetupMenuValFloat* p) {
    return s2fModeChange(nullptr);
}

int unitChangeS(SetupMenuSelect* p) {
    Units::setAll();
    return 0;
}

static int exitFactoryMenu(SetupMenuSelect* p){
    if (p->getSelect() == 1) {
#ifndef DEBUG_AND_TEST
        // check if factory tasks are done
        axes_i16_abi accbias = accl_bias.get();
        if ( factory_volt_adjust.get() == 0.f ) {
            // not done, show warning
            p->menuPrintLn("Bat. volt. adjust not done.", 9, 5);
            p->setSelect(0);
        }
        else if ( accSensor && (accbias == axes_i16_abi()) ) {
            // not done, show warning
            p->menuPrintLn("Accel. bias not done.", 9, 5);
            p->setSelect(0);
        }
        else if (leak_test_loss.get() > LeakTest::LEAK_TEST_MAX_LOSS
                || leak_test_loss.get() == 0.f) {
            // not done, show warning
            p->menuPrintLn("Leak test not passed.", 9, 5);
            p->setSelect(0);
        }
        p->menuPrintLn("Pls. confirm", 10, 5);
        if ( p->getSelect() == 0 ) {
            while (!Rotary->readSwitch(100)) ;
        }
        else
#endif
        {
            factory_menu.set(1); // dont show it any more
            p->setTerminateMenu();
        }
    }
    return 0;
}

void vario_menu_create_damping(SetupMenu *top) {
	SetupMenuValFloat *vda = new SetupMenuValFloat("Damping", "sec", vario_setup, &vario_delay, RST_NONE, false);
	vda->setHelp("Response time, time constant of Vario low pass filter");
	vda->setPrecision(1);
	top->addEntry(vda);

	SetupMenuValFloat *vdav = new SetupMenuValFloat("Averager", "sec", vario_setup, &vario_av_delay, RST_NONE, false);
	vdav->setHelp("Response time, time constant of digital Average Vario Display");
	top->addEntry(vdav);
}

void vario_menu_create_meanclimb(SetupMenu *top) {
	SetupMenuValFloat *vccm = new SetupMenuValFloat("Minimum climb", "", nullptr, &core_climb_min, RST_NONE, false);
	vccm->setHelp("Minimum climb rate that counts for arithmetic mean climb value");
	top->addEntry(vccm);

	SetupMenuValFloat *vcch = new SetupMenuValFloat("Duration", "min", nullptr, &core_climb_history, RST_NONE, false);
	vcch->setHelp(
			"Duration in minutes over which mean climb rate is computed, default is last 3 thermals or 45 min");
	top->addEntry(vcch);

	SetupMenuValFloat *vcp = new SetupMenuValFloat("Cycle", "sec", nullptr, &core_climb_period, RST_NONE, false);
	vcp->setHelp(
			"Cycle: number of seconds when mean climb value is recalculated, default is every 60 seconds");
	top->addEntry(vcp);

	SetupMenuValFloat *vcmc = new SetupMenuValFloat("Major Change", "m/s", nullptr, &mean_climb_major_change, RST_NONE, false);
	vcmc->setHelp(
			"Change in mean climb during last cycle (minute), that results in a major change indication (with arrow symbol)");
	top->addEntry(vcmc);
}

void vario_menu_create_s2f(SetupMenu *top) {
	SetupMenuValFloat *vds2 = new SetupMenuValFloat("Damping", "sec", s2f_change_action, &s2f_delay, RST_NONE, false);
	vds2->setHelp("Time constant of S2F low pass filter");
	top->addEntry(vds2);

	SetupMenuSelect *blck = new SetupMenuSelect("Blockspeed", RST_NONE, nullptr, &s2f_blockspeed);
	blck->setHelp(
			"With Blockspeed enabled, vertical movement of airmass or G-load is not considered for speed to fly calculation");
	blck->mkEnable();
	top->addEntry(blck);

	SetupMenuSelect *s2fmod = new SetupMenuSelect("S2F Mode", RST_NONE, s2fModeChange, &s2f_switch_mode);
	s2fmod->setHelp("Select data source for switching between S2F and Vario modes");
	s2fmod->addEntry("Manual", AM_MANUALLY);
	s2fmod->addEntry("AutoSpeed", AM_AUTOSPEED);
	if ( FLAP ) {
		s2fmod->addEntry("AutoFlap", AM_FLAP); // not dynamic, exit setup to change
	}
	s2fmod->addEntry("AutoTurn", AM_AHRS);
	s2fmod->addEntry("Vario fix", AM_VARIO);
	s2fmod->addEntry("Cruise fix", AM_S2F);
	top->addEntry(s2fmod);

	SetupMenuSelect *s2fsw = new SetupMenuSelect("S2F Switch", RST_NONE, s2fModeChange, &s2f_switch_type);
	top->addEntry(s2fsw);
	s2fsw->setHelp("Select S2F switch type: normal switch, push button (toggling S2F mode on each press), or disabled");
	s2fsw->addEntry("Disable", S2F_SWITCH_DISABLE );
	s2fsw->addEntry("Switch", S2F_HW_SWITCH);
	s2fsw->addEntry("Switch Invert", S2F_HW_SWITCH_INVERTED);
	s2fsw->addEntry("Push Button", S2F_HW_PUSH_BUTTON);

	SetupMenuValFloat *autospeed = new SetupMenuValFloat("AutoSpeed Thresh.", "", nullptr, &s2f_threshold, RST_NONE, false);
	top->addEntry(autospeed);
	autospeed->setHelp("Transition speed for the AutoSpeed S2F switch");

	if ( FLAP ) {
		SetupMenuValFloat *s2f_flap = new SetupMenuValFloat("AutoFlap Position", "flp", nullptr, &s2f_flap_pos, RST_NONE, false);
		top->addEntry(s2f_flap);
		s2f_flap->setHelp("Precise flap position for the AutoFlap S2F switch");
	}

	SetupMenuValFloat *s2f_gyro = new SetupMenuValFloat("AutoTurn Rate", "°/s", nullptr, &s2f_gyro_deg, RST_NONE, false);
	top->addEntry(s2f_gyro);
	s2f_gyro->setHelp("Turnrate for the AutoTurnrate switch");

	SetupMenuValFloat *s2flag = new SetupMenuValFloat("Switch Lag", "sec", s2fModeChangeF, &s2f_auto_lag, RST_NONE, false);
	s2flag->setHelp("Lag to delay the auto switch event (2-20sec)");
	top->addEntry(s2flag);
}

void vario_menu_create_ec(SetupMenu *top) {
	SetupMenuSelect *enac = new SetupMenuSelect("eCompensation", RST_NONE, nullptr, &te_comp_enable);
	enac->setHelp(
			"Enable/Disable electronic TE compensation option; Enable only when TE port is connected to ST (static) pressure");
	enac->addEntry("TEK Probe");
	enac->addEntry("EPOT");
	enac->addEntry("PRESSURE");
	top->addEntry(enac);

	SetupMenuValFloat *elca = new SetupMenuValFloat("Adjustment", "%", nullptr, &te_comp_adjust, RST_NONE, false);
	elca->setHelp(
			"Adjustment option for electronic TE compensation in %. This affects the energy altitude calculated from airspeed");
	top->addEntry(elca);
}

void wiper_menu_create(SetupMenu *top) {
	SetupAction *wiperL = new SetupAction("Wipe left         ", wiper_button, 1);
	JumboCmdMsg::LeftAction = wiperL;
	SetupAction *wiperR = new SetupAction("Wipe       right", wiper_button, 0);
	JumboCmdMsg::RightAction = wiperR;
	top->addEntry(wiperL);
	top->addEntry(wiperR);

	bugs_item_create(top);
}

void bugs_item_create(SetupMenu *top) {
	SetupMenuValFloat *bgs = new SetupMenuValFloat("Bugs", "%", nullptr, &bugs, RST_NONE, false);
	bgs->setHelp("Percent degradation of gliding performance due to bugs contamination");
	bgs->setTerminateMenu();
	top->addEntry(bgs);
}

void vario_menu_create(SetupMenu *vae) {
	ESP_LOGI(FNAME,"SetupMenu::vario_menu_create( %p )", vae );

	SetupMenuValFloat *vga = new SetupMenuValFloat("Range", "", audio_setup_f, &scale_range, RST_NONE, false);
	vga->setHelp("Upper and lower value for Vario scale range");
	vga->setPrecision(0);
	vae->addEntry(vga);

	SetupMenuSelect *vlogscale = new SetupMenuSelect("Log. Scale", RST_NONE, nullptr, &log_scale);
	vlogscale->mkEnable();
	vae->addEntry(vlogscale);

	SetupMenuSelect *vamod = new SetupMenuSelect("Mode", RST_NONE, nullptr, &vario_mode);
	vamod->setHelp(
			"Controls if vario considers polar sink (=Netto), or not (=Brutto), or if Netto vario applies only in Cruise Mode");
	vamod->addEntry("Brutto");
	vamod->addEntry("Netto");
	vamod->addEntry("Cruise-Netto");
	vae->addEntry(vamod);

	SetupMenuSelect *nemod = new SetupMenuSelect("Netto Mode", RST_NONE, nullptr, &netto_mode);
	nemod->setHelp(
			"In 'Relative' mode (also called 'Super-Netto') circling sink is considered,  to show climb rate as if you were circling there");
	nemod->addEntry("Normal");
	nemod->addEntry("Relative");
	vae->addEntry(nemod);

	SetupMenu *vdamp = new SetupMenu("Vario Damping", vario_menu_create_damping);
	vae->addEntry(vdamp);

	SetupMenu *meanclimb = new SetupMenu("Mean Climb", vario_menu_create_meanclimb);
	meanclimb->setHelp("Options for calculation of Mean Climb (MC recommendation) displayed by green/red dot");
	vae->addEntry(meanclimb);

	SetupMenu *s2fs = new SetupMenu("S2F Settings", vario_menu_create_s2f);
	vae->addEntry(s2fs);

	SetupMenu *elco = new SetupMenu("Electronic Compensation", vario_menu_create_ec);
	vae->addEntry(elco);
}

void options_menu_create_units(SetupMenu *top) {
	SetupMenuSelect *alu = new SetupMenuSelect("Altimeter", RST_NONE, unitChangeS, &alt_unit);
	alu->addEntry("Meter (m)");
	alu->addEntry("Feet (ft)");
	alu->addEntry("FL (FL)");
	top->addEntry(alu);
	SetupMenuSelect *iau = new SetupMenuSelect("Airspeed", RST_NONE, unitChangeS, &ias_unit);
	iau->addEntry("Kilom./hour (km/h)");
	iau->addEntry("Miles/hour (mph)");
	iau->addEntry("Knots (kt)");
	top->addEntry(iau);
	SetupMenuSelect *vau = new SetupMenuSelect("Vario", RST_NONE, update_range_entry_s, &vario_unit);
	vau->addEntry("Meters/sec (m/s)");
	vau->addEntry("100Feet/min (hfpm)");
	vau->addEntry("Knots (kt)");
	top->addEntry(vau);
	SetupMenuSelect *teu = new SetupMenuSelect("Temperature", RST_NONE, unitChangeS, &temperature_unit);
	teu->addEntry("Celsius");
	teu->addEntry("Fahrenheit");
#ifdef DEBUG_AND_TEST
	teu->addEntry("Kelvin");
#endif
	top->addEntry(teu);
	SetupMenuSelect *qnhi = new SetupMenuSelect("QNH", RST_NONE, unitChangeS, &qnh_unit);
	qnhi->addEntry("Hectopascal");
	qnhi->addEntry("InchMercury");
	top->addEntry(qnhi);
	SetupMenuSelect *dst = new SetupMenuSelect("Distance", RST_NONE, unitChangeS, &dst_unit);
	dst->addEntry("Meter (m)");
	dst->addEntry("Feet (ft)");
	top->addEntry(dst);
}

static void system_menu_create_airspeed(SetupMenu *top) {
    SetupMenuValFloat* spc = new SetupMenuValFloat("AS Calibration", "%", speedcal_change, &speedcal, RST_NONE, false);
    spc->setHelp("Calibration of airspeed sensor (AS). Normally not needed, unless the pressure probe has a systematic error");
    top->addEntry(spc);

    SetupMenuSelect* stawaen = new SetupMenuSelect("Stall Warning", RST_NONE, nullptr, &stall_warning);
    stawaen->setHelp("Enable alarm sound when speed goes below configured stall speed (until 30% less)");
    stawaen->mkEnable();
    top->addEntry(stawaen);

    if ( !airborne.get() ) {
        SetupMenuSelect* asze = new SetupMenuSelect("Zero Airspeed Sensor", RST_NONE, airspeed_zero, nullptr);
        top->addEntry(asze);
        asze->setHelp("Recalculate zero point for airspeed sensor right now");
        asze->addEntry("Cancel");
        asze->addEntry("Start");
    }

    if ( airspeed_sensor.get() == AirspeedSensor::ABPMRR || airspeed_sensor.get() == AirspeedSensor::TE4525 ) {
        // offer the option to switch in-between them, call it swapped tubes
        SetupMenuSelect* asswap = new SetupMenuSelect("Swapped Tubes", RST_NONE, tube_swap, &airspeed_sensor);
        top->addEntry(asswap);
        asswap->setHelp("Some airspeed sensors have pressure tubes swapped, resulting in no IAS indication.");
        asswap->addEntry("Straight", AirspeedSensor::ABPMRR);
        asswap->addEntry("Swapped", AirspeedSensor::TE4525);
    }
}

static void options_menu_create_altimeter(SetupMenu *top) {
	SetupMenuSelect *altDisplayMode = new SetupMenuSelect("Altitude Mode", RST_NONE, update_alti_buzz, &alt_display_mode);
	top->addEntry(altDisplayMode);
	altDisplayMode->setHelp("Select altitude display mode");
	altDisplayMode->addEntry(alti_mode[0].data());
	altDisplayMode->addEntry(alti_mode[1].data());

	SetupMenuSelect *atrans = new SetupMenuSelect("Auto Transition", RST_NONE, nullptr, &fl_auto_transition);
	top->addEntry(atrans);
	atrans->setHelp("Option to enable automatic altitude transition to QNH Standard (1013.25) above 'Transition Altitude'");
	atrans->mkEnable();

	SetupMenuValFloat *tral = new SetupMenuValFloat("Transition Altitude", "FL", nullptr, &transition_alt, RST_NONE, false);
	tral->setHelp("Transition altitude (or transition height, when using QFE) is the altitude/height above which standard pressure (QNE) is set (1013.2 mb/hPa)");
	top->addEntry(tral);

	if ( SetupCommon::isMaster() ) {
		// only the master XCV has this choice
		SetupMenuSelect *als = new SetupMenuSelect("Alt. Source", RST_NONE, SetupMenu::switch_alt_source, &alt_select);
		top->addEntry(als);
		als->setHelp("Select source for the altitude, either TE sensor or Baro sensor (default), or an external source e.g. FLARM");
		als->addEntry("Baro Sensor", ALT_BARO_SENSOR);
		als->addEntry("TE Sensor", ALT_TE_SENSOR);
		als->addEntry("GPS/FLARM", ALT_EXTERNAL);
	}

	SetupMenuSelect *alq = new SetupMenuSelect("Alt. Quantization", RST_NONE, nullptr, &alt_quantization);
	alq->setHelp("Set altimeter mode with discrete steps and rolling last digits");
	alq->addEntry("Disable");
	alq->addEntry("2");
	alq->addEntry("5");
	alq->addEntry("10");
	alq->addEntry("20");
	top->addEntry(alq);
}

void options_menu_create_flarm(SetupMenu* top) {
    if (top->getNrChilds() == 0) {
        top->setDynContent();

        SetupMenuSelect* flarml = new SetupMenuSelect("Level Threshold", RST_NONE, set_parent_dirty, &flarm_warning);
        flarml->setHelp("Level of FLARM alarm to enable: 1 is lowest (13-18 sec), 2 medium (9-12 sec), 3 highest (0-8 sec) until impact");
        flarml->addEntry("Disable", 4);
        flarml->addEntry("Level 1", 1);
        flarml->addEntry("Level 2", 2);
        flarml->addEntry("Level 3", 3);
        top->addEntry(flarml);

        SetupMenuValFloat* flarmt = new SetupMenuValFloat("Alarm Timeout", "sec", nullptr, &flarm_alarm_time);
        flarmt->setHelp("The time FLARM alarm warning keeps displayed after alarm went off");
        top->addEntry(flarmt);

        SetupMenuSelect* flarms = new SetupMenuSelect("Alarm Check", RST_NONE, startFlarmSimulation);
        flarms->setHelp("Simulate an airplane crossing from left to right with different alarm levels and vertical distance in 5 seconds");
        flarms->addEntry("Cancel");
        flarms->addEntry("Cross Deeper");
        flarms->addEntry("Cross Higher");
        flarms->addEntry("Head-on Deeper");
        flarms->addEntry("Overtake Left");
        flarms->addEntry("Cross Level");
        flarms->addEntry("Circling Left");
        top->addEntry(flarms);
    }
    SetupMenuSelect *tmp_menu = static_cast<SetupMenuSelect*>(top->getEntry(2)); // alarm check
	if ( flarm_warning.get() != 4 ) {
		tmp_menu->unlock();
	}
	else {
		tmp_menu->lock();
	}
}

static void screens_menu_create_vario(SetupMenu *top) {
    SetupMenuSelect *tgauge = new SetupMenuSelect("Upper Gauge", RST_NONE, nullptr, &vario_upper_gauge);
    tgauge->setHelp("Choose the content for this gauge");
    tgauge->addEntry("Disable", MultiGauge::GAUGE_NONE);
    tgauge->addEntry("IAS Speed", MultiGauge::GAUGE_IAS_SPEED);
    tgauge->addEntry("TAS Speed", MultiGauge::GAUGE_TAS_SPEED);
    tgauge->addEntry("GND Speed", MultiGauge::GAUGE_GND_SPEED);
    tgauge->addEntry("Altitude", MultiGauge::GAUGE_ALTIMETER);
    tgauge->addEntry("Speed2Fly", MultiGauge::GAUGE_S2F);
    tgauge->addEntry("McCready", MultiGauge::GAUGE_MC);
    tgauge->addEntry("Net. Vario", MultiGauge::GAUGE_NETTO);
    tgauge->addEntry("OATemp.", MultiGauge::GAUGE_OAT);
    tgauge->addEntry("Heading", MultiGauge::GAUGE_HEADING);
    if (gflags.expert) {
        tgauge->addEntry("Slip Angle", MultiGauge::GAUGE_SLIP);
    }
    top->addEntry(tgauge);

    SetupMenuSelect *bgauge = new SetupMenuSelect("Lower Gauge", RST_NONE, nullptr, &vario_lower_gauge);
    bgauge->setHelp("Choose the content for this gauge");
    bgauge->addEntry("Disable", MultiGauge::GAUGE_NONE);
    bgauge->addEntry("Altimeter", MultiGauge::GAUGE_ALTIMETER);
    bgauge->addEntry("Wind", MultiGauge::GAUGE_WIND);
    top->addEntry(bgauge);

    SetupMenuSelect* mc = new SetupMenuSelect("Speed2Fly", RST_NONE, nullptr, &vario_mc_gauge);
    mc->setHelp("Show the McCready setting and the recommended speed to fly");
    mc->mkEnable();
    top->addEntry(mc);

    SetupMenuSelect *scrcaid = new SetupMenuSelect("Therm.-Assist", RST_NONE, caid_reference, &vario_centeraid);
    scrcaid->setHelp("The thermal assistant; with reference on top, or on the side.");
    scrcaid->addEntry(ENABLE_MODE[0].data());
    scrcaid->addEntry("Topref");
    scrcaid->addEntry("Sideref");
    top->addEntry(scrcaid);

    SetupMenuSelect *wke = new SetupMenuSelect("Flap-Assist", RST_NONE, nullptr, &flapbox_enable);
    wke->mkEnable();
	wke->addEntry("Vis.Only");
    wke->setHelp("An indicator to assist optimum flap setting depending on speed, G-load and ballast");
    top->addEntry(wke);

    SetupMenuSelect *batv = new SetupMenuSelect("Battery Display", RST_NONE, nullptr, &battery_display);
    batv->setHelp("Display battery charge state either in Percentage e.g. 75% or Voltage e.g. 12.5V");
    batv->addEntry("Disable");
    batv->addEntry("Percentage");
    batv->addEntry("Voltage", 2);
    top->addEntry(batv);
}

static void options_menu_create_screens(SetupMenu *top) { // dynamic!
	if ( top->getNrChilds() == 0 ) {
		top->setDynContent();

		SetupMenu *vario = new SetupMenu("Variometer", screens_menu_create_vario);
		top->addEntry(vario);

		SetupMenu *gload = new SetupMenu("G-Meter", screens_menu_create_gload);
        if ( !accSensor ) {
            gload->lock();
            gload->setBuzzword("n/a");
        }
		top->addEntry(gload);

        SetupMenuSelect *horizon = new SetupMenuSelect("Horizon", RST_NONE, set_parent_dirty, &screen_horizon);
        horizon->mkEnable();
        if ( !accSensor ) {
            horizon->lock();
        }
        top->addEntry(horizon);

        SetupMenuSelect* ncolor = new SetupMenuSelect("Needle Color", RST_NONE, nullptr, &needle_color);
        ncolor->addEntry("Orange");
        ncolor->addEntry("Cream");
        ncolor->addEntry("White");
        top->addEntry(ncolor);

#ifdef DEBUG_AND_TEST
        SetupMenuSelect* disva = new SetupMenuSelect("Color Variant", RST_NONE, nullptr, &display_variant);
        top->addEntry(disva);
        disva->setHelp("Display variant white on black (W/B) or black on white (B/W)");
        disva->addEntry("W/B");
        disva->addEntry("B/W");
#endif
	}

	SetupMenu *tmp_menu = static_cast<SetupMenu*>(top->getEntry(0)); // vario
	if ( screen_gmeter.get() != SCREEN_PRIMARY ) {
		tmp_menu->setBuzzword("Primary");
		tmp_menu->unlock();
	}
	else {
		tmp_menu->setBuzzword("Disabled");
		tmp_menu->lock();
	}

	tmp_menu = static_cast<SetupMenu*>(top->getEntry(1)); // gload
	tmp_menu->setBuzzword(ENABLE_MODE[screen_gmeter.get()].data());
}

void options_menu_create(SetupMenu *opt) { // dynamic!
	if ( opt->getNrChilds() == 0 ) {
		opt->setDynContent();
		SetupMenuSelect *stumo = new SetupMenuSelect("Student Mode", RST_NONE, student_mode_action, &student_mode);
		opt->addEntry(stumo);
		stumo->setHelp(
				"Student mode, disables all sophisticated setup to just basic pre-flight related items like MC, ballast or bugs");
		stumo->mkEnable();
		
		// Vario
		SetupMenu *va = new SetupMenu("Vario and Speed 2 Fly", vario_menu_create);
		opt->addEntry(va);

		// Audio
		SetupMenu *ad = new SetupMenu("Audio", audio_menu_create);
		opt->addEntry(ad);

		// Airspeed
		SetupMenu *velocity = new SetupMenu("Airspeed", system_menu_create_airspeed);
		opt->addEntry(velocity);

		// Altimeter
		SetupMenu *alti = new SetupMenu("Altimeter", options_menu_create_altimeter);
		alti->setBuzzword(alti_mode[alt_display_mode.get()].data());
		opt->addEntry(alti);

		SetupMenu *flarm = new SetupMenu("FLARM", options_menu_create_flarm);
		opt->addEntry(flarm);
		flarm->setHelp("Option to display FLARM Warnings depending on FLARM alarm level");

		SetupMenu *compassWindMenu = new SetupMenu("Wind", options_menu_create_wind);
		opt->addEntry(compassWindMenu);
		compassWindMenu->setHelp("Setup Compass and Wind");

		SetupMenu *screens = new SetupMenu("Screens & Gauges", options_menu_create_screens);
		opt->addEntry(screens);
	}
	SetupMenu *flarm = static_cast<SetupMenu*>(opt->getEntry(5));
	if ( DEVMAN->getDevice(FLARM_DEV) != nullptr ) {
		flarm->unlock();
		flarm->setBuzzword();
	}
	else {
		flarm->lock();
		flarm->setBuzzword("n/a");
	}
}

void system_menu_create_software(SetupMenu *top) {
    SetupMenuSelect *ahrsid = new SetupMenuSelect("XCVario S/N", RST_NONE);
    ahrsid->addEntry(SetupCommon::getDefaultID());
    ahrsid->lock();
    top->addEntry(ahrsid);

    SetupMenuSelect* ver = new SetupMenuSelect("Rev.", RST_NONE, nullptr);
    ver->addEntry(FW_VERSION);
    ver->lock();
    top->addEntry(ver);

    SetupMenuSelect* upd = new SetupMenuSelect("Update", RST_IMMEDIATE, nullptr, &software_update);
    upd->setHelp("Update using the internet connection of your smart phone, or upload a binary using the ESP32 webserver.");
    upd->addEntry("Cancel");
    upd->addEntry("Easy Connect");
    upd->addEntry("Webserver");
    top->addEntry(upd);

    if (logged_tests.size() > 0) {
        SetupMenuDisplay* dis = new SetupMenuDisplay("Show Boot Messages", show_boot_log);
        top->addEntry(dis);
    }

    SetupMenuSelect* fa = new SetupMenuSelect("Factory Reset", RST_IMMEDIATE, factory_nvs_action);
    fa->setHelp("Reset all settings to factory defaults (metric system, 5 m/s vario range, etc.)");
    fa->addEntry("Cancel");
    fa->addEntry("ResetAll");
#ifdef DEBUG_AND_TEST
    fa->addEntry("ClearNVS", 2);
#endif
    if (gflags.expert) {
        fa->addEntry("Enter Factory Menu", 3);
    }
    top->addEntry(fa);

    small_buf[0] = ' ';
    small_buf[1] = '\0';
    SetupMenuChar* exp = new SetupMenuChar("Expert Menu",  "0A", 4, RST_NONE, expert_menu_action, small_buf);
    exp->setHelp("Enter code to unlock expert menu settings");
    top->addEntry(exp);
}

static void system_menu_create_battery(SetupMenu *top) {
	SetupMenuSelect *btype = new SetupMenuSelect("Battery Type", RST_NONE, select_battery_type);
	btype->setHelp("Apply template battery threshold voltages for different battery types");
	btype->addEntry("Cancel");
	btype->addEntry("LeadAcid");
	btype->addEntry("LiFePo4");
	top->addEntry(btype);

	SetupMenuValFloat *blow = new SetupMenuValFloat("Empty", "Volt ", nullptr, &bat_low_volt, RST_NONE, false);
	top->addEntry(blow);
	SetupMenuValFloat *bred = new SetupMenuValFloat("Critical", "Volt ", nullptr, &bat_red_volt, RST_NONE, false);
	top->addEntry(bred);
	SetupMenuValFloat *byellow = new SetupMenuValFloat("Moderate", "Volt ", nullptr, &bat_yellow_volt, RST_NONE, false);
	top->addEntry(byellow);
	SetupMenuValFloat *bfull = new SetupMenuValFloat("Full", "Volt ", nullptr, &bat_full_volt, RST_NONE, false);
	top->addEntry(bfull);

	SetupMenuValFloat *met_adj = SetupMenu::createVoltmeterAdjustMenu();
	top->addEntry(met_adj);
}


void system_menu_create_hardware_type(SetupMenu *top) {
	// Display Orientation
	SetupMenuSelect * diso = new SetupMenuSelect( "Orientation", RST_ON_EXIT, nullptr, &display_orientation );
	top->addEntry( diso );
	diso->setHelp( "Display Orientation.  NORMAL means Rotary on left, TOPDOWN means Rotary on right  (reboots). A change will reset the IMU reference.");
	diso->addEntry( "NORMAL");
	diso->addEntry( "TOPDOWN");
    if ( gflags.expert ) {
        diso->addEntry( "NINETY");
    }

	SetupAction *dtest = new SetupAction("Display Test", do_display_test, 0);
	dtest->setHelp("Start display test screens, press rotary to cancel");
	top->addEntry(dtest);

	SetupMenuValFloat *dcadj = new SetupMenuValFloat("Display Clk Adj", "%", nullptr, &display_clock_adj, RST_IMMEDIATE);
	dcadj->setHelp("Modify display clock by given percentage (restarts on exit)");
	top->addEntry(dcadj);

}

void system_menu_create_hardware_rotary(SetupMenu *top) {
	SetupMenuSelect *roinc = new SetupMenuSelect("Sensitivity", RST_NONE, set_rotary_increment, &rotary_inc);
	top->addEntry(roinc);
	roinc->setHelp(
			"Select rotary sensitivity in number of tick's for one increment, to your personal preference, 1 tick is most sensitive");
	roinc->addEntry("1 indent", 1);
	roinc->addEntry("2 indent", 2);

    // Rotary Default
#ifdef DEBUG_AND_TEST
    SetupMenuSelect* rd = new SetupMenuSelect("Primary Use", RST_NONE, nullptr, &rot_default);
    top->addEntry(rd);
    rd->setHelp("Select value to be altered at rotary movement outside of setup menu (reboots)");
    rd->addEntry("Volume");
    rd->addEntry("MC Value");
#endif

    SetupMenuSelect *sact = new SetupMenuSelect("Enter Setup by", RST_NONE, nullptr, &menu_long_press);
	top->addEntry(sact);
	sact->setHelp("Activate setup menu either by short or long button press");
	sact->addEntry("Short Press");
	sact->addEntry("Long Press");
}

static void system_menu_create_hardware(SetupMenu *top) {
    if (top->getNrChilds() == 0) {
        top->setDynContent();

        SetupMenu* display = new SetupMenu("Display Type", system_menu_create_hardware_type);
        top->addEntry(display);

        SetupMenu* rotary = new SetupMenu("Rotary Knob", system_menu_create_hardware_rotary);
        top->addEntry(rotary);

        // Flap::setupMenue(top);
        SetupMenu* wkm = new SetupMenu("Flap Sensor", flap_menu_create_flap_sensor);
        top->addEntry(wkm);

        SetupMenuSelect *gear = new SetupMenuSelect("Gear Warn", RST_NONE, config_gear_warning, &gear_warning);
		top->addEntry(gear);
		gear->setHelp("Enable gear warning on S2 flap sensor or serial RS232 pin (pos. or neg. signal) or by external command");
		gear->addEntry("Disable");
		gear->addEntry("S2 Flap positive"); // A positive signal, high signal or > 2V will start alarm
		gear->addEntry("S2 RS232 positive");
		gear->addEntry("S2 Flap negative"); // A negative signal, low signal or < 1V will start alarm
		gear->addEntry("S2 RS232 negative");
		gear->addEntry("External");  // A $g,w<n>*CS command from an external device

        if (accSensor) {
            SetupMenu* ahrs = new SetupMenu("IMU & AHRS", system_menu_create_hardware_imu);
            top->addEntry(ahrs);
        }

        SetupMenu *bat = new SetupMenu("Battery Meter", system_menu_create_battery);
        bat->setHelp("Adjust voltage thresholds for battery state indication");
        top->addEntry(bat);
    }
    SetupMenu* wkm = static_cast<SetupMenu*>(top->getEntry(2));  // Flap Sensor
    if (Speed2Fly.hasFlaps()) {
        wkm->unlock();
        if (flap_sensor.get()) {
            wkm->setBuzzword(ENABLE_MODE[1].data());  // enabled
        } else if (Flap::sensAvailable()) {
            wkm->setBuzzword(ENABLE_MODE[4].data());  // from peer
        } else {
            wkm->setBuzzword(ENABLE_MODE[0].data());  // disabled
        }
    } else {
        wkm->lock();
        wkm->setBuzzword("n/a");
    }
}

void system_menu_create(SetupMenu *sye) {
	SetupMenu *soft = new SetupMenu("Software", system_menu_create_software);
	sye->addEntry(soft);

	// Glider Setup
	SetupMenu *po = new SetupMenu("Glider Type", glider_menu_create);
	po->setBuzzword(Polars::getGliderType(Speed2Fly.getMyGliderIdx()));
	po->setHelp("Polar, weight and all attributes of the glider");
	sye->addEntry(po);

	// Units
	SetupMenu *un = new SetupMenu("Units", options_menu_create_units);
	sye->addEntry(un);
	un->setHelp("Setup altimeter, airspeed indicator and variometer with European Metric, American, British or Australian units");

	SetupMenu *hardware = new SetupMenu("Hardware & Sensors", system_menu_create_hardware);
	hardware->setHelp("Setup variometer hardware e.g. display, rotary, AS and AHRS sensor, voltmeter, etc");
	sye->addEntry(hardware);

	// XCV role
	SetupMenuSelect *role = new SetupMenuSelect("XCV device role", RST_IMMEDIATE, nullptr, &xcv_role);
	role->setHelp("Set the intended role of this device first (needs a reboot)");
	role->addEntry("Master", MASTER_ROLE);
	role->addEntry("Second", SECOND_ROLE);
	sye->addEntry(role);

	// Devices menu
	SetupMenu *devices = new SetupMenu("Connected Devices", system_menu_connected_devices);
	devices->setHelp("Devices, Interf.s, Protocols");
	sye->addEntry(devices);

    if (gflags.expert) {
        SetupMenuSelect *logg = new SetupMenuSelect("Logging", RST_NONE, nullptr, &logging);
        logg->setHelp("R&D option, do not just enable!\n Collect data with NMEA logger in XCSoar");
        logg->addEntry("Disable");
        logg->addEntry("Wind");
        logg->addEntry("GYRO/MAG");
        logg->addEntry("Both");
        logg->addEntry("All Sensor Data");
        sye->addEntry(logg);

        // SetupAction *devdump = new SetupAction("Device Setup Dump", deviceDumpAction, 0);
        // sye->addEntry(devdump);
        SetupMenuDisplay *info = new ShowFlightInfo("Flight Info");
        sye->addEntry(info);
    }
}

void setup_create_root(SetupMenu *top) {
	ESP_LOGI(FNAME,"setup_create_root()");
	if (rot_default.get() == 0) {
		SetupMenuValFloat *mc = new SetupMenuValFloat("MC", "", nullptr, &MC);
		mc->setHelp(
				"Mac Cready value for optimum cruise speed or average climb rate, in same unit as the variometer");
		mc->setPrecision(1);
		mc->setTerminateMenu();
		top->addEntry(mc);
	} else {
		SetupMenuValFloat *vol = new SetupMenuValFloat("Audio Volume", "%", nullptr, &audio_volume, RST_NONE, true);
		vol->setHelp("Audio volume level for variometer tone on internal and external speaker");
		vol->setTerminateMenu();
		top->addEntry(vol);
	}

	ESP_LOGI(FNAME, "Create bugs menu");
	Device *jumbo = DEVMAN->getDevice(DeviceId::JUMBO_DEV);
	if (jumbo) {
		sprintf(small_buf, "Bugs at %d%%", int(bugs.get()));
		SetupMenu *wiper = new SetupMenu(small_buf, wiper_menu_create);
		top->addEntry(wiper);
	} else {
		bugs_item_create(top);
	}

	SetupMenuValFloat *bal = SetupMenu::createBallastMenu();
	top->addEntry(bal);

    SetupMenuValFloat* crewball = new SetupMenuValFloat("Crew Weight", "kg", start_weight_adj, &crew_weight, RST_NONE, true);
    crewball->setPrecision(0);
    crewball->setHelp("Weight of the pilot(s) including parachute (everything on top of the empty weight apart from ballast)");
    crewball->setNeverInline();
    top->addEntry(crewball);

    SetupMenuValFloat *qnh_menu = SetupMenu::createQNHMenu(true);
    qnh_menu->setHelp("QNH pressure value as from next ATC");
	top->addEntry(qnh_menu);

	SetupMenuValFloat *afe = new SetupMenuValFloat("Airfield Elevation", "", nullptr, &airfield_elevation);
	afe->setHelp(
			"Airfield elevation in meters for QNH auto adjust");
	afe->setRotDynamic(3.0);
	top->addEntry(afe);

	// Clear student mode, password correct
	if ((int) (password.get()) == 271) {
		student_mode.set(0);
		password.set(0);
	}
	// Student mode: Query password
	if (student_mode.get()) {
		SetupMenuValFloat *passw = new SetupMenuValFloat("Expert Password", "", nullptr, &password);
		passw->setPrecision(0);
		passw->setHelp(
				"To exit from student mode enter expert password and restart device after expert password has been set correctly");
		passw->setTerminateMenu();
		top->addEntry(passw);
	} else {
		// Options Setup
		SetupMenu *opt = new SetupMenu("Options", options_menu_create);
		top->addEntry(opt);

		// System Setup
		SetupMenu *sy = new SetupMenu("System", system_menu_create);
		top->addEntry(sy);
	}
}



////////////////////////////////////
// Central menu creation
////////////////////////////////////


gpio_num_t SetupMenu::getGearWarningIO() {
	gpio_num_t io = GPIO_NUM_0;
	if (gear_warning.get() == GW_FLAP_SENSOR
			|| gear_warning.get() == GW_FLAP_SENSOR_INV) {
		io = GPIO_NUM_34;
	} else if (gear_warning.get() == GW_S2_RS232_RX
			|| gear_warning.get() == GW_S2_RS232_RX_INV) {
		io = GPIO_NUM_18;
	}
	return io;
}

void SetupMenu::initGearWarning() {
	gpio_num_t io = SetupMenu::getGearWarningIO();
	if (io != GPIO_NUM_0) {
		gpio_reset_pin(io);
		gpio_set_direction(io, GPIO_MODE_INPUT);
		gpio_set_pull_mode(io, GPIO_PULLUP_ONLY);
		gpio_pullup_en(io);
        // add gear warn to my caps
        CANPeerCaps::addCapability(XcvCaps::GEARSENS_CAP);
	}
	ESP_LOGI(FNAME,"initGearWarning: IO: %d", io );
}

int SetupMenu::switch_alt_source(SetupMenuSelect* p) {
    if (alt_select.get() == ALT_BARO_SENSOR) {
        // point baro pressure input to publish pressure
        dynamic_cast<PressureSensor*>(baroSensor)->setNVSVar(&statp);
        dynamic_cast<PressureSensor*>(teSensor)->setNVSVar(nullptr);
    } else if (alt_select.get() == ALT_TE_SENSOR) {
        // suppress baro pressure input
        dynamic_cast<PressureSensor*>(baroSensor)->setNVSVar(nullptr);
        dynamic_cast<PressureSensor*>(teSensor)->setNVSVar(&statp);
    } else {
        // GPS sets the statp directly, suppress pressure input
        dynamic_cast<PressureSensor*>(baroSensor)->setNVSVar(nullptr);
        dynamic_cast<PressureSensor*>(teSensor)->setNVSVar(nullptr);
    }
    return 0;
}


SetupMenu* SetupMenu::createTopSetup() {
	const char *top_menu_name = "Setup";
	if (glider_type.get() != 1000 || ! S2F::isPolarEqualTo(0)) {
		top_menu_name =	Polars::getPolarName(Speed2Fly.getMyGliderIdx());
	}
	SetupMenu *setup = new  SetupMenu(top_menu_name, setup_create_root);
	return setup;
}


SetupMenu* SetupMenu::createFactorySetup() {
    sprintf(small_buf, "%s - %s", SetupCommon::getDefaultID(), FW_VERSION);
    SetupMenu *setup = new SetupMenu(small_buf, nullptr);

    SetupMenuValFloat *met_adj = SetupMenu::createVoltmeterAdjustMenu();
    setup->addEntry(met_adj);

    SetupAction *dtest = new SetupAction("Display Test", do_display_test, 0);
    setup->addEntry(dtest);

    if( accSensor ){
    	SetupMenuSelect* bias_zero = new SetupMenuSelect("IMU Biases", RST_NONE, imu_calib);
    	bias_zero->addEntry("Cancel");
    	bias_zero->addEntry("Acc. Bias Calib.", 3);
    	bias_zero->addEntry("AccBias Check", 6);
    	bias_zero->addEntry("Gyro Reset", 4);
    	bias_zero->addEntry("Acc. Reset", 5);
    	setup->addEntry(bias_zero);
    }

    SetupMenuDisplay *leak = new LeakTest("Leak Test");
    setup->addEntry(leak);

    SetupMenuSelect* fa = new SetupMenuSelect("Exit Factory Menu", RST_NONE, exitFactoryMenu);
    fa->addEntry("Cancel");
    fa->addEntry("Exit");
    setup->addEntry(fa);
    return setup;
}

SetupMenuValFloat* SetupMenu::createQNHMenu() {
	SetupMenuValFloat *qnh = new SetupMenuValFloat("QNH", "", qnh_adj, &QNH, RST_NONE, true);
    qnh->setPrecision(2);
	qnh->setTerminateMenu();
	qnh->setHelp("QNH pressure value from ATC. On ground you may adjust to airfield altitude above MSL");
	return qnh;
}

SetupMenuValFloat* SetupMenu::createBallastMenu() {
    SetupMenuValFloat *bal = new SetupMenuValFloat("Ballast", "liter", water_adj, &ballast_kg, RST_NONE, true);
    bal->setHelp("Amount of water ballast added to the over all weight");
    bal->setPrecision(0);
    bal->setNeverInline();
    bal->setMax(polar_max_ballast.get());
    return bal;
}

SetupMenuValFloat* SetupMenu::createVoltmeterAdjustMenu() {
	SetupMenuValFloat *met_adj = new SetupMenuValFloat("Voltmeter Adjust", "%", factv_adj, &factory_volt_adjust, RST_NONE, true);
	met_adj->setHelp("Factory fine adjust voltmeter");
	met_adj->setNeverInline();
	return met_adj;
}

