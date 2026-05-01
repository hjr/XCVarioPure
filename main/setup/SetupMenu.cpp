/*
 * SetupMenu.cpp
 *
 *  Created on: Feb 4, 2018
 *      Author: iltis
 */

#include "SetupMenu.h"

#include "CenterAid.h"
#include "SetupCommon.h"
#include "math/Floats.h"
#include "setup/SubMenuAudio.h"
#include "setup/SubMenuDevices.h"
#include "setup/SubMenuCompassWind.h"
#include "setup/SubMenuGlider.h"
#include "setup/SubMenuFlap.h"
#include "setup/ShowBootMsg.h"
#include "setup/ShowFlightInfo.h"
#include "screen/element/MultiGauge.h"
#include "Colors.h"
#include "sensor/VarioFilter.h"
#include "S2F.h"
#include "Version.h"
#include "glider/Polars.h"
#include "math/Units.h"
#include "Atmosphere.h"
#include "driver/gpio/S2fSwitch.h"
#include "Flap.h"
#include "setup/SetupMenuSelect.h"
#include "setup/SetupMenuValFloat.h"
#include "setup/SetupMenuDisplay.h"
#include "setup/SetupAction.h"
#include "setup/SetupNG.h"
#include "Flarm.h"
#include "protocol/FlarmSim.h"
#include "sensor/imu/AccMPU6050.h"
#include "sensor/imu/GyroMPU6050.h"
#include "sensor/pressure/PressureSensor.h"
#include "driver/audio/ESPAudio.h"
#include "driver/gpio/AnalogInput.h"
#include "sensor/press_diff/AirspeedSensor.h"
#include "AdaptUGC.h"
#include "sensor.h"
#include "test/LeakTest.h"
#include "logdefnone.h"

#include "comm/DeviceMgr.h"
#include "math/Trigonometry.h"
#include "protocol/NMEA.h"
#include "protocol/nmea/JumboCmdMsg.h"
#include "protocol/CANPeerCaps.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <string_view>

extern AdaptUGC *MYUCG;

static void setup_create_root(SetupMenu *top);

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
static void screens_menu_create_gload(SetupMenu *top);
static void screens_menu_create_extreme_records(SetupMenu *top);

static void system_menu_create(SetupMenu *top);
static void system_menu_create_software(SetupMenu *top);
static void system_menu_create_battery(SetupMenu *top);
static void system_menu_create_hardware(SetupMenu *top);
static void system_menu_create_hardware_type(SetupMenu *top);
static void system_menu_create_hardware_rotary(SetupMenu *top);
static void system_menu_create_hardware_imu(SetupMenu *top);
static void system_menu_create_hardware_ahrs_parameter(SetupMenu *top);


static char small_buf[32];

static int set_parent_parent_dirty(SetupMenuSelect* p) {
    p->getParent()->getParent()->setDirty();
    return 0;
}
static int set_parent_dirty(SetupMenuSelect* p) {
    p->getParent()->setDirty();
    return 0;
}

int gload_reset(SetupMenuSelect *p) {
	if ( p->getSelect() == 1 ) {
		gload_pos_max.set(1);
		gload_neg_max.set(0);
		airspeed_max.set(0);
		p->setSelect(0);
		set_parent_dirty(p);
	}
	return 0;
}

static int caid_reference(SetupMenuSelect* p) {
    if ( theCenteraid ) {
        theCenteraid->setGliderOnTop(p->getSelect() != 2);
    }
    return 0;
}

static int factory_nvs_action(SetupMenuSelect* p) {
    if (p->getSelect() == 1) {
        factory_flag.set(factory_flag.get() | 1);
    }
    else if (p->getSelect() == 2) {
        ESP_LOGI(FNAME, "Clearing NVS...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_flash_init();
        p->reBoot();
    }
    else if (p->getSelect() == 3) {
        factory_flag.set(factory_flag.get() & ~2);
    }
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

int set_ahrs_defaults(SetupMenuSelect *p) {
	if (p->getSelect() == 1) {
		ahrs_gyro_factor.set(ahrs_gyro_factor.getDefault());
		ahrs_min_gyro_factor.set(ahrs_min_gyro_factor.getDefault());
		ahrs_dynamic_factor.set(ahrs_dynamic_factor.getDefault());
		gyro_gating.set(gyro_gating.getDefault());
	}
	p->setSelect(0);
	return 0;
}

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

int config_gear_warning(SetupMenuSelect *p) {
	SetupMenu::initGearWarning();
	return 0;
}

int do_display_test(SetupMenuSelect* p) {
    if (display_test.get()) {
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
    }
    p->setSelect(0);
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

// static int add_key(SetupMenuChar *p) {
//     ESP_LOGI(FNAME, "add_key( %s ) ", p->value());
//     if (p->value()[0] == '@') {
//         // hidden short-cut to delete the license key
//         ahrs_licence.set("");
//         p->reset();
//     } else {
//         ahrs_licence.set(p->value());
//     }
//     Cipher crypt;
//     gflags.ahrsKeyValid = crypt.checkKeyAHRS();
//     return 0;
// }

static int imu_gaa(SetupMenuValFloat *f) {
    if (accSensor && !(imu_reference.get() == Quaternion())) {
        accSensor->getMpu().applyImuReference(f->get(), imu_reference.get());
    }
    return 0;
}

// 1: left wing down, -1: right wing down, 0: leveled
static void drawGliderPic(int16_t lev) {
    int X = DISPLAY_W / 2;
    int Y = DISPLAY_H / 2;
    MYUCG->setColor(COLOR_BLACK);
    MYUCG->drawBox(X-70, Y-20, 140, 40);
    MYUCG->setColor(COLOR_WHITE);
    MYUCG->drawDisc(X, Y+4, 6, UCG_DRAW_ALL);
    MYUCG->drawLine(X-50, Y+7*lev, X+50, Y-7*lev); // wing
    MYUCG->drawLine(X, Y, X-1*lev, Y-10);
    MYUCG->drawLine(X-9-1*lev, Y-10+1*lev, X+9+1*lev, Y-10-1*lev); // tail wing
    if ( lev != 0 ) {
        // some dots depicting the motion
        MYUCG->setColor(COLOR_RED);
        MYUCG->drawDisc(X+61*lev, Y-15, 2, UCG_DRAW_ALL);
        MYUCG->drawDisc(X+64*lev, Y-7, 2, UCG_DRAW_ALL);
        MYUCG->drawDisc(X+63*lev, Y+1, 2, UCG_DRAW_ALL);
        MYUCG->drawTriangle(X+62*lev, Y+12, X+62*lev+5, Y+7, X+62*lev-5, Y+7);
    }
}

static void doImuCalibration(SetupMenuSelect* p) {
    MYUCG->setFont(ucg_font_ncenR14_hr, true);
    p->clear();
    p->menuPrintLn("IMU Glider Reference", 2, 18);
    constexpr int16_t next_step = 4;
    int16_t nlidx = next_step;
    p->menuPrintLn("Checking Gyro Bias", nlidx++);
    nlidx += 3;
    p->menuPrintLn("Wait for the Chimes, ", nlidx++);
    p->menuPrintLn("abort with button.", nlidx);

    accSensor->getMpu().resetCalibProgress();
    // load the default reference to the IMU
    Quaternion backup = accSensor->getMpu().getRefRot();
    accSensor->getMpu().applyImuReference(0, MpuImu::getDefaultImuReference());
    // reset lever arm
    accSensor->getMpu().setLeverArm(0.f);

    // double check that the gyro bias is set ok, otherwise the result would be useless
    vector_f gyro;
    float gnorm;
    bool abort = false;
    do {
        abort = Rotary->readSwitch(500); // fixme -> interferes with MBOX and malfunct totaly when MBOX activ
        gyro = gyroSensor->getAVG(1000) - gyroSensor->getBias();
        gnorm = gyro.get_norm();
        ESP_LOGI(FNAME, "gyro norm: %f, gcalm %d acalm %d", gnorm, gyroSensor->isResting(), accSensor->isResting());
    } while ((gnorm > GyroMPU6050::GYRO_THRESHOLD || !gyroSensor->isResting()) && !abort);

    ESP_LOGI(FNAME, "gyro reading: (%f/%f/%f): %f < %f", gyro.x, gyro.y, gyro.z, gnorm, GyroMPU6050::GYRO_THRESHOLD);

    p->clear();
    p->menuPrintLn("IMU Glider Reference", 2, 18);
    nlidx = next_step;

    float angle;
    float ground_angle;
    int ret = 0;
    bool once = true;
    while (once && !abort)
    {
        uint32_t start_time;
        uint32_t stop_time;
        vector_f gyro_integral;
        bool button = false;
        once = false;

        AUDIO->startSound(AUDIO_TADDA | PRIO_SND_MASK, false, 100);

        for (int i=0; i<3; i++) {

            if (i == 0) {
                // first wing down
                p->menuPrintLn("Move first wing down    ", next_step);
                // draw a picture of the expected movement to support the user
                drawGliderPic(1);
            }
            else if (i == 1) {
                // second wing down
                p->menuClearLn(next_step);
                MYUCG->setColor(COLOR_RED);
                p->menuPrintLn("Move other wing down    ", next_step);
                drawGliderPic(-1);
            }
            else if (i == 2) {
                // wings level
                p->menuClearLn(next_step);
                MYUCG->setColor(COLOR_RED);
                p->menuPrintLn("Hold wings level      ", next_step);
                drawGliderPic(0);
            }
            MYUCG->setColor(COLOR_WHITE);
            nlidx = next_step + 3;
            p->menuClearLn(nlidx);
            p->menuPrintLn("Start with button press.", nlidx);
            p->menuClearLn(nlidx+1);
            while (!Rotary->readSwitch(700)) ;
            p->menuClearLn(nlidx);
            p->menuPrintLn("Wait for the Chimes,", nlidx++);
            p->menuPrintLn("after movement finished.", nlidx++);

            // wait for the first wing movement
            start_time = Clock::getMillis(); // save the time when the movement starts, to calculate the gyro integral later
            gyroSensor->resetRest();
            accSensor->resetRest();
            
            // wait until movement stops, save the gyro average and integral
            button = false;
            while ( (!gyroSensor->isResting() || !accSensor->isResting()) && !button) {
                // make a noise if the movement is detected, to support the user in the procedure
                if (i < 2) AUDIO->startSound(AUDIO_KNOCK | PRIO_SND_MASK, false, 100);
                ESP_LOGI(FNAME, "Wait for standstill, gcalm %d acalm %d", gyroSensor->isResting(), accSensor->isResting());
                button = Rotary->readSwitch(700);
            }
            // wing movement stopped, save the gyro integral
            stop_time = Clock::getMillis();
            gyro_integral = gyroSensor->getIntegral(stop_time - start_time);
            // compensate bias to get actual movement integral
            gyro_integral -= gyroSensor->getBias() * (float)((stop_time - start_time) / 100.f); // 10 Hz
            // sample the accel for the bob vector
            ret = accSensor->getMpu().getAccelSamplesAndCalib(gyro_integral, angle, ground_angle);
            if (i<2 || ret == 4) AUDIO->startSound(AUDIO_TADDA | PRIO_SND_MASK, false, 100);
        }
    }

    // set lever arm again
    accSensor->getMpu().setLeverArm(imu_leverarm.get());

    if (ret < 4 || abort) {
        p->clear();
        p->menuPrintLn("... aborted ...", 2);
        nlidx = 4;
        if ( gnorm > GyroMPU6050::GYRO_THRESHOLD ) {
            p->menuPrintLn("IMU bias too high,", nlidx++);
            p->menuPrintLn("try a restart.", nlidx++);
        } else {
            p->menuPrintLn("The movement covered", nlidx++);
            p->menuPrintLn("too small an angle.", nlidx++);
        }
        accSensor->getMpu().setRefRot(backup);
        p->menuPrintLn("press button to return", 8, 1);
        while (!Rotary->readSwitch(100))
            ;
        return;
    }

    p->clear();
    p->menuPrintLn("Success !", 2, 20);
    MYUCG->setPrintPos(1, 150);
    MYUCG->printf("Wing Angle: %.1f°", rad2deg(angle));
    MYUCG->setPrintPos(1, 175);
    MYUCG->printf("Ground Angle: %.1f°", rad2deg(ground_angle));
    p->menuPrintLn("press button to return", 10, 1);
    while (!Rotary->readSwitch(100))
        ;
}


static void factoryAccCalibration(SetupMenuSelect* p) {
    MYUCG->setFont(ucg_font_ncenR14_hr, true);
    p->clear();
    p->menuPrintLn("Factory Acc. Calibration", 2, 18);
    constexpr int16_t next_step = 4;
    int16_t nlidx = next_step;
    p->menuPrintLn("Choose 6 unique orientations.", nlidx++);
    nlidx++;
    p->menuPrintLn("Start with button.", nlidx);
    while (!Rotary->readSwitch(200)) ;
    p->menuClearLn(nlidx);
    p->menuPrintLn("Wait for the Chimes, ", nlidx++);
    p->menuPrintLn("to go on.", nlidx++);
    p->menuPrintLn("Abort with button.", nlidx++);
    nlidx++;

    // reset lever arm
    accSensor->getMpu().setLeverArm(0.f);
    // need to reset the acc bias, because this is the only way we can really measure it
    accSensor->resetBias();

    int pos = 0;
    bool abort = false;
    vector_f samples[6];
    while (pos < 6 && !abort)
    {
        // wait until calm
        do {
            abort = Rotary->readSwitch(500);
            ESP_LOGI(FNAME, "Rest gcalm %d acalm %d", gyroSensor->isResting(), accSensor->isResting());
        } while ((!accSensor->isResting() || !gyroSensor->isResting()) && !abort);

        char buf[64];
        snprintf(buf, sizeof(buf), "Position %d/6", pos+1);
        p->menuPrintLn(buf, nlidx);

        // rest condition ensures 2 seconds calm history
        if ( ! abort ) {
            // read the acc average
            samples[pos] = accSensor->getAVG(2000);
            if (pos < 5) { AUDIO->startSound(AUDIO_TADDA | PRIO_SND_MASK, false, 100); }
            pos++;
        }
        gyroSensor->resetRest();
        accSensor->resetRest();
    }

    // rough quality check on samples
    // the sum should be close to zero, otherwise the device was not really moved around all axes
    vector_f sum;
    for ( int i=0; i < 6; i++ ) {
        sum += samples[i];
        ESP_LOGI(FNAME, "Sample %d: %f,%f,%f", i, samples[i].x, samples[i].y, samples[i].z);
    }
    ESP_LOGI(FNAME, "Sample sum: %f,%f,%f, norm: %f", sum.x, sum.y, sum.z, sum.get_norm());

    if (pos < 6 || sum.get_norm() > 0.8f || abort) {
        p->clear();
        p->menuPrintLn("... aborted ...", 2);
        nlidx = 4;
        if ( ! abort && pos == 6 ) {
            p->menuPrintLn("Too simillar samples", nlidx++);
        }
        accSensor->getMpu().restoreAccelOffset();
    }
    else {
        // calculate bias from samples and push to sensor
        vector_f bias = accSensor->getMpu().extractAccBias(samples, 6);
        accSensor->pushBias(bias);
        AUDIO->startSound(AUDIO_TADDA | PRIO_SND_MASK, false, 100);
        p->clear();
        p->menuPrintLn("Success !", 2, 20);
    }

    // restore IMU setup
    accSensor->getMpu().setLeverArm(imu_leverarm.get());

    p->menuPrintLn("press button to return", 8, 1);
    while (!Rotary->readSwitch(100)) ;
}

static int imu_calib(SetupMenuSelect* p) {
    ESP_LOGI(FNAME, "Collect AHRS data (%d)", p->getValue());
    int sel = p->getValue();
    switch (sel) {
        case 0:
            break;  // cancel
        case 1:
            // do the calibration procedure
            doImuCalibration(p);
            break;
        case 2:
            // reset to default
            accSensor->getMpu().resetImuReference();
            break;
        case 3:
            // factory acc calib
            factoryAccCalibration(p);
            break;
        case 4:
            // set Gyro bias to zero
            accSensor->getMpu().zeroGyroBias();
            break;
        case 5:
            // set Acc bias to zero
            accSensor->getMpu().zeroAccBias();
            break;
        default:
            break;
    }
    p->setSelect(0);
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
    MYUCG->printf("%5d %s  ", fast_iroundf(alt), AltUnit->getName());

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

int wiper_button(SetupAction *p) {
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

///////////////////////////////////////////////////////////////////////////////////////////////////////
// SetupMenu
///////////////////////////////////////////////////////////////////////////////////////////////////////
SetupMenu::SetupMenu(const char *title, SetupMenuCreator_t menu_create, int cont_id) :
	MenuEntry(title),
	populateMenu(menu_create),
	content_id(cont_id)
{
	// ESP_LOGI(FNAME,"SetupMenu::SetupMenu( %s ) ", title );
	setRotDynamic(1.f);
}

SetupMenu::~SetupMenu() {
	ESP_LOGI(FNAME,"del SetupMenu( %s ) ", _title.c_str() );
	for (auto *c : _childs) {
		delete c;
	}
	_childs.clear();
}

void SetupMenu::enter()
{
    if (isLocked()) { return; }

	ESP_LOGI(FNAME,"enter inSet %d, mptr: %p", gflags.inSetup, populateMenu );
	if ((_childs.empty() || dyn_content) && populateMenu) {
		(populateMenu)(this); // callback needs to be designed for this !!!
		ESP_LOGI(FNAME,"created childs %d", _childs.size());
	}
	MenuEntry::enter();
}

void SetupMenu::display(int mode)
{
	if (dirty && dyn_content && populateMenu) {
		// Cope with changes in menu item presence
		ESP_LOGI(FNAME,"SetupMenu display() dirty %d", dirty );
		(populateMenu)(this);
		ESP_LOGI(FNAME,"create_childs %d", _childs.size());
	}
	ESP_LOGI(FNAME,"SetupMenu display(%s-%d)", _title.c_str(), highlight );
	if ( highlight >= (int)(_childs.size()) ) {
		highlight = _childs.size()-1;
	}
	ESP_LOGI(FNAME,"SetupMenu display %d", highlight );
	clear();
	ESP_LOGI(FNAME,"Title: %s child size:%d", _title.c_str(), _childs.size());
	MYUCG->setFont(ucg_font_ncenR14_hr, true);
	MYUCG->setFontPosBottom();
	menuPrintLn("  <", 0);
	menuPrintLn(_title.c_str(), 0, 30);
	doHighlight(highlight);
	for (int i = 0; i < _childs.size(); i++) {
		MenuEntry *child = _childs[i];
		// cope with potential external change to the e.g. nvs representation of values
		if ( dirty ) { child->refresh(); }
		
		// Menu entry
		MYUCG->setColor(COLOR_WHITE);
        const char *cv = child->value();
		if ( ! child->isLeaf()  || cv ) {
			MYUCG->setColor(COLOR_BBLUE);
		}
		menuPrintLn(child->getTitle(), i+1);

		// Optional value or detail
		// ESP_LOGI(FNAME,"Child Title: %s - %p", child->getTitle(), child->value() );
		if (cv && *cv != '\0')
		{
			// Detail seperator
			const char *sep = "> ";
			int entry_len = MYUCG->getStrWidth(child->getTitle());
			if ( child->isLeaf() ) {
				sep = ": ";
			}
			menuPrintLn(sep, i+1, 1+entry_len);

			if (child->isLeaf() ) {
				MYUCG->setColor(COLOR_WHITE);
			} else {
				MYUCG->setColor(COLOR_HEADER_LIGHT);
			}
			menuPrintLn(cv, i+1, 1 + entry_len + MYUCG->getStrWidth(sep));
			ESP_LOGI(FNAME,"Child V: %s", cv );
		}
		// ESP_LOGI(FNAME,"Child: %s y=%d",child->getTitle() ,y );
	}
	showhelp(true);
	dirty = false;
}

int16_t SetupMenu::freeBottomLines() const
{
    return dheight/25 - getNrChilds() - 1;
}

int16_t SetupMenu::firstHelpLine() const
{
    return getNrChilds() + 1;
}

// void SetupMenu::setHighlight(MenuEntry *value)
// {
// 	int found = -1;
// 	for (int i = 0; i < _childs.size(); ++i) {
// 		if (_childs[i] == value) {
// 			found = i;
// 			break;
// 		}
// 	}
// 	if ( found >= 0 ) {
// 		highlight = found;
// 	}
// }
void SetupMenu::setHighlight(int chnr)
{
	if ( chnr >= 0 && chnr < (int)_childs.size() ) {
		highlight = chnr;
	}
}

MenuEntry* SetupMenu::addEntry( MenuEntry * item )
{
	ESP_LOGI(FNAME,"add to childs");
	item->regParent(this);
	_childs.push_back( item );
	return item;
}

void SetupMenu::delEntry( MenuEntry * item ) {
	ESP_LOGI(FNAME,"SetupMenu delMenu() title %s", item->getTitle() );
	std::vector<MenuEntry *>::iterator position = std::find(_childs.begin(), _childs.end(), item );
	if (position != _childs.end()) {
		ESP_LOGI(FNAME,"found entry, now erase" );
		delete *position;
		_childs.erase(position);
	}
}

// not yet used
// const MenuEntry* SetupMenu::findMenu(const char *title) const
// {
// 	ESP_LOGI(FNAME,"MenuEntry findMenu() %s %p", title, this );
// 	if( _title == title ) {
// 		ESP_LOGI(FNAME,"Menu entry found for start %s", title );
// 		return this;
// 	}
// 	for (const MenuEntry* child : static_cast<const SetupMenu*>(this)->_childs) {
// 		const MenuEntry* m = child->findMenu(title);
// 		if( m != nullptr ) {
// 			ESP_LOGI(FNAME,"Menu entry found for %s", title);
// 			return m;
// 		}
// 	}
// 	ESP_LOGW(FNAME,"Menu entry not found for %s", title);
// 	return nullptr;
// }
//
// int SetupMenu::findMenuIdx(int contId) const
// {
// 	int idx = 0;
// 	for (const MenuEntry* child : _childs) {
// 		if( !child->isLeaf() && static_cast<const SetupMenu*>(child)->getContId() == contId ) {
// 			ESP_LOGI(FNAME,"Menu entry found for %s", title);
// 			return idx;
// 		}
// 		idx++;
// 	}

// 	return -1; // not found
// }


MenuEntry *SetupMenu::getEntry(int n) const
{
	if ( n < _childs.size() ) {
		return _childs[n];
	}
	return nullptr;
}

static int modulo(int a, int b) {
	return (a % b + b) % b;
}

void SetupMenu::rot(int count)
{
    // ESP_LOGI(FNAME,"select %d: %d/%d", count, highlight, _childs.size() );
    unHighlight(highlight);
    highlight = modulo(highlight + 1 + count, _childs.size() + 1) - 1;
    doHighlight(highlight);

    // show help like a mouse over when menu get highlighted
    MenuEntry* child = (highlight >= 0) ? _childs[highlight] : nullptr;
    if (child && child->canInline() && child->hasHelp() ) {
        child->showhelp(true);
        _help_dirty = true;
    }
    else {
        if ( _help_dirty ) {
            if ( hasHelp() ) { showhelp(true); }
            else { clearHelpLines(firstHelpLine()); }
            _help_dirty = false;
        }
    }
    ESP_LOGI(FNAME, "new highlight %d dyn %.1f", highlight, getRotDynamic());
}

void SetupMenu::press()
{
	ESP_LOGI(FNAME,"press() inSet %d highl: %d", gflags.inSetup, highlight );
	if (highlight == -1) {
		_parent->highlightTop();
		exit();
	} else {
		ESP_LOGI(FNAME,"SetupMenu to child");
		if ((highlight >= 0) && (highlight < _childs.size())) {
			_childs[highlight]->enter();
		}
	}
}

void SetupMenu::longPress()
{
	if (highlight == -1) {
		exit(-1); // fast exit
	} else {
		press();
	}
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
            p->menuPrintLn("Bat. volt. adjust not done.", 10, 5);
            p->setSelect(0);
        }
        else if ( accbias == axes_i16_abi() ) {
            // not done, show warning
            p->menuPrintLn("Accel. bias not done.", 10, 5);
            p->setSelect(0);
            return 0;
        }
        if ( p->getSelect() == 0 ) {
            while (!Rotary->readSwitch(100)) ;
        }
        else
#endif
        {
            factory_flag.set(factory_flag.get() | 2);
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
	meanclimb->setHelp(
			"Options for calculation of Mean Climb (MC recommendation) displayed by green/red rhombus");
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

void screens_menu_create_extreme_records(SetupMenu *top) {
	SetupMenuValFloat *gmpos = new SetupMenuValFloat("Peak Positive G", "", nullptr, &gload_pos_max, RST_NONE, false);
	top->addEntry(gmpos);
	gmpos->lock();

	SetupMenuValFloat *gmneg = new SetupMenuValFloat("Peak Negative G", "", nullptr, &gload_neg_max, RST_NONE, false);
	top->addEntry(gmneg);
	gmneg->lock();

	SetupMenuValFloat *maxias = new SetupMenuValFloat("Peak Airspeed", "", nullptr, &airspeed_max, RST_NONE, false);
	top->addEntry(maxias);
	maxias->lock();

	SetupMenuSelect *gloadres = new SetupMenuSelect("Reset Peak-Hold", RST_NONE, gload_reset);
	gloadres->addEntry("Cancel");
	gloadres->addEntry("Reset");
	top->addEntry(gloadres);
}

static void screens_menu_create_vario(SetupMenu *top) {
    SetupMenuSelect *tgauge = new SetupMenuSelect("Upper Gauge", RST_NONE, nullptr, &vario_upper_gauge);
    tgauge->setHelp("Choose the content for this gauge");
    tgauge->addEntry("Disable", MultiGauge::GAUGE_NONE);
    tgauge->addEntry("IAS Speed", MultiGauge::GAUGE_IAS_SPEED);
    tgauge->addEntry("TAS Speed", MultiGauge::GAUGE_TAS_SPEED);
    tgauge->addEntry("GND Speed", MultiGauge::GAUGE_GND_SPEED);
    tgauge->addEntry("Speed2Fly", MultiGauge::GAUGE_S2F);
    tgauge->addEntry("McCready", MultiGauge::GAUGE_MC);
    tgauge->addEntry("Net. Vario", MultiGauge::GAUGE_NETTO);
    tgauge->addEntry("OATemp.", MultiGauge::GAUGE_OAT);
    tgauge->addEntry("Heading", MultiGauge::GAUGE_HEADING);
#ifdef DEBUG_AND_TEST
    tgauge->addEntry("Slip Angle", MultiGauge::GAUGE_SLIP);
#endif
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
    batv->addEntry("Voltage");
    batv->addEntry("Voltage Big");
    top->addEntry(batv);
}

void screens_menu_create_gload(SetupMenu *top) {
    SetupMenuSelect *glmod = new SetupMenuSelect("Screen Mode", RST_NONE, set_parent_parent_dirty, &screen_gmeter);
    glmod->setHelp("Switch off G-Force screen, switch it 'On', or choose it as 'Primary' screen");
    glmod->addEntry(ENABLE_MODE[0].data());
    glmod->addEntry(ENABLE_MODE[1].data());
    glmod->addEntry(ENABLE_MODE[3].data(), 3);
    top->addEntry(glmod);

	SetupMenuValFloat *glpos = new SetupMenuValFloat("Red positive Limit", "", nullptr, &gload_pos_limit, RST_NONE, false);
	top->addEntry(glpos);
	glpos->setPrecision(1);
	glpos->setHelp(
			"Positive g load factor limit the aircraft is able to handle below maneuvering speed, see manual");

	SetupMenuValFloat *glposl = new SetupMenuValFloat("Yellow pos. Limit", "", nullptr, &gload_pos_limit_low, RST_NONE, false);
	top->addEntry(glposl);
	glposl->setPrecision(1);
	glposl->setHelp(
			"Positive g load factor limit the aircraft is able to handle above maneuvering speed, see manual");

	SetupMenuValFloat *glneg = new SetupMenuValFloat("Red negative Limit", "", nullptr, &gload_neg_limit, RST_NONE, false);
	top->addEntry(glneg);
	glneg->setPrecision(1);
	glneg->setHelp(
			"Negative g load factor limit the aircraft is able to handle below maneuvering speed, see manual");

	SetupMenuValFloat *glnegl = new SetupMenuValFloat("Yellow neg. Limit", "", nullptr, &gload_neg_limit_low, RST_NONE, false);
	top->addEntry(glnegl);
	glnegl->setPrecision(1);
	glnegl->setHelp(
			"Negative g load factor limit the aircraft is able to handle above maneuvering speed, see manual");

	SetupMenu *extreme = new SetupMenu("Extreme Recordings", screens_menu_create_extreme_records);
	top->addEntry(extreme);
}

void screens_menu_create_horizon(SetupMenu *top) {
	SetupMenuSelect *horizon = new SetupMenuSelect("Horizon", RST_NONE, set_parent_parent_dirty, &screen_horizon);
	horizon->mkEnable();
	top->addEntry(horizon);
}

static void options_menu_create_screens(SetupMenu *top) { // dynamic!
	if ( top->getNrChilds() == 0 ) {
		top->setDynContent();

		SetupMenu *vario = new SetupMenu("Variometer", screens_menu_create_vario);
		top->addEntry(vario);

		SetupMenu *gload = new SetupMenu("G-Meter", screens_menu_create_gload);
		top->addEntry(gload);

		SetupMenu *horizon = new SetupMenu("Horizon", screens_menu_create_horizon);
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

	tmp_menu = static_cast<SetupMenu*>(top->getEntry(2)); // horizon
	if ( screen_gmeter.get() == SCREEN_PRIMARY ) {
		tmp_menu->setBuzzword(ENABLE_MODE[0].data());
		tmp_menu->lock();
	}
	else if ( ! gflags.ahrsKeyValid ) {
		tmp_menu->setBuzzword("no license");
		tmp_menu->lock();
	}
	else {
		tmp_menu->setBuzzword(ENABLE_MODE[screen_horizon.get()].data());
		tmp_menu->unlock();
	}
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
    SetupMenuSelect *ahrsid = new SetupMenuSelect("XCV unique Id", RST_NONE);
    ahrsid->addEntry(SetupCommon::getDefaultID());
    ahrsid->lock();
    top->addEntry(ahrsid);

    Version V;
    SetupMenuSelect* ver = new SetupMenuSelect("Version", RST_NONE, nullptr);
    ver->addEntry(V.version());
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
    fa->addEntry("ClearNVS");
    fa->addEntry("Enter Factory Menu");
#endif
    top->addEntry(fa);
}

void system_menu_create_battery(SetupMenu *top) {
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
#ifdef DEBUG_AND_TEST
	diso->addEntry( "NINETY");
#endif

	SetupMenuSelect *dtest = new SetupMenuSelect("Display Test", RST_NONE, do_display_test, &display_test);
	top->addEntry(dtest);
	dtest->setHelp("Start display test screens, press rotary to cancel");
	dtest->addEntry("Cancel");
	dtest->addEntry("Start Test");

	SetupMenuValFloat *dcadj = new SetupMenuValFloat("Display Clk Adj", "%", nullptr, &display_clock_adj, RST_IMMEDIATE);
	dcadj->setHelp(
			"Modify display clock by given percentage (restarts on exit)");
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

#ifdef DEBUG_AND_TEST
void system_menu_create_hardware_ahrs_parameter(SetupMenu *top) {
    SetupMenuValFloat* lever_arm = new SetupMenuValFloat("CG Lever Arm", "m", nullptr, &imu_leverarm, RST_NONE, false);
    lever_arm->setHelp(
        "Distance from XCVario back to the CG of the glider. Used to compensate accelerometer readings.");
    top->addEntry(lever_arm);

	SetupMenuValFloat *ahrsgf = new SetupMenuValFloat("Gyro Max Trust", "x", nullptr, &ahrs_gyro_factor, RST_NONE, false);
	ahrsgf->setPrecision(0);
	ahrsgf->setHelp("Maximum Gyro trust factor in artifical horizon");
	top->addEntry(ahrsgf);

	SetupMenuValFloat *ahrsgfm = new SetupMenuValFloat("Gyro Min Trust", "x", nullptr, &ahrs_min_gyro_factor, RST_NONE, false);
	ahrsgfm->setPrecision(0);
	ahrsgfm->setHelp("Minimum Gyro trust factor in artifical horizon");
	top->addEntry(ahrsgfm);

	SetupMenuValFloat *ahrsdgf = new SetupMenuValFloat("Gyro Dynamics", "", nullptr, &ahrs_dynamic_factor, RST_NONE, false);
	ahrsdgf->setHelp(
			"Gyro dynamics factor, higher value trusts gyro more when load factor is different from one");
	top->addEntry(ahrsdgf);

	SetupMenuValFloat *gyrog = new SetupMenuValFloat("Gyro Gating", "°", nullptr, &gyro_gating, RST_NONE, false);
	gyrog->setHelp("Minimum accepted gyro rate in degree per second");
	top->addEntry(gyrog);

    SetupMenuSelect *ahrsdef = new SetupMenuSelect("Reset to Defaults", RST_NONE, set_ahrs_defaults);
	top->addEntry(ahrsdef);
	ahrsdef->setHelp(
			"Set optimum default values for all AHRS Parameters as determined to the best practice");
	ahrsdef->addEntry("Cancel");
	ahrsdef->addEntry("Set Defaults");
}
#endif

void system_menu_create_hardware_imu(SetupMenu *top) {
#ifdef DEBUG_AND_TEST
    SetupMenuSelect* imu_calib_collect = new SetupMenuSelect("IMU Reference", RST_NONE, imu_calib);
    imu_calib_collect->setHelp("Calibrate IMU to glider reference. Run the procedure by selecting Start.");
    imu_calib_collect->addEntry("Cancel", 0);
    if (!airborne.get()) {
        imu_calib_collect->addEntry("Start Calib.", 1);
    }
    imu_calib_collect->addEntry("Reset", 2);
    top->addEntry(imu_calib_collect);

    SetupMenuSelect* gyro_reset = new SetupMenuSelect("Gyro Zero", RST_NONE, imu_calib);
    gyro_reset->setHelp("Reset gyro bias to zero");
    gyro_reset->addEntry("Cancel", 0);
    gyro_reset->addEntry("Gyro Reset", 4);
    top->addEntry(gyro_reset);
#endif
#ifdef DEBUG_AND_TEST
    SetupMenuValFloat* ahrs_ground_aa = new SetupMenuValFloat("Ground Angle of Attack", "°", imu_gaa, &glider_ground_aa, RST_NONE, false);
    ahrs_ground_aa->setHelp(
        "Angle of attack with tail skid on the ground to adjust the AHRS horizon level. Change this any time");
    ahrs_ground_aa->setPrecision(0);
    top->addEntry(ahrs_ground_aa);
#endif
	SetupMenuValFloat* tcontrol = new SetupMenuValFloat("Temp Control", "°C", nullptr, &mpu_temperature, RST_NONE, false);
    tcontrol->setPrecision(0);
    tcontrol->setHelp("Target temperature of AHRS sensor temp-controler, if supported in hardware (model > 2023)");
    top->addEntry(tcontrol);

	SetupMenuSelect *rpyl = new SetupMenuSelect("AHRS RPYL", RST_NONE, nullptr, &ahrs_rpyl_dataset);
	top->addEntry(rpyl);
	rpyl->setHelp("Send LEVIL AHRS like $RPYL sentence for artifical horizon");
	rpyl->mkEnable();

	SetupMenuSelect * araw = new SetupMenuSelect( "AHRS RAW", RST_NONE , nullptr, &ahrs_raw_data );
	top->addEntry( araw );
	araw->setHelp( "Send RAW AHRS gyro and accelerator data in XCV,G..,A.. format");
	araw->mkEnable();

	// SetupMenuChar *ahrslc = new SetupMenuChar("License Key", "0A#", 4, RST_NONE, add_key, ahrs_licence.get().id);
	// ahrslc->setHelp("Enter valid AHRS License Key to enabled the 'AHRS Option'");
	// top->addEntry(ahrslc);

#ifdef DEBUG_AND_TEST
	SetupMenu *ahrspa = new SetupMenu("Parameters", system_menu_create_hardware_ahrs_parameter);
	ahrspa->setHelp("AHRS constants such as gyro trust and filtering");
	top->addEntry(ahrspa);
#endif
}

void system_menu_create_hardware(SetupMenu *top) {
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
    if (FLAP) {
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

#ifdef DEBUG_AND_TEST
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
#endif
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

    SetupMenuValFloat *qnh_menu = SetupMenu::createQNHMenu();
	top->addEntry(qnh_menu);

	SetupMenuValFloat *afe = new SetupMenuValFloat("Airfield Elevation", "", nullptr, &airfield_elevation);
	afe->setHelp(
			"Airfield elevation in meters for QNH auto adjust on ground according to this elevation");
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

	SetupMenu* SetupMenu::createTopSetup() {
	const char *top_menu_name = "Setup";
	if (glider_type.get() != 1000 || ! S2F::isPolarEqualTo(0)) {
		top_menu_name =	Polars::getPolarName(Speed2Fly.getMyGliderIdx());
	}
	SetupMenu *setup = new  SetupMenu(top_menu_name, setup_create_root);
	return setup;
}


SetupMenu* SetupMenu::createFactorySetup() {
    SetupMenu *setup = new SetupMenu("Factory Setup", nullptr);

    SetupMenu *soft = new SetupMenu("Software", system_menu_create_software);
    setup->addEntry(soft);

    SetupMenuValFloat *met_adj = SetupMenu::createVoltmeterAdjustMenu();
    setup->addEntry(met_adj);

    SetupMenuSelect* bias_zero = new SetupMenuSelect("IMU Biases", RST_NONE, imu_calib);
    bias_zero->addEntry("Cancel");
    bias_zero->addEntry("Acc. Bias Calib.", 3);
    bias_zero->addEntry("Gyro Reset", 4);
    bias_zero->addEntry("Acc. Reset", 5);
    setup->addEntry(bias_zero);

    SetupMenuDisplay *leak = new LeakTest("Leak Test");
    setup->addEntry(leak);

    SetupMenuSelect* fa = new SetupMenuSelect("Exit Factory Menu", RST_NONE, exitFactoryMenu);
    fa->addEntry("Cancel");
    fa->addEntry("Exit");
    setup->addEntry(fa);
    return setup;
}

SetupMenuValFloat* SetupMenu::createQNHMenu() {
	SetupMenuValFloat *qnh = new SetupMenuValFloat("QNH", "", qnh_adj, &QNH, RST_NONE, false);
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

