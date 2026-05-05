/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "setup/SetupMenu.h"
#include "setup/SetupMenuSelect.h"
#include "setup/SetupMenuValFloat.h"
#include "setup/SetupNG.h"
#include "sensor/imu/AccMPU6050.h"
#include "sensor/imu/GyroMPU6050.h"
#include "driver/audio/ESPAudio.h"
#include "Colors.h"
#include "AdaptUGC.h"
#include "sensor.h"
#include "logdef.h"

#include <cstdint>

extern AdaptUGC *MYUCG;

std::string imu_menu_help;

static int gload_reset(SetupMenuSelect *p) {
	if ( p->getSelect() == 1 ) {
		gload_pos_max.set(1);
		gload_neg_max.set(0);
		airspeed_max.set(0);
		p->setSelect(0);
		set_parent_dirty(p);
	}
	return 0;
}

static int set_ahrs_defaults(SetupMenuSelect *p) {
	if (p->getSelect() == 1) {
		ahrs_gyro_factor.set(ahrs_gyro_factor.getDefault());
		ahrs_min_gyro_factor.set(ahrs_min_gyro_factor.getDefault());
		ahrs_dynamic_factor.set(ahrs_dynamic_factor.getDefault());
		gyro_gating.set(gyro_gating.getDefault());
	}
	p->setSelect(0);
	return 0;
}

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

        AUDIO->startSound(AUDIO_TADDA_SHORT | PRIO_SND_MASK, false, 100);

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
            if (i<2 || ret == 4) AUDIO->startSound(AUDIO_TADDA_SHORT | PRIO_SND_MASK, false, 100);
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


static void factoryAccCalibration(SetupMenuSelect* p, bool check_only=false) {
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
    if ( ! check_only ) accSensor->resetBias();

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
            if (pos < 5) { AUDIO->startSound(AUDIO_TADDA_SHORT | PRIO_SND_MASK, false, 100); }
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
        float res0, res;
        vector_f bias = accSensor->getMpu().extractAccBias(samples, 6, &res0, &res);
        if ( check_only ) {
            ESP_LOGI(FNAME, "Acc bias check: %f/%f/%f", bias.x, bias.y, bias.z);
        }
        else {
            accSensor->pushBias(bias);
            // set menu help to show the quality of the result
            imu_menu_help = "Before: \n" + std::to_string(res0);
            axes_i16_abi tmp = accl_bias.get();
            imu_menu_help += " Bias: (" + std::to_string(tmp.x) + "," + std::to_string(tmp.y) + "," + std::to_string(tmp.z) + ")\n";
            imu_menu_help += " After: " + std::to_string(res);
            SetupMenu *menu = p->getParent();
            menu->setHelp(imu_menu_help.c_str());
        }
        AUDIO->startSound(AUDIO_TADDA_LONG | PRIO_SND_MASK, false, 100);
        p->clear();
        p->menuPrintLn("Success !", 2, 20);
    }

    // restore IMU setup
    accSensor->getMpu().setLeverArm(imu_leverarm.get());

    p->menuPrintLn("press button to return", 8, 1);
    while (!Rotary->readSwitch(100)) ;
}

int imu_calib(SetupMenuSelect* p) {
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
        case 6:
            factoryAccCalibration(p, true);
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

void system_menu_create_hardware_imu(SetupMenu *top) {
    if (gflags.expert) {
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

        SetupMenuValFloat* ahrs_ground_aa = new SetupMenuValFloat("Ground Angle of Attack", "°", imu_gaa, &glider_ground_aa, RST_NONE, false);
        ahrs_ground_aa->setHelp(
            "Angle of attack with tail skid on the ground to adjust the AHRS horizon level. Change this any time");
        ahrs_ground_aa->setPrecision(0);
        top->addEntry(ahrs_ground_aa);
    }
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


void free_imu_menu()
{
    imu_menu_help.clear();
    imu_menu_help.shrink_to_fit();
}
