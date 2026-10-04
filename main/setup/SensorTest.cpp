/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "SensorTest.h"

#include "driver/time/Clock.h"
#include "screen/UiEvents.h"
#include "screen/DrawDisplay.h"
#include "sensor/pressure/PressureSensor.h"
#include "sensor/press_diff/AirspeedSensor.h"
#include "sensor/temp/TempVSensor.h"
#include "sensor/imu/AccMPU6050.h"
#include "sensor/imu/GyroMPU6050.h"
#include "sensor/adc/BatteryVoltage.h"
#include "driver/gpio/S2fSwitch.h"
#include "Colors.h"
#include "IpsDisplay.h"
#include "Atmosphere.h"
#include "AdaptUGC.h"
#include "logdefnone.h"


extern AdaptUGC *MYUCG;

constexpr int16_t LINEHIGHT = 25;

SensorTest* ST = nullptr;

SensorTest::SensorTest() :
    MenuEntry("Sensor Test"),
    Clock_I(20) // 5Hz
{
    bits._never_inline = true;
}
SensorTest::~SensorTest()
{
    ST = nullptr;
}

SensorTest* SensorTest::create() {
    if ( !ST ) {
        ST = new SensorTest();
    }
    return ST;
}

void SensorTest::update() {
    if ( ST ) {
        ST->draw();
    }
}

void SensorTest::display(int mode)
{
    // if ( mode == 0 ) {
    //     Display->clear();
    // }

    ESP_LOGI(FNAME, "sensor test mode %d", mode);
    menuPrintLn("  <", 0);
    menuPrintLn(_title.c_str(), 0, 30);

    // draw item list and start timer
    MYUCG->setColor(COLOR_LBBLUE);
    MYUCG->setFont(ucg_font_fub14_hr, true);
    
    int16_t line = 1;
    menuPrintLn("STp", line++, 1);
    menuPrintLn("TEp", line++, 1);
    menuPrintLn("DYp", line++, 1);
    menuPrintLn("OAt", line++, 1);
    menuPrintLn("Bat", line++, 1);
    menuPrintLn("IMU  acc./*g, gyro/dsp", line++, 1);
    line += 2;
    menuPrintLn("S2f", line++, 1);
    line++;
    menuPrintLn("press button to return", line, 10);

    MYUCG->setColor( COLOR_WHITE );
    Clock::start(this);
}

// called in DrawDisplay context
void SensorTest::draw()
{

    ESP_LOGI(FNAME, "sensor test draw");
    MYUCG->setColor( COLOR_WHITE );
    MYUCG->setFont(ucg_font_fub14_hr, true);

    int16_t line = 1;
    char buf[120];
    if ( baroSensor ) {
        sprintf(buf, "%.1f %s ", AltUnit->apply(baroSensor->getHeadValid() ? altitude_isa.get() : 0.f), AltUnit->getName());
        menuPrintLn(buf, line, 42);
        if (baroSensor && SetupCommon::isMaster()) {
            sprintf(buf, "%7.1f Pa  ", baroSensor->getHead());
            MYUCG->print(buf);
        }
    }
    line++;
    if ( teSensor ) {
        sprintf(buf, "%.1f m ", Units::calcAltitudeISA(teSensor->getHeadValid() ? teSensor->get() : 0.f));
        menuPrintLn(buf, line, 42);
        if (SetupCommon::isMaster()) {
            sprintf(buf, "%7.1f Pa  ", teSensor->getHead());
            MYUCG->print(buf);
        }
    }
    line++;
    if ( asSensor ) {
        sprintf(buf, "%.1f %s ", SpeedUnit->apply(asSensor->getHeadValid() ? ias.get() : 0.f), SpeedUnit->getName());
        menuPrintLn(buf, line, 42);
        if (SetupCommon::isMaster()) {
            sprintf(buf, "%+7.2f Pa  ", asSensor->getHead());
            MYUCG->print(buf);
        }
    }
    line++;
    sprintf(buf, "%4.1f %s ", (oatSensor && oatSensor->getHeadValid()) || !oatSensor ? TempUnit->apply(OAT.get()) : 0.f, TempUnit->getName());
    menuPrintLn(buf, line, 42);
    if ( oatSensor ) {
        sprintf(buf, "% 5.2f K  ", oatSensor->getHead());
        MYUCG->print(buf);
    }
    line++;
    sprintf(buf, "%.1f V ", battery_voltage.get());
    menuPrintLn(buf, line, 42);
    if ( batSensor ) {
        sprintf(buf, "%5.2f V ", batSensor->getHead());
        MYUCG->print(buf);
    }
    line += 2;
    if ( accSensor ) {
        vector_f acc;
        if ( accSensor->getHeadValid() ) {
            acc = accSensor->get();
        }
        sprintf(buf, "(% 7.4f % 7.4f % 7.4f)  ", acc.x, acc.y, acc.z);
        menuPrintLn(buf, line, 2);
    }
    line++;
    if ( gyroSensor ) {
        vector_f gyr;
        if ( gyroSensor->getHeadValid() ) {
            gyr = gyroSensor->get();
        }
        gyr *= Units::rad_to_deg(1.f);
        sprintf(buf, "(% 7.3f % 7.3f % 7.3f)  ", gyr.x, gyr.y, gyr.z);
        menuPrintLn(buf, line, 2);
    }
    line++;
    if ( S2FSWITCH ) { // Master and Client option for this switch
        sprintf(buf, "%s ", S2FSWITCH->getRawState() ? "closed" : "  open");
        menuPrintLn(buf, line, 42);
    }
}

void SensorTest::press() {
    Clock::stop(this);
    ESP_LOGI(FNAME, "sensor test press");
    exit();
    delete this;
}


bool SensorTest::tick()
{
    int evt = ScreenEvent(ScreenEvent::SENSOR_TEST).raw;
    xQueueSend(uiEventQueue, &evt, 0);
    return false;
}