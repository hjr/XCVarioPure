/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "FlarmScreen.h"

#include "Colors.h"
#include "UiEvents.h"
#include "IpsDisplay.h"
#include "DrawDisplay.h"
#include "ESPAudio.h"
#include "sensor/imu/KalmanMPU6050.h"
#include "Flarm.h"
#include "math/Trigonometry.h"
#include "math/Quaternion.h"
#include "AdaptUGC.h"
#include "logdefnone.h"


extern AdaptUGC *MYUCG;
FlarmScreen *FLARMSCREEN = nullptr;
constexpr int FLARM_ALARM_HOLDING_TIME = 7000; // msec


FlarmScreen *FlarmScreen::create()
{
    if ( ! FLARMSCREEN ) {
        FLARMSCREEN = new FlarmScreen();
    }
    return FLARMSCREEN;
}

FlarmScreen::FlarmScreen() :
    MenuEntry(""),
    _time_out( this )
{
    ESP_LOGI(FNAME,"FlarmScreen created");
    _time_out.start( FLARM_ALARM_HOLDING_TIME );
}

void FlarmScreen::exit(int ups)
{
    ESP_LOGI(FNAME,"FlarmScreen exit");
    FLARMSCREEN = nullptr;
    MenuEntry::exit();
    if ( ! isRoot() ) {
        delete this;
    }
}

void FlarmScreen::display(int mode)
{
    if ( mode == 0 ) {
        AUDIO->startSound(AUDIO_ALARM_FLARM);
        Display->clear();
    }
    _time_out.pet();
    _tick++;

    ESP_LOGI(FNAME, "flarm_screen mode %d", mode);
    // calc horizon line
    Quaternion attq = IMU::getAHRSQuaternion();
    Line l( attq, dwidth/2, dheight/2 );
    Point above[6], below[6];
    int na, nb;
    IpsDisplay::clipRectByLine(nullptr, l, above, &na, below, &nb);

    ESP_LOGI(FNAME,"Target in B%d°, dH%dm, dV%dm", Flarm::RelativeBearing, Flarm::RelativeDistance, Flarm::RelativeVertical );
    // calc from distance and bearing the vector to the target
    Quaternion qtmp(deg2rad(static_cast<float>(-Flarm::RelativeBearing)), vector_f(0.f, 0.f, 1.f));
    vector_f bearingVec = qtmp * vector_f(Flarm::RelativeDistance, 0.f, Flarm::RelativeVertical);
    ESP_LOGI(FNAME,"BearingVec %1.1f,%1.1f,%1.1f", bearingVec.x, bearingVec.y, bearingVec.z );

    // determine side and altDiff for audio alarm
    constexpr const int   MID_FUNNEL_DEG = 30;
    constexpr const float MID_FUNNEL_RAD = 0.577350269; // eval manually .. fast_tan_deg(static_cast<float>(MID_FUNNEL_DEG));
    float dist = (Flarm::RelativeDistance>0) ? Flarm::RelativeDistance : 0.1f;
    int alt_bear = (std::abs((float)Flarm::RelativeVertical/dist) < MID_FUNNEL_RAD) ? 1 : (Flarm::RelativeVertical > 0 ? 2 : 0);
    int side_bear = (std::abs(std::abs(Flarm::RelativeBearing) - 90) < (90 - MID_FUNNEL_DEG)) ? 2 : 1;
    if ( Flarm::RelativeBearing < 0 ) { side_bear = 0; }
        
    // rotate according to own attitude
    vector_f buddyVec = attq * bearingVec;
    ESP_LOGI(FNAME,"BuddyVec %1.1f,%1.1f,%1.1f", buddyVec.x, buddyVec.y, buddyVec.z );
    Point p = IpsDisplay::projectToDisplayPlane(buddyVec, 100.f);
    ESP_LOGI(FNAME,"DisplPt %d,%d", p.x, p.y);
    if ( p.x < -9000 || p.y < -9000 ) {
        // cannot project point
        ESP_LOGI(FNAME,"Cannot project point");
        return;
    }

    // draw horizon
    MYUCG->setColor( COLOR_SKYBLUE );
    IpsDisplay::drawPolygon(above, na);
    MYUCG->setColor( COLOR_EARTH );
    IpsDisplay::drawPolygon(below, nb);

    // clip to display area
    p = IpsDisplay::clipToScreenCenter(p);
    ESP_LOGI(FNAME,"ClippPt %d,%d", p.x, p.y);

    // Put alarm level into color
    if ( Flarm::AlarmLevel <= 1 ) {
        MYUCG->setColor(COLOR_LGREY);
    }
    else if ( Flarm::AlarmLevel == 2 ) {
        MYUCG->setColor(COLOR_YELLOW);
    }
    else if ( Flarm::AlarmLevel >= 3 ) {
        MYUCG->setColor(COLOR_RED);
    }

    // draw target as a  disc, depict the distance through the size
    int16_t size = (300 - Flarm::RelativeDistance) / 5;
    if ( size < 20 ) {
        size = 20;
    }
    MYUCG->drawDisc( p.x, p.y, size, UCG_DRAW_ALL);
    // Embedd a little glider symbol
    float roll = -IMU::getRollRad();
    if ( buddyVec.x > 0.f ) {
        MYUCG->setColor(COLOR_WHITE);
        MYUCG->drawDisc( p.x, p.y, size/3, UCG_DRAW_ALL);
        // draw little wings as tetragon rolled according to horizontal angle
        Point w1 = Point( size, 0 ).rotate(roll);
        Point th = Point(0, size/6).rotate(roll);
        MYUCG->drawTetragon( p.x + w1.x + th.x, p.y + w1.y + th.y,
                             p.x - w1.x + th.x, p.y - w1.y + th.y,
                             p.x - w1.x - th.x, p.y - w1.y - th.y,
                             p.x + w1.x - th.x, p.y + w1.y - th.y );
        p += Point(0, -size * 2 / 3).rotate(roll);
        w1 = Point(size / 3, 0).rotate(roll);
        MYUCG->drawTetragon( p.x + w1.x + th.x, p.y + w1.y + th.y,
                             p.x - w1.x + th.x, p.y - w1.y + th.y,
                             p.x - w1.x, p.y - w1.y,
                             p.x + w1.x, p.y + w1.y);

    }
    else {
        MYUCG->setColor(COLOR_BLACK);
        // target is behind -> draw a black X and a circle around the disc
        MYUCG->drawCircle( p.x, p.y, size, UCG_DRAW_ALL);
        // draw a diagonal X line
        Point x1 = Point(size, size).rotate(roll);
        MYUCG->drawLine( p.x - x1.x, p.y - x1.y, p.x + x1.x, p.y + x1.y );
        MYUCG->drawLine( p.x - x1.y, p.y + x1.x, p.x + x1.y, p.y - x1.x );
    }

    // start encoded audio alarm
    uint16_t alarm = Audio::encFlarmParam(AUDIO_ALARM_FCODE, Flarm::AlarmLevel, side_bear, alt_bear);
    int pause = 4;
    if ( Flarm::AlarmLevel > 1 ) { pause = 4 - Flarm::AlarmLevel; };
    if (mode > 0 && Flarm::AlarmLevel > 0 && (alarm > _prev_alarm || (_tick-_alarmtick) > pause)) {
        _prev_alarm = alarm;
        _alarmtick = _tick;
        ESP_LOGI(FNAME,"Start sound alarm %u", alarm);
        AUDIO->startSound(alarm);
    }
}

void FlarmScreen::press() {
    ESP_LOGI(FNAME,"FlarmScreen press - exit");
    int expected = 0;
    if ( atomic_compare_exchange_strong(&_done, &expected, 1) ) {
        ESP_LOGI(FNAME,"FlarmScreen press - done");
        // set the confirmed time to mute this alarm for 30 seconds
        Flarm::setConfirmed();
        exit();
    }
}

void FlarmScreen::remove()
{
    ESP_LOGI(FNAME,"FlarmScreen remove - called");
    // protect double/competing exit calls from watch dog and e.g. UI interaction
    int expected = 0;
    if ( atomic_compare_exchange_strong(&_done, &expected, 1) ) {
        ESP_LOGI(FNAME,"FlarmScreen remove - done");
        exit();
    }
}

void FlarmScreen::barked()
{
    ESP_LOGI(FNAME,"FlarmScreen barked - timeout");
    // timeout
    int exitWarn = ScreenEvent(ScreenEvent::FLARM_ALARM_TIMEOUT).raw;
    // Route this event to the DrawDisplay context
    xQueueSend(uiEventQueue, &exitWarn, 0);
}