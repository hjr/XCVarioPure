/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "FlarmScreen.h"

#include "MessageBox.h"
#include "Colors.h"
#include "UiEvents.h"
#include "IpsDisplay.h"
#include "DrawDisplay.h"
#include "driver/audio/ESPAudio.h"
#include "sensor/imu/AccMPU6050.h"
#include "sensor/gps/GpsVSensor.h"
#include "Flarm.h"
#include "vector.h"
#include "math/Trigonometry.h"
#include "math/Quaternion.h"
#include "Atmosphere.h"
#include "AdaptUGC.h"
#include "logdefnone.h"

#include <algorithm>

extern AdaptUGC *MYUCG;
FlarmScreen *FLARMSCREEN = nullptr;


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
    _time_out.start( (int)(flarm_alarm_time.get()) * 1000 );
}

void FlarmScreen::exit(int ups)
{
    ESP_LOGI(FNAME,"FlarmScreen exit");

    FLARMSCREEN = nullptr;
    MenuEntry::exit();
    delete this;
}

void FlarmScreen::display(int mode)
{
    if ( mode == 0 ) {
        AUDIO->startSound(AUDIO_ALARM_FLARM | PRIO_SND_MASK); // start a priority alarm
        Display->clear();
    }
    _time_out.pet();
    _tick++;

    ESP_LOGI(FNAME, "flarm_screen mode %d", mode);
    // calc horizon line
    Quaternion attq = accSensor->getAHRSQuaternion();
    Line l( attq, true);
    Point above[6], below[6];
    int na, nb;
    IpsDisplay::clipPolygonByLine(nullptr, 0, l, above, &na, below, &nb);

    // ESP_LOGI(FNAME,"Target in B%.1f°, dH%dm, dV%dm", Units::rad_to_deg(Flarm::RelativeBearing), Flarm::HorizontalDistance, Flarm::RelativeVertical );
    // calc the vector to the target from distance and bearing in ground nav frame
    // add the wind correction angle
    rad_t rel_target_bearing = Vector::normalizePI(Flarm::RelativeBearing + heading_wca.get());
    vector_f bearingVec;
    bearingVec.x = Flarm::HorizontalDistance * fast_cos_rad(rel_target_bearing);
    bearingVec.y = Flarm::HorizontalDistance * fast_sin_rad(rel_target_bearing);
    float hrel = Flarm::RelativeVertical / 20.f; // magnify vertical position for small alt diff
    bearingVec.z = - hrel / (4.f + std::abs(hrel)) * 250.f; // "-" for NED frame, 
    ESP_LOGI(FNAME,"BearingVec %1.1f,%1.1f,%1.1f", bearingVec.x, bearingVec.y, bearingVec.z );

    // determine side and altDiff for audio alarm code
    constexpr const rad_t MID_FUNNEL_RAD = Units::deg_to_rad(30.f);
    meter_t dist = (Flarm::HorizontalDistance>0) ? Flarm::HorizontalDistance : 0.1f;
    int alt_bear = (std::abs(fast_atan((float)Flarm::RelativeVertical/dist)) < MID_FUNNEL_RAD) ? 1 : (Flarm::RelativeVertical > 0 ? 2 : 0);
    int side_bear = (std::abs(std::abs(rel_target_bearing)) < MID_FUNNEL_RAD) ? 1 : 2;
    if ( rel_target_bearing < 0 ) { side_bear = 0; }
        
    // project in NAV frame
    Point p = Point::centralProjection(bearingVec, 100.f); // start clipping from ca. 50° on
    ESP_LOGI(FNAME,"CentralProj %d,%d", p.x, p.y);
    // map the target point according to the current horizon line on the display
    p = l.mapToHorizon(p);
    ESP_LOGI(FNAME,"DisplPt %d,%d vertHorizon %.1f", p.x, p.y, l.getDistance(p) );
    if ( p.x < -9000 || p.y < -9000 ) {
        // cannot project point
        ESP_LOGI(FNAME,"Cannot project point");
        return;
    }

    // put the word TRAFFIC under the screen
    if ( mode == 0 ) {
        _msgid = MBOX->pushMessage(4, "! TRAFFIC !", flarm_alarm_time.get());
    }
    else {
        MBOX->keepMessage(_msgid);
    }

    // draw horizon
    MYUCG->setColor( COLOR_SKYBLUE );
    IpsDisplay::drawPolygon(above, na);
    MYUCG->setColor( COLOR_EARTH );
    IpsDisplay::drawPolygon(below, nb);

    // limit target to display area
    p = l.limitToScreen(p, true);
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

    // draw target as a disc, depict the non linear compressed distance through the size
    int16_t size = 300 - (300 * Flarm::HorizontalDistance/ (Flarm::HorizontalDistance + 30));
    if ( size < 20 ) {
        size = 20;
    }
    MYUCG->drawDisc( p.x, p.y, size, UCG_DRAW_ALL);
    // Embedd a little glider symbol
    float roll = -accSensor->getRoll();
    if ( bearingVec.x > 0.f ) {
        MYUCG->setColor(COLOR_WHITE);
        // fuselage as disc
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
    // write the alt difference to the target, left, or right of it
    MYUCG->setFont(ucg_font_fub20_hr);
    MYUCG->setFontPosCenter();
    MYUCG->setColor(COLOR_WHITE);
    char buf[20];
    snprintf(buf, sizeof(buf), "%+d%s", (int)AltUnit->apply(Flarm::RelativeVertical), AltUnit->getName());
    p.y = std::clamp(p.y, (int16_t)20, (int16_t)(DISPLAY_H - 20));
    int16_t swidth = MYUCG->getStrWidth(buf);
    if (l.fct(Point(p.x + swidth/2, p.y)) > 0.f) {
        MYUCG->setColor(1, COLOR_SKYBLUE); // bg color sky
    } else {
        MYUCG->setColor(1, COLOR_EARTH);   // bg color earth
    }
    if ( p.x < DISPLAY_W / 2 ) {
        MYUCG->setPrintPos(p.x + size + 5, p.y);
    }
    else {
        MYUCG->setPrintPos(p.x - size - swidth - 5, p.y);
    }
    MYUCG->print(buf);
    MYUCG->setFontPosBottom();
    MYUCG->setColor(1, COLOR_BLACK); // bg color

    // start encoded audio alarm
    uint16_t alarm = Audio::encFlarmParam(AUDIO_ALARM_FCODE, Flarm::AlarmLevel, side_bear, alt_bear);
    int pause = 6;
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
    MBOX->popMessage(_msgid); // remove the message box if still there
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