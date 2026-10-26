/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "ThermalAssist.h"

#include "sensor/VarioFilter.h"
#include "math/Units.h"
#include "screen/element/PolarGauge.h"
#include "driver/time/Clock.h"
#include "math/Trigonometry.h"
#include "Flarm.h"
#include "setup/SetupNG.h"
#include "AdaptUGC.h"
#include "Colors.h"
#include "logdefnone.h"

#include <algorithm>


constexpr rad_t CA_STEP = Units::deg_to_rad(360/CA_NUM_DIRS); // 15
constexpr rad_t CA_STEP_2 = CA_STEP/2.f;  // 7.5
constexpr int16_t MAX_DISK_RAD = 7;

extern AdaptUGC *MYUCG;
ThermalAssist  *thrmAssist = nullptr;

enum class circdir_t : uint8_t {
    no_circle,
    circlLeft,
    circlRight
};

// the Thermal helper struct
void Thermal::set(mps_t s) {
    strength = s;
    timestamp = Clock::getMillis();
}

// strength -oo .. 0 .. oo
float Thermal::getStrength() const {
    // scale to peak value and limit to max disk radius
    // as well as fade with time passing
    float agescale = 1.f - std::max(0.f, (Clock::getMillis() - timestamp) - 30000.f) / 30000.f;
    if ( agescale < 0.f ) {
        return 0.f;
    }
    return strength * agescale;
}

// Thermal assistant
ThermalAssist *ThermalAssist::create(PolarGauge &g)
{
    if ( ! thrmAssist ) {
        // create the singleton
        thrmAssist = new ThermalAssist(g);
    }
    return thrmAssist;
}
void ThermalAssist::remove()
{
    if ( thrmAssist ) {
        // remove the singleton
        ThermalAssist *tmp = thrmAssist;
        thrmAssist = nullptr;
        delete tmp;
    }
}

ThermalAssist::ThermalAssist(PolarGauge &g) :
    _gauge(g),
    _glider_on_top(true),
    _confidence(LowPassFilterT<float>::alphaFromTau(2.0, 0.1f)),
    _th_norm(LowPassFilterT<float>::alphaFromTau(vario_av_delay.get() * 2, .5f))
{
    _glider_on_top = thermal_assist.get() != 2;
    _th_norm.reset(std::max(1.f, MC.get()));
}

// th_strength normalized to 0 .. 1
void ThermalAssist::drawThermal(float th_strength, int idir) {
    static const uint8_t hem_flag[4] = {1, 8, 2, 4};
    // ESP_LOGI(FNAME,"drawThermal, th_strength: %.1f, idir: %d", th_strength, idir);
    if (idir >= CA_NUM_DIRS || idir < 0) {
        ESP_LOGE(FNAME, "index out of range: %d", idir);
        return;
    }
    int ddir = idir;
    if (!_glider_on_top) {
        if (_cdir == (uint8_t)circdir_t::circlRight) {
            ddir = (idir + 3 * CA_NUM_DIRS / 4) % CA_NUM_DIRS;  // move reference to the left
        } else if (_cdir == (uint8_t)circdir_t::circlLeft) {
            ddir = (idir + CA_NUM_DIRS / 4) % CA_NUM_DIRS;  // move reference to the right
        }
    }
    Point center = {(int16_t)(fast_sin_rad(ddir * CA_STEP) * _gauge._radius), (int16_t)(fast_cos_rad(ddir * CA_STEP) * _gauge._radius)};
    uint8_t hem = (center.y < 0) ? 2 : 0;
    hem |= (center.x < 0);
    center += _gauge._ref;

    // a green thermal spot that brightens with increasing thermal strength (up to 10%)
    ucg_color_t col = { COLOR_GREEN };
    col.fadeTo(col, th_strength * 1.2f);
    // ESP_LOGI(FNAME,"drawThermal, th: %.1f, ths: %.3f, color: %d,%d,%d", th.strength, th_strength, col.color[0], col.color[1], col.color[2]);
    MYUCG->setColor(col.color[0], col.color[1], col.color[2]);
    if (idir != 0) {
        MYUCG->startBuffering(center.x-MAX_DISK_RAD, center.y-MAX_DISK_RAD, 2*MAX_DISK_RAD+1, 2*MAX_DISK_RAD);
        // MYUCG->drawFrame(center.x-MAX_DISK_RAD, center.y-MAX_DISK_RAD, 2*MAX_DISK_RAD, 2*MAX_DISK_RAD-1);
        int16_t radius = std::clamp((int16_t)fast_iroundf_positive(th_strength * MAX_DISK_RAD), (int16_t)1, MAX_DISK_RAD);
        MYUCG->drawDisc(center.x, center.y, radius, UCG_DRAW_ALL);
        if ( _last_radius > 5 ) {
            // fill potential bbox overlap with the previous disc
            MYUCG->drawDisc(_last_center.x, _last_center.y, _last_radius, hem_flag[hem]);
        }
        _last_radius = radius;
        MYUCG->finishBuffering();
    }
    else {
        // draw glider icon here
        drawGlider(center);
        _last_radius = 0;
    }
    _last_center = center;
}

//  x Cxy + (A,-W)
//  .  .
//  .     .
//  .         .
//  .  Cxy      x Cxy + (B,0)
//  .         .
//  .     .
//  .  .
//  x Cxy + (A,W)
//
void ThermalAssist::drawGlider(Point center) {
    constexpr int16_t A = -3;
    constexpr int16_t B = 3;
    constexpr int16_t W = 8;
    static const int16_t triangle[3][6] = { { A, -W, A, W, B, 0}, {-A, -W, -B, 0, -A, W}, {-W, B, W, B, 0, A} }; // left, right, tail

    // select triangle for circling direction
    MYUCG->setColor(COLOR_WHITE);
    int16_t *trptr = (int16_t*)triangle[2];
    if (_glider_on_top) {
        MYUCG->startBuffering(center.x-B, center.y-W, 2*B+1, 2*W+1);
        // MYUCG->drawFrame(cx-B, cy-W, 2*B, 2*W);
        if ( _cdir == (uint8_t)circdir_t::circlRight )
            trptr = (int16_t*)triangle[0];
        else {
            trptr = (int16_t*)triangle[1];
        }
    } else {
        MYUCG->startBuffering(center.x-W, center.y-B, 2*W+1, 2*B+1);
        // MYUCG->drawFrame(cx-W, cy-B, 2*W, 2*B);
    }
    if ( _confidence.get() > 0.7f ) {
        MYUCG->setColor(COLOR_WHITE);
        MYUCG->drawTriangle(center.x + trptr[0], center.y + trptr[1], center.x + trptr[2], center.y + trptr[3], center.x + trptr[4], center.y + trptr[5]);
    }
    MYUCG->finishBuffering();
}

// int ThermalAssist::maxClimbIndex() {
//     int max = 0;
//     int max_index = -1;
//     for (int i = 0; i < CA_NUM_DIRS; i++) {
//         if (max < thermals[i].strength) {
//             max = thermals[i].strength;
//             max_index = i;
//         }
//     }
//     return max_index;
// }

Point ThermalAssist::getThermalCG() const {
    float sx = 0.0f, sy = 0.0f;
    float sum = 0.f;

    for (int i = 0; i < 24; i++) {
        float w = thermals[i].strength;
        sum += w;
        sx += w * fast_cos_idx(i * 360/CA_NUM_DIRS*2);
        sy += w * fast_sin_idx(i * 360/CA_NUM_DIRS*2);
    }

    float magnitude = std::sqrt(sx*sx + sy*sy);
    float eccentricity = magnitude / sum;  // 0.0 .. ~1.0
    float roundness = 1.0f - eccentricity;


    // float angle = std::atan2(sy, sx);
    return Point(sx, sy);
}


float ThermalAssist::getTop8Norm() const {
    constexpr int TOP_N = 5;
    float top[TOP_N] = {};
    int n = 0;

    for (const Thermal &x : thermals) {
        if (n < TOP_N) {
            top[n++] = x.strength;
            continue;
        }

        // smalles of the top TOP_N
        int minIdx = 0;
        for (int i = 1; i < TOP_N; ++i) {
            if (top[i] < top[minIdx]) {
                minIdx = i;
            }
        }

        if (x.strength > top[minIdx]) {
            top[minIdx] = x.strength;
        }
    }
    float sum = 0;
    for (int i = 0; i < TOP_N; ++i) {
        sum += top[i];
    }

    return sum / TOP_N;
}

void ThermalAssist::draw() {
    // calculate a thermals norm
    float th_top = std::min(getTop8Norm(), MC.get() + 2.f); // max. norm according to MC
    float th_min = std::max(th_top - 3.0f, .0f);
    _th_norm.filter(std::max(th_top - th_min, 1.f));

    // ESP_LOGI(FNAME,"TA draw, peak norm: %.2f, %.2f, %.2f", th_min, th_top, _th_norm.get());
    for (int i = 0; i < CA_NUM_DIRS; i++) {
        int d = (i + _idir) % CA_NUM_DIRS;
        float ths = std::min((thermals[d].getStrength() - th_min) / _th_norm.get(), 1.f); // normalized strength 0..1

        // ESP_LOGI(FNAME,"dir:%d TE:%.1f", d, thermals[d].strength );
        drawThermal(ths, i);
    }
}

// void ThermalAssist::redrawAt(int deg) {
//     int i = (deg * CA_NUM_DIRS / 360) % CA_NUM_DIRS;
//     int d = (i+idir) % CA_NUM_DIRS;
//     drawThermal(thermals[d], i);
//     i = (i + CA_NUM_DIRS / 2) % CA_NUM_DIRS; // opposite direction
//     d = (i+idir) % CA_NUM_DIRS;
//     drawThermal(thermals[d], i);
// }


// 10 Hz called from AHRS / heading sensor
void ThermalAssist::checkHeading(rad_t vheading, rad_t omega, rad_t bank) {

    // Combined fidelity on "this is thermaling"
    // 1. turn rate confidence
    float c_turn = std::clamp((fabsf(omega) - Units::deg_to_rad(2.0f)) / Units::deg_to_rad(6.0f), 0.0f, 1.0f);
    // 2. bank angle confidence
    float c_bank = std::clamp((fabsf(bank) - Units::deg_to_rad(8.0f)) / Units::deg_to_rad(16.0f), 0.0f, 1.0f);
    // 3. steadiness/duration confidence
    _confidence.filter( (c_turn + c_bank) / 2.f );

    // integrate footing/heading, create a new thermal when the heading has changed by 15° or more
    rad_t diff = Vector::angleDiff( vheading, cur_heading );
    if ( fabsf(diff) > CA_STEP ) {
        ESP_LOGI(FNAME,"CheckHeading, heading: %.1f, diff: %.1f", Units::rad_to_deg(vheading), Units::rad_to_deg(diff) );
        // advance in CA_STEP's
        cur_heading = Vector::normalizePI2(cur_heading + (std::signbit(diff) ? -CA_STEP : CA_STEP));

        // calc next thermal
        uint32_t dt = Clock::getMillis() - thermals[_idir].timestamp; // last updated at
        if ( std::signbit(diff) ) { // go against the turn direction
            _idir += 1;
            if (_idir >= CA_NUM_DIRS) _idir = 0;
        } else {
            _idir -= 1;
            if (_idir < 0) _idir = CA_NUM_DIRS - 1;
        }
        ESP_LOGI(FNAME,"VHeading: %.1f, omega: %.1f, bank: %.1f, c_turn: %.2f, c_bank: %.2f, confidence: %.2f",
            Units::rad_to_deg(vheading), Units::rad_to_deg(omega), Units::rad_to_deg(bank), c_turn, c_bank, _confidence.get() );

        mps_t te = 0.f;
        if ( _confidence.get() > 0.7f && dt < 20000 ) { // 15° in max 20sec

            te = varioSensor->getAVG(dt);
            ESP_LOGI(FNAME,"New thermal at heading %.1f, TE: %.2f, %dmsec", Units::rad_to_deg(cur_heading), te, (int)dt );
            uint8_t new_c_dir = std::signbit(diff) ? (uint8_t)circdir_t::circlLeft : (uint8_t)circdir_t::circlRight;
            if ( new_c_dir != _cdir ) {
                ESP_LOGI(FNAME,"ThermalAssist checkHeading, circling direction changed to %s", new_c_dir == (uint8_t)circdir_t::circlLeft ? "left" : "right");
            }
            _cdir = new_c_dir;
        }
        else {
            _th_norm.filter(std::max(1.f, MC.get()));
            ESP_LOGI(FNAME,"ThermalAssist checkHeading, no thermaling detected, reset peak value %.2f, dt: %d", _confidence.get(), (int)dt);
        }
        thermals[_idir].set(te);
    }
}

