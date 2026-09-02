/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "ThermalAssist.h"

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
    _th_norm(LowPassFilterT<float>::alphaFromTau(20.0, 1.0f))
{
    _glider_on_top = thermal_assist.get() != 2;
    resetNorm();
}

// th_strength normalized to 0 .. 1
void ThermalAssist::drawThermal(float th_strength, int idir) {
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
    int16_t cx = _gauge._ref.x + fast_sin_rad(ddir * CA_STEP) * _gauge._radius;
    int16_t cy = _gauge._ref.y - fast_cos_rad(ddir * CA_STEP) * _gauge._radius;

    // a green thermal spot that brightens with increasing thermal strength (up to 10%)
    ucg_color_t col = { COLOR_GREEN };
    col.fadeTo(col, th_strength * 1.2f);
    // ESP_LOGI(FNAME,"drawThermal, th: %.1f, ths: %.3f, color: %d,%d,%d", th.strength, th_strength, col.color[0], col.color[1], col.color[2]);
    MYUCG->setColor(col.color[0], col.color[1], col.color[2]);
    if (idir != 0) {
        MYUCG->startBuffering(cx-MAX_DISK_RAD, cy-MAX_DISK_RAD, 2*MAX_DISK_RAD+1, 2*MAX_DISK_RAD);
        // MYUCG->drawFrame(cx-MAX_DISK_RAD, cy-MAX_DISK_RAD, 2*MAX_DISK_RAD, 2*MAX_DISK_RAD-1);
        int16_t radius = std::clamp((int16_t)fast_iroundf_positive(th_strength * MAX_DISK_RAD), (int16_t)1, MAX_DISK_RAD);
        MYUCG->drawDisc(cx, cy, radius, UCG_DRAW_ALL);
        MYUCG->finishBuffering();
    }
    else {
        // draw glider icon here
        drawGlider(cx, cy);
    }
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
void ThermalAssist::drawGlider(int16_t cx, int16_t cy) {
    constexpr int16_t A = -3;
    constexpr int16_t B = 3;
    constexpr int16_t W = 8;
    const int16_t triangle[3][6] = { { A, -W, A, W, B, 0}, {-A, -W, -B, 0, -A, W}, {-W, B, W, B, 0, A} }; // left, right, tail

    // select triangle for circling direction
    MYUCG->setColor(COLOR_WHITE);
    int16_t *trptr = (int16_t*)triangle[2];
    if (_glider_on_top) {
        MYUCG->startBuffering(cx-B, cy-W, 2*B+1, 2*W+1);
        // MYUCG->drawFrame(cx-B, cy-W, 2*B, 2*W);
        if ( _cdir == (uint8_t)circdir_t::circlRight )
            trptr = (int16_t*)triangle[0];
        else {
            trptr = (int16_t*)triangle[1];
        }
    } else {
        MYUCG->startBuffering(cx-W, cy-B, 2*W+1, 2*B+1);
        // MYUCG->drawFrame(cx-W, cy-B, 2*W, 2*B);
    }
    if ( _confidence.get() > 0.7f ) {
        MYUCG->setColor(COLOR_WHITE);
        MYUCG->drawTriangle(cx + trptr[0], cy + trptr[1], cx + trptr[2], cy + trptr[3], cx + trptr[4], cy + trptr[5]);
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

    float magnitude = sqrtf(sx*sx + sy*sy);
    float eccentricity = magnitude / sum;  // 0.0 .. ~1.0
    float roundness = 1.0f - eccentricity;


    // float angle = atan2f(sy, sx);
    return Point(sx, sy);
}

void ThermalAssist::resetNorm() {
    _th_norm.reset(std::min(1.f, MC.get()));
}

void ThermalAssist::draw() {
    // get the peak and min thermals
    float th_min = thermals[0].strength;
    float th_max = th_min;
    for ( int i = 1; i < CA_NUM_DIRS; i++) {
        float tmp = thermals[i].strength;
        if ( tmp > th_max) {
            th_max = tmp;
        }
        if ( tmp < th_min) {
            th_min = tmp;
        }
    }
    // calc peak norm
    th_min = std::max(th_min - 0.2f, .0f);
    _th_norm.filter(std::max(th_max - th_min, 1.f));

    ESP_LOGI(FNAME,"TA draw, peak norm: %.2f, %.2f, %.2f", th_min, th_max, _th_norm.get());
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


// 2 Hz called from AHRS / heading sensor
// > tick : a 10 Hz counter
void ThermalAssist::checkHeading(rad_t vheading, rad_t omega, rad_t bank) {

    // Combined fidelity on "this is thermaling"
    // 1. turn rate confidence
    float c_turn = std::clamp((fabsf(omega) - Units::deg_to_rad(2.0f)) / Units::deg_to_rad(6.0f), 0.0f, 1.0f);
    // 2. bank angle confidence
    float c_bank = std::clamp((fabsf(bank)-Units::deg_to_rad(8.0f)) / Units::deg_to_rad(16.0f), 0.0f, 1.0f);
    // 3. steadiness/duration confidence
    _confidence.filter( (c_turn + c_bank) / 2.f );
    debugvar.set(_confidence.get());

    // integrate footing/heading, create a new thermal when the heading has changed by 15° or more
    rad_t diff = Vector::angleDiff( vheading, cur_heading );
    if ( fabsf(diff) > CA_STEP ) {
        ESP_LOGI(FNAME,"CheckHeading, heading diff: %.1f", Units::rad_to_deg(diff) );
        cur_heading = Vector::normalizePI2(cur_heading + diff);

        // calc next thermal
        uint32_t dt = Clock::getMillis() - thermals[_idir].timestamp; // last updated at
        _idir += std::signbit(diff) ? -1 : 1;
        if (_idir < 0) _idir += CA_NUM_DIRS;
        if (_idir >= CA_NUM_DIRS) _idir -= CA_NUM_DIRS;
        ESP_LOGI(FNAME,"VHeading: %.1f, omega: %.1f, bank: %.1f, c_turn: %.2f, c_bank: %.2f, confidence: %.2f",
            Units::rad_to_deg(vheading), Units::rad_to_deg(omega), Units::rad_to_deg(bank), c_turn, c_bank, _confidence.get() );

        if ( _confidence.get() > 0.7f && dt < 10000 ) {

            ESP_LOGI(FNAME,"New thermal avg over %dsec", (int)dt / 1000);
            mps_t te = te_vario.get(); // (alt - _te_alt) * 1000.f / dt;
            thermals[_idir].set(te);
            ESP_LOGI(FNAME,"New thermal at heading %.1f, TE: %.2f", Units::rad_to_deg(cur_heading), te );
            uint8_t new_c_dir = std::signbit(diff) ? (uint8_t)circdir_t::circlLeft : (uint8_t)circdir_t::circlRight;
            if ( new_c_dir != _cdir ) {
                ESP_LOGI(FNAME,"ThermalAssist checkHeading, circling direction changed to %s", new_c_dir == (uint8_t)circdir_t::circlLeft ? "left" : "right");

            }
            _cdir = new_c_dir;
        }
        else {
            resetNorm();
            ESP_LOGI(FNAME,"ThermalAssist checkHeading, no thermaling detected, reset peak value");
            thermals[_idir].set(0.f);
        }

    }
}

