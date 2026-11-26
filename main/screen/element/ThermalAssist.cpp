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
#include "driver/audio/ESPAudio.h"
#include "vector.h"
#include "sensor/imu/Gyroscope.h"
#include "setup/SetupNG.h"
#include "AdaptUGC.h"
#include "Colors.h"
#include "screen/MessageBox.h"
#include "logdefnone.h"

#include <algorithm>
#include <cmath>


constexpr rad_t CA_STEP = Units::deg_to_rad(360/CA_NUM_DIRS); // 15
constexpr rad_t CA_STEP_2 = CA_STEP/2.f;  // 7.5
constexpr int16_t MAX_DISK_RAD = 7;
constexpr int16_t CG_MIN_DISK_RAD = 2;   // the thermal CG mark, smallest
constexpr float CG_MIN_WEIGHT = 2.f;      // min. sum of normalized strengths to show a CG
constexpr float CG_MIN_ECC = 0.15f;       // below: evenly spread, well centered, no mark
constexpr float CG_FULL_ECC = 0.6f;       // from here on: max. mark size and full red
constexpr uint32_t CG_MAX_AGE_MS = 40000; // slots older than this do not count as observed
constexpr int CG_MIN_SLOTS = CA_NUM_DIRS * 3 / 4; // observed part of the circle to show a CG

// acoustic centering commands
constexpr float    CMD_MIN_CONFIDENCE = 0.8f;  // steady thermaling
constexpr float    CMD_MIN_LEVEL = 0.2f;       // min. CG level to bother the pilot
constexpr uint32_t CMD_CG_MAX_AGE_MS = 3000;   // CG is refreshed with every 15° heading step
constexpr float    CG_SHIFT_GAIN = 1.0f;       // center shift = gain * CG eccentricity * circle radius
constexpr float    CMD_MIN_SHIFT = 12.f;       // [m] smaller corrections are not worth it
constexpr float    CMD_MAX_SHIFT_R = 1.0f;     // max. shift in circle radii per maneuver
constexpr second_t CMD_REACT_TIMEOUT = 4.f;    // pilot ignored the command
constexpr second_t LEAD_INIT = 1.5f;           // pilot lag command -> half rolled out [s]
constexpr second_t LEAD_MIN = 0.5f;
constexpr second_t LEAD_MAX = 3.5f;
constexpr rad_t    ROLL_STEADY = Units::deg_to_rad(5.f); // [/s] max. roll rate for steady circling
constexpr rad_t    ROLL_ONSET = Units::deg_to_rad(8.f);  // [/s] pilot starts to roll out
constexpr rad_t    CMD_HOLDOFF_TURN = 2.5f * My_PIf;     // 1.25 circles to re-observe after a maneuver

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
    _th_norm(LowPassFilterT<float>::alphaFromTau(vario_av_delay.get() * 2, .5f)),
    _step_ms(0.2f),   // per heading step, ~5 steps
    _lead_s(0.3f),    // per maneuver
    _roll_rate(LowPassFilterT<float>::alphaFromTau(0.3f, 0.1f))
{
    _glider_on_top = thermal_assist.get() != 2;
    _th_norm.reset(std::max(1.f, MC.get()));
    _lead_s.reset(LEAD_INIT);
}

// display slot (0 = glider position) to the drawing direction on the gauge [CA_STEP]
int ThermalAssist::slotDir(int i) const {
    if (!_glider_on_top) {
        if (_cdir == (uint8_t)circdir_t::circlRight) {
            return (i + CA_NUM_DIRS / 4) % CA_NUM_DIRS;  // move reference to the left
        } else if (_cdir == (uint8_t)circdir_t::circlLeft) {
            return (i + 3 * CA_NUM_DIRS / 4) % CA_NUM_DIRS;  // move reference to the right
        }
    }
    return i;
}

// th_strength normalized to 0 .. 1
// cg_level: the thermal's center of gravity points to this spot, with a CG offset level 0..1
//           (white and small .. red and filling the spot), < 0 for no mark
void ThermalAssist::drawThermal(float th_strength, int idir, float cg_level) {
    // the quadrant of the previous (clockwise neighbor) disc reaching into this disc's box,
    // indexed by this disc's quadrant: lower right, lower left, upper right, upper left
    static const uint8_t hem_flag[4] = {UCG_DRAW_UPPER_RIGHT, UCG_DRAW_LOWER_RIGHT, UCG_DRAW_UPPER_LEFT, UCG_DRAW_LOWER_LEFT};
    // ESP_LOGI(FNAME,"drawThermal, th_strength: %.1f, idir: %d", th_strength, idir);
    if (idir >= CA_NUM_DIRS || idir < 0) {
        ESP_LOGE(FNAME, "index out of range: %d", idir);
        return;
    }
    int ddir = slotDir(idir);
    // slot 0 on top, counting counter clockwise (slot i was flown at heading - i*CA_STEP, screen y grows downwards),
    // so the spots move against the glider's flight direction in both circling directions
    Point center = {(int16_t)(-fast_sin_rad(ddir * CA_STEP) * _gauge._radius), (int16_t)(-fast_cos_rad(ddir * CA_STEP) * _gauge._radius)};
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
        if ( cg_level >= 0.f ) {
            // piggy-back the CG mark, the buffered box wipes it again with the next redraw
            // grows with the CG offset up to the full spot size, dominating the green spot,
            // and turns from white to red
            int16_t cg_rad = CG_MIN_DISK_RAD + fast_iroundf_positive(cg_level * (MAX_DISK_RAD - CG_MIN_DISK_RAD));
            const ucg_color_t white = { COLOR_WHITE }; // runtime display variant
            constexpr ucg_color_t red = { COLOR_RED };
            ucg_color_t mc;
            for (int k = 0; k < 3; k++) {
                mc.color[k] = (uint8_t)(white.color[k] + (red.color[k] - white.color[k]) * cg_level);
            }
            MYUCG->setColor(mc.color[0], mc.color[1], mc.color[2]);
            MYUCG->drawDisc(center.x, center.y, cg_rad, UCG_DRAW_ALL);
        }
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

// Center of gravity of the observed climb around the circle.
// ths: normalized strengths per display slot (0 = glider position).
// The CG is the strength weighted mean of the slot direction vectors. Its length (0..1) tells
// how much the climb concentrates on one side, its direction where to shift the circle.
// Returns false for none (not enough data, or well centered), else the CG with the display slot
// it points to, the offset level 0..1 for the mark, the raw eccentricity and the direction.
bool ThermalAssist::calcThermalCG(const float ths[], ThermalCG &cg) const {
    uint32_t now = Clock::getMillis();
    float sx = 0.f, sy = 0.f, sum = 0.f;
    int observed = 0;

    cg.slot = -1;
    for (int i = 0; i < CA_NUM_DIRS; i++) {
        const Thermal &th = thermals[(i + _idir) % CA_NUM_DIRS];
        if (th.timestamp == 0 || now - th.timestamp > CG_MAX_AGE_MS) {
            continue; // nothing (recent) observed in this direction
        }
        observed++;
        float w = std::clamp(ths[i], 0.f, 1.f);
        rad_t dir = i * CA_STEP; // slot frame, the display rotation applies equally to all slots
        sx += w * fast_sin_rad(dir);
        sy += w * fast_cos_rad(dir);
        sum += w;
    }
    if (observed < CG_MIN_SLOTS || sum < CG_MIN_WEIGHT) {
        return false;
    }
    cg.ecc = std::sqrt(sx * sx + sy * sy) / sum;
    if (cg.ecc < CG_MIN_ECC) {
        return false;
    }
    cg.level = std::clamp((cg.ecc - CG_MIN_ECC) / (CG_FULL_ECC - CG_MIN_ECC), 0.f, 1.f);
    cg.angle = std::atan2(sx, sy);
    int slot = fast_iroundf(cg.angle / CA_STEP);
    cg.slot = (slot + CA_NUM_DIRS) % CA_NUM_DIRS;
    // ESP_LOGI(FNAME, "thermal CG: ecc %.2f slot %d, observed %d, weight %.1f", cg.ecc, cg.slot, observed, sum);
    return true;
}

// normalized strength 0..1 per display slot (0 = glider position)
void ThermalAssist::normStrengths(float ths[]) const {
    float th_top = std::min(getTop8Norm(), MC.get() + 2.f); // max. norm according to MC
    float th_min = std::max(th_top - 3.0f, .0f);
    float norm = std::max(_th_norm.get(), 1.f);
    for (int i = 0; i < CA_NUM_DIRS; i++) {
        int d = (i + _idir) % CA_NUM_DIRS;
        ths[i] = std::min((thermals[d].getStrength() - th_min) / norm, 1.f);
    }
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
    float ths[CA_NUM_DIRS];
    normStrengths(ths);
    // the spot the thermal CG points to, while circling
    ThermalCG cg;
    if (_confidence.get() > 0.7f) {
        calcThermalCG(ths, cg);
    }

    for (int i = 0; i < CA_NUM_DIRS; i++) {
        drawThermal(ths[i], i, (i == cg.slot) ? cg.level : -1.f);
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
        // ESP_LOGI(FNAME,"CheckHeading, heading: %.1f, diff: %.1f", Units::rad_to_deg(vheading), Units::rad_to_deg(diff) );
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
        // ESP_LOGI(FNAME,"VHeading: %.1f, omega: %.1f, bank: %.1f, c_turn: %.2f, c_bank: %.2f, confidence: %.2f",
        //     Units::rad_to_deg(vheading), Units::rad_to_deg(omega), Units::rad_to_deg(bank), c_turn, c_bank, _confidence.get() );

        mps_t te = 0.f;
        bool circling = _confidence.get() > 0.7f && dt < 20000; // 15° in max 20sec
        if ( circling ) {

            te = varioSensor->getAVG(dt * 2.f);
            // ESP_LOGI(FNAME,"New thermal at heading %.1f, TE: %.2f, %dmsec", Units::rad_to_deg(cur_heading), te, (int)dt );
            uint8_t new_c_dir = std::signbit(diff) ? (uint8_t)circdir_t::circlLeft : (uint8_t)circdir_t::circlRight;
            // if ( new_c_dir != _cdir ) {
            //     ESP_LOGI(FNAME,"ThermalAssist checkHeading, circling direction changed to %s", new_c_dir == (uint8_t)circdir_t::circlLeft ? "left" : "right");
            // }
            _cdir = new_c_dir;
        }
        else {
            _th_norm.filter(std::max(1.f, MC.get()));
            // ESP_LOGI(FNAME,"ThermalAssist checkHeading, no thermaling detected, reset peak value %.2f, dt: %d", _confidence.get(), (int)dt);
        }
        thermals[_idir].set(te);

        // refresh the CG and circle geometry for the centering aid
        _cg_fresh = false;
        if ( circling ) {
            updateCircleGeometry(dt);
            float ths[CA_NUM_DIRS];
            normStrengths(ths);
            _cg_fresh = calcThermalCG(ths, _cg);
            _cg_time = Clock::getMillis();
        }
    }
    centeringAid(vheading, omega, bank);
}

// learn the time per heading step during steady circling (not while a centering maneuver distorts it)
void ThermalAssist::updateCircleGeometry(uint32_t step_ms)
{
    if ( (_cmd_state != cmd_state_t::IDLE && _cmd_state != cmd_state_t::HOLDOFF)
        || std::fabs(_roll_rate.get()) > ROLL_STEADY
        || step_ms < 300 || step_ms > 5000 ) { // 3..50 °/s
        return;
    }
    if ( _step_ms.get() <= 0.f ) {
        _step_ms.reset(step_ms);
    } else {
        _step_ms.filter(step_ms);
    }
}

//
// Acoustic centering aid, 10Hz
//
// Shifting the circle: flying (more) straight for a moment on heading h moves the circle center along h.
// The lift CG direction seen from the circle center is the CG heading -/+ 90° (right/left turn), which
// in turn is the heading to fly straight on. So the "turn out" command is given a quarter circle ahead of
// the CG, early by the learned pilot lag. The pilot's response is monitored through roll rate and
// turn rate, the achieved center shift is integrated, and "turn in" is commanded when it will meet the
// wanted shift, which is derived from the CG eccentricity and the circle radius:
//   circle radius   R = TAS / omega, omega = 360° / circle time (cross checked with R = TAS² / (g tan(bank)))
//   wanted shift    d = gain * ecc * R
//   achieved shift  d = integral TAS * (1 - omega/omega_ref) dt   (center of the reference circle tangent to the path)
// After a maneuver the TA needs at least one fresh circle before the next command.
//
void ThermalAssist::centeringAid(rad_t vheading, rad_t omega, rad_t bank)
{
    uint32_t now = Clock::getMillis();
    second_t dt = (now - _prev_tick) / 1000.f;
    _prev_tick = now;
    if ( dt <= 0.f || dt > 0.5f || ! gyroSensor ) { // first call or a gap
        return;
    }
    // body roll rate ~ bank rate at the small pitch angles of circling, lightly damped against turbulence
    _roll_rate.filter(gyroSensor->getRollRate());

    const rad_t aomega = std::fabs(omega);
    const mps_t v = tas.get();

    switch ( _cmd_state ) {
    case cmd_state_t::IDLE:
    {
        if ( thermal_assist_audio.get() == 0 || ! AUDIO || ! _cg_fresh || now - _cg_time > CMD_CG_MAX_AGE_MS
            || _confidence.get() < CMD_MIN_CONFIDENCE || _cg.level < CMD_MIN_LEVEL || _step_ms.get() <= 0.f ) {
            break;
        }
        // steady circling, no pilot action in progress
        rad_t omega0 = CA_STEP * 1000.f / _step_ms.get(); // mean circling turn rate
        if ( std::fabs(_roll_rate.get()) > ROLL_STEADY || std::fabs(aomega - omega0) > 0.25f * omega0 ) {
            break;
        }
        // circle radius from speed and circle time, cross checked with the bank angle
        float radius = v / omega0;
        float tb = std::tan(std::fabs(bank));
        if ( tb < 0.1f ) {
            break;
        }
        float radius_bank = v * v / (Units::g0 * tb);
        if ( radius_bank < 0.6f * radius || radius_bank > 1.6f * radius ) {
            ESP_LOGI(FNAME, "implausible circle R %.0fm, bank R %.0fm", radius, radius_bank);
            break;
        }
        float target = std::min(CG_SHIFT_GAIN * _cg.ecc * radius, CMD_MAX_SHIFT_R * radius);
        if ( target < CMD_MIN_SHIFT ) {
            break;
        }
        // CG heading, slot i was flown at cur_heading - i*CA_STEP
        bool right = omega > 0.f;
        rad_t h_cg = cur_heading - _cg.angle;
        // the lift lies 90° into the turn from there, fly straight on that bearing
        rad_t h_cmd = h_cg + (right ? -My_PIf / 2.f : My_PIf / 2.f);
        rad_t togo = Vector::normalizePI2(right ? h_cmd - vheading : vheading - h_cmd);
        if ( togo > aomega * _lead_s.get() ) {
            break; // not yet
        }
        AUDIO->startSound(AUDIO_CMD_CIRCLE_OUT, true);
        MBOX->pushMessage(1, "  turn now", 10);
        _cmd_state = cmd_state_t::OUT_CMD;
        _cmd_time = now;
        _omega_ref = omega0;
        _shift = 0.f;
        _shift_target = target;
        _roll_onset = false;
        ESP_LOGI(FNAME, "turn out: R %.0fm (bank R %.0fm), ecc %.2f, shift %.0fm towards %.0f°, lead %.1fs",
            radius, radius_bank, _cg.ecc, target, Units::rad_to_deg(Vector::normalizePI2(h_cmd)), _lead_s.get());
        break;
    }
    case cmd_state_t::OUT_CMD:
    case cmd_state_t::STRAIGHT:
    {
        second_t elapsed = (now - _cmd_time) / 1000.f;
        // the center of a reference circle tangent to the flight path moves with TAS * (1 - omega/omega_ref)
        float rate = v * std::max(0.f, 1.f - aomega / _omega_ref);
        _shift += rate * dt;

        if ( _cmd_state == cmd_state_t::OUT_CMD ) {
            if ( ! _roll_onset && _roll_rate.get() * bank < 0.f && std::fabs(_roll_rate.get()) > ROLL_ONSET ) {
                _roll_onset = true; // pilot rolls towards wings level
                ESP_LOGI(FNAME, "roll out after %.1fs", elapsed);
            }
            if ( aomega < 0.5f * _omega_ref ) {
                // half rolled out, learn the pilot's lag for the next command
                _lead_s.filter(std::clamp(elapsed, LEAD_MIN, LEAD_MAX));
                _cmd_state = cmd_state_t::STRAIGHT;
            }
            else if ( elapsed > CMD_REACT_TIMEOUT ) {
                if ( _shift < 0.3f * _shift_target ) {
                    // ignored, no further command
                    ESP_LOGI(FNAME, "turn out ignored (onset %d, shift %.0fm)", _roll_onset, _shift);
                    _turned = 0.f;
                    _cmd_state = cmd_state_t::HOLDOFF;
                    break;
                }
                _cmd_state = cmd_state_t::STRAIGHT; // only shallowed the turn, that does it as well
            }
        }
        if ( _cmd_state == cmd_state_t::STRAIGHT ) {
            // turn in again, anticipating the shift until the pilot is back in the turn
            second_t t_max = CMD_REACT_TIMEOUT + 2.f * _lead_s.get() + _shift_target / std::max(v, 10.f);
            if ( _shift + rate * _lead_s.get() >= _shift_target || elapsed > t_max ) {
                AUDIO->startSound(AUDIO_CMD_CIRCLE_IN, true);
                MBOX->pushMessage(1, "  go straight", 10);
                ESP_LOGI(FNAME, "turn in: shift %.0fm of %.0fm after %.1fs", _shift, _shift_target, elapsed);
                _turned = 0.f;
                _cmd_state = cmd_state_t::HOLDOFF;
            }
        }
        break;
    }
    case cmd_state_t::HOLDOFF:
        // let the TA observe at least one more full circle around the new center
        if ( _confidence.get() > 0.7f ) {
            _turned += aomega * dt;
        }
        if ( _turned > CMD_HOLDOFF_TURN ) {
            _cmd_state = cmd_state_t::IDLE;
        }
        break;
    }
}

