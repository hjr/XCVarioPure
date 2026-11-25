/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "IpsDisplay.h"
#include "sensor/Filters.h"
#include "math/Units.h"

#include <cstdint>

constexpr int CA_NUM_DIRS = 24;  // every 15°

class PolarGauge;

// center of gravity of the observed climb around the circle
struct ThermalCG {
    int   slot = -1;    // display slot the CG points to, -1 for none
    float level = 0.f;  // CG offset scaled 0..1 for the display mark
    float ecc = 0.f;    // CG offset, |weighted mean direction vector| 0..1
    rad_t angle = 0.f;  // CG direction in the slot frame (0 = glider position, + towards higher slots)
};

struct Thermal {
    mps_t    strength;
    uint32_t timestamp;
    constexpr Thermal() : strength(0.f), timestamp(0) {}
    void set(mps_t s);
    float getStrength() const;
};

class ThermalAssist {
    friend Thermal;
public:
    static ThermalAssist *create(PolarGauge &g);
    static void remove();
    void checkHeading(rad_t vheading, rad_t omega, rad_t bank); // 10Hz ticker
    void draw();
    void setGliderOnTop(bool onTop) { _glider_on_top = onTop; }

private:
    ThermalAssist(PolarGauge &g);
    int slotDir(int i) const;
    void drawThermal(float th_strength, int idir, float cg_level = -1.f);
    void drawGlider(Point center);
    // int maxClimbIndex();
    bool calcThermalCG(const float ths[], ThermalCG &cg) const;
    void normStrengths(float ths[]) const;
    float getTop8Norm() const;
    // acoustic centering commands
    void centeringAid(rad_t vheading, rad_t omega, rad_t bank);
    void updateCircleGeometry(uint32_t step_ms);

    const PolarGauge &_gauge;
    bool _glider_on_top; // circle aid reference on top, or 90° on the side.

    Thermal thermals[CA_NUM_DIRS] = {};  // every 15°: [m/s]
    int8_t _idir = 0;
    uint8_t _cdir = 0; // right & left
    LowPassFilterT<float> _confidence; // 0..1, how confident we are that we are thermaling;
    rad_t cur_heading = 0.f; // accumulating heading delta and dice in e.g.15° steps
    LowPassFilterT<float> _th_norm; // normalization factor for thermal strength
    // acoustic centering
    enum class cmd_state_t : uint8_t { IDLE, OUT_CMD, STRAIGHT, HOLDOFF };
    cmd_state_t _cmd_state = cmd_state_t::IDLE;
    ThermalCG _cg;                   // latest CG, updated with every heading step
    bool      _cg_fresh = false;     // _cg computed from a steady circle
    uint32_t  _cg_time = 0;
    LowPassFilterT<float> _step_ms;  // time per heading step while circling [ms]
    LowPassFilterT<float> _lead_s;   // learned pilot lag: command to half rolled out [s]
    LowPassFilterT<float> _roll_rate; // gyro roll rate, damped [rad/s]
    uint32_t _prev_tick = 0;
    uint32_t _cmd_time = 0;          // time the turn out command was given
    rad_t    _omega_ref = 0.f;       // circling turn rate the maneuver refers to [rad/s]
    float    _shift = 0.f;           // achieved center shift [m]
    float    _shift_target = 0.f;    // wanted center shift [m]
    rad_t    _turned = 0.f;          // holdoff: turned angle since last maneuver
    bool     _roll_onset = false;
    // pixel clean bit bubbles
    Point _last_center;
    int16_t _last_radius = 0;
};

extern ThermalAssist  *thrmAssist;
