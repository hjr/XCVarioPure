/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "ScreenElement.h"
#include "math/Trigonometry.h"
#include "math/Units.h"

#include <algorithm>

class ArrowIndicator;
class WindIndicator;
class LargeFigure;
class CenterAid;


// map gauge values to gauge scale angles
class GaugeFunc
{
public:
    GaugeFunc(float scale, float zero) : _scale_k(scale), _zero_at(zero) {}
    virtual ~GaugeFunc() = default;
    // calculate a gauge indicator position in rad (-_scale_max .. _scale_max) for an gauge indicated value
    virtual float operator()(float a) const { return 0.; }
    // inverse to operator. Get the value for an indicator position in radian
    virtual float invers(float rad) const { return 0.; }
    float getZero() const { return _zero_at; }

protected:
    float _scale_k; // scale factor
    float _zero_at; // value at the middle of the scale
};

// a polar gauge
class PolarGauge : public ScreenElement
{
    friend class ArrowIndicator;
    friend class WindIndicator;
    friend class LargeFigure;
    friend class CenterAid;

public:
    using GaugeFlavor = enum : uint8_t { CLUB, XCVPRO, GLOAD, COMPASS};
    using WindReference = enum : uint8_t { WR_HEADING, WR_NORTH };

    PolarGauge(int16_t refx, int16_t refy, int16_t scale_end, int16_t radius, GaugeFlavor flavor);
    ~PolarGauge();
    void enableWindIndicator(bool avg, bool live);
    void forceAllRedraw();
    void setRange(float pos_range, float center_at, bool log);
    float getMRange() const { return _mrange; }
    int16_t getDist05() const { return _dist05; }
    void setUnit(float uf) { _unit_fac = uf; }
    void setColor(int color_idx);
    void setFigOffset(int16_t ox, int16_t oy);
    void setWindRef(int wref) { _wind_ref = static_cast<WindReference>(wref); }

    void draw(float a);
    void drawIndicator(float a);
    void drawPolarSink(mps_t a);
    void drawAVG();
    void drawFigure(float a);
    void drawWind(int16_t wdir, mps_t wval, int16_t idir, mps_t ival);
    using BowColorIdx = enum { GREEN, BLUE, ORANGE, RED };
    void colorRange(float from, float to, int16_t color);
    void drawScale(float from = -1000., float to = -1000.);
    void drawScaleBottom();
    void drawRose(int16_t at = -1000) const;
    void clearGauge();

  private:
    // indicator and attributes
    ArrowIndicator *_arrow = nullptr;
    WindIndicator *_wind_avg = nullptr;
    WindIndicator *_wind_live = nullptr;
    WindReference _wind_ref = WR_HEADING;
    GaugeFlavor _flavor = CLUB;
    float _scale_max = 1.57f; // half scale extend in rad
    int16_t _radius = 50; // pixel
    float _range = 5.; // max positive value of the scale
    float _mrange = -5.; // resulting from range and zero_at, assuming an always symetric scale
    int16_t _dist05;
    static constexpr const float IDX_SCALE = 360.f/My_PIf; // cut the scale range into discrete 0.5deg steps [rad]
    float _unit_fac = 1.f; // scale  from SI units to the gauge units
    int16_t _old_idx = 360; // discretized previous index value
    int16_t _old_bow_idx = 0;
    int16_t _old_polar_sink = 0;
    float _old_avc = -1.f;

    // gauge value as average or figures
    LargeFigure *_figure = nullptr;

    // gauge function
    GaugeFunc *func = nullptr; // map value range to scale range [rad]
    // dice up into (arbitrary) discrete steps, map scale range into a 0,5° step counter
    int16_t dice_up(float a) const { return (int16_t)(IDX_SCALE*(*func)(a)); } // output in 0.5° steps
    int16_t dice_rad(float rad) const { return (int16_t)(rad*IDX_SCALE); } // input in [rad], output in 0.5° steps

    // trigonometric macros
    int16_t SinCentered(float val, int16_t len) const; // output in pixel
    int16_t CosCentered(float val, int16_t len) const;
    int16_t SinCenteredDeg2(int16_t val, int16_t len) const;
    int16_t CosCenteredDeg2(int16_t val, int16_t len) const;
    int16_t Sin(float val, int16_t len) const;
    int16_t Cos(float val, int16_t len) const;
    int16_t SinDeg2(int16_t val, int16_t len) const;
    int16_t CosDeg2(int16_t val, int16_t len) const;

    // gauge helpers
    inline float clipValue(float a) const { return std::clamp(a, _mrange, _range); };
    void drawDisc(float val, bool clean=false) const;
    void drawOneScaleLine(float a, int16_t l2, int16_t w, int16_t cidx) const;
    void drawBow(int16_t idx, int16_t &old, int16_t w, int16_t off, int16_t cidx = 0) const;
    void drawOneLabel(float val, int16_t labl) const;
    void drawTwoDots(int16_t a, int16_t size, int16_t cidx) const;
};
