/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "PolarGauge.h"

#include "ArrowIndicator.h"
#include "WindIndicator.h"
#include "LargeFigure.h"
#include "math/Trigonometry.h"
#include "math/Floats.h"
#include "math/Units.h"
#include "Atmosphere.h"
#include "Colors.h"
#include "AdaptUGC.h"
#include "setup/SetupNG.h"
#include "logdefnone.h"

#include <cmath>
#include <algorithm>

float getHeading(); // fixme

extern AdaptUGC *MYUCG;
static constexpr const ucg_color_t bow_color[4] = {{COLOR_DGREEN}, {COLOR_BLUE}, {COLOR_ORANGE}, {COLOR_RED}};
static const ucg_color_t lne_color[3] = {{COLOR_LGREY}, {COLOR_MGREY}, {COLOR_WHITE}};


////////////////////////////
// Function mapping the gauges value range to rad, or diced 0,5° steps

class LinGaugeFunc : public GaugeFunc
{
public:
    LinGaugeFunc(float scale, float zero) : GaugeFunc(scale, zero) {}
    float operator()(float a) const override { return (a - _mid_at) * _scale_k; }
    float invers(float rad) const override { return (rad / _scale_k) + _mid_at; }
};
class LogGaugeFunc : public GaugeFunc
{
public:
    LogGaugeFunc(float scale, float zero) : GaugeFunc(scale, zero) {}
    float operator()(float a) const override { return fast_log2f(std::abs((a-_mid_at)) + 1.f) * _scale_k * (std::signbit((a-_mid_at)) ? -1. : 1.); }
    float invers(float rad) const override { return ((pow(2., std::abs(rad)) - 1.f) / _scale_k * (std::signbit(rad) ? -1.f : 1.f)) + _mid_at; }
};
// class RoseGaugeFunc : public GaugeFunc
// {
// public:
//     RoseGaugeFunc() : GaugeFunc(My_PIf/180.f, 90.f) {}
//     float operator()(float a) const override { return (a + _zero_at) * _scale_k; }
//     float invers(float rad) const override { return (rad / _scale_k) - _zero_at; }
// };

////////////////////////////
// PolarGauge

// pixel, pixel, degree, pixel
PolarGauge::PolarGauge(int16_t refx, int16_t refy, int16_t scale_end, int16_t radius, GaugeFlavor flavor) :
    ScreenElement(refx, refy),
    _flavor(flavor),
    _scale_max(deg2rad((float)scale_end)),
    _radius(radius)
{
    func = new GaugeFunc(1.,0.);
    if ( flavor != COMPASS ) {
        if ( _flavor == XCVPRO ) {
            _arrow = new ArrowIndicator(*this, _radius-4, 22, 10);
        }
        else {
            int16_t arrow_len = display_orientation.get() == DISPLAY_NINETY ? 40 : 50;
            _arrow = new ArrowIndicator(*this, _radius+8, arrow_len, 5, 7);
        }
    }
    else {
        _wind_avg = new WindIndicator(*this, false);

        _unit_fac = 1.;
        _range = 360.;
        _mrange = 0.;
        // func = new RoseGaugeFunc();
    }
}
PolarGauge::~PolarGauge()
{
    if ( _arrow ) {
        delete _arrow;
    }
    if ( _wind_avg ) {
        delete _wind_avg;
    }
    if ( _wind_live ) {
        delete _wind_live;
    }
    if ( _figure ) {
        delete _figure;
    }
    if ( func ) {
        delete func;
    }
}

void PolarGauge::enableWindIndicator(bool avg, bool live) {
    if (avg && !_wind_avg) {
        _wind_avg = new WindIndicator(*this, false);
    }
    else if (!avg && _wind_avg) {
        delete _wind_avg;
        _wind_avg = nullptr;
    }
    if (live && !_wind_live) {
        _wind_live = new WindIndicator(*this, true);
    }
    else if (!live && _wind_live) {
        delete _wind_live;
        _wind_live = nullptr;
    }
}

void PolarGauge::forceAllRedraw()
{
    _dirty = true;
    if ( _figure ) {
        _figure->forceRedraw();
    }
    _old_bow_idx = 0; // redraw bows
    _old_polar_sink = 0;
    _old_avc = -1.f;
}

void PolarGauge::setRange(float pos_range, float center_at, bool log)
{
    _range = pos_range * _unit_fac;
    _mrange = 2 * center_at - _range;
    if ( func ) delete func;
    if ( log ) {
        func = new LogGaugeFunc(_scale_max / fast_log2f((_range - center_at) + 1.), center_at);
    }
    else {
        func = new LinGaugeFunc(_scale_max / (_range - center_at), center_at);
    }
    // pre-calc pixel dist for interval 0.5-1 on scale bow
    _dist05 = fast_iroundf(((*func)(1.) - (*func)(0.5)) * _radius); // in pixel
}

void PolarGauge::setColor(int color_idx)
{
    if ( _arrow ) {
        _arrow->setColor(ndl_color[color_idx & 3]);
    }
    if ( _wind_live ) {
        _wind_live->setColor(ndl_color[color_idx & 3]);
    }
}

void PolarGauge::setFigOffset(int16_t ox, int16_t oy)
{
    if ( ! _figure ) {
        _figure = new LargeFigure(_ref.x+ox, _ref.y+oy);
    }
    else {
        _figure->setRef(_ref.x+ox, _ref.y+oy);
    }
}

// > in [m/s]
void PolarGauge::draw(float a)
{
    a *= _unit_fac;
    int16_t val = dice_up(clipValue(a));
    // ESP_LOGI(FNAME,"draw  a:%f - %d", a, val);
    if ( _flavor == XCVPRO ) {
        if (_arrow->draw(val))
        {
            // Draw colored bow
            int16_t bar_val = (val > 0) ? val : 0;
            // draw green vario bar
            drawBow(bar_val, _old_bow_idx, 5, 1, GREEN);
        }
    }
    else {
        _arrow->drawOver(val, a);
    }
    _dirty = false;
}

void PolarGauge::drawIndicator(float a)
{
    if ( _flavor == XCVPRO ) {
        _arrow->draw(dice_up(clipValue(a)));
    }
    else {
        _arrow->drawOver(dice_up(clipValue(a)), a);
    }
}

// sink speed in [m/s]
void PolarGauge::drawPolarSink(mps_t a)
{
    a *= _unit_fac;
    int16_t val = dice_up(clipValue(a));
    // ESP_LOGI(FNAME,"sink  a:%f - %d", a, val);
    drawBow(val, _old_polar_sink, 5, 1, BLUE);
}

void PolarGauge::drawAvgClimb(){
    // average climb in [m/sec]
    float avclimb = VarioUnit->apply(average_climb.get());
    float delta = avclimb - _old_avc;

    ESP_LOGI(FNAME, "drawAVG: av=%.2f delta=%.2f", avclimb, delta);
    if ( std::fabs(delta) < 0.05 || avclimb < 0.05) {
        return; // that is just noise
    }
    if (_old_avc > .0) {
        drawDisc(_old_avc, true);
        ESP_LOGI(FNAME, "clean scale at: %f", _old_avc);
        drawScale(_old_avc, _old_avc);
    }
    if (delta > (mean_climb_major_change.get()) / core_climb_history.get()) {
        MYUCG->setColor(COLOR_GREEN);
    } else if (delta < -(mean_climb_major_change.get()) / core_climb_history.get()) {
        MYUCG->setColor(COLOR_RED);
    } else if (delta > (mean_climb_major_change.get() / 2.0) / core_climb_history.get()) {
        MYUCG->setColor(COLOR_GREEN);
    } else if (delta < -(mean_climb_major_change.get() / 2.0) / core_climb_history.get()) {
        MYUCG->setColor(COLOR_RED);
    } else {
        MYUCG->setColor(COLOR_WHITE);
    }

    drawDisc(avclimb);
    _old_avc = avclimb;
    MYUCG->setColor(COLOR_WHITE);
}

void PolarGauge::drawFigure(float a)
{
    if ( _figure && _figure->changed(a) ) {
        const BoundingBox& prev = _figure->getBoundingBox();
        MYUCG->startBuffering(prev[0].x, prev[0].y, prev[1].x, prev[1].y);
        
        if (_wind_avg) {
            _wind_avg->redrawBG();
        }
        if (_wind_live) {
            _wind_live->redrawBG();
        }
        a *= VarioUnit->scale; // fixme that is a bit hacky
        _figure->draw(a);
        MYUCG->finishBuffering();
    }
}

void PolarGauge::drawWind(WindData s, WindData i)
{
    int16_t heading = 0;

    if ( _wind_ref == WR_HEADING ) {
        heading = fast_iroundf(getHeading());
        s.inclHeading(heading);
        i.inclHeading(heading);
    }

    // ESP_LOGI(FNAME,"PW  d:%d - %d", wdir%360, wval);
    if ( _wind_avg ) {
        if (_wind_avg->changed(s) || _dirty) {
            _wind_avg->drawWind();
        }
    }

    if ( _wind_live ) {
        if (_wind_live->changed(i) || _dirty) {
            _wind_live->drawWind();
        }
    }
}

// a : in [scale units]
// set color before calling
void PolarGauge::drawDisc(float a, bool clean) const
{
    if ( clean ) {
        MYUCG->setColor(COLOR_BLACK);
        // siz += 1;
    }
    int16_t val = dice_up(clipValue(a));
    ESP_LOGI( FNAME,"draw disc val %.2f diced: %d", a, val );
    int16_t pos = _radius + 15;
    int x = CosCenteredDeg2(val, pos);
    int y = SinCenteredDeg2(val, pos);
    MYUCG->drawDisc(x, y, 4, UCG_DRAW_ALL);
}

// a : in [rad]
// l2 : line end radius in [pixel]
// w : line width in [pixel]
// cidx : color index
void PolarGauge::drawOneScaleLine(float a, int16_t l2, int16_t w, int16_t cidx) const
{
    float si = fast_sin_rad(a);
    float co = fast_cos_rad(a);
    int16_t l1 = _radius;
    int16_t w0 = w / 2;
    int16_t w1 = w - w0; // total width := w1 + w0
    int16_t xn_0 = _ref.x - fast_iroundf(co * l1 - si * w0);
    int16_t yn_0 = _ref.y - fast_iroundf(si * l1 + co * w0);
    int16_t xn_1 = _ref.x - fast_iroundf(co * l1 + si * w1);
    int16_t yn_1 = _ref.y - fast_iroundf(si * l1 - co * w1);
    int16_t xn_2 = _ref.x - fast_iroundf(co * l2 + si * w1);
    int16_t yn_2 = _ref.y - fast_iroundf(si * l2 - co * w1);
    int16_t xn_3 = _ref.x - fast_iroundf(co * l2 - si * w0);
    int16_t yn_3 = _ref.y - fast_iroundf(si * l2 + co * w0);
    // ESP_LOGI(FNAME,"drawTetragon  x0:%d y0:%d x1:%d y1:%d x2:%d y2:%d x3:%d y3:%d", xn_0, yn_0, xn_1, yn_1, xn_2, yn_2, xn_3, yn_3 );
    cidx = std::clamp(cidx, (int16_t)0, (int16_t)2);  
    MYUCG->setColor(lne_color[cidx].color[0], lne_color[cidx].color[1], lne_color[cidx].color[2]);
    MYUCG->drawTetragon(xn_0, yn_0, xn_1, yn_1, xn_2, yn_2, xn_3, yn_3);
}

// draw incremental bow up to indicator given in diced 0.5° steps, pos
void PolarGauge::drawBow(int16_t idx, int16_t &old, int16_t w, int16_t off, int16_t cidx) const
{
    if (idx == old) {
        return;
    }
    // ESP_LOGI(FNAME,"drawBbow af= level=%d old_level=%d", idx, old );

    // potentially clean first
    if (std::abs(idx) < std::abs(old) || idx * old < 0) {
        MYUCG->setColor(COLOR_MGREY);
    }
    else {
        if (cidx>=0) {
            MYUCG->setColor(bow_color[cidx].color[0], bow_color[cidx].color[1], bow_color[cidx].color[2]);

        } else {
            MYUCG->setColor(COLOR_BLACK);
        }
    }
    // ESP_LOGI(FNAME,"bow lev %d", idx);
    int16_t l1 = _radius + off;
    if (  w < 0 ) {
        w = -w;
        l1 += w;
    }
    int inc = (idx - old > 0) ? 1 : -1;
    if ( std::abs(w) > 5 && std::abs(old - idx) > 8)
    {
        inc *= 6;
    }
    int16_t step = std::abs(inc) * ((old<0||idx<0)?-1:1);
    for (int i = old; i != idx; i += inc)
    {
        int16_t x0 = CosCenteredDeg2(i, l1);
        int16_t y0 = SinCenteredDeg2(i, l1);
        int16_t x1 = CosCenteredDeg2(i + step, l1);
        int16_t y1 = SinCenteredDeg2(i + step, l1);
        int16_t xe = CosDeg2(i, w);
        int16_t ye = SinDeg2(i, w);

        MYUCG->drawTetragon(x0, y0, x1, y1, x1 + xe, y1 + ye, x0 + xe, y0 + ye);
        if (std::abs(inc) > 1 && std::abs(i - idx) < 8)
        {
            inc /= std::abs(inc);;
            step /= std::abs(step);
        }
    }
    old = idx;
}

// Draw scale label numbers for -_range .. _range w/o sign
// val: in [rad]
// labl: label value
void PolarGauge::drawOneLabel(float val, int16_t labl) const
{
    // ESP_LOGI( FNAME,"drawOneLabel val %.2f, label %d", val, labl );
    int16_t x, y;
    char s[10];
    if ( _flavor == XCVPRO) {
        float to_side = 0.05;
        float incr = (_scale_max - std::abs(val)) * 2; // increase pos towards 0
        int16_t pos = _radius + 12 + (int)incr - 3;
        if (val > 0)
        {
            to_side += incr / ((My_PIf / 2.f) * 80);
        }
        else
        {
            to_side = -to_side;
            to_side -= incr / ((My_PIf / 2.f) * 80);
        }
        // ESP_LOGI( FNAME,"drawOneLabel val:%.2f label:%d  toside:%.2f inc:%.2f", val, labl, to_side, incr );
        x = CosCentered(val + to_side, pos + (2 * incr));
        y = SinCentered(val + to_side, pos + (2 * incr));

        MYUCG->setColor(COLOR_LBBLUE);
    }
    else {
        std::sprintf(s, "%d", std::abs(labl));
        MYUCG->setFont(ucg_font_fub20_hn, false);
        x = CosCenteredDeg2(dice_rad(val), _radius - 16) - MYUCG->getStrWidth(s)/2;
        y = SinCenteredDeg2(dice_rad(val), _radius - 16);
        MYUCG->setColor(COLOR_WGREY);
    }
    MYUCG->setPrintPos(x, y);
    MYUCG->print(s);
}
void PolarGauge::drawDirLabel(int16_t deg2, const char *labl) const
{
    MYUCG->setColor(COLOR_LGREY);
    MYUCG->setFont(ucg_font_fub20_hn, false);
    int16_t x = CosCenteredDeg2(deg2, _radius + 4) - MYUCG->getStrWidth(labl)/2;
    int16_t y = SinCenteredDeg2(deg2, _radius + 4);
    MYUCG->setPrintPos(x, y);
    MYUCG->print(labl);
}


void PolarGauge::colorRange(float from, float to, int16_t color)
{
    int16_t ol = dice_up(from);
    drawBow(dice_up(to), ol, -10, 0, color);
}

// draw a scale from _range down to _mrange
// with radius _radius
// to _ref center [pixel]
// and zero label offset according to (_mrange + _range) / 2
// optional: (when from < -1000)
// refresh small area from [scale], to [scale] ( from > to .. always)
void PolarGauge::drawScale(float from, float to)
{
    // line width in pixel
    int16_t w1 = 1, w2 = 2, w3 = 3;
    if ( _flavor == CLUB ) {
        w1 = 0;
        w2 = 0;
        w3 = 6;
    }
    // line density on outer scale area
    int16_t modulo = (_range > 10) ? 20 : (_range < 6) ? 5 : 10; // typically start in "no details" area

    // for larger ranges put at least on extra label in the middle of each half scale
    int16_t mid_lpos_upper = fast_iroundf(func->invers(0.5 * (*func)(_range))) * 10;
    mid_lpos_upper = (mid_lpos_upper / modulo) * modulo; // round down to the next modulo hit
    int16_t mid_lpos_lower = fast_iroundf(func->invers(0.5 * (*func)(_mrange))) * 10;
    mid_lpos_lower = (mid_lpos_lower / modulo) * modulo;
    ESP_LOGI(FNAME, "range %f/%f lines go m%d %d %d", _range, _mrange, modulo, _dist05, mid_lpos_upper);

    // increment in 1/10 scale steps
    int16_t start = fast_iroundf(_range)*10, stop = fast_iroundf(_mrange)*10;
    int16_t l_start = start, l_stop = stop; // for labels
    if (from > -1000.)
    {
        // partial scale repainting
        start = (int)(clipValue(from) * 10) + 2; // alias .2
        stop = (int)(clipValue(to) * 10) - 2;
        if (std::abs(start) <= 10) {
            modulo = (_dist05 > 24) ? 1 : (_dist05 > 16) ? 2 : (_dist05 > 8) ? 5 : 10;
        }
    }
    else {
        // put scale unit on to of the last scale
        int16_t ival = dice_up(_range);
        int16_t x0 = CosCenteredDeg2(ival, _radius+30) - MYUCG->getStrWidth(VarioUnit->getName());
        int16_t y0 = SinCenteredDeg2(ival, _radius+30);
        MYUCG->setFont(ucg_font_fub11_hr);
        MYUCG->setPrintPos(x0, y0);
        MYUCG->setColor(COLOR_HEADER);
        MYUCG->print(VarioUnit->getName());
    }

    MYUCG->setFontPosCenter();
    MYUCG->setFont(ucg_font_fub14_hn);
    ESP_LOGI(FNAME, "scale from %d to %d", start, stop);
    bool draw_label = false;
    int middleat = 10 * func->getZero();
    for (int16_t a = start; a >= stop; a--)
    {

        if (a == middleat + 10 || a == middleat - 10)
        {
            draw_label = true;
            if (a > 0)
            {
                modulo = (_dist05 > 24) ? 1 : (_dist05 > 16) ? 2 : (_dist05 > 8) ? 5 : 10; // go into the details around zero
            }
            else
            {
                modulo = (_range > 10) ? 20 : (_range < 6) ? 5 : 10; // leave the details around zero
            }
        }
        if ( a == 4 ) {
            int16_t tmp = dice_rad((*func)(static_cast<float>(0.f)));
            drawDirLabel(tmp + 20, "+");
        }
        else if ( a == -4 ) {
            int16_t tmp = dice_rad((*func)(static_cast<float>(0.f)));
            drawDirLabel(tmp - 16, "-");
        }

        if (!(a % modulo))
        {
            float val = (*func)(static_cast<float>(a) / 10.);
            // a line to draw
            int16_t width = w1; // .1 little/short lines
            int16_t end = _radius + 7;

            if (!(a % 5))
            {
                // .5 small line
                width = w2;
                end = _radius + (_flavor == CLUB ? 24 : 12);
            }

            if (!(a % 10))
            {
                // every integer big line
                if (modulo < 11 || a == l_start || a == l_stop || a == mid_lpos_upper || a == mid_lpos_lower || a == middleat)
                {
                    width = w3;
                    end = _radius + 15 + (_flavor == CLUB ? 7 : 0);
                }
                draw_label = a != middleat && (draw_label || _range < 5. || a == mid_lpos_upper || a == mid_lpos_lower || a == l_start || a == l_stop);
            }
            ESP_LOGI(FNAME, "lines a:%d end:%d label: %d  width: %d", a, end, draw_label, width );

            if (width) {
                drawOneScaleLine(val, end, width, width-1);
                if (draw_label || _flavor == CLUB)
                {
                    drawOneLabel(val, a / 10);
                }
            }
            draw_label = false;
        }
    }
    if (_flavor == XCVPRO) {
        int16_t prev = dice_up(start/10.) +12; // overdraw a bit
        int16_t to = dice_up(stop/10.) -12;
        if (start > 0)
        {
            drawBow(to, prev, _radius - _arrow->getBase(), 0);
        }
        else
        {
            // draw bow towards zero to get the background color
            drawBow(prev, to, _radius - _arrow->getBase(), 0);
        }
    }
    MYUCG->setFontPosBottom();
}

void PolarGauge::drawScaleBottom()
{
    // Area of -60° to -120° is concerned
    // start repaint from /60def onwards to the end of the scale
    ESP_LOGI(FNAME, "restore scale at: %f", func->invers(deg2rad(-60.)) );
    drawScale( func->invers(deg2rad(-60.)), _mrange );
}

// a [deg]; 0° ref on top
void PolarGauge::drawOneDot(int16_t a, int16_t size, int16_t cidx) const
{
    int16_t bx = fast_iroundf(fast_sin_idx(a*2) * _radius);
    int16_t by = fast_iroundf(fast_cos_idx(a*2) * _radius);
    MYUCG->setColor(lne_color[cidx].color[0], lne_color[cidx].color[1], lne_color[cidx].color[2]);
    MYUCG->drawDisc(_ref.x + bx,_ref.y - by, size, UCG_DRAW_ALL );
}

void PolarGauge::drawRose(int16_t at) const
{
    int16_t start = _range - 10.f;
    int16_t stop = 0;
    if (at != -1000)
    {
        // partial scale repainting
        start = at + 15; // 30° range
        stop = at - 15;
        start = (start/10)*10; // iterate on 10° raster
        // stop = (stop/10)*10;
    }
    for (int16_t a = start; a >= stop; a-=10)
    {
        // ESP_LOGI(FNAME, "dot a:%d", a%360);
        if ( a == 0 ) {
            MYUCG->setColor(COLOR_LBBLUE);
            // Draw a blue triangle for heading-up, or N for north-up
            if ( _wind_ref == WR_NORTH) {
                MYUCG->setFont(ucg_font_fub11_hr);
                char c = 'N';
                int16_t w2 = MYUCG->getCharWidth(c)/2;
                MYUCG->setPrintPos(_ref.x-w2+1, _ref.y-_radius+9);
                MYUCG->print(c);
            }
            else {
                MYUCG->drawTriangle(_ref.x,_ref.y-_radius-6, _ref.x-4, _ref.y-_radius+2, _ref.x+4, _ref.y-_radius+2);
            }
        }
        else if (!(a % 30)) {
            drawOneDot( a, 2, 1);
        }
    }
}

void PolarGauge::clearGauge()
{
    int16_t tmp = 0;
    drawBow(720, tmp, 15, 8, -1 );
    // remove last indicator print
    if (_wind_live) {
        _wind_live->drawWind(true);
    }
    if (_wind_avg) {
        _wind_avg->drawWind(true);
    }
}

////////////////////////////
// trigonometric helpers for gauge painters

// get sin/cos position from gauge index in radian with gauge centered mapping
int16_t PolarGauge::SinCentered(float val, int16_t len) const {
    return _ref.y - static_cast<int16_t>(fast_iroundf(fast_sin_rad(val) * len));
}
int16_t PolarGauge::CosCentered(float val, int16_t len) const { return _ref.x - static_cast<int16_t>(fast_iroundf(fast_cos_rad(val) * len)); }
// based on discrete integral values with 0.5deg resolution
int16_t PolarGauge::SinCenteredDeg2(int16_t val, int16_t len) const { return _ref.y - fast_iroundf(fast_sin_idx(val) * len); }
int16_t PolarGauge::CosCenteredDeg2(int16_t val, int16_t len) const { return _ref.x - fast_iroundf(fast_cos_idx(val) * len); }
int16_t PolarGauge::Sin(float val, int16_t len) const { return static_cast<int16_t>(fast_iroundf(fast_sin_rad(val) * len)); }
int16_t PolarGauge::Cos(float val, int16_t len) const { return static_cast<int16_t>(fast_iroundf(fast_cos_rad(val) * len)); }
int16_t PolarGauge::SinDeg2(int16_t val, int16_t len) const { return fast_iroundf(fast_sin_idx(val) * len); }
int16_t PolarGauge::CosDeg2(int16_t val, int16_t len) const { return fast_iroundf(fast_cos_idx(val) * len); }
