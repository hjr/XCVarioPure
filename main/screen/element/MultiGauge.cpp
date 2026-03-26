/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "MultiGauge.h"

#include "Colors.h"
#include "math/Floats.h"
#include "sensor/press_diff/AirspeedSensor.h"
#include "setup/SetupNG.h"
#include "math/Units.h"
#include "AdaptUGC.h"
#include "logdefnone.h"

#include <cstdio>
#include <cstring>


extern AdaptUGC *MYUCG;

MultiGauge::MultiGauge(int16_t cx, int16_t cy, MultiDisplay d) :
    ScreenElement(cx, cy),
    _display(d)
{
    update_nvs();
}

void MultiGauge::setDisplay(MultiDisplay d)
{
    _display = d;
    update_nvs();
}

// accepts speed in kmh IAS/TAS, translates into configured unit
// right-aligned to value
void MultiGauge::draw()
{
    float fval = 0;
    if ( ! _nvsvar->getValid() ) {
        _dirty = true;
    }
    else {
        switch (_display) {
        case GAUGE_IAS_SPEED:
        case GAUGE_TAS_SPEED:
        case GAUGE_GND_SPEED:
        case GAUGE_S2F:
            fval = SpeedUnit->apply(_nvsvar->get());
            break;
        case GAUGE_NETTO:
            fval = VarioUnit->apply(_nvsvar->get()) * 10.f;
            break;
        case GAUGE_HEADING:
        case GAUGE_SLIP:
            fval = Units::rad_to_deg(_nvsvar->get());
            break;
        default:
            fval = _nvsvar->get();
            break;
        }
    }
    int val = fast_iroundf(fval);

    if (val_prev == val && ! _dirty) return;
    ESP_LOGI(FNAME, "draw val %d (old: %d)", val, val_prev);

    MYUCG->setColor(COLOR_WHITE);
    MYUCG->setFont(ucg_font_fub25_hn, true);

    char s[32] = {"   ---"};
    if (_nvsvar->getValid()) {
        if (vario_upper_gauge.get() == GAUGE_SLIP || vario_upper_gauge.get() == GAUGE_NETTO ) {
            sprintf(s, "  %.1f", fval);
        } else {
            // here we have only positive values
            if ( val >= 0 ) {
                sprintf(s, "  %3d", val);
            }
        }
    } else {
        ESP_LOGI(FNAME, "nvs val not valid");
    }

    MYUCG->setPrintPos(_ref_x - MYUCG->getStrWidth(s), _ref_y);
    MYUCG->print(s);

    val_prev = val;
    _dirty = false;
}

void MultiGauge::drawUnit() const
{
    MYUCG->setFont(ucg_font_fub11_hr);
    MYUCG->setColor( COLOR_HEADER );
    const char *unit_str = "";
    const char *mode_str = nullptr;
    switch (_display) {
    case GAUGE_GND_SPEED:
        mode_str = "GndV";
        [[fallthrough]];
    case GAUGE_S2F:
        if (!mode_str) mode_str = "S2F";
        [[fallthrough]];
    case GAUGE_IAS_SPEED:
        if (!mode_str) mode_str = "IAS";
        [[fallthrough]];
    case GAUGE_TAS_SPEED:
        if (!mode_str) mode_str = "TAS";
        unit_str = SpeedUnit->getName();
        break;
    case GAUGE_NETTO:
        mode_str = "NET";
        unit_str = VarioUnit->getName();
        break;
    case GAUGE_HEADING:
        mode_str = "HDG";
        [[fallthrough]];
    case GAUGE_SLIP:
        if (!mode_str) mode_str = "SLIP";
        unit_str = "deg";
        break;
    default:
        mode_str = "";
        break;
    }
    MYUCG->setPrintPos(_ref_x+5,_ref_y-3);
    MYUCG->print(unit_str);
    MYUCG->setPrintPos(_ref_x+5,_ref_y-17);
    MYUCG->print(mode_str);
}

void MultiGauge::update_nvs()
{
    switch (_display) {
    case MultiGauge::GAUGE_IAS_SPEED:
        _nvsvar = &ias;
        break;
    case MultiGauge::GAUGE_TAS_SPEED:
        _nvsvar = &tas;
        break;
    case MultiGauge::GAUGE_GND_SPEED:
        _nvsvar = &gnd_speed;
        break;
    case MultiGauge::GAUGE_S2F:
        _nvsvar = &s2f_ideal;
        break;
    case MultiGauge::GAUGE_NETTO:
        _nvsvar = &te_netto;
        break;
    case MultiGauge::GAUGE_HEADING:
        _nvsvar = &mag_hdt;
        break;
    case MultiGauge::GAUGE_SLIP:
        _nvsvar = &slip_angle;
        break;
    // case GAUGE_TRACK:
    // 	_nvsvar = &;
    // 	break;
    default:
        _nvsvar = &ias;
        break;
    }
}