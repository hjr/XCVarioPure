
#include "mp50040p.h"

#include "../adc/mcp3221.h"
#include "logdefnone.h"

#include <I2Cbus.hpp>

constexpr float MP50040P_CORR = 5000.0/4096.0;  // according to above formula, this is the relation between adc readout and Pascal

const float min_pascal = 10.0;

// Analog / Digital conversion MP3V5004DP:
// - MP3V5004DP Vout -> (0.2*VS) - VS = 0.66..3.3V
// - MCP3221    0...4095 (12 bit)
// - VS = 3.3 V

// 1)
// V = VS*[(0.2*P) +0.2]
// adc = 4096* Vout/VS
// Vout = (adc/4096) * VS

// 2)
// after offset compensation:  ( P in kPA )
// Vout = VS*0.2*Pkpa
// P = Vout / VS*0.2


// 1) in 2)
// P = (adc / 4096)*VS / VS*0.2
// VS surplus:
// P = (adc /4096) / 0.2
// or
// P = adc * 5/4096     (kPa)
// or in Pascal:
// P = 5000/4096 * adc

// 100 mm H2O:  P = 1000 Pascal o. 0.6 V o. 0.2*4096 = 819.2


MP5004DP::MP5004DP() : AirspeedSensor(), _mcp(&i2c1)
{
    _multiplier = MP50040P_CORR;
    changeConfig();
}

bool MP5004DP::probe()
{
    if (_mcp.selfTest() != ESP_OK)
    {
        return false;
    }
    return _mcp.readVal() >= 0;
}

void MP5004DP::changeConfig()
{
    _multiplier = MP50040P_CORR * ((100.0 + speedcal.get()) / 100.0);
}

bool MP5004DP::fetch_pressure(int32_t &p, uint16_t &t)
{
    p = _mcp.readVal();
    return p != 0;
}


/*
 * Offset Voltage according to datasheet:
 *  0.45 0.6 0.75  V  @ 3V Vs
 *  15% min 20% (typ) 25% (max)
 *  @ 4096 full swing Vs adc val:
 *  614   812   1024 + add +-1% for ADC
 */
bool MP5004DP::offsetPlausible(int32_t offset)
{
    ESP_LOGI(FNAME, "MP5004DP offsetPlausible( %d )", offset);
    constexpr int lower_val = 608;
    constexpr int upper_val = 1067;

    return (offset > lower_val) && (offset < upper_val);
}

int MP5004DP::getMaxACOffset()
{
    return 20;
}

