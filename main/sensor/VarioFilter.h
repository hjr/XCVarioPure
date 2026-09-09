
#pragma once

#include "SensorBase.h"
#include "Filters.h"
#include "S2F.h"
#include "average.h"

#include <driver/gpio.h>


#define FILTER 0

//
// Implementation of a stable Vario with flight optimized Iltis-Kalman filter
//

class VarioFilter final : public SensorTP<meter_t> {
   public:
    VarioFilter();
    const char* name() const override { return "tek_vario"; }
    bool probe() override { return true; }
    bool setup() override;

    bool doRead(meter_t& val) override;
    void postProcess() override;

    void configChange();
    void prepareForSimJump() { _prepare_sim_jump = 40; } // prepare for a disruptive jump in altitude in simulation mode
    float getAvgVario() const { return _avg_vario; }
    float getPolarSink() const { return _polar_sink; }
    bool gotPositive() const { return _got_positive; }

   private:
    void init(meter_t alt);

    LowPassFilterT<float> _tealt_lpf;
    LeakyIntegratorT<float> _Gact;
    LeakyIntegratorT<float> _Goptimal;
    uint32_t _prev_time = 0;
    Average<34, float, float> TEavg;
#if FILTER == 0
    Average<60, float, float> avgTE;
#endif
    LowPassFilterT<float> _lpf{0.25f};
    int16_t _avg_filter_idx;
    // some usefull derived values
    float _avg_vario = 0.f;
    float _polar_sink = 0.f;
    int8_t _prepare_sim_jump = 0;
    int8_t _got_positive : 1 = 0;
};

extern VarioFilter bmpVario;
