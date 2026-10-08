
#pragma once

#include "SensorBase.h"
#include "Filters.h"


//
// Stable Vario with Kalman filter post processing
// Individual damping for master and client devices
//
class VarioFilter final : public SensorTP<mps_t> {
   public:
    VarioFilter();
    ~VarioFilter() = default;
    
    const char* name() const override { return "Vario"; }
    bool probe() override { return true; }
    bool setup() override;
    void setECompAdjust(float adjust) { _eCompAdjust = adjust / 100.f; }

    void postProcess() override;

    void prepareForSimJump() { _prepare_jump = 3; } // prepare for a disruptive jump in altitude in simulation mode
    mps_t getAvgVario() const { return _avg_vario.get(); }
    float getPolarSink() const { return _polar_sink; }
    bool gotPositive() const { return _got_positive; }

   private:
    void init(meter_t alt);

    LeakyIntegratorT<float> _Gact; // experimental thermal score
    LeakyIntegratorT<float> _Goptimal;

    uint32_t _prev_time = 0;
    LowPassFilterT<mps_t> _vario_lpf;
    LowPassFilterT<mps_t> _avg_vario;
    float _eCompAdjust = 1.f;
    mps_t _polar_sink = 0.f;
    int8_t _prepare_jump = 0;
    int8_t _got_positive : 1 = 0;
};

extern VarioFilter *varioSensor;
