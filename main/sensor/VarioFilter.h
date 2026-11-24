
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
    // Add to the indicator (history) value to get the gross climb.
    // HACK: the history records whatever the gauge shows (gross, netto or super netto, as per CRMOD),
    // so readers needing gross climb (ClimbStats, ThermalAssist) have to compensate with this offset,
    // which is only the latest one and not the one of each history sample.
    // TODO: record a fixed gross history instead and derive the shown value from it:
    //   needle te_vario = gross + display offset (0 | -polar_sink | -polar_sink + circle_sink),
    //   average figure  = getRunningAvg() + low pass filtered display offset (no restart on mode change),
    //   then drop this getter and use getAVG() directly in ClimbStats.
    mps_t getGrossOffset() const { return _gross_offset; }
    bool gotPositive() const { return _got_positive; }

   private:
    void init(meter_t alt);

    SensorTP<pascal_t>* pSensor = nullptr; // te or baro sensor
    LeakyIntegratorT<float> _Gact; // experimental thermal score
    LeakyIntegratorT<float> _Goptimal;

    uint32_t _prev_time = 0;
    LowPassFilterT<mps_t> _vario_lpf;
    LowPassFilterT<mps_t> _avg_vario;
    float _eCompAdjust = 1.f;
    mps_t _polar_sink = 0.f;
    mps_t _gross_offset = 0.f;
    uint16_t count = 0;
    int8_t _prepare_jump = 0;
    int8_t _got_positive : 1 = 0;
};

extern VarioFilter *varioSensor;

