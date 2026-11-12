
#pragma once

#include "SensorBase.h"
#include "Filters.h"
#include "time/Sampling.h"

#include <optional>


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
    std::optional<meter_t> teAltitude(bool te_probe) const;
    void filterAltitude(meter_t curr_altitude, second_t dt);
    void publish(mps_t v, int t, bool slow) {
        if (slow) {
            pushToHistory(v, t);
        }
        publishNVS(v);
    }

    SensorTP<pascal_t>* pSensor = nullptr; // te or baro sensor
    LeakyIntegratorT<float> _Gact; // experimental thermal score
    LeakyIntegratorT<float> _Goptimal;

    int _reading_ms = 0;
    SampleClock _source_clock;
    Periodic _slow_timer;
    mps_t _te_rate = 0.f;    // altitude rate of the last reading
    VarioDamper _te_damper;
    LowPassFilterT<mps_t> _avg_vario;
    float _eCompAdjust = 1.f;
    mps_t _polar_sink = 0.f;
    uint16_t count = 0;
    int8_t _prepare_jump = 0;
    int8_t _got_positive : 1 = 0;
};

extern VarioFilter *varioSensor;

