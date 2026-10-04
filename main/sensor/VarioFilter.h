
#pragma once

#include "SensorBase.h"
#include "Filters.h"

#include <driver/gpio.h>


//
// a virtual sensor creating a total energy compensated altimeter trace
// it is used on a master device to read the real pressure data 
// and create a total energy compensated altitude trace form it.
// A client device does not have it.
//
class TEcompFilter final : public SensorTP<meter_t> {
   public:
    TEcompFilter();
    ~TEcompFilter() = default;
    
    const char* name() const override { return "TeComp"; }
    bool probe() override { return true; }
    bool setup() override { return true; }

    bool doRead(meter_t& val) override;

   private:
    LowPassFilterT<float> _tealt_lpf;
};


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

    void postProcess() override;

    void prepareForSimJump() { _prepare_jump = 40; } // prepare for a disruptive jump in altitude in simulation mode
    mps_t getAvgVario() const { return _avg_vario.get(); }
    float getPolarSink() const { return _polar_sink; }
    bool gotPositive() const { return _got_positive; }

   private:
    void init(meter_t alt);

    LeakyIntegratorT<float> _Gact;
    LeakyIntegratorT<float> _Goptimal;
    uint32_t _prev_time = 0;
    LowPassFilterT<mps_t> _avg_vario;
    mps_t _polar_sink = 0.f;
    int8_t _prepare_jump = 0;
    int8_t _got_positive : 1 = 0;
};

extern TEcompFilter *tecompSensor;
extern VarioFilter *varioSensor;
