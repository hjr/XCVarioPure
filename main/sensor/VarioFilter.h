
#pragma once

#include "SensorBase.h"
#include "Filters.h"
#include "S2F.h"

#include <driver/gpio.h>


//
// Implementation of a stable Vario with flight optimized Iltis-Kalman filter
//

class PressureSensor;
class LowPassFilter;

constexpr const int FILTER_LEN = 34; // Max Filter length
constexpr const float ALPHA = 0.2; // Kalman Gain alpha
#define ERRORVAL 1.6             // damping Factor for values off the weeds
constexpr const float STANDARD = 1013.25; // ICAO standard pressure

class VarioFilter final : public SensorTP<float> {
   public:
    using TekComp_Type = enum : uint8_t { TE_TEK_PROBE, TE_TEK_EPOT, TE_TEK_PRESSURE, TE_TEK_IMU, TE_TEK_MAX_TYPES };

    VarioFilter();
    const char* name() const override { return "tek_vario"; }
    bool probe() override { return true; }
    bool setup() override;

    bool doRead(float& val) override;
    void postProcess() override;

    void inject(float te_alt);  // te raw feed from master
    void configChange();
    float getAvgVario() const { return _avg_vario; }
    // float readAVGalt() { return averageAlt; };    // get average Altitude
    // float readCuralt() { return _currentAlt; };   // get current Altitude

   private:
   int16_t _te_filter_idx;
    LowPassFilter _lpf{0.25f};
    int16_t _avg_filter_idx;
    // some usefull derived values
    float _avg_vario = 0.f;
};

extern VarioFilter* bmpVario;
