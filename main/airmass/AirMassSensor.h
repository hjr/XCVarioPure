#pragma once

#include "AirMassEKF.h"
#include "AirMassAttitude.h"
#include "AirMassFlaps.h"
#include "sensor/SensorBase.h"
#include "math/vector_3d.h"
#include "wind/Wind.h"

#include <cmath>

//
// Virtual sensor running the AirMass estimator in the sensor loop (master only).
// Publishes the vertical air mass movement and the instant wind on the black board,
// the 3D wind vector (NED) goes into the sensor history.
//
class AirMassSensor final : public SensorTP<vector_f>
{
public:
    AirMassSensor();
    virtual ~AirMassSensor();

    const char *name() const override { return "airmass"; }
    bool probe() override { return false; }
    bool setup() override;
    bool doRead(vector_f& val) override { return false; }
    void postProcess() override;

    void configChanged() { _config_changed = true; }
    const AirMassEKF& getEKF() const { return _ekf; }

private:
    void applyConfig();
    bool readBaroAltitude(float& alt) const;
    void start();
    void stop();
    void persistLearned();
    void publish();

    AirMassEKF _ekf;
    airmass::Attitude _att; // gyro propagated, air data aided: robust in pull-ups unlike the accelerometer referenced AHRS
    airmass::FlapOffsets _flaps; // pitch offset per flap position
    uint32_t _prev_time = 0;
    uint16_t _gnss_epoch = 0;
    uint16_t _count = 0;
    LowPassFilterT<vector_f> _avg_wind;
    LowPassFilterT<vector_f> _gyro_dot; // angular acceleration for the lever arm compensation
    vector_f _gyro_prev;
    bool _active = false;
    bool _config_changed = true;
};

extern AirMassSensor *airmassSensor;

// wind data from the air movement vector (north, east), the inverse of WindData::getVector()
inline WindData windFromVector(const vector_f& v)
{
    return WindData(std::atan2(-v.y, -v.x), std::hypot(v.x, v.y));
}
