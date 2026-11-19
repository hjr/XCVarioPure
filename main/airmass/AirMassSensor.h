#pragma once

#include "AirMassEKF.h"
#include "AirMassPipeline.h"
#include "sensor/SensorBase.h"
#include "math/Quaternion.h"
#include "math/vector_3d.h"
#include "wind/Wind.h"

#include <atomic>
#include <cmath>
#include <string>

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

    const AirMassEKF& getEKF() const { return _pipe.ekf(); }
    float imuRoll() const { return _pipe.imuRoll(); }
    // instrument replay: restart for a disruptive jump of the replayed flight (any task)
    void prepareForSimJump() { _jump = true; }

private:
    airmass::Setup currentSetup() const;
    void applyConfig();
    bool readBaroAltitude(float& alt) const;
    void start(bool keep_learned = false);
    void turnBody();
    void stop();
    void persistLearned();
    void publish();
    void withdrawWind();

    // estimator, own attitude and flap offsets, the same code the host simulation and replay run
    airmass::Pipeline _pipe;
    uint16_t _gnss_epoch = 0;
    airmass::Periodic _setup_timer, _compass_timer, _persist_timer, _output_timer;
    airmass::Periodic _sync_timer, _wind_timer, _average_timer, _diagnostic_timer;
    airmass::SampleClock _compass_sample;
    bool _active = false;
    bool _recording = false;
    uint8_t _gust_msg = 0;
    int _sig_pitch = 0, _sig_speed = 0; // the installation signatures the running estimator started with
    Quaternion _ref; // the IMU reference of the estimator's body axes
    bool _average_published = false; // the synoptic wind on the blackboard is the average of the estimator
    uint32_t _last_ms = 0; // the clock of the last cycle
    std::atomic<bool> _jump{false}; // a replay jump requested
};

// Recording of the estimator (System > Logging > "All Sensor Data"): $PXCVA and the sentences of AirMassRecord.h
// but the input frames ($SENS has the sensor readings), queued by the sensor loop in the order they happened. Appends the queued lines, false when empty.
bool airmassNmeaDrain(std::string& lines);

extern AirMassSensor *airmassSensor;

// wind data from the air movement vector (north, east), the inverse of WindData::getVector()
inline WindData windFromVector(const vector_f& v)
{
    return WindData(std::atan2(-v.y, -v.x), std::hypot(v.x, v.y));
}
