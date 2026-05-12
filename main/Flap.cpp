
#include "Flap.h"

#include "glider/Polars.h"
#include "driver/gpio/AnalogInput.h"
#include "setup/SetupNG.h"
#include "sensor/imu/AccMPU6050.h"
#include "math/Floats.h"
#include "sensor.h"
#include "logdefnone.h"

#include <array>
#include <algorithm>


constexpr mps_t GENERAL_V_MIN = Units::kmh_to_mps(50);

Flap* Flap::_instance = nullptr;
Flap* FLAP = nullptr;
const FlapLevel Flap::dummy = {.0, (int)('x'<<24), 0};

// Center responsibility to store the flap settings permanetly here
// ================================================================
// all nvs setup data enumerable
struct FLConf
{
    SetupNG<kmh_t> *speed;
    SetupNG<int>   *label;
    SetupNG<int>   *sensval;
    constexpr FLConf(SetupNG<kmh_t> *s, SetupNG<int> *l, SetupNG<int> *sv) : speed(s), label(l), sensval(sv) {}
    const char *getLabel() const {
        return (const char *)(label->getPtr());
    }
    int getLabelInt() const {
        return label->get();
    }
};

// storage of all flap configuration entries
static const std::array<FLConf, Flap::MAX_NR_POS> FL_STORE = {{
    {&wk_speed_0, &wk_label_0, &wk_sens_pos_0},
    {&wk_speed_1, &wk_label_1, &wk_sens_pos_1},
    {&wk_speed_2, &wk_label_2, &wk_sens_pos_2},
    {&wk_speed_3, &wk_label_3, &wk_sens_pos_3},
    {&wk_speed_4, &wk_label_4, &wk_sens_pos_4},
    {&wk_speed_5, &wk_label_5, &wk_sens_pos_5},
    {&wk_speed_6, &wk_label_6, &wk_sens_pos_6}}};

///////////////////////////////////////
// Flap class implementation
Flap::Flap() {
    configureADC();
    initFromNVS();
    prepLevels();
    FLAP = this;
}
Flap::~Flap() {
    if (sensorAdc) {
        delete sensorAdc;
        sensorAdc = nullptr;
    }
    _instance = nullptr;
    FLAP = nullptr;
}
Flap *Flap::theFlap() {
    if (!_instance) {
        _instance = new Flap();
    }
    return _instance;
}

// setup API
SetupNG<kmh_t> *Flap::getSpeedNVS(int idx)
{
    return FL_STORE[idx].speed;
}
SetupNG<int> *Flap::getLblNVS(int idx)
{
    return FL_STORE[idx].label;
}
SetupNG<int>   *Flap::getSensNVS(int idx)
{
    return FL_STORE[idx].sensval;
}

void Flap::setFromFlapTable(const flap_table& fte) {
    for (int i = 0; i < MAX_NR_POS; i++) {
        getSpeedNVS(i)->set(static_cast<kmh_t>(fte.speeds[i]), true, false);
        int tmp;
        std::strncpy((char *)&tmp, fte.labels[i].data(), 4);
        getLblNVS(i)->set(tmp, true, false);
        ESP_LOGI(FNAME, "set flap level %d: speed %d, label %s", i, fte.speeds[i], fte.labels[i].data());
    }
    initFromNVS();
    prepLevels();
}

void Flap::setSensCal(int idx, int val) {
    if (idx < flevel.size()) {
        flevel[idx].sensval = val;
    }
}
void Flap::setLabel(int idx, const char *lab) {
    if (idx < flevel.size()) {
        std::strncpy(flevel[idx].label, lab, 4);
        flevel[idx].label[3] = '\0';
    }
}
void Flap::setSpeed(int idx, kmh_t spd) {
    if (idx < flevel.size()) {
        flevel[idx].nvs_speed = spd;
    }
}

void Flap::prepLevels()
{
    if ( flevel.size() > 0 )
    {
        // adapt speeds to actual wingload
        for ( FlapLevel &fl : flevel ) {
            fl.prep_speed = Units::kmh_to_mps(fl.nvs_speed) * std::sqrtf( (ballast.get()+100.0) / 100.0 );
            ESP_LOGI( FNAME, "Adjusted flap speed %.1f", fl.prep_speed );
        }

        // some precalculations for the flap levels
        _sens_order = flevel[0].sensval > flevel.back().sensval;
        FlapLevel vne(v_max.get(), 0, 0);
        vne.prep_speed = Units::kmh_to_mps(vne.nvs_speed);
        vne.sensval = flevel[0].sensval;
        FlapLevel *prev = &vne;
        int sdelta = 0;
        float vdelta = 0.0f;
        for (FlapLevel &fl : flevel) { // iterate 0, 1, ..
            sdelta = fl.sensval - prev->sensval;
            if (sdelta == 0) {
                sdelta = _sens_order ? -1 : 1; // avoid zero deltas
            }
            fl.sens_delta = sdelta;
            vdelta = fl.prep_speed - prev->prep_speed;
            if (vdelta > -1.f ) {
                vdelta = -1.f;
            }
            fl.speed_delta = vdelta;
            ESP_LOGI( FNAME, "sens delta %d, vdelta %.1f", sdelta, vdelta);
            prev = &fl;
        }
    }
}

// setup actions
void Flap::modLevels()
{
    // keep sorted by speed
    std::sort(flevel.begin(), flevel.end(), [](const FlapLevel &a, const FlapLevel &b) {
        return a.nvs_speed > b.nvs_speed;
    });
    ESP_LOGI(FNAME, "Flap levels sorted");
    saveToNVS();
    prepLevels();
}
void Flap::reLoadLevels()
{
    initFromNVS();
    prepLevels();
}
void Flap::addLevel(FlapLevel &lev)
{
    flevel.push_back(lev);
    modLevels();
}
void Flap::removeLevel(int idx)
{
    flevel.erase(flevel.begin() + idx);
    saveToNVS();
    prepLevels();
}


// 10 Hz update
void Flap::progress(int count) {
    if ( sensorAdc ) {
        int wkraw = std::clamp(getSensorRaw(), -1, 4096);
        if (wkraw < 0) {
            // drop erratic negative readings
            ESP_LOGW(FNAME, "negative flap sensor reading: %d", wkraw);
            return;
        }
        // ESP_LOGI(FNAME,"flap sensor =%d", wkraw );
        rawFiltered = rawFiltered + (wkraw - rawFiltered) / 4;
        if (!(count % 5)) { // 2 Hz
            float lever = sensorToLeverPosition(rawFiltered);
            // ESP_LOGI(FNAME, "wk sensor=%1.2f  raw=%d", lever, rawFiltered);
            if (lever < 0.) {
                lever = 0.;
            } else if (lever > flevel.size() - 1) {
                lever = flevel.size() - 1;
            }

            if ((int)(flap_pos.get() * 10) != (int)(lever * 10)) {
                flap_pos.set(lever); // update secondary vario
                // ESP_LOGI(FNAME, "wk sensor=%1.2f  raw=%d", lever, rawFiltered);
            }
        }
    }
}

/////////////////////////////////////////////////
// the core API functions for flap recommendations
/////////////////////////////////////////////////

//
// all of them have to cope with an optional empty or missing flap level definition !
//

// get optimum flap position for given speed
float Flap::getOptimum(mps_t spd) const {
    // Correct for current g load
    float g_force = accSensor ? accSensor->getGload() : 1.f;
    if (g_force < 0.3) {
        g_force = 0.3; // Ignore meaningless values below 0.3g
    }
    mps_t g_speed = spd / sqrtf(g_force); // reduce current speed, instead of increase switch points
    // ESP_LOGI(FNAME, "g force: %.1f, g corrected speed: %3.1f", g_force, g_speed);

    int wki = 0; // find the min speed flap index
    for (auto &l : flevel) {
        if (g_speed > l.prep_speed) {
            break;
        }
        wki++;
    }
    if (wki >= flevel.size()) {
        if ( flevel.size() == 0 ) {
            return 0.f;
        }
        wki = flevel.size() - 1;
    }

    // correct above GENERAL_V_MIN, but not below (too far extrapolated)
    float wkf = wki + 0.5 + (g_speed - flevel[wki].prep_speed) / flevel[wki].speed_delta;
    if (g_speed < GENERAL_V_MIN) {
        wkf = flap_takeoff.get();
    } else {
        wkf = std::clamp(wkf, -0.1f, (float)(flevel.size() - 1));
    }
    ESP_LOGI(FNAME, "opt: g-ias:%.1fmps wki:%d wkf:%.1f (p%.1fd%.1f)", g_speed, wki, wkf, flevel[wki].prep_speed, flevel[wki].speed_delta);
    return wkf;
}

int Flap::getWkIndex(float wkf) const {
    if ( flevel.size() == 0 ) { return -1; }
    return std::clamp((int)(wkf + 0.5), 0, (int)flevel.size()-1);
}

// get speed band for given flap position wk
// 0 < wk < (# positions - 1)
mps_t Flap::getSpeedBand(float wkf, mps_t &maxv) const
{
    mps_t minv = 0.;
    maxv = 0.;

    // pick min/max speeds for given flap position index
    int wki = getWkIndex(wkf);
    if ( wki < 0 ) {
        return 0.;
    }

    minv = flevel[wki].prep_speed;
    if( wki == 0 ) {
        maxv = v_max.get(); // upper speed end
    }
    else {
        maxv = flevel[wki-1].prep_speed;
        // push speed band according to fractional flap position
        // assuming linear interpolation between flap positions
        // map band so that at a full flap position
        // the speed band is centered between the two flap speeds (no shift)
        mps_t shift = (wkf - wki) * flevel[wki].speed_delta;
        minv += shift;
        maxv += shift;
        ESP_LOGI(FNAME,"shift:%.1f center speed:%.1fmps", shift, (minv + maxv)/2);
    }
    ESP_LOGI(FNAME,"wki:%d wkf:%.1f minv:%.1f maxv:%.1f", wki, wkf, minv, maxv);

    return minv;
}

mps_t Flap::getSpeed(float wkf) const
{
    int wki = getWkIndex(wkf);
    if ( wki < 0 ) {
        return 0.f;
    }

    // speed from prep_speed@(wki+0.5) up to prep_speed+speed_delta@(wki-0.5)
    float ret = flevel[wki].prep_speed + (wkf - wki) * flevel[wki].speed_delta;
    ESP_LOGI(FNAME, "getspeed: %.1f (f%.1fi%d)", ret, wkf, wki);
    return ret;
}

float Flap::getFlapPosition() const
{
    return flap_pos.get();
}

//////////////////////////////////
// sensor access
//////////////////////////////////

// sudo wrapper to the flap_sensor setup variable considering also the peer capabilities
bool Flap::sensAvailable()
{
    return flap_sensor.get() || (peer_caps.get() & XcvCaps::FLAPSENS_CAP);
}

// create the optional flap sensor
void Flap::configureADC() {
    ESP_LOGI(FNAME, "Flap::configureADC");
    if (sensorAdc) {
        delete sensorAdc;
        sensorAdc = nullptr;
    }

    if ( flap_sensor.get() ) {
        // nonzero -> configured, only one port needed for XCV23+ HW
        sensorAdc = new AnalogInput(-1, ADC_CHANNEL_6);
    }
    if (sensorAdc != 0) {
        ESP_LOGI(FNAME, "Flap sensor properly configured");
        sensorAdc->begin(ADC_ATTEN_DB_0, ADC_UNIT_1, false);
        delay(10);
        uint32_t read = sensorAdc->getRaw();
        if (read == 0 || read >= 4096) { // try GPIO pin 34, series 2021-2
            ESP_LOGI(FNAME, "Flap sensor not found or edge value, reading: %d", (int)read);
        } else {
            ESP_LOGI(FNAME, "Flap sensor looks good, reading: %d", (int)read);
        }
    } else {
        ESP_LOGI(FNAME, "Sensor ADC NOT properly configured");
    }
}

int Flap::getSensorRaw() const
{
    return sensorAdc ? sensorAdc->getRaw() : 0;
}


////////////////////////////////////////////
// private helper routines
////////////////////////////////////////////

// returns true for old setup migration
bool Flap::initFromNVS()
{
    // Capture configured flap positions
    flevel.clear();
    for (int i = 0; i < MAX_NR_POS; i++)
    {
        kmh_t nvsspeed = FL_STORE[i].speed->get();
        if (nvsspeed > 0)
        {
            // a valid entry
            flevel.push_back(FlapLevel{nvsspeed, FL_STORE[i].getLabelInt(), FL_STORE[i].sensval->get()});
            ESP_LOGI( FNAME, "new flap level %d %s: %.1f (%d)", i, FL_STORE[i].getLabel(), nvsspeed, FL_STORE[i].sensval->get() );
        }
    }

    ESP_LOGI(FNAME, "found %d flap levels", (int)flevel.size());
    return false;
}


void Flap::saveToNVS()
{
    // go through all levels and write back when changed
    // sensor values are not touched
    for (int i = 0; i < MAX_NR_POS; i++)
    {
        if ( i < (int)flevel.size() )
        {
            // speed
            if ( ! floatEqualFast(FL_STORE[i].speed->get(), flevel[i].nvs_speed) ) {
                FL_STORE[i].speed->set( flevel[i].nvs_speed, true, false ); // synch, but no action
            }
            // label
            if ( FL_STORE[i].getLabelInt() != flevel[i].label_int ) {
                FL_STORE[i].label->set( flevel[i].label_int, true, false ); // synch, but no action
            }
            // sensor cal value
            if ( FL_STORE[i].sensval->get() != flevel[i].sensval ) {
                FL_STORE[i].sensval->set( flevel[i].sensval, false, false ); // no synch, no action
            }
        }
        else {
            FL_STORE[i].speed->set(0);
            FL_STORE[i].label->set(0);
            FL_STORE[i].sensval->set(0);
        }
    }
}


float Flap::sensorToLeverPosition( int val ) const
{
    int wk = flevel.size()-1;
    if ( wk > 0 ) {
        for (int i = 0; i < flevel.size(); i++)
        {
            if (_sens_order) {
                // sensor readings going down with increasing flap index
                if (val > flevel[i].sensval) {
                    wk = i;
                    break;
                }
            }
            else {
                if (val < flevel[i].sensval) {
                    wk = i;
                    break;
                }
            }
        }
        float wkf = wk + (float)(val - flevel[wk].sensval) / flevel[wk].sens_delta;
        // ESP_LOGI(FNAME,"getLeverPos(%d): wk: %d, cal %d, delta %d, frac: %1.2f ", val, wk, flevel[wk].sensval, flevel[wk].sens_delta, (float)(val - flevel[wk].sensval) / flevel[wk].sens_delta);
        return wkf;
    }
    return 0.;
}
