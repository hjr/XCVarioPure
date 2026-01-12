
#include "Flap.h"

#include "AnalogInput.h"
#include "setup/SubMenuFlap.h"
#include "setup/SetupNG.h"
#include "sensor/imu/KalmanMPU6050.h"
#include "math/Floats.h"
#include "sensor.h"
#include "logdefnone.h"

#include <string_view>
#include <array>
#include <algorithm>


constexpr float GENERAL_V_MIN = 50;

Flap* Flap::_instance = nullptr;
Flap* FLAP = nullptr;
FlapLevel Flap::dummy = {.0, (int)('x'<<24), 0};

// Center responsibility to store the flap settings permanetly here
// ================================================================
// all nvs setup data enumerable
struct FLConf
{
    SetupNG<float> *speed;
    SetupNG<int>   *label;
    SetupNG<int>   *sensval;
    constexpr FLConf(SetupNG<float> *s, SetupNG<int> *l, SetupNG<int> *sv) : speed(s), label(l), sensval(sv) {}
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
    if (flap_sensor.get() & 0x3) {
        // migration from old settings with multiple IO's
        flap_sensor.set(FLAP_SENSOR_ENABLE);
    }
    configureADC();
    if ( initFromNVS() ) {
        // migrated from old settings may need to sort levels according speed.
        modLevels();
    }
    else {
        prepLevels();
    }
}
Flap::~Flap() {
    if (sensorAdc) {
        delete sensorAdc;
        sensorAdc = nullptr;
    }
    _instance = nullptr;
}
Flap *Flap::theFlap() {
    if (!_instance) {
        _instance = new Flap();
    }
    return _instance;
}

// setup API
SetupNG<float> *Flap::getSpeedNVS(int idx)
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
void Flap::setSpeed(int idx, float spd) {
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
            fl.prep_speed = fl.nvs_speed * std::sqrtf( (ballast.get()+100.0) / 100.0 );
            ESP_LOGI( FNAME, "Adjusted flap speed %.1f", fl.prep_speed );
        }

        // some precalculations for the flap levels
        _sens_order = flevel[0].sensval > flevel.back().sensval;
        FlapLevel* prev = nullptr;
        int sdelta = 0;
        float vdelta = 0.0f;
        for (FlapLevel &fl : flevel) { // iterate 0, 1, ..
            if (prev) {
                sdelta = fl.sensval - prev->sensval;
                if (sdelta == 0) {
                    sdelta = _sens_order ? -1 : 1; // avoid zero deltas
                }
                prev->sens_delta = sdelta;
                vdelta = fl.prep_speed - prev->prep_speed;
                if (vdelta > -1.f ) {
                    vdelta = -1.f;
                }
                prev->speed_delta = vdelta;
                ESP_LOGI( FNAME, "sens delta %d, vdelta %.1f", sdelta, vdelta);
            }
            prev = &fl;
        }
        // also define on last entry to e.g. extrapolate
        if (prev) {
            prev->sens_delta = sdelta;
            prev->speed_delta = vdelta;
            ESP_LOGI( FNAME, "sens delta %d, vdelta %.1f", sdelta, vdelta);
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
void Flap::progress() {
    if ( sensorAdc ) {
        int wkraw = getSensorRaw();
        if (wkraw > 4096)
            wkraw = 4096;
        if (wkraw < 0) {
            // drop erratic negative readings
            ESP_LOGW(FNAME, "negative flap sensor reading: %d", wkraw);
            return;
        }
        // ESP_LOGI(FNAME,"flap sensor =%d", wkraw );
        rawFiltered = rawFiltered + (wkraw - rawFiltered) / 4;
        tick++;
        if (!(tick % 5)) { // 2 Hz
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
// the core API functions for wk recommendations
/////////////////////////////////////////////////

float Flap::getOptimum(float spd) const {
    // Correct for current g load
    g_force += (IMU::getGliderAccelZ() - g_force) * 0.5;
    if (g_force < 0.3) {
        g_force = 0.3; // Ignore meaningless values below 0.3g
    }
    float g_speed = spd / sqrtf(g_force); // reduce current speed, instead of increase switch points
    // ESP_LOGI(FNAME, "g force: %.1f, g corrected speed: %3.1f", g_force, g_speed);

    int wki = 0; // find the wk index one index above the current speed
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
    else if (wki > 0) {
        wki--; // to get the speed index above
    }

    // correct above GENERAL_V_MIN, but not below (too far extrapolated)
    float wkf = wki + (g_speed - flevel[wki].prep_speed) / flevel[wki].speed_delta;
    if (g_speed < GENERAL_V_MIN) {
        wkf = flap_takeoff.get();
    } else if (wkf < 0.) {
        wkf = -0.1; // stop indicator a little beyond
    } else if (wkf > flevel.size() - 1) {
        wkf = flevel.size() - 1;
    }
    ESP_LOGI(FNAME, "opt: g-ias:%.1f wki:%d wkf:%.1f", g_speed, wki, wkf);
    return wkf;
}

static int getWkIndex(float wkf) {
     return std::max(0, fast_iroundf(wkf));
}

// get speed band for given flap position wk
// 0 < wk < (# positions - 1)
float Flap::getSpeedBand(float wkf, float &maxv) const
{
    float minv = 0.;
    maxv = 0.;

    // pick min/max speeds for given flap position index
    int wki = getWkIndex(wkf);
    if ( wki < flevel.size() ) {
        minv = flevel[wki].prep_speed;
        if( wki == 0 ) {
            maxv = v_max.get();
        }
        else {
            maxv = flevel[wki-1].prep_speed;
        }
        ESP_LOGI(FNAME,"wkf:%.1f minv:%.1f maxv:%.1f", wkf, minv, maxv);

        // push speed band according to fractional flap position
        // assuming linear interpolation between flap positions
        // map band so that at a full flap position
        // the speed band is centered between the two flap speeds (no shift)
        float shift = (wkf - wki) * flevel[wki].speed_delta;
        minv += shift;
        maxv += shift;
        ESP_LOGI(FNAME,"shift:%.1f center speed:%.1f", shift, (minv + maxv)/2);
    }
    return minv;
}

float Flap::getSpeed(float wkf) const
{
    int wki = getWkIndex(wkf);
    if ( wki < flevel.size() ) {
       return flevel[wki].prep_speed + (wkf - wki) * flevel[wki].speed_delta;
    }
    return 0.f;
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

unsigned int Flap::getSensorRaw() const
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
        float nvsspeed = FL_STORE[i].speed->get();
        if (nvsspeed > 0)
        {
            // a valid entry
            if ( i == MAX_NR_POS-1 && nvsspeed >= flevel.back().nvs_speed ) {
                // last entry must be slower than previous
                nvsspeed = std::max(flevel.back().nvs_speed - 20.0f, 0.f);
            }
            flevel.push_back(FlapLevel{nvsspeed, FL_STORE[i].getLabelInt(), FL_STORE[i].sensval->get()});
            ESP_LOGI( FNAME, "new flap level %d %s: %.1f (%d)", i, FL_STORE[i].getLabel(), nvsspeed, FL_STORE[i].sensval->get() );
        }
    }

    if ( flevel.size() == 0 && ! gflags.flaps_nvs_defined ) {
        // try to read from old settings
        const char *old_speed[] = {"FLAP_MINUS_3", "FLAP_MINUS_2", "FLAP_MINUS_1", "FLAP_0", "FLAP_PLUS_1", "FLAP_PLUS_2", ""};
        const char *old_lblidx[] = {"WKLM3", "WKLM2", "WKLM1", "WKL0", "WKLP1", "WKLP2", "WKLP3"};
        const char *old_sens[] = {"WKSM3", "WKSM2", "WKSM1", "WKSP0", "WKSP1", "WKSP2", "WKSP3"};
        // legacy predefined flap labels         // -9,..,-2,-1,+0,+1,+2,..,+9
        const std::string_view flap_labels[55] = { "-9", "-8", "-7", "-6", "-5", "-4", "-3", "-2", "-1", "+0",  // 9
            "+1", "+2", "+3", "+4", "+5", "+6", "+7", "+8", "+9",     // 18
            " 0", " 1", " 2", " 3", " 4", " 5", " 6", " 7", " 8", " 9", "10",  // 29
            "11", "12", "13", "14", "15", "16", "17", "18", "19", "20",   // 39
            " N", " L", " S", "3a", "3b", " A", "21", "22", "23", "24", "25", "26", "27", "T", "" };  // 55

        ESP_LOGI(FNAME, "migrating old flap settings");
        float flstart, flend;
        if ( SetupCommon::getOldFloat("FL_NEG_M", flstart) && SetupCommon::getOldFloat("FL_POS_M", flend) ) {
            // old settings with min/max flags
            const int end = std::min((int)(flend - flstart + 1), MAX_NR_POS);
            ESP_LOGI( FNAME, "found old flap range %.1f to %.1f, levels %d", flstart, flend, end ); 
            int old_iter = flstart + 3;
            for ( int i=0; i<end; i++ ) {
                float nvsspeed;
                bool good = SetupCommon::getOldFloat( old_speed[old_iter], nvsspeed );
                if ( ! good && old_iter == 6 ) {
                    nvsspeed = GENERAL_V_MIN; // last position default
                    good = true;
                }
                int lblidx;
                good &= SetupCommon::getOldInt( old_lblidx[old_iter], lblidx );
                int sensval;
                good &= SetupCommon::getOldInt( old_sens[old_iter], sensval );
                if ( good && lblidx >=0 && lblidx < 55 ) {
                    int ilabel;
                    std::strncpy((char*)&ilabel,flap_labels[lblidx].data(), 4);
                    ((char*)&ilabel)[3] = '\0';
                    flevel.push_back( FlapLevel{ nvsspeed, ilabel, sensval } );
                    ESP_LOGI( FNAME, "migrated old flap level %d %s: %.1f (%d)", i, (char*)&ilabel, nvsspeed, sensval );
                }
                old_iter++;
            }
        }
        return true;
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
            if ( FL_STORE[i].speed->get() != flevel[i].nvs_speed ) {
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
                    wk = i-1;
                    break;
                }
            }
            else {
                if (val < flevel[i].sensval) {
                    wk = i-1;
                    break;
                }
            }
        }
        if (wk < 0) {
            wk = 0;
        }
        float wkf = wk + (float)(val - flevel[wk].sensval) / flevel[wk].sens_delta;
        // ESP_LOGI(FNAME,"getLeverPos(%d): wk: %d, cal %d, delta %d, frac: %1.2f ", val, wk, flevel[wk].sensval, flevel[wk].sens_delta, (float)(val - flevel[wk].sensval) / flevel[wk].sens_delta);
        return wkf;
    }
    return 0.;
}
