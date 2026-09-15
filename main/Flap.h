/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "setup/Capability.h"
#include "sensor/Filters.h"
#include "math/Units.h"

#include <cmath>
#include <vector>


// Flap level internal representation
// ==================================
// Levels enumerate from 0 (most negative flap) to n-1 (most positive flap) for n flap levels.
// Each NVS stored level defines a speed, a label, and a sensor value (ADC reading).
//
//   0         1        2   ...   n-2          n-1    -> level index
//   ^- max speed for L1   ...     ^- max speed for L(n-1)
//             ^- min speed for L1 ...          ^- min speed for L(n-1)
//             ^- speed_delta := L1speed - L0speed ... etc. (speed_delta < 0)
//   |--           Sensor Value     ...       --|      -> steady increase/decrease of sensor reading
//             ^- sens_delta := L1calsens - L0calsens ... etc.
// Level 0 | Level 1 |  ...  | Level n-2 | Level n-1 | -> recommended speed band
//         ^idx0.5.. ^idx1.5 maps to speed band prep_speed(L0) > v >= prep_speed(L1)
//
// Flap levels need to be prepared for the actual wingload on initialization and after changes. The resulting
// prepared speeds are used for all calculations. -> prep_speed
// The NVS stored speed belongs to the reference wing load of the selected glider polar.
// 
// The flap level x is recommended for speeds >= speed(x)!
// For the sake of saving NVS entries (legacy), there is no speed for level (n-1) defined (when n=7). Then
// the last level n-1 is recommended for all speeds < speed(n-2).
//
// The sensor values can be in ascending or descending order, depending on the flap sensor installation.
// 
// NVS storage items order is defined through the FLAP_STORE array.
//
// Flap level addition/removal: A modified level list gets sorted according to speeds and is saved back 
// to NVS on exit of the flap setup menu
//
// Simple application sample "Teka-Off:"
//    For take-off phase you want to show a flap setting according to the take-off flap level index.
//    As the flap level store let you retrieve the recommended speed band for e.g. level 4 with:
//    Level(idx4) stores the min. speed for L4 and the sensor value for L4.
//    Level(idx4) max. speed is found by level(idx3) min. speed, and so on.
//    Choose to display speed half ways to level(3).min speed to have the flap box indicator showing the recommended 
//    flap setting for take-off "centered".
//

class AnalogInput;
template <typename T>
class SetupNG;
struct flap_table;

struct FlapLevel
{
    kmh_t nvs_speed;
    mps_t prep_speed;
    mps_t speed_delta;
    union {
        char label[4];
        int label_int;
    };
    int sensval;
    int sens_delta;
    constexpr FlapLevel(kmh_t s, int label_int, int sv) : nvs_speed(s), prep_speed(0.), speed_delta(0.), label_int(label_int), sensval(sv), sens_delta(0) {}
    // FlapLevel(float s, const char *lc, int sv) : nvs_speed(s), prep_speed(0.), speed_delta(0.), sensval(sv), sens_delta(0) {
    //     std::strncpy(label, lc, 4);
    //     label[3] = '\0';
    // }
};

/*
 * This class handels flap display and Flap sensor
 *
 */

class Flap
{
private:
    Flap();

public:
    ~Flap();
    static Flap *theFlap();

    // config access
    static SetupNG<mps_t> *getSpeedNVS(int idx);
    static SetupNG<int> *getLblNVS(int idx);
    static SetupNG<int> *getSensNVS(int idx);
    void setFromFlapTable(const flap_table &fte);
    const FlapLevel *getFL(int idx) const { return (idx < flevel.size()) ? &flevel[idx] : &dummy; }
    void setSensCal(int idx, int val);
    void setLabel(int idx, const char *lab);
    void setSpeed(int idx, kmh_t spd);
    void prepLevels();
    void modLevels();
    void reLoadLevels();
    void addLevel(FlapLevel &lev);
    void removeLevel(int idx);
    void clearAllLevels() { flevel.clear(); prepLevels(); }

    // recommendations
    float getOptimum(mps_t speed) const;
    mps_t getSpeedBand(float wkf, mps_t &maxv) const;
    mps_t getSpeed(float wkf) const;

    // sensor access
    static inline bool sensAvailable() { return XcvCaps::haveCap(XcvCaps::FLAPSENS_CAP); }
    static void configureADC();
    static void removeADC();
    bool sensorToLeverPosition(int sensorreading, float &wkf) const;
    int getNrPositions() const { return flevel.size(); }
    static constexpr const int MAX_NR_POS = 7;

private:
    // helper
    bool initFromNVS();
    void saveToNVS();
    int getWkIndex(float wkf) const;

    // attributes
    static Flap *_instance;
    std::vector<FlapLevel> flevel;
    bool _sens_order = true; // if true, sensval are in descending order from flap level 0, 1, 2, ...
    static const FlapLevel dummy;
    int rawFiltered = 0;
};

extern Flap* FLAP;
