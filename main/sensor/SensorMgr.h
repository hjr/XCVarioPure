/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include <array>
#include <cstdint>

class SensorBase;

enum SensorId : uint8_t {
    NONE = 0,
    TEMPERATURE,
    DIFFPRESSURE,
    STATIC_PRESSURE,
    TE_PRESSURE,
    POSITION,
    ALTITUDE,
    VARIO,
    MAGNETO,
    GYRO,
    HUMIDITY,
    FLAP_POSITION,
    MAX_SENSOR_ID
};

struct SensorEntry {
    SensorId    id = SensorId::NONE; // enum
    SensorBase* sensor = nullptr;   // polymorph
    int         dutycycle = 0;      // x 100msec loops, 0 for not part of the loop
    constexpr bool isActive() const { return sensor != nullptr; }
};

class SensorRegistry
{
public:
    static constexpr int MaxSensors = 10;

    static bool registerSensor(SensorId id, SensorBase* sensor);
    static void deregisterSensor(SensorBase* sensor);
    static void removeFromUpdateLoop(SensorId id);
    static void enterSimMode();
    static SensorEntry* find(SensorId id);

    static auto begin() { return all_sensors.begin(); }
    static auto end()   { return all_sensors.end(); }

private:
    static std::array<SensorEntry, MaxSensors> all_sensors;
};

