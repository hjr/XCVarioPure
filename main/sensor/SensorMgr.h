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
    VARIOMETER,
    MAGNETO,
    ACC_INERTIAL,
    GYRO_INERTIAL,
    HUMIDITY,
    FLAP_POSITION,
    MAX_SENSOR_ID
};

enum SensorFlags : uint8_t {
    SENSOR_LOCAL     = 0x80,
    SENSOR_ESSENTIAL = 0x40
};

constexpr bool isLocalSensor(SensorId id) {
    return (static_cast<uint8_t>(id) & static_cast<uint8_t>(SensorFlags::SENSOR_LOCAL)) != 0;
}
constexpr bool isEssentialSensor(SensorId id) {
    return (static_cast<uint8_t>(id) & static_cast<uint8_t>(SensorFlags::SENSOR_ESSENTIAL)) != 0;
}
constexpr SensorId operator|(SensorId a, SensorFlags b) {
    return static_cast<SensorId>(
        static_cast<uint8_t>(a) | static_cast<uint8_t>(b)
    );
}
constexpr SensorId operator&(SensorId a, int b) {
    return static_cast<SensorId>(
        static_cast<uint8_t>(a) & b
    );
}
constexpr bool operator==(SensorId a, SensorId b) {
    return static_cast<uint8_t>(a & 0x1f) == static_cast<uint8_t>(b & 0x1f);
}

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

    static bool registerSensor(SensorBase* sensor);
    static void disable(SensorId id);
    static void enterSimMode();

    static auto begin() { return all_sensors.begin(); }
    static auto end()   { return all_sensors.end(); }

    // for debug purposes
    static void dump();

private:
    static SensorEntry* find(SensorId id);
    static std::array<SensorEntry, MaxSensors> all_sensors;
};

