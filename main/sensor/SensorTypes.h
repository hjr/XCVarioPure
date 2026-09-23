/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include <cstdint>

enum SensorType : uint8_t {
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
    BATTERY_VOLTAGE,
    MAX_SENSOR_ID
};

struct SensorId {
    static constexpr uint8_t SENSOR_LOCAL     = 1<<4;
    static constexpr uint8_t SENSOR_ESSENTIAL = 1<<5;

    SensorType type = SensorType::NONE;
    union {
        struct {
            uint8_t prio      : 4; // 0..15 - high..low priority
            uint8_t local     : 1;
            uint8_t essential : 1;
        };
        uint8_t flags = 0;
    };

    SensorId() = delete;
    constexpr explicit SensorId(SensorType t, uint8_t f = 0) : type(t), flags(f) {}
    constexpr bool isLocalSensor() const {
        return (flags & SensorId::SENSOR_LOCAL) != 0;
    }
    constexpr bool isEssentialSensor() const {
        return (flags & SensorId::SENSOR_ESSENTIAL) != 0;
    }
    constexpr bool operator==(SensorId b) const {
        return type == b.type;
    }
};

