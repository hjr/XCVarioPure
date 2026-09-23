/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "SensorTypes.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <array>
#include <cstdint>

class SensorBase;

struct SensorEntry {
    SensorId    id = SensorId(SensorType::NONE);
    SensorBase* sensor = nullptr;   // polymorph
    uint16_t    dutycycle = 0;      // x 100msec loops
    uint16_t    postproccycle = 0;  // x 100msec loops, 0 for not part of the loop
    constexpr bool isActive() const { return sensor != nullptr; }
};

class SensorRegistry
{
public:
    static constexpr int MaxSensors = 14;
    static void createQueue();

    static bool registerSensor(SensorBase* sensor);
    static bool deregisterSensor(SensorBase* sensor);
    static void applyChange();
    static bool isRegistered(SensorType typ);
    static void disable(SensorId id);
    static void enterSimMode();

    static SensorEntry* begin() { return all_sensors.begin(); }
    static SensorEntry* end() { return all_sensors.data() + numSensors; }

    // for debug purposes
    static void dump();

private:
    static bool addSensor(SensorBase* sensor);
    static bool removeSensor(SensorBase* sensor);
    static void goSimMode();
    static SensorEntry* find(SensorType typ);
    // data structures for managing sensors and pending changes
    static QueueHandle_t sensChangeQueue;
    static std::array<SensorEntry, MaxSensors> all_sensors;
    static int numSensors;
};

