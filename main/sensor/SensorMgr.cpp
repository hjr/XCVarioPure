/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "SensorMgr.h"

#include "SensorBase.h"
#include "logdef.h"

#include <cstdint>

// manage max. 10 sensors at a time (incl. all virtual filter sensors)
std::array<SensorEntry, SensorRegistry::MaxSensors> SensorRegistry::all_sensors {};

bool SensorRegistry::registerSensor(SensorId id, SensorBase *s)
{
    if (!s) {
        ESP_LOGE(FNAME, "Attempt to register nullptr sensor");
        return false;
    }

    if ( find(id) ) {
        ESP_LOGI(FNAME, "Sensor with id %d already registered", static_cast<int>(id));
        return false; // already registered
    }

    for (auto& e : all_sensors) {
        if (!e.isActive()) {
            e = { id, s, s->getDutyCycle() / 100 }; // store dutycycle in 100ms units
            ESP_LOGI(FNAME, "Sensor registered with id %d", static_cast<int>(id));
            return true;
        }
    }
    return false; // full
}

void SensorRegistry::deregisterSensor(SensorBase* s) {
    
    for (auto& e : all_sensors) {
        if (e.sensor == s) {
            ESP_LOGI(FNAME, "Sensor %d deregistered", static_cast<int>(e.id));
            e.id = SensorId::NONE;
            e.sensor = nullptr;
            e.dutycycle = 0;
            return;
        }
    }
}

void SensorRegistry::removeFromUpdateLoop(SensorId id)
{
    SensorEntry *entry = find(id);
    if (entry) {
        ESP_LOGI(FNAME, "Sensor %d removed from update loop", static_cast<int>(entry->id));
        entry->dutycycle = 0;
    }
}

void SensorRegistry::enterSimMode()
{
    ESP_LOGI(FNAME, "SensorRegistry entering SIMULATION MODE");
    for (auto& e : all_sensors) {
        if (e.isActive()) {
            e.dutycycle = 0; // no further sensor reading
        }
    }
}

SensorEntry* SensorRegistry::find(SensorId id) {
    for (auto& e : all_sensors)
        if (e.id == id) return &e;
    return nullptr;
}
