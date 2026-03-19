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

#ifndef CONFIG_LOG_DISABLED
const char *idmemo[] = { "", "Tmp", "dP", "sP", "teP", "Pos", "Alt", "Var", "Mag", "Acc", "Gyr", "HUM", "FLP" };
#endif

// manage max. 10 sensors at a time (incl. all virtual filter sensors)
std::array<SensorEntry, SensorRegistry::MaxSensors> SensorRegistry::all_sensors {};

bool SensorRegistry::registerSensor(SensorBase *s)
{
    // ESP_LOGI(FNAME, "Sensor registration");
    if (!s) {
        ESP_LOGE(FNAME, "Attempt to register nullptr sensor");
        return false;
    }
    SensorId id = s->getId();
    if ( find(id) ) {
        ESP_LOGW(FNAME, "Sensor %s already registered", idmemo[static_cast<int>(id) & 0x3f]);
        return false; // already registered
    }

    int idx = 0;
    for (auto& e : all_sensors) {
        if (!e.isActive()) {
            e = { id, s, s->getDutyCycle() / 100 }; // store dutycycle in 100ms units
            ESP_LOGW(FNAME, "%d. %s sensor 0x%x registered with dutycycle %d00msec", idx, idmemo[static_cast<int>(id) & 0x3f], static_cast<int>(id), e.dutycycle);
            return true;
        }
        idx++;
    }
    return false; // full
}

void SensorRegistry::deregisterSensor(SensorBase* s) {
    
    ESP_LOGI(FNAME, "Sensor deregistration");
    for (auto& e : all_sensors) {
        if (e.sensor == s) {
            ESP_LOGW(FNAME, "Sensor %s deregistered", idmemo[static_cast<int>(e.id) & 0x3f]);
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
        ESP_LOGW(FNAME, "Sensor %s removed from update loop", idmemo[static_cast<int>(entry->id) & 0x3f]);
        entry->id = entry->id & ~SensorFlags::SENSOR_LOCAL; // clear local sensor flag
    }
}

void SensorRegistry::enterSimMode()
{
    ESP_LOGI(FNAME, "SensorRegistry entering SIMULATION MODE");
    for (auto& e : all_sensors) {
        if (e.isActive() && !isEssentialSensor(e.id)) {
            e.id = e.id & ~SensorFlags::SENSOR_LOCAL; // no further sensor reading
        }
    }
}

SensorEntry* SensorRegistry::find(SensorId id) {
    for (auto& e : all_sensors)
        if (e.id == id) return &e;
    return nullptr;
}

#ifdef DEBUG_AND_TEST
void SensorRegistry::dump() {
    printf("Dumping registered sensors:\n");
    for (const auto& e : all_sensors) {
        if (e.isActive()) {
            printf("  Sensor %s (0x%x), dutycycle: %d00ms\n", idmemo[static_cast<int>(e.id) & 0x3f], static_cast<int>(e.id), e.dutycycle);
        }
    }
}
#endif
