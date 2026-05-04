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

#ifndef ALL_LOGS_DISABLED
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
    SensorEntry *existing = find(id);
    if ( existing ) {
        if ( existing->sensor != s ) {
            ESP_LOGW(FNAME, "Sensor %s already registered, replacing it", idmemo[static_cast<int>(id) & 0x3f]);
            SensorBase *old_sensor = existing->sensor;
            *existing = { id, s, s->getDutyCycle() / 100 };
            delete old_sensor;
        }
        return true;
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

bool SensorRegistry::isRegistered(SensorId id) {
    return find(id) != nullptr;
}


// suppress the doRead call for this sensor, but keep the sensor registered for post processing and data access
// one way action (for e.g. sim mode), needs a reboot to revert.
void SensorRegistry::disable(SensorId id)
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
    ESP_LOGI(FNAME, "Dumping registered sensors:");
    for (const auto& e : all_sensors) {
        if (e.isActive()) {
            ESP_LOGI(FNAME, "  Sensor %s (0x%x), dutycycle: %d00ms\n", idmemo[static_cast<int>(e.id) & 0x3f], static_cast<int>(e.id), e.dutycycle);
        }
    }
}
#endif
