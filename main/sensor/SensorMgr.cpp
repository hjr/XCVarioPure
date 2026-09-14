/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "SensorMgr.h"

#include "SensorBase.h"
#include "sensor.h"
#include "logdef.h"

#include <atomic>
#ifndef ALL_LOGS_DISABLED
const char *idmemo[] = { "", "Tmp", "dP", "sP", "teP", "Pos", "Alt", "Var", "Mag", "Acc", "Gyr", "HUM", "FLP" };
#endif

// manage max. 14 sensors at a time (incl. all virtual filter sensors)
std::array<SensorEntry, SensorRegistry::MaxSensors> SensorRegistry::all_sensors {};

// pending sensor change (add/remove) while sensor reading is running
struct SensorChange {
    SensorBase* sensor;
    enum Type : uint8_t { Add, Remove } type;
};
std::atomic<SensorChange*> _pending;

bool SensorRegistry::registerSensor(SensorBase *s)
{
    // ESP_LOGI(FNAME, "Sensor registration");
    if (!s) {
        ESP_LOGE(FNAME, "Attempt to register nullptr sensor");
        return false;
    }

    if ( gflags.sensread_running ) {
        if (_pending.load() != nullptr || all_sensors.size() >= SensorRegistry::MaxSensors) {
            ESP_LOGE(FNAME, "Cannot register sensor");
            return false;
        }
        _pending.store(new SensorChange{ s, SensorChange::Add });
        ESP_LOGI(FNAME, "Sensor registration scheduled");
        return true;
    }

    return addSensor(s);
}

bool SensorRegistry::deregisterSensor(SensorBase* s)
{
    if (!s) {
        ESP_LOGE(FNAME, "Attempt to deregister nullptr sensor");
        return false;
    }

    if ( gflags.sensread_running ) {
        if (_pending.load() != nullptr) {
            ESP_LOGE(FNAME, "Cannot deregister sensor");
            return false;
        }
        _pending.store(new SensorChange{ s, SensorChange::Remove });
        ESP_LOGI(FNAME, "Sensor deregistration scheduled");
        return true;
    }

    return removeSensor(s);
}

void SensorRegistry::applyChange()
{
    SensorChange* change = _pending.load();
    if (!change) { return; }

    ESP_LOGI(FNAME, "Apply sensor change of type %s for sensor %s", change->type == SensorChange::Add ? "Add" : "Remove", idmemo[static_cast<int>(change->sensor->getId()) & 0x3f]);
    if (change->type == SensorChange::Add) {
        addSensor(change->sensor);
    } else if (change->type == SensorChange::Remove) {
        removeSensor(change->sensor);
    }
    _pending.store(nullptr);
    delete change;
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

bool SensorRegistry::addSensor(SensorBase* s)
{
    SensorId id = s->getId();
    SensorEntry *existing = find(id);
    if ( existing ) {
        if ( existing->sensor != s ) {
            ESP_LOGW(FNAME, "Sensor %s already registered, replacing it", idmemo[static_cast<int>(id) & 0x3f]);
            SensorBase *old_sensor = existing->sensor;
            *existing = { id, s, (uint16_t)(s->getDutyCycle() / 100), (uint16_t)(s->getProcessInterval() / 100) };
            delete old_sensor;
        }
        return true;
    }

    int idx = 0;
    for (auto& e : all_sensors) {
        if (!e.isActive()) {
            e = { id, s, (uint16_t)(s->getDutyCycle() / 100), (uint16_t)(s->getProcessInterval() / 100) }; // store dutycycle in 100ms units
            ESP_LOGW(FNAME, "%d. %s::%s sensor (%s) 0x%x registered with dutycycle %dmsec and postproccycle %dmsec", 
                idx, isLocalSensor(id) ? "local" : "extern", idmemo[static_cast<int>(id) & 0x3f], s->name(), static_cast<int>(id), e.dutycycle * 100, e.postproccycle * 100);
            return true;
        }
        idx++;
    }
    return false; // full
}

bool SensorRegistry::removeSensor(SensorBase* s)
{
    for (int i = 0; i < all_sensors.size(); ++i) {
        if (all_sensors[i].sensor == s) {
            delete all_sensors[i].sensor;
            for (int j = i + 1; j < all_sensors.size(); ++j) {
                all_sensors[j - 1] = all_sensors[j];
            }
            break;
        }
    }

    return true;
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
