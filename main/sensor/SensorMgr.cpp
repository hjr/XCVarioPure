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
const char *idmemo[] = { "", "Tmp", "dP", "sP", "teP", "Pos", "Alt", "Vrt", "Var", "Mag", "Acc", "Gyr", "HUM", "FLP", "Bat" };
#endif

// queue changes
QueueHandle_t SensorRegistry::sensChangeQueue = nullptr;

// manage max. 14 sensors at a time (incl. all virtual filter sensors)
std::array<SensorEntry, SensorRegistry::MaxSensors> SensorRegistry::all_sensors {};
int SensorRegistry::numSensors = 0;

// pending sensor change (add/remove) while sensor reading is running
struct SensorChange {
    SensorBase* sensor;
    enum Type : uint8_t { Add, Remove, Sim } type;
};

// initiate the synchronized sensor chenge by creating the queue
void SensorRegistry::createQueue()
{
    sensChangeQueue = xQueueCreate(10, sizeof(SensorChange*));
}

bool SensorRegistry::registerSensor(SensorBase *s)
{
    // ESP_LOGI(FNAME, "Sensor registration");
    if (!s) {
        ESP_LOGE(FNAME, "Attempt to register nullptr sensor");
        return false;
    }

    if (sensChangeQueue) {
        SensorChange* change = new SensorChange{ s, SensorChange::Add };
        if (numSensors >= SensorRegistry::MaxSensors || xQueueSend(sensChangeQueue, &change, 0) != pdTRUE) {
            delete change;
            ESP_LOGE(FNAME, "Cannot register sensor");
            return false;
        }
        
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

    if (sensChangeQueue) {
        SensorChange* change = new SensorChange{ s, SensorChange::Remove };
        if (xQueueSend(sensChangeQueue, &change, 0) != pdTRUE) {
            delete change;
            ESP_LOGE(FNAME, "Cannot deregister sensor");
            return false;
        }
        
        ESP_LOGI(FNAME, "Sensor deregistration scheduled");
        return true;
    }

    return removeSensor(s);
}

void SensorRegistry::updateCycleTimes(SensorBase* s)
{
    if (!s) {
        ESP_LOGE(FNAME, "Attempt to update cycle times for nullptr sensor");
        return;
    }

    SensorEntry *entry = find(s->getId().type);
    if (entry && entry->sensor == s) {
        entry->dutycycle = (uint16_t)(s->getDutyCycle() / 100);
        entry->postproccycle = (uint16_t)(s->getProcessInterval() / 100);
    }
}

void SensorRegistry::applyChange()
{
    SensorChange* change = nullptr;
    while (xQueueReceive(sensChangeQueue, &change, 0) == pdTRUE) {
        if (change->type == SensorChange::Add) {
            addSensor(change->sensor);
        } else if (change->type == SensorChange::Remove) {
            removeSensor(change->sensor);
        } else if (change->type == SensorChange::Sim) {
            goSimMode();
        }
        delete change;
    }
}

bool SensorRegistry::isRegistered(SensorType typ) {
    return find(typ) != nullptr;
}


// suppress the doRead call for this sensor, but keep the sensor registered for post processing and data access
// one way action (for e.g. sim mode), needs a reboot to revert.
void SensorRegistry::disable(SensorId id)
{
    SensorEntry *entry = find(id.type);
    if (entry) {
        ESP_LOGW(FNAME, "Sensor %s removed from update loop", idmemo[id.type]);
        entry->id.flags = entry->id.flags & ~SensorId::SENSOR_LOCAL; // clear local sensor flag
    }
}

void SensorRegistry::enterSimMode()
{
    if (sensChangeQueue) {
        SensorChange* change = new SensorChange{ nullptr, SensorChange::Sim };
        if (xQueueSend(sensChangeQueue, &change, 0) != pdTRUE) {
            delete change;
            ESP_LOGE(FNAME, "Cannot enter simulation mode");
        }
    }
}

bool SensorRegistry::addSensor(SensorBase* s)
{
    SensorId id = s->getId();
    SensorEntry *existing = find(id.type);
    if (existing) {
        if (existing->sensor != s) {
            if (existing->id.prio == id.prio) {
                ESP_LOGW(FNAME, "Sensor %s already registered, replacing it", idmemo[id.type]);
                SensorBase *old_sensor = existing->sensor;
                *existing = { id, s, (uint16_t)(s->getDutyCycle() / 100), (uint16_t)(s->getProcessInterval() / 100) };
                delete old_sensor;
                return true;
            } else {
                ESP_LOGW(FNAME, "Sensor %s already registered with different priority", idmemo[id.type]);
                removeSensor(existing->sensor);
            }
        }
    }

    if (numSensors < SensorRegistry::MaxSensors) {
        // respect sensor priority
        int i = 0;
        SensorEntry* e = nullptr;
        for (; i < numSensors; ++i) {
            if (all_sensors[i].id.prio > id.prio) {
                e = &all_sensors[i];
                break;
            }
        }
        if ( e && i < numSensors) {
            // shift all sensors with lower priority one position to the right
            for (int j = numSensors; j > i; --j) {
                all_sensors[j] = all_sensors[j - 1];
            }
        } else {
            e = end();
        }
        *e = { id, s, (uint16_t)(s->getDutyCycle() / 100), (uint16_t)(s->getProcessInterval() / 100) }; // store dutycycle in 100ms units
            ESP_LOGW(FNAME, "%d. %s::%s sensor%s (%s) registered with dutycycle %dmsec and postproccycle %dmsec", 
                numSensors, (id.isLocalSensor() ? "local" : "extern"), idmemo[id.type], (id.isEssentialSensor() ? "*" : ""), s->name(), e->dutycycle * 100, e->postproccycle * 100);
        // increment numSensors to account for the new sensor
        numSensors++;
#ifdef DEBUG_AND_TEST
        // dump();
#endif
        return true;
    }
    return false; // full
}

bool SensorRegistry::removeSensor(SensorBase* s)
{
    for (int i = 0; i < numSensors; ++i) {
        if (all_sensors[i].sensor == s) {
            ESP_LOGW(FNAME, "%d. remove %s sensor (%s)", i, idmemo[s->getId().type], s->name());
            delete s;
            for (int j = i + 1; j < all_sensors.size(); ++j) {
                all_sensors[j - 1] = all_sensors[j];
            }
            numSensors--;
            break;
        }
    }

    return true;
}

void SensorRegistry::goSimMode()
{
    ESP_LOGW(FNAME, "SensorRegistry entering SIMULATION MODE");
    for (SensorEntry* e = all_sensors.data(); e != end(); ++e) {
        if (e->isActive() && !e->id.isEssentialSensor()) {
            e->id.flags = e->id.flags & ~SensorId::SENSOR_LOCAL; // no further sensor reading
        }
    }
}


SensorEntry* SensorRegistry::find(SensorType typ) {
    for (SensorEntry *e = all_sensors.data(); e != end(); ++e)
        if (e->id.type == typ) return e;
    return nullptr;
}

#ifdef DEBUG_AND_TEST
void SensorRegistry::dump() {
    ESP_LOGI(FNAME, "Dumping registered sensors:");
    int i = 1;
    for (const auto& e : all_sensors) {
        if (e.isActive()) {
            ESP_LOGI(FNAME, "%d: prio%02d Sensor %s::%s (f:%x), dutycycle: %d00ms/%d00ms", i++, e.id.prio, idmemo[e.id.type], e.sensor->name(), static_cast<int>(e.id.flags), e.dutycycle, e.postproccycle);
        }
    }
}
#endif
