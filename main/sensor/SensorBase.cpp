/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/


#include "SensorBase.h"

#include "SensorMgr.h"
#include "logdef.h"

SensorBase::SensorBase(int ums) : _update_interval_ms(ums), _latency_ms(0), _last_update_time_ms(0),
    _id(SensorId::NONE)
{
    // Pls register sensors as needed for the read-sensor loop and in the proper order
    // SensorRegistry::registerSensor(this);
}

SensorBase::~SensorBase()
{
    // deregister is done automatically
    SensorRegistry::deregisterSensor(this);
}

template <typename T>
void SensorTP<T>::dump(int interval_ms) const {
    int count = getCount(interval_ms);
    ESP_LOGI(FNAME, "Dumping last %d ms, count=%d", interval_ms, count);
    for (int i = 0; i < count; ++i) {
        if constexpr (std::is_same_v<T, float>) {
            float val = _history[i];
            ESP_LOGI(FNAME, "  [%03d]: %f", i, val);
        }
        else if constexpr (std::is_same_v<T, vector_f>) {
            vector_f val = _history[i];
            ESP_LOGI(FNAME, "  [%03d]: (%f, %f, %f)", i, val.x, val.y, val.z);
        }
    }
}

// explicit instantiation
template void SensorTP<float>::dump(int interval_ms) const;
template void SensorTP<vector_f>::dump(int interval_ms) const;