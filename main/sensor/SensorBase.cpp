/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/


#include "SensorBase.h"

#include "SensorMgr.h"

SensorBase::SensorBase(int ums) : _update_interval_ms(ums), _latency_ms(0), _last_update_time_ms(0),
    _id(SensorId::NONE)
{
    // Pls register sensors as needed in the sensor loop and in the proper order
    // SensorRegistry::registerSensor(this);
}

SensorBase::~SensorBase()
{
    // deregister is done automatically
    SensorRegistry::deregisterSensor(this);
}