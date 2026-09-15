/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "BatteryVoltage.h"
#include "sensor/SensorMgr.h"
#include "logdefnone.h"

BatteryVoltage *batSensor = nullptr;

constexpr int SENSOR_HISTORY_DURATION_MS = 300;  // .3 seconds for the adc to average readings
constexpr int DUTY_CYCLE_MS = 100; // 100ms cycle time for adc
constexpr size_t HSIZE = SENSOR_HISTORY_DURATION_MS / DUTY_CYCLE_MS;
static __attribute__((aligned(4))) volt_t batv_buffer[ HSIZE + 1 ];

// multiplier - Uin(Umeasured) := ? ; including 1/1000, because adc measures in mV
const float BatteryVoltage::_Multiplier = (22.0+1.2)/1200.f;

BatteryVoltage::BatteryVoltage() :
    AnalogInput(batv_buffer, HSIZE, DUTY_CYCLE_MS, 0),
    _lpf_volt(0.35)
{
    _id = SensorId::BATTERY_VOLTAGE | SensorFlags::SENSOR_LOCAL | SensorFlags::SENSOR_ESSENTIAL,
    _valid_time_ms = 10000; // 10 seconds
    setNVSVar(&battery_voltage);
    _lpf_volt.reset(12.8f); // expect a 12V system by default
    setFilter(&_lpf_volt);
}

bool BatteryVoltage::setup() {
    begin(ADC_ATTEN_DB_0, ADC_UNIT_1, ADC_CHANNEL_7, true);
    setAdjust(factory_volt_adjust.get());

    // Check the battery monitor
    float value;
    for (int i=0; i<3; i++) {
        doRead(value);
        pushAndPublish(value, Clock::getMillis());
    }
    value = get();
    return (value > 1 && value < 28.0);
}

void BatteryVoltage::setAdjust(float adj){
    _adjust_factor = _Multiplier * ((100.0 + adj) / 100.0);
}

bool BatteryVoltage::doRead(volt_t& val) {

    if ( AnalogInput::doRead(val) ) {
        ESP_LOGI(FNAME, "Battery reading: %.3f/%.3f", val, val * _adjust_factor);
        val *= _adjust_factor;
        return true;
    }
    return false;
}
