/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "BatteryVoltage.h"
#include "sensor/SensorMgr.h"
#include "math/Floats.h"
#include "logdefnone.h"

BatteryVoltage *batSensor = nullptr;

constexpr int SENSOR_HISTORY_LENGTH_MS = 100;  // kind of no history
constexpr int DUTY_CYCLE_MS = 100; // 100ms cycle time for adc
constexpr size_t HSIZE = SENSOR_HISTORY_LENGTH_MS / DUTY_CYCLE_MS;
static __attribute__((aligned(4))) volt_t batv_buffer[ HSIZE + 1 ];

// multiplier - Uin(Umeasured) := ? ; including 1/1000, because adc measures in mV
const float BatteryVoltage::_Multiplier = (22.0+1.2)/1200.f;

BatteryVoltage::BatteryVoltage() :
    AnalogInput(batv_buffer, HSIZE, DUTY_CYCLE_MS, 0),
    _lpf_volt(0.35)
{
    _id = SensorId(SensorType::BATTERY_VOLTAGE, SensorId::SENSOR_LOCAL | SensorId::SENSOR_ESSENTIAL | 15);
    // it is essential, because it is not part of the Sim dataset
    _valid_time_ms = 10000; // 10 seconds
    _lpf_volt.reset(12.8f); // expect a 12V system by default
    // do not implant the filter, because it would mess the .1 rounded values
}

BatteryVoltage::~BatteryVoltage() {
    batSensor = nullptr;
}

bool BatteryVoltage::setup() {
    begin(ADC_ATTEN_DB_0, ADC_UNIT_1, ADC_CHANNEL_7, true);
    setAdjust(factory_volt_adjust.get());

    // check the battery monitor
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
        val = _lpf_volt.filter(val * _adjust_factor);
        // only push changes beyond .1V difference to the blackboard
        // a way to avoid frequent updates for minor voltage changes
        // and non conform with the sensor base's usual behavior
        if (std::fabsf(val - battery_voltage.get()) > 0.1) {
            // round and set
            val = fast_roundf(val);
            battery_voltage.set(val);
        }
        return true;
    }
    return false;
}

