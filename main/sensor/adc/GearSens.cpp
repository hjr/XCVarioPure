/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "GearSens.h"

#include "logdefnone.h"

GearSens *gearSensor = nullptr;

constexpr int DUTY_CYCLE_MS = 500; // 2Hz sampling is plenty for a gear warning

GearSens::GearSens(bool inverted) :
    SensorBase(DUTY_CYCLE_MS, 0),
    _inverted(inverted)
{
    _id = SensorId(SensorType::GEAR_SIGNAL, SensorId::SENSOR_LOCAL | 15);
}

GearSens::~GearSens() {
    gearSensor = nullptr;
}

GearSens* GearSens::create(bool inverted) {
    if (!gearSensor) {
        gearSensor = new GearSens(inverted);
    }
    else {
        gearSensor->setInverted(inverted);
    }
    gearSensor->setup();
    ESP_LOGI(FNAME, "Gear sensor configured, inverted %d", inverted);
    return gearSensor;
}

bool GearSens::setup() {
    gpio_reset_pin(GEAR_IO);
    gpio_set_direction(GEAR_IO, GPIO_MODE_INPUT);
    // Mind: GPIO 34 has no internal pull-up, the external logic has to drive the pin.
    gpio_set_pull_mode(GEAR_IO, GPIO_PULLUP_ONLY);
    return true;
}

bool GearSens::update(uint32_t now_ms) {
    _alarm = (gpio_get_level(GEAR_IO) != 0) != _inverted;
    _last_update_time_ms = now_ms;
    return true;
}
