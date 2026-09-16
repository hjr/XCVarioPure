/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "FlapSens.h"
#include "Flap.h"
#include "sensor/SensorMgr.h"
#include "math/Floats.h"
#include "logdefnone.h"

#include <algorithm>

FlapSens *flapSensor = nullptr;

constexpr int SENSOR_HISTORY_DURATION_MS = 200;
constexpr int DUTY_CYCLE_MS = 200; // 200ms cycle time for adc
constexpr size_t HSIZE = SENSOR_HISTORY_DURATION_MS / DUTY_CYCLE_MS;
static __attribute__((aligned(4))) float  flps_buffer[ HSIZE + 1 ];

FlapSens::FlapSens() :
    AnalogInput(flps_buffer, HSIZE, DUTY_CYCLE_MS, 1),
    _alp_filter{0.08f, 0.5f}
{
    _id = SensorId::FLAP_POSITION | SensorFlags::SENSOR_LOCAL,
    _valid_time_ms = 3000; // 3 seconds
}

FlapSens::~FlapSens() {
    flapSensor = nullptr;
}

FlapSens* FlapSens::create() {
    if (!flapSensor) {
        flapSensor = new FlapSens();
        flapSensor->setup();
        ESP_LOGI(FNAME, "Flap sensor configured");

        // Check the sensor
        float value;
        for (int i=0; i<3; i++) {
            flapSensor->doRead(value);
            flapSensor->pushAndPublish(value, Clock::getMillis());
        }
        uint32_t read = (uint32_t)flapSensor->getHead();
        if (read == 0 || read >= 4096) { // try GPIO pin 34, series 2021-2
            ESP_LOGI(FNAME, "Flap sensor not found or edge value, reading: %d", (int)read);
        } else {
            ESP_LOGI(FNAME, "Flap sensor looks good, reading: %d", (int)read);
        }

    }
    return flapSensor;
}

bool FlapSens::setup() {
    begin(ADC_ATTEN_DB_0, ADC_UNIT_1, ADC_CHANNEL_6, false);
    return true;
}

// 5 Hz update
void FlapSens::postProcess() {
    float wkraw = std::clamp(getHead(), -1.f, 4096.f);
    if (wkraw < 0) {
        // drop erratic negative readings
        ESP_LOGW(FNAME, "negative flap sensor reading: %f", wkraw);
        return;
    }
    // ESP_LOGI(FNAME,"flap sensor =%d", wkraw );
    int raw_filtered = fast_iroundf(_alp_filter.filter(wkraw));

    if (FLAP) {
        float lever;
        if (FLAP->sensorToLeverPosition(raw_filtered, lever)) {

            if (fabsf(flap_pos.get() - lever) >= 0.1f) {
                flap_pos.set(lever); // update secondary vario
                ESP_LOGI(FNAME, "wk sensor=%1.2f  raw=%d", lever, raw_filtered);
            }
        }
    }
}