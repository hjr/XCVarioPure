/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "sensor/SensorBase.h"

#include <esp_adc/adc_oneshot.h>

class AnalogInput : public SensorTP<float> {
   public:
    explicit AnalogInput(void *buf, size_t cap, uint32_t ums, int pm);
    virtual ~AnalogInput();

    // partial sensor API
    bool probe() override { return true; }; // they are known at compile time
    bool doRead(float& val) override; // read raw adc value

   protected:
    void begin(adc_atten_t attenuation, adc_unit_t unit, adc_channel_t ch, bool calibration);

   private:
    adc_channel_t _adc_ch;
    static adc_oneshot_unit_handle_t _adc_handle;
    adc_cali_handle_t _adc_cali = nullptr;
};
