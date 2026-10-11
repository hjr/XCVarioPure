/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "AnalogInput.h"

class FlapSens final : public AnalogInput {
private:
    explicit FlapSens(uint8_t port);

public:
    using ADC_PORT = enum : uint8_t { UNIT1_CHANNEL6, UNIT2_CHANNEL2, UNIT2_CHANNEL9 }; // GPIO-34, GPIO-2, GPIO-26
    static FlapSens* create(uint8_t port);
    virtual ~FlapSens();

    // sensor API
    const char* name() const override { return "FlapSens"; }
    bool setup() override;
    void postProcess() override;

    void changePort(uint8_t port);
    uint8_t getPort() const { return static_cast<uint8_t>(_port); }

    // sim mode
    void enterSimMode();

private:
    ADC_PORT _port;
    AdaptiveLowPassFilterT<float> _alp_filter;
};

extern FlapSens *flapSensor;
