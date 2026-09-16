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
   public:
    explicit FlapSens();
    virtual ~FlapSens();

    // sensor API
    const char* name() const override { return "FlapSens"; }
    bool setup() override;
    void postProcess() override;

   private:
    AdaptiveLowPassFilterT<float> _alp_filter;
};

extern FlapSens *flapSensor;
