/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "InterfaceCtrl.h"

// GPIO, or dummy interface

template<const char* Id, InterfaceId IdVal>
class DmyItf : public InterfaceCtrl
{
public:
    DmyItf() : InterfaceCtrl(true, false) {}
    InterfaceId getId() const override { return IdVal; }
    const char* getStringId() const override { return Id; }
    void ConfigureIntf(int) override {}
    int Send(const char*, int&, int = 0) override { return 0; }
};

extern constexpr char GPIO_ID[] = "S2P6_GPIO";
using AnalogSignal = DmyItf<GPIO_ID, S2_PIN6>;
extern InterfaceCtrl *S2_6_Signal;