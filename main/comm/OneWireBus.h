/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "InterfaceCtrl.h"

#include <cstdint>
#include <vector>


struct onewire_bus_t;
class SensorBase;
class OwSens;

class OneWireBus final : public InterfaceCtrl
{
private:
    OneWireBus();
public:
    static OneWireBus* create();
    ~OneWireBus();

    using onewire_device_address_t = uint64_t;

    // OneWire specific
    onewire_bus_t *getOneWire() const { return _onewire; }
    SensorBase* probeAndSetup(uint8_t famid);
    void busReset();
    esp_err_t sendCommand(onewire_device_address_t addr, uint8_t cmd);
    esp_err_t writeBytes(const uint8_t *buf, uint8_t size);
    esp_err_t readBytes(uint8_t *buf, size_t size);
    uint8_t crc8(const uint8_t *data, size_t len);
    bool groupUpdate(uint32_t now_ms);

    // Ctrl
    InterfaceId getId() const override { return OW_BUS; }
    const char *getStringId() const override { return "OneWire"; }
    void ConfigureIntf(int cfg) override;
    int Send(const char*, int&, int) override { return 0; }

private:
    OwSens* getSensorByAddress(onewire_device_address_t addr);
    // void recover();
    onewire_bus_t *_onewire = nullptr;
    static const int _ONEWIRE_BUS_GPIO;
    std::vector<OwSens*> _all_sensor;
    // int8_t errors = 0;
};

extern OneWireBus *OneWIRE;
