/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "OneWireBus.h"

#include "sensor/temp/ds18b20.h"
#include "comm/DeviceMgr.h"
#include "logdefnone.h"

#include <onewire_bus.h>
#include <onewire_bus_interface.h>
#include <onewire_cmd.h>
#include <onewire_device.h>
#include <onewire_crc.h>
#include <driver/gpio.h>

#include <vector>

// 
// The OneWire bus is sadly dur to low stability, low support from chip manufacturer besides Dallas, etc. a dedicated
// temperature sensor peer to peer connection.
// So, this module is dedicated to discover and connect a temperature sensor of the family (id 0x28), nothing more.
// 

// Singleton instance of the OneWireBus
OneWireBus *OneWIRE = nullptr;

const int OneWireBus::_ONEWIRE_BUS_GPIO = GPIO_NUM_23;

OneWireBus::OneWireBus() : InterfaceCtrl(false, false)
{
   	gpio_set_drive_capability((gpio_num_t)_ONEWIRE_BUS_GPIO, GPIO_DRIVE_CAP_1);
}

OneWireBus* OneWireBus::create()
{
    if ( ! OneWIRE ) {
        OneWIRE = new OneWireBus();
    }
    return OneWIRE;
}

OneWireBus::~OneWireBus()
{
    OneWIRE = nullptr;
    if ( _onewire) {
        // onewire_bus_del( (onewire_bus_handle_t)_onewire );
        _onewire->del(_onewire);
    }
}

void OneWireBus::ConfigureIntf(int cfg)
{
    // install 1-wire bus
    onewire_bus_handle_t bus;
    onewire_bus_config_t bus_config = {
        .bus_gpio_num = (gpio_num_t)_ONEWIRE_BUS_GPIO,
        .flags = {
            .en_pull_up = true,
        }
    };
    onewire_bus_rmt_config_t rmt_config = {
        .max_rx_bytes = 10, // 1byte ROM command + 8byte ROM number + 1byte device command
    };
    if ( onewire_new_bus_rmt(&bus_config, &rmt_config, &bus) == ESP_OK ) {
        ESP_LOGI(FNAME, "1-Wire bus installed on GPIO%d", _ONEWIRE_BUS_GPIO);
        _onewire = bus;
        _functional = true;
    }
    else {
        ESP_LOGE(FNAME, "1-Wire bus installation failed on GPIO%d", _ONEWIRE_BUS_GPIO);
    }
}

SensorBase* OneWireBus::probeAndSetup(uint8_t famid) {

    // Search for devices
    onewire_device_iter_handle_t iter = nullptr;
    onewire_device_t next_device;
    std::vector<onewire_device_t> devices;

    SensorBase* found_sensor = nullptr;

    esp_err_t err = onewire_new_device_iter(_onewire, &iter);
    if (err != ESP_OK) return nullptr;

    while (onewire_device_iter_get_next(iter, &next_device) == ESP_OK) {
        devices.push_back(next_device);
    }
    onewire_del_device_iter(iter);

    if (devices.empty()) {
        ESP_LOGW(FNAME, "No OneWire devices found on GPIO %d", _ONEWIRE_BUS_GPIO);
        return nullptr;
    }

    ESP_LOGI(FNAME, "Found %d OneWire device(s) on GPIO %d", devices.size(), _ONEWIRE_BUS_GPIO);
    // Process each device by family code
    for (const auto& dev : devices) {
        uint8_t family = dev.address & 0xFF;  // LSB is family code

        ESP_LOGI(FNAME, "Device ROM: %016llX, Family: 0x%02X", dev.address, family);

        if ( family == famid ) {
            OwSens *s = getSensorByAddress(dev.address);
            if ( s ) {
                // Already registered
                s->setup(); // re-setup
                ESP_LOGI(FNAME, "Device %016llX already registered", dev.address);
                continue;
            }

            switch (family) {
                case DS18B20_FAMILY:  // DS18B20 (also covers DS18S20 variants in some libs)
                // case 0x10:  // DS18S20
                // case 0x22:  // DS1822
                    {
                        auto* sensor = new DS18B20(dev.address);
                        sensor->setup();
                        _all_sensor.push_back(sensor);
                        found_sensor = sensor;
                        ESP_LOGI(FNAME, "Registered DS18B20-like temperature sensor");
                    }
                    break;

                // Add more cases here for other devices, e.g.:
                // case 0x01: // DS2401 Silicon Serial Number (ID chip)
                // case 0x2D: // DS2431 1Kb EEPROM
                // etc.

                default:
                    ESP_LOGW(FNAME, "Unsupported OneWire family 0x%02X – ignored", family);
                    break;
            }
        }
    }

    // Optional: set resolution for all DS18B20 (if using espressif/ds18b20 component)
    // for (auto s : temp_sensors_) ds18b20_set_resolution(...);

    return found_sensor;
}

void OneWireBus::busReset()
{
    _onewire->reset(_onewire);
}

esp_err_t OneWireBus::sendCommand(onewire_device_address_t addr, uint8_t cmd)
{
    // send command
    uint8_t tx_buffer[10] = {0};
    tx_buffer[0] = ONEWIRE_CMD_MATCH_ROM;
    memcpy(&tx_buffer[1], &addr, sizeof(addr));
    tx_buffer[sizeof(addr) + 1] = cmd;

    return _onewire->write_bytes(_onewire, tx_buffer, sizeof(tx_buffer));
}

esp_err_t OneWireBus::writeBytes(const uint8_t *buf, uint8_t size)
{
    return _onewire->write_bytes(_onewire, buf, size);
}

esp_err_t OneWireBus::readBytes(uint8_t *buf, size_t size)
{
    return _onewire->read_bytes(_onewire, buf, size);
}

uint8_t OneWireBus::crc8(const uint8_t *data, size_t len)
{
    return onewire_crc8(0, data, len);
}

// @brief Group update for all temperature sensors (non-blocking).
// Call every 100ms) from main sensor task.
bool OneWireBus::groupUpdate(uint32_t now_ms)
{
    const uint32_t UPDATE_INTERVAL_MS = 1000;  // Desired update interval
    const uint32_t MAX_CONVERSION_TIME_MS = 750;  // Max time for 12-bit conversion

    if ( errors > 100 ) {
        ESP_LOGW(FNAME, "Too many errors on OneWire bus, attempting recovery");
        busReset();
        for ( auto s : _all_sensor ) {
            probeAndSetup(s->family()); // a reconnect attempt for all known devices
        }
        if ( _all_sensor.empty() ) {
            // a delayed first connect
            ESP_LOGE(FNAME, "No OneWire devices found after recovery");
            Device *dev = DEVMAN->getDevice(TEMPSENS_DEV);
            if ( dev && !dev->_sensor ) {
                dev->_sensor = probeAndSetup(DS18B20_FAMILY);
            }
        }
        errors = 0; // reset counter in any case
        return false;
    }

    // Read all sensors individually
    for (auto* sensor : _all_sensor) {
        if ( ! sensor->isConverting() ) {
            if ((now_ms - sensor->getLastUpdateTimeMs()) < (UPDATE_INTERVAL_MS + sensor->getLatency())) {
                continue;
            }

            // Start conversion
            if (! sensor->primeRead(now_ms)) {
                ESP_LOGE(FNAME, "Failed to trigger conversion");
                errors++;
            }
            continue;
        }

        if (now_ms - sensor->getConvertStartMs() < MAX_CONVERSION_TIME_MS) {
            continue;
        }
        
        float val;
        if ( !sensor->doRead(val) ) {
            errors++;
            ESP_LOGE(FNAME, "Failed to read sensor %016llX", sensor->getAddress());
            val = NAN; // Invalid value to indicate error (temp only)
        }
        sensor->pushAndPublish(val, sensor->getConvertStartMs());

        ESP_LOGI(FNAME, "OW sensor %016llX: %umsec %.2f", sensor->getAddress(), (unsigned)sensor->getConvertStartMs(), val);
    }

    // Auto connect the temp sensor in case it was not connected during first probe
    // The existance of the bus instance is prove of a configured devices for it.
    if ( _all_sensor.empty() ) {
        errors++;
    }

    return true;
}

//
// private helper
//

OwSens *OneWireBus::getSensorByAddress(onewire_device_address_t addr)
{
    for ( auto s : _all_sensor ) {
        if ( s->getAddress() == addr ) {
            return s;
        }
    }
    return nullptr;
}
