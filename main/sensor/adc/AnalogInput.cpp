/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "AnalogInput.h"
#include "sensor/SensorMgr.h"
#include "logdefnone.h"

#include <esp_adc/adc_cali.h>


// ADC_ATTEN_DB_0 No : Input attenumation, ADC can measure up to approx. 800 mV.
// ADC_ATTEN_DB_2_5  : The input voltage of ADC will be attenuated, extending the range of measurement to up to approx. 1100 mV.
// ADC_ATTEN_DB_6    : The input voltage of ADC will be attenuated, extending the range of measurement to up to approx. 1350 mV.
// ADC_ATTEN_DB_11   : The input voltage of ADC will be attenuated, extending the range of measurement to up to approx. 2600 mV.

//  ADC_CHANNEL_7,     /*!< ADC1 channel 7 is GPIO_NUM_35 */
//  ADC_CHANNEL_6,     /*!< ADC1 channel 6 is GPIO_NUM_34 */
//  ADC_CHANNEL_2,     /*!< ADC2 channel 2 is GPIO_NUM_2 */

constexpr const unsigned DEFAULT_VREF = 1100;

adc_oneshot_unit_handle_t AnalogInput::_adc_handle = nullptr;

AnalogInput::AnalogInput(void *buf, size_t cap, uint32_t ums, int pm) :
    SensorTP<float>(buf, cap, ums, pm),
    _adc_ch(ADC_CHANNEL_7)
{
}

AnalogInput::~AnalogInput()
{
    if (_adc_cali) {
        adc_cali_delete_scheme_line_fitting(_adc_cali);
        _adc_cali = nullptr;
    }
    // the _adc_handle is a shared resource so that it would need a use counter,
    // or just to leave it.
    // if (_adc_handle) {
    //     adc_oneshot_del_unit(_adc_handle);
    //     _adc_handle = nullptr;
    // }
}

// can handle only one unit, but multiple channels on it
void AnalogInput::begin(adc_atten_t attenuation, adc_unit_t unit, adc_channel_t ch, bool calibration)
{
    ESP_LOGI(FNAME, "begin() unit: %d ch:%d cal:%d att: %d", unit, ch, calibration, attenuation);

    if (!_adc_handle) {
        adc_oneshot_unit_init_cfg_t init_config = {
            .unit_id = unit,
            .clk_src = (adc_oneshot_clk_src_t)0,
            .ulp_mode = ADC_ULP_MODE_DISABLE,
        };
        ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &_adc_handle));
    }

    adc_oneshot_chan_cfg_t config = {
        .atten = attenuation,
        .bitwidth = ADC_BITWIDTH_12,
    };
    _adc_ch = ch;
    ESP_ERROR_CHECK(adc_oneshot_config_channel(_adc_handle, _adc_ch, &config));

    if (calibration) {
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = attenuation,
            .bitwidth = ADC_BITWIDTH_12,
            .default_vref = DEFAULT_VREF,
        };
        if (_adc_cali) {
            adc_cali_delete_scheme_line_fitting(_adc_cali);
            _adc_cali = nullptr;
        }
        adc_cali_create_scheme_line_fitting(&cali_config, &_adc_cali);
    }
}

bool AnalogInput::doRead(float& val) {
    constexpr int BATCH = 3;
    int adc = 0;
    esp_err_t err = ESP_OK;
    for (int i = 0; i < BATCH; i++) {
        int rawadc = 0;
        err = adc_oneshot_read(_adc_handle, _adc_ch, &rawadc);
        if ( err != ESP_OK ) { break; }
        adc += rawadc;
    }
    if ( err == ESP_OK ) {
        int raw = adc / BATCH;
        if ( _adc_cali ) {
            adc = raw;
            adc_cali_raw_to_voltage(_adc_cali, adc, &raw);
        }
        val = static_cast<float>(raw);
        return true;
    }

    return false;
}
