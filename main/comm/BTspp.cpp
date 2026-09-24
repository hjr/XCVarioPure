/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "BTspp.h"


#include "BlueTooth.h"
#include "setup/SetupCommon.h"
#include "DataLink.h"

#include "logdefnone.h"

#include <driver/uart.h>
#include <esp_bt.h>
#include <esp_bt_main.h>
#include <esp_bt_device.h>
#include <esp_gap_bt_api.h>
#include <esp_spp_api.h>

constexpr int RFCOMM_SERVER_CHANNEL = 1;
// #define HEARTBEAT_PERIOD_MS 50
// #define SPP_SERVICE_BUFFER_SIZE 1024


BTspp *BLUEspp = nullptr;

class BTspp_EVENT_HANDLER
{
public:

// SPP Callback function
static void spp_event_handler(esp_spp_cb_event_t event, esp_spp_cb_param_t* param) {
    // assert(BLUEspp) this is
    switch (event) {
    case ESP_SPP_INIT_EVT:
        ESP_LOGI(FNAME, "SPP initialized");
        // Start listening for incoming connections
        esp_spp_start_srv(ESP_SPP_SEC_AUTHENTICATE, ESP_SPP_ROLE_SLAVE, RFCOMM_SERVER_CHANNEL, SetupCommon::getID());
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        BLUEspp->_server_running = true;
        break;

    case ESP_SPP_SRV_STOP_EVT:
        ESP_LOGI(FNAME, "SPP server stoped");
        BLUEspp->_server_running = true;
        break;

    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(FNAME, "SPP rcomm opened, handle: %u", (unsigned)param->open.handle);
        BLUEspp->_client_handle = param->open.handle;
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
        break;

        // never received this for rfcomm channels
        // case ESP_SPP_OPEN_EVT:
        // 	ESP_LOGI(FNAME, "SPP connection opened, handle: %d", param->open.handle);
        // 	BLUEspp->_client_handle = param->open.handle;
        // 	esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
        // 	break;

    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(FNAME, "SPP connection closed, handle: %u", (unsigned)param->close.handle);
        BLUEspp->_client_handle = 0;
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        break;

    case ESP_SPP_DATA_IND_EVT: {
        ESP_LOGI(FNAME, "Received data, handle: %u, length: %d", (unsigned)param->data_ind.handle, param->data_ind.len);
        // Process received data
        int count = param->data_ind.len;
        char* rxBuf = (char*)param->data_ind.data;
        if (count > 0) {
            rxBuf[count] = '\0';
            DataLink* dltarget = BLUEspp->_dlink.begin()->link;  // only the first entry is used here
            if (dltarget) {
                dltarget->process(rxBuf, count);
            }
        }
        break;
    }
    default:
        break;
    }
}
};

BTspp::BTspp() :
    InterfaceCtrl(true),
    core(BlueTooth::instance())
{
    ESP_LOGI(FNAME, "BTspp constructor");
    core.acquire();
}

BTspp::~BTspp()
{
    stop();
    core.release();
}

// bool BTspp::selfTest()
// {
//    return isRunning();
// }

// void BTspp::ConfigureIntf(int cfg)
// {
//     // maybe fine like this
// }

int BTspp::Send(const char *msg, int &len, int port)
{
    if (_client_handle)
    {
        // ESP_LOGI(FNAME,"Send BTspp, len %d", len);
        esp_err_t err = esp_spp_write(_client_handle, len, (uint8_t *)msg);
        if (err == ESP_OK)
        {
            return 0;
        }
        return 4; // wild guess
    }
    return 0;
}

bool BTspp::start()
{
    ESP_LOGI(FNAME, "BTspp start()");

    // Register SPP callback function
    if (esp_spp_register_callback(BTspp_EVENT_HANDLER::spp_event_handler))
    {
        ESP_LOGE(FNAME, "SPP callback registration failed");
        return false;
    }

    // Initialize SPP
    esp_spp_cfg_t spp_cfg = {
        .mode = ESP_SPP_MODE_CB,
        .enable_l2cap_ertm = false,
        .tx_buffer_size = 0 // only for VFS mode
    };
    if (esp_spp_enhanced_init(&spp_cfg) != ESP_OK)
    {
        ESP_LOGE(FNAME, "SPP initialization failed");
        return false;
    }
    ESP_LOGI(FNAME, "SPP server started and waiting for connections...");

    return true;
}

void BTspp::stop()
{
    if (_client_handle)
    {
        ESP_LOGI(FNAME, "Disconnecting SPP client");
        esp_spp_disconnect(_client_handle);

        // Wait for the disconnection event
        while (_client_handle != 0)
        {
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        ESP_LOGI(FNAME, "SPP client disconnected");
        esp_spp_deinit();
        _server_running = false;
    }
}
