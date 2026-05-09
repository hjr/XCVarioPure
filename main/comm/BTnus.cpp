/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "BTnus.h"

#include "BlueTooth.h"
#include "DataLink.h"
#include "setup/SetupCommon.h"
#include "math/Floats.h"
#include "logdefnone.h"

#include <esp_bt.h>
#include <esp_gatts_api.h>
#include <esp_gap_ble_api.h>
#include <esp_bt_main.h>

#include <string>
#include <cstring>
#include <mutex>


// Nordic UART Service
// #define NUS_SERVICE_UUID   0x6E400001
// #define NUS_RX_UUID        0x6E400002  // Client → ESP (WRITE)
// #define NUS_TX_UUID        0x6E400003  // ESP → Client (NOTIFY)

BTnus *BLUEnus = nullptr;
static esp_gatt_if_t my_gatts_if = ESP_GATT_IF_NONE;
// static uint8_t nus_rx_buf[256];

static const uint8_t nus_service_uuid128[ESP_UUID_LEN_128] = {
    0x9E,0xCA,0xDC,0x24,0x0E,0xE5,0xA9,0xE0,  0x93,0xF3,0xA3,0xB5,0x01,0x00,0x40,0x6E
};
static const uint8_t nus_rx_uuid128[ESP_UUID_LEN_128] = {
    0x9E,0xCA,0xDC,0x24,0x0E,0xE5,0xA9,0xE0,  0x93,0xF3,0xA3,0xB5,0x02,0x00,0x40,0x6E
};
static const uint8_t nus_tx_uuid128[ESP_UUID_LEN_128] = {
    0x9E,0xCA,0xDC,0x24,0x0E,0xE5,0xA9,0xE0,  0x93,0xF3,0xA3,0xB5,0x03,0x00,0x40,0x6E
};
static const uint8_t issc_service_uuid128[ESP_UUID_LEN_128] = {
    0x55,0xe4,0x05,0xd2,0xaf,0x9f,0xa9,0x8f,  0xe5,0x4a,0x7d,0xfe,0x43,0x53,0x53,0x49
};
static const uint8_t issc_rx_uuid128[ESP_UUID_LEN_128] = {
    0xb3,0x9b,0x72,0x34,0xbe,0xec,0xd4,0xa8,  0xf4,0x43,0x41,0x88,0x43,0x53,0x53,0x49
};
static const uint8_t issc_tx_uuid128[ESP_UUID_LEN_128] = {
    0x16,0x96,0x24,0x47,0xc6,0x23,0x61,0xba,  0xd9,0x4b,0x1e,0x53,0x53,0x35,0x39,0x49
};
static uint8_t* ble_service_uuid128 = (uint8_t*)nus_service_uuid128;
static uint8_t* ble_rx_uuid128 = (uint8_t*)nus_rx_uuid128;
static uint8_t* ble_tx_uuid128 = (uint8_t*)nus_tx_uuid128;

static const std::string_view service_name[] = { "BT nrf", "BT issc" };

static const esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .peer_addr = { 0, 0, 0, 0, 0, 0 },
    .peer_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};


class BTnus_EVENT_HANDLER
{
public:
    // BLE server event handler
    static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                    esp_ble_gatts_cb_param_t *param)
    {
        if(!BLUEnus) return; // safety check

        switch (event)
        {
        case ESP_GATTS_MTU_EVT:
            BLUEnus->peer_mtu = param->mtu.mtu;
            ESP_LOGI(FNAME, "Peer MTU: %d", BLUEnus->peer_mtu);
            break;

        case ESP_GATTS_CONNECT_EVT: // Client connected
            ESP_LOGI(FNAME, "Client connected, conn_id: %d", param->connect.conn_id);
            BLUEnus->my_conn_id = param->connect.conn_id;
            BLUEnus->nus_notify_enabled = true;
            break;

        case ESP_GATTS_DISCONNECT_EVT: // Client disconnected
        {
            ESP_LOGI(FNAME, "Client disconnected, reason: %d", param->disconnect.reason);
            BLUEnus->nus_notify_enabled = false;
            BLUEnus->my_conn_id = 0xFFFF;

            // Restart advertising
            esp_ble_gap_start_advertising((esp_ble_adv_params_t *)&adv_params);
            break;
        }
        case ESP_GATTS_READ_EVT:
        {
            ESP_LOGI(FNAME, "Char read, handle: %d", param->read.handle);
            // Optionally, modify the value here before sending it to the client.
            esp_gatt_rsp_t rsp = {};
            rsp.attr_value.handle = param->read.handle;
            rsp.attr_value.len = 0;
            esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
            break;
        }
        case ESP_GATTS_WRITE_EVT:
            ESP_LOGI(FNAME, "Char write, handle: %d", param->write.handle);
            if (param->write.is_prep)
            {
                // Handle prepare write (for large data writes)
            }
            else
            {
                // Check if the write is to the CCCD for notifications/indications
                if (param->write.len == 2 && param->write.handle == BLUEnus->tx_cccd_handle)
                {
                    uint16_t cccd_value = param->write.value[1] << 8 | param->write.value[0];
                    if (cccd_value == 0x0001)
                    {
                        BLUEnus->nus_notify_enabled = true;
                        ESP_LOGI(FNAME, "Notifications enabled");
                    }
                    else if (cccd_value == 0x0002)
                    {
                        ESP_LOGI(FNAME, "Indications enabled");
                    }
                    else if (cccd_value == 0x0000)
                    {
                        ESP_LOGI(FNAME, "Notifications/indications disabled");
                        BLUEnus->nus_notify_enabled = false;
                    }

                    // Respond explicitly
                    esp_gatt_rsp_t rsp = {};
                    rsp.attr_value.handle = param->write.handle;
                    rsp.attr_value.len = 2;
                    rsp.attr_value.value[0] = cccd_value & 0xFF;
                    rsp.attr_value.value[1] = cccd_value >> 8;
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, &rsp);
                }
                else
                {
                    // Handle normal tx data
                    int count = param->write.len;
                    char *rxBuf = (char *)param->write.value;
                    if (count > 0)
                    {
                        // rxBuf[count] = '\0';
                        DataLink *dltarget = nullptr;
                        {
                            std::lock_guard<SemaphoreMutex> lock(BLUEnus->_dlink_mutex);
                            auto dlit = BLUEnus->_dlink.begin();
                            if (dlit != BLUEnus->_dlink.end())
                            {
                                dltarget = dlit->second;
                            }
                        }
                        if (dltarget)
                        {
                            dltarget->process(rxBuf, count);
                        }
                    }

                    std::string rx((char *)param->write.value, param->write.len);
                    if (rx.size()>0)
                    {
                        ESP_LOGI(FNAME, ">BTLE RX %d bytes: %s", rx.length(), rx.c_str());
                    }
                }
            }
            break;

        case ESP_GATTS_CONF_EVT:
            static int congestion = 0;
            if (param->conf.status != ESP_GATT_OK)
            {
                ESP_LOGE(FNAME, "Notification/Indication status: %d", param->conf.status);
                if (param->conf.status == ESP_GATT_CONGESTED)
                {
                    ESP_LOGI(FNAME, "Congested, pacing: %d", congestion);
                    if (congestion < 300)
                        congestion++;
                }
            }
            else
            {
                ESP_LOGD(FNAME, "Send OK, pacing: %d", congestion);
                if (congestion)
                    congestion--;
            }
            break;

        case ESP_GATTS_REG_EVT:
        {
            ESP_LOGI(FNAME, "ESP_GATTS_REG_EVT, registering service...");
            my_gatts_if = gatts_if; // Store the GATT server interface

            esp_gatt_srvc_id_t service_id = {
                .id = {
                    .uuid = {
                        .len = ESP_UUID_LEN_128,
                        .uuid = {},
                    },
                    .inst_id = 0,
                },
                .is_primary = true,
            };
            memcpy(service_id.id.uuid.uuid.uuid128, ble_service_uuid128, ESP_UUID_LEN_128);
            esp_ble_gatts_create_service(gatts_if, &service_id, 16); // Create service with max 16 handles
            break;
        }
        case ESP_GATTS_CREATE_EVT:
        {
            ESP_LOGI(FNAME, "ESP_GATTS_CREATE_EVT, service created with handle %d", param->create.service_handle);
            BLUEnus->service_handle = param->create.service_handle; // Store the service handle
            esp_ble_gatts_start_service(BLUEnus->service_handle);
            break;
        }
        case ESP_GATTS_START_EVT:
        {
            // Add characteristics to the service
            esp_bt_uuid_t char_uuid = {
                .len = ESP_UUID_LEN_128,
                .uuid = {},
            };
            memcpy(char_uuid.uuid.uuid128, ble_rx_uuid128, ESP_UUID_LEN_128);
            // esp_attr_value_t ble_attr = {
            //     .attr_max_len = sizeof(nus_rx_buf),
            //     .attr_len     = 0,
            //     .attr_value   = ble_rx_uuid128,
            // };
            esp_ble_gatts_add_char(BLUEnus->service_handle, &char_uuid,
                                   ESP_GATT_PERM_WRITE,
                                   ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR,
                                   NULL, NULL);

            memcpy(char_uuid.uuid.uuid128, ble_tx_uuid128, ESP_UUID_LEN_128);
            esp_ble_gatts_add_char(BLUEnus->service_handle, &char_uuid,
                                   0,
                                   ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                                   NULL, NULL);

            esp_bt_uuid_t cccd_uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG}, // 0x2902
            };
            esp_attr_value_t cccd_val = {
                .attr_max_len = 2,
                .attr_len = 2,
                .attr_value = (uint8_t[]){0x00, 0x00}, // default: notifications disabled
            };
            esp_ble_gatts_add_char_descr(BLUEnus->service_handle, &cccd_uuid,
                                    ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                    &cccd_val, NULL);

            esp_ble_gap_start_advertising((esp_ble_adv_params_t *)&adv_params);
            BLUEnus->_server_running = true;
            break;
        }
        case ESP_GATTS_STOP_EVT:
        {
            if ( BLUEnus->service_handle == param->stop.service_handle ) {
                ESP_LOGI(FNAME, "ESP_GATTS_STOP_EVT, service stopped");
                esp_ble_gatts_delete_service(param->stop.service_handle);
            }
            break;
        }
        case ESP_GATTS_DELETE_EVT:
        {
            if ( BLUEnus->service_handle == param->stop.service_handle ) {
                esp_ble_gatts_delete_service(param->stop.service_handle);
                BLUEnus->service_handle = 0;
                BLUEnus->_server_running = false;
            }
            break;
        }
        case ESP_GATTS_ADD_CHAR_EVT:
        {
            const esp_bt_uuid_t *uuid = &param->add_char.char_uuid;
            if (uuid->len == ESP_UUID_LEN_128 && memcmp(uuid->uuid.uuid128, ble_rx_uuid128, ESP_UUID_LEN_128) == 0)
            {
                BLUEnus->rx_char_handle = param->add_char.attr_handle;
                ESP_LOGI(FNAME, "NUS RX characteristic added, handle=%d status=%d", BLUEnus->rx_char_handle, param->add_char.status);
            }
            else
            {
                if (uuid->len == ESP_UUID_LEN_128 && memcmp(uuid->uuid.uuid128, ble_tx_uuid128, ESP_UUID_LEN_128) == 0)
                {
                    BLUEnus->tx_char_handle = param->add_char.attr_handle;
                    ESP_LOGI(FNAME, "NUS TX characteristic added, handle=%d status=%d", BLUEnus->tx_char_handle, param->add_char.status);
                }
            }
            break;
        }
        case ESP_GATTS_ADD_CHAR_DESCR_EVT:
            BLUEnus->tx_cccd_handle = param->add_char_descr.attr_handle;
            ESP_LOGI(FNAME, "NUS characteristic desc added, handle=%d status=%d", BLUEnus->tx_cccd_handle, param->add_char_descr.status);
            break;

        case ESP_GATTS_EXEC_WRITE_EVT:
        case ESP_GATTS_UNREG_EVT:
        case ESP_GATTS_ADD_INCL_SRVC_EVT:
        case ESP_GATTS_OPEN_EVT:
        case ESP_GATTS_CANCEL_OPEN_EVT:
        case ESP_GATTS_CLOSE_EVT:
        case ESP_GATTS_LISTEN_EVT:
        case ESP_GATTS_CONGEST_EVT:
        case ESP_GATTS_RESPONSE_EVT:
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        case ESP_GATTS_SET_ATTR_VAL_EVT:
        case ESP_GATTS_SEND_SERVICE_CHANGE_EVT:
        default:
            ESP_LOGI(FNAME, "Unhandled GATTS event: %d", event);
            break;
        }
    }

    // GAP Event Handler
    static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
    {
        switch (event)
        {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        {
            // Advertising data was successfully set
            ESP_LOGI(FNAME, "Advertising data set successfully");
            break;
        }
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        {
            // Scanning parameters were successfully set
            ESP_LOGI(FNAME, "Scan parameters set successfully");
            break;
        }
        case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        {
            // Scanning started
            if (param->scan_start_cmpl.status == ESP_BT_STATUS_SUCCESS)
            {
                ESP_LOGI(FNAME, "Scan started successfully");
            }
            else
            {
                ESP_LOGE(FNAME, "Scan start failed, status: %d", param->scan_start_cmpl.status);
            }
            break;
        }
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        {
            // Connection parameters have been updated
            ESP_LOGI(FNAME, "Connection parameters updated: min_int=%d, max_int=%d, latency=%d, timeout=%d",
                     param->update_conn_params.min_int,
                     param->update_conn_params.max_int,
                     param->update_conn_params.latency,
                     param->update_conn_params.timeout);
            break;
        }
        case ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT:
            if (param->pkt_data_length_cmpl.status == ESP_OK) {
                ESP_LOGI(FNAME, "Packet length set to %d", param->pkt_data_length_cmpl.params.tx_len);
            }
            break;

        default:
            ESP_LOGI(FNAME, "Unhandled GAP event: %d", event);
            break;
        }
    }
};

BTnus::BTnus() :
    InterfaceCtrl(true),
    core(BlueTooth::instance())
{
    ESP_LOGI(FNAME, "BTnus constructor");
    core.acquire();
}
BTnus::~BTnus()
{
    stop();
    BLUEnus = nullptr;
    core.release();
}

// bool BTnus::selfTest(){
// 	ESP_LOGI(FNAME,"SerialBLE::selfTest");
// 	return true;
// }

const char *BTnus::getStringId() const
{
    return  (ble_service_uuid128 == (uint8_t*)nus_service_uuid128) ? service_name[0].data() : service_name[1].data();
}


void BTnus::ConfigureIntf(int cfg)
{
    if ( ! _server_running ) {
        ESP_LOGI(FNAME, "BTle ConfigureIntf: %d", cfg);
        // if ( cfg == SEEYOU_P ) {
        //     // Configure for SeeYou
        //     ble_service_uuid128 = (uint8_t*)issc_service_uuid128;
        //     ble_rx_uuid128 = (uint8_t*)issc_rx_uuid128;
        //     ble_tx_uuid128 = (uint8_t*)issc_tx_uuid128;
        // }
        // else {
        //     ble_service_uuid128 = (uint8_t*)nus_service_uuid128;
        //     ble_rx_uuid128 = (uint8_t*)nus_rx_uuid128;
        //     ble_tx_uuid128 = (uint8_t*)nus_tx_uuid128;
        // }
        start();
    }
    else {
        ESP_LOGI(FNAME, "BTle already running");
    }
}

int BTnus::Send(const char *msg, int &len, int port)
{
    int ret = 0;
    if (my_conn_id==0xFFFF || !tx_char_handle || my_gatts_if==ESP_GATT_IF_NONE) {
        ESP_LOGD(FNAME,"SendTry BTnus, id %d, tx %d, gatt %d", my_conn_id, tx_char_handle, my_gatts_if);
        return 0; // not connected; pretend everything is fine
    }
    if (nus_notify_enabled) {
        ESP_LOGD(FNAME,"Send BTnus: %s", msg);
        if ( len >  peer_mtu - 3 ) {
            len = peer_mtu - 3; // adjust to MTU
            ret = fast_ceilf(peer_mtu / 700.f) * 15; // estimate time for larger packets (700 byte chunks, 15ms each)
        }
        esp_ble_gatts_send_indicate(my_gatts_if, my_conn_id, tx_char_handle, len, (uint8_t*)msg, false);
    }
    return ret;
}

bool BTnus::start()
{
	ESP_LOGI(FNAME, "BTle start()");

	// Register GAP and GATT events
	esp_ble_gap_register_callback(BTnus_EVENT_HANDLER::gap_event_handler);

	esp_ble_gatts_register_callback(BTnus_EVENT_HANDLER::gatts_event_handler);
    esp_ble_gatts_app_register(0x42);

    esp_ble_gap_set_device_name(SetupCommon::getID());
    static esp_ble_adv_data_t adv_data = {
        .set_scan_rsp = false,
        .include_name = true,
        .include_txpower = false,
        .min_interval = 0x20,
        .max_interval = 0x40,
        .appearance = 0x00,
        .manufacturer_len = 0,
        .p_manufacturer_data =  nullptr,
        .service_data_len = 0,
        .p_service_data = nullptr,
        .service_uuid_len = ESP_UUID_LEN_128,
        .p_service_uuid = (uint8_t*)nus_service_uuid128,
        .flag = ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT,
    };
    esp_ble_gap_config_adv_data(&adv_data);

    esp_log_level_set("BT_GATT", ESP_LOG_DEBUG);
    esp_log_level_set("GATTS", ESP_LOG_DEBUG);

    return true;
}

void BTnus::stop()
{
    ESP_LOGI(FNAME, "BTle stop()");
    esp_ble_gap_stop_advertising(); // should actually wait for ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT
    vTaskDelay(100 / portTICK_PERIOD_MS);
    if ( BLUEnus->my_conn_id != 0xFFFF ) {
        ESP_LOGI(FNAME, "Closing connection with conn_id: %d", BLUEnus->my_conn_id);
        esp_ble_gatts_close(my_gatts_if, BLUEnus->my_conn_id);
        BLUEnus->my_conn_id = 0xFFFF;
    }
    esp_ble_gatts_stop_service(service_handle); // -> ESP_GATTS_STOP_EVT
    // wait max. 2 seconds for service to be stopped and deleted, otherwise just continue with unregistration
    int wait_time = 0;
    while(_server_running && wait_time < 2000) { // wait for service to be stopped and deleted
        vTaskDelay(50 / portTICK_PERIOD_MS);
        wait_time += 50;
    }
    esp_ble_gatts_app_unregister(my_gatts_if); // -> ESP_GATTS_UNREG_EVT
    _server_running = false;
    my_gatts_if = ESP_GATT_IF_NONE;
}