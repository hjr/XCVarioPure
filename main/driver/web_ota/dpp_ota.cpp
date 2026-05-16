/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "dpp_ota.h"
#include "comm/WifiApSta.h"
#include "logdef.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include <esp_dpp.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_ota_ops.h>
#include <esp_https_ota.h>
#include <cJSON.h>

#include <cstring>
#include <cstdio>


#define OTA_RESPONSE_BUF_SIZE  512

static char* response_buf;
static int  response_len = 0;

static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    if (evt->event_id == HTTP_EVENT_ON_DATA) {
        if (!esp_http_client_is_chunked_response(evt->client)) {
            int copy_len = std::min((int)evt->data_len,
                            OTA_RESPONSE_BUF_SIZE - response_len - 1);
            if (copy_len > 0) {
                memcpy(response_buf + response_len, evt->data, copy_len);
                response_len += copy_len;
                response_buf[response_len] = '\0';
            }
        }
    }
    return ESP_OK;
}

// step 2: load OTA binary
static esp_err_t ota_download(const char *binary_url, const char *expected_sha256, const char *cert_pem)
{
    esp_http_client_config_t ota_http_cfg = {
        .url        = binary_url,
        .cert_pem   = cert_pem,
        .timeout_ms = 60000,
        .keep_alive_enable = true,
    };

    esp_https_ota_config_t ota_cfg = {
        .http_config = &ota_http_cfg,
    };

    esp_https_ota_handle_t ota_handle = NULL;
    esp_err_t ret = esp_https_ota_begin(&ota_cfg, &ota_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(FNAME, "esp_https_ota_begin fehlgeschlagen: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    // Download-Loop mit Fortschrittslog
    int image_size = esp_https_ota_get_image_size(ota_handle);
    ESP_LOGI(FNAME, "Binary Größe: %d bytes", image_size);

    while (true) {
        ret = esp_https_ota_perform(ota_handle);
        if (ret != ESP_ERR_HTTPS_OTA_IN_PROGRESS) break;
    }

    if (ret != ESP_OK) {
        ESP_LOGE(FNAME, "OTA Download fehlgeschlagen: %s", esp_err_to_name(ret));
        esp_https_ota_abort(ota_handle);
        return ret;
    }

    // SHA256 prüfen
    uint8_t sha256_raw[32];
    char    sha256_str[65] = {0};
    const esp_partition_t *ota_partition = esp_ota_get_next_update_partition(NULL);
    esp_partition_get_sha256(ota_partition, sha256_raw);
    for (int i = 0; i < 32; i++) {
        sprintf(sha256_str + (i * 2), "%02x", sha256_raw[i]);
    }

    if (strncasecmp(sha256_str, expected_sha256, 64) != 0) {
        ESP_LOGE(FNAME, "SHA256 Mismatch!\n  Erwartet: %s\n  Erhalten: %s",
                 expected_sha256, sha256_str);
        esp_https_ota_abort(ota_handle);
        return ESP_FAIL;
    }

    ESP_LOGI(FNAME, "SHA256 OK");

    ret = esp_https_ota_finish(ota_handle);
    if (ret == ESP_OK) {
        ESP_LOGI(FNAME, "OTA erfolgreich — Reboot in 2s");
        vTaskDelay(pdMS_TO_TICKS(2000));
        esp_restart();
    }
    return ret;
}

// exec ota
// static esp_err_t run_ota(const dpp_ota_config_t *config)
// {
//     // URL build up: /api/firmware?hw=1.2&sn=SN-00042
//     char url[256];
//     snprintf(url, sizeof(url), "%s?hw=%s&sn=%s",
//              config->update_server_url,
//              config->hw_version,
//              config->serial_number);

//     ESP_LOGI(FNAME, "OTA URL: %s", url);

//     esp_http_client_config_t http_cfg = {
//         .url            = url,
//         .cert_pem       = config->server_cert_pem,
//         .timeout_ms     = 30000,
//     };

//     esp_https_ota_config_t ota_cfg = {
//         .http_config = &http_cfg,
//     };

//     esp_err_t ret = esp_https_ota(&ota_cfg);
//     if (ret == ESP_OK) {
//         ESP_LOGI(FNAME, "OTA successful, reboot...");
//         esp_restart();
//     } else {
//         ESP_LOGE(FNAME, "OTA failed: %s", esp_err_to_name(ret));
//     }
//     return ret;
// }

// step 1: firmware-info request + JSON parsing
static esp_err_t run_ota(const dpp_ota_config_t *config)
{
    // URL aufbauen
    char url[256];
    snprintf(url, sizeof(url), "%s?hw=%s&sn=%s&sw=%s",
             config->update_server_url,
             config->hw_version,
             config->serial_number,
             config->sw_version);

    ESP_LOGI(FNAME, "Firmware-Info Request: %s", url);

    response_len = 0;
    response_buf = (char*)malloc(OTA_RESPONSE_BUF_SIZE);
    memset(response_buf, 0, OTA_RESPONSE_BUF_SIZE);

    esp_http_client_config_t http_cfg = {
        .url           = url,
        .cert_pem      = config->server_cert_pem,
        .timeout_ms    = 15000,
        .event_handler = http_event_handler,
    };

    esp_http_client_handle_t client = esp_http_client_init(&http_cfg);
    esp_err_t ret = esp_http_client_perform(client);
    int status    = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);

    if (ret != ESP_OK || status != 200) {
        ESP_LOGE(FNAME, "HTTP Fehler: ret=%s status=%d",
                 esp_err_to_name(ret), status);
        free(response_buf);
        return ESP_FAIL;
    }

    ESP_LOGI(FNAME, "Server: %s", response_buf);

    // JSON parsen
    cJSON *root      = cJSON_Parse(response_buf);
    if (!root) {
        ESP_LOGE(FNAME, "JSON Parse fehlgeschlagen");
        free(response_buf);
        return ESP_FAIL;
    }

    cJSON *j_update  = cJSON_GetObjectItem(root, "update_available");
    cJSON *j_version = cJSON_GetObjectItem(root, "version");
    cJSON *j_url     = cJSON_GetObjectItem(root, "url");
    cJSON *j_sha256  = cJSON_GetObjectItem(root, "sha256");

    // Kein Update?
    if (cJSON_IsBool(j_update) && !cJSON_IsTrue(j_update)) {
        ESP_LOGI(FNAME, "Bereits aktuell (v%s)",
                 cJSON_IsString(j_version) ? j_version->valuestring : "?");
        cJSON_Delete(root);
        free(response_buf);
        return ESP_OK;
    }

    // Pflichtfelder prüfen
    if (!cJSON_IsString(j_url) || !cJSON_IsString(j_sha256)) {
        ESP_LOGE(FNAME, "JSON: url oder sha256 fehlt");
        cJSON_Delete(root);
        free(response_buf);
        return ESP_FAIL;
    }

    // Werte sichern vor cJSON_Delete
    char binary_url[256];
    char expected_sha256[65];
    char new_version[32];
    strncpy(binary_url,      j_url->valuestring,    sizeof(binary_url) - 1);
    strncpy(expected_sha256, j_sha256->valuestring, sizeof(expected_sha256) - 1);
    strncpy(new_version,
            cJSON_IsString(j_version) ? j_version->valuestring : "?",
            sizeof(new_version) - 1);
    cJSON_Delete(root);

    ESP_LOGI(FNAME, "Update verfügbar: v%s → v%s", config->sw_version, new_version);
    free(response_buf);

    return ota_download(binary_url, expected_sha256, config->server_cert_pem);
}

// dpp ota task
void dpp_ota_task(void *pvParam)
{

    const dpp_ota_config_t *config = (const dpp_ota_config_t *)pvParam;
    EventGroupHandle_t event_group = WIFI->getDppEventGroup();

    // Wait for the internet connection or error (Timeout: 3 minutes)
    EventBits_t bits = xEventGroupWaitBits(event_group,
                           DPP_CONNECTED_BIT | DPP_FAILED_BIT,
                           pdFALSE, pdFALSE,
                           pdMS_TO_TICKS(180000));

    if (bits & DPP_CONNECTED_BIT) {
        run_ota(config);
    } else {
        ESP_LOGE(FNAME, "DPP/WiFi Timeout oder Fehler");
    }

    vTaskDelete(NULL);
}

