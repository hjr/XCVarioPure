/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

typedef struct {
    const char     *update_server_url; // e.g., "https://update.xcvario.com/api/firmware"
    const char     *hw_version;        // e.g., "2.0"
    const char     *serial_number;     // e.g., "8974"
    const char     *sw_version;        // e.g., "1.0.3"
    const char     *server_cert_pem;   // Root-CA for HTTPS verification
} dpp_ota_config_t;

void dpp_ota_task(void *pvParam);
