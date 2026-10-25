/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "SubMenuOTA.h"

#include "comm/WifiApSta.h"
#include "primitive/SetupMenu.h"
#include "primitive/SetupMenuSelect.h"
#include "primitive/SetupMenuChar.h"
#include "SetupNG.h"
#include "Colors.h"
#include "AdaptUGC.h"
#include "logdef.h"

#include <cstdint>

extern AdaptUGC *MYUCG;

wifi_ap_record_t *wifi_scan_results = nullptr;
constexpr const uint16_t MAX_WIFI_SCAN_RESULTS = 20;

static int action_select_ap(SetupMenuSelect* p) {
    if (p->getValue() == 101) {
        if ( !WIFI ) {
            WifiApSta::createWifiApSta();
        }
        if ( !WIFI->isSTA() ) { 
            ESP_LOGI(FNAME, "Switch to STA mode");
            WIFI->ConfigureIntf(8884);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        if ( wifi_scan_results == nullptr ) {
            wifi_scan_results = new wifi_ap_record_t[MAX_WIFI_SCAN_RESULTS];
        }
        uint16_t num_aps = 0;
        WIFI->scanAPs(wifi_scan_results, num_aps, MAX_WIFI_SCAN_RESULTS);
        ESP_LOGI(FNAME, "Found %d APs", num_aps);
        p->delAllEntries();
        p->addEntry("Select One", 100);
        for (int i = 0; i < num_aps; ++i) {
            p->addEntry((const char*)wifi_scan_results[i].ssid, i);
            ESP_LOGI(FNAME, "Found AP: %s", wifi_scan_results[i].ssid);
        }
        p->addEntry("Refresh Scan", 101);
    }
    else if (p->getValue() < 100) {
        int idx = p->getValue();
        ESP_LOGI(FNAME, "Selected AP: %s", wifi_scan_results[idx].ssid);
        ota_ap.set((const char*)wifi_scan_results[idx].ssid);
        p->delAllEntries();
        p->addEntry((const char*)wifi_scan_results[idx].ssid, idx);
        p->addEntry("Refresh Scan", 101);
    }
    else if (p->getValue() == 100) {
        ESP_LOGI(FNAME, "Cancel AP selection");
    }
    p->setSelect(0);
    return 0;
}

static int set_dirty(SetupMenuChar* p) {
    ota_pwd.setDirty();
    return 0;
}

static int action_check_updates(SetupMenuSelect* p) {
    if (p->getSelect() == 1) {
        // check for updates
    }
    else if ( p->getSelect() == 2) {
        // re-start into webserver mode
        p->reBoot();
    }
    p->setSelect(0);
    return 0;
}

void software_menu_create_OTA(SetupMenu *top) {

    // text
    SetupMenuSelect *text = new SetupMenuSelect(">Internet access<", RST_NONE);
    text->lock();
    top->addEntry(text);

    SetupMenuSelect *ap_list = new SetupMenuSelect("AP", RST_NONE, action_select_ap, nullptr);
    ap_list->setHelp("Select an access point to connect to the internet");
    if (ota_ap.get().id[0] != '\0') {
        ap_list->addEntry(ota_ap.get().id, 0);
    }
    else {
        ap_list->addEntry("Cancel", 100);
    }
    ap_list->addEntry("Scan", 101);
    top->addEntry(ap_list);

    SetupMenuChar *ap_pw = new SetupMenuChar("PW", "Aa0+#", 32, RST_NONE, set_dirty, ota_pwd.get().id);
    ap_pw->setHelp("Enter the password for the selected access point");
    top->addEntry(ap_pw);

    SetupMenuSelect* upd = new SetupMenuSelect("Update", RST_NONE, action_check_updates, &software_update);
    upd->setHelp("Update using the internet connection of your smart phone, or upload a binary using the embedded webserver.");
    upd->addEntry("Cancel");
    upd->addEntry("Check for");
    upd->addEntry("Start Webserver");
    top->addEntry(upd);

}


void free_ota_menu()
{
    if ( wifi_scan_results ) {
        delete[] wifi_scan_results;
        wifi_scan_results = nullptr;
    }
}
