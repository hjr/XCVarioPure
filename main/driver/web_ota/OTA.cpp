/*
 * OTA.cpp
 *
 *  Created on: Mar 17, 2020
 *      Author: iltis
 *
 *
 *
 */

#include "OTA.h"

#include "Webserver.h"
#include "screen/ScreenRoot.h"
#include "IpsDisplay.h"
#include "AdaptUGC.h"
#include "Colors.h"
#include "driver/time/Clock.h"
#include "comm/WifiApSta.h"
#include "logdef.h"

#include "qrcodegen.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_netif.h>
#include <esp_ota_ops.h>

extern AdaptUGC *MYUCG;

typedef void (*dpp_ota_qr_cb_t)(const char *uri);  // callback QR renderer


// the menu to select classic upload or the easy connect method
// static int ota_select(SetupMenuSelect *p) {
//     if (p->getSelect() == 0) {
//         MenuRoot->begin(new OTAdpp(""));
//     }
//     else {
//         MenuRoot->begin(new OTAap(""));
//     }
//     return 0;
// }
// void ota_selection_create(SetupMenu* top) {
//     SetupMenuSelect *space = new SetupMenuSelect("", RST_NONE);
//     space->lock();
//     top->addEntry(space);
//     SetupMenuSelect *ota = new SetupMenuSelect("Firmware Update", RST_NONE, ota_select);
//     ota->addEntry("OTA with Easy Connect");
//     ota->addEntry("I provide a binary");
//     ota->setHelp("OTA using the internet connection of your smart phone, "
//         "or upload a binary using the ESP32 webserver.");
//     ota->setSelect(0);
//     ota->setTerminateSetup();
//     ota->setNeverInline();
//     top->addEntry(ota);
//     top->setHighlight(1);
//     int event = ButtonEvent(ButtonEvent::SHORT_PRESS).raw;
//     xQueueSend(uiEventQueue, &event, 0); // virtually press the button to straight enter the selection
// }

// SetupMenu *createOtaMenu() {
//     SetupMenu *ota_menu = new SetupMenu("", ota_selection_create);
//     return ota_menu;
// }

static constexpr const int16_t qrCodeMaxWidth = 114;

static bool drawQrCode(const char *textBuffer,int16_t xref, int16_t yref) {

    uint8_t *tempBuffer = (uint8_t *)malloc(qrcodegen_BUFFER_LEN_FOR_VERSION(4));
    uint8_t *qrcodeBuffer = (uint8_t *)malloc(qrcodegen_BUFFER_LEN_FOR_VERSION(4));

    bool qrSuccess = qrcodegen_encodeText(textBuffer, tempBuffer, qrcodeBuffer, qrcodegen_Ecc_LOW, 4, 4, qrcodegen_Mask_AUTO, true);

    if ( qrSuccess ) {
        // Calculate module size for best fit
        int16_t size = qrcodegen_getSize(qrcodeBuffer);
        int16_t dotSize = qrCodeMaxWidth / size;

        // Center QRCode
        xref += (120 - (size * dotSize)) / 2;

        for (int16_t y = 0; y < size; y++) {
            MYUCG->startBuffering(xref, yref + (y * dotSize), (size*dotSize)+1, dotSize+1);
            for (int16_t x = 0; x < size; x++) {
                if (qrcodegen_getModule(qrcodeBuffer, x, y)) {
                    MYUCG->drawBox(xref + (x * dotSize), yref + (y * dotSize), dotSize, dotSize);
                }
            }
            MYUCG->finishBuffering();
        }
    }
    free(tempBuffer);
    free(qrcodeBuffer);

    return qrSuccess;
}


OTA::OTA(bool dpp) :
    SetupMenuDisplay(""),
    Clock_I(100),
    _do_dpp(dpp)
{
    scheduleReboot();
}

OTA::~OTA()
{
    Webserver.stop();
    Clock::stop(this);
}

// OTA with legacy access point
void OTA::display(int some){
	ESP_LOGI(FNAME,"Now start Wifi OTA");
	WifiApSta *wifi = WifiApSta::createWifiApSta();
	wifi->ConfigureIntf(80);

	Display->clear();
	int line=0;
	MYUCG->setFont(ucg_font_ncenR14_hr, true );
	MYUCG->setColor(COLOR_WHITE);
    menuPrintLn("Firmware Upload", line++, 30);
	menuPrintLn("Use Wifi: ESP32 OTA", line++);
	menuPrintLn("Password: xcvario-21", line++);
	menuPrintLn("Open: http://192.168.4.1", line++);

	// Make and print the QR Code symbol
	constexpr int textBufferSize = 128;
	char textBuffer[textBufferSize];
	int16_t xOffset = 0;
	int16_t yOffset = 130;

	const char *wifiText = "WIFI:";
	int16_t strWidth = MYUCG->getStrWidth(wifiText);
	MYUCG->setPrintPos((120 - strWidth) / 2, 130);
	MYUCG->print(wifiText);
	snprintf(textBuffer, textBufferSize, "WIFI:S:%s;T:WPA;P:%s;;", OTA_SSID, AP_PASSPHARSE);

	if( ! drawQrCode(textBuffer, xOffset, yOffset) ) {
        // In case of error draw empty rectangle
		MYUCG->drawFrame(xOffset + ((120 - qrCodeMaxWidth) / 2), yOffset, qrCodeMaxWidth, qrCodeMaxWidth);
	}

	// Generate URL QR Code using the AP IP-address
	// TODO: Show after Wifi has been connected?
	esp_netif_ip_info_t ip_info;
	esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_AP_DEF"), &ip_info);    // Return ESPs IP for everybody else


	const char *urlText = "URL:";
	strWidth = MYUCG->getStrWidth(urlText);
	MYUCG->setPrintPos(120 + (120 - strWidth) / 2, 130);
	MYUCG->print(urlText);
	snprintf(textBuffer, textBufferSize, "http://" IPSTR, IP2STR(&ip_info.ip));

	xOffset = 120;
	if( ! drawQrCode(textBuffer, xOffset, yOffset) ) {
        // In case of error draw empty rectangle
		MYUCG->drawFrame(xOffset + ((120 - qrCodeMaxWidth) / 2), yOffset, qrCodeMaxWidth, qrCodeMaxWidth);
	}

	Webserver.start();
    Clock::start(this);
}

void OTA::press() {
    Clock::stop(this);
    exit();
}

// exceptional use of the ticker for display output
bool OTA::tick() {
    int16_t line = 8;
    char txt[40];
    sprintf(txt,"Timeout in %d sec  ", _count-- );
    menuPrintLn(txt, line+2);
    sprintf(txt,"Progress: %d %%  ", Webserver.getOtaProgress() );
    menuPrintLn(txt, line+3);

    bool update_end = _count < 0;
    if( Webserver.getOtaStatus() == otaStatus::DONE ){
        ESP_LOGI(FNAME,"Flash status, Now restart");
        menuPrintLn("Download SUCCESS !", line+3);
        vTaskDelay(3000/portTICK_PERIOD_MS);
        update_end = true;
    }

    if ( update_end ) {
        ESP_LOGI(FNAME,"Now restart");
        exit(-1);
        return true;
    }
    return false;
}
