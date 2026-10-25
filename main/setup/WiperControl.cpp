/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "WiperControl.h"

#include "IpsDisplay.h"
#include "AdaptUGC.h"
#include "Colors.h"
#include "comm/DeviceMgr.h"
#include "protocol/NMEA.h"
#include "sensor.h"
#include "driver/time/WatchDog.h"
#include "logdef.h"

WiperControl* WPControl = nullptr;

extern AdaptUGC *MYUCG;

enum WiperControlAction : int {
    WCA_Init,
    WCA_SelLeft,
    WCA_SelExit,
    WCA_SelRight,
    WCA_Press,
    WCA_LongPress,
    WCA_Release
};

const Point but_geom[] = { { -90, 50 }, { -30, 170 }, { 30, 50 } };
const char *but_labels[] = { "Left", "Exit", "Right" };

WiperControl::WiperControl() : SetupMenuDisplay("Wiper Control"),
    display_width(MYUCG->getDisplayWidth())
{
    setNeverInline();
    setRotDynamic(1);
}
WiperControl::~WiperControl() {
    WPControl = nullptr;
}

void WiperControl::enter() {
    MenuEntry::enter();
    WPControl = this;
}

void WiperControl::exit(int ups) {
    WPControl = nullptr; // control the time/scope for valid callbacks
    MenuEntry::exit(ups);
}

void WiperControl::display(int event) {
    const int16_t dw = MYUCG->getDisplayWidth();
    const int16_t ab = getUserAttr();

    if (event == WCA_Init) {
        printTitle();
        event = WCA_SelLeft;
        setUserAttr(WCA_SelLeft);
    }

    if ( event <= WCA_SelRight) {
        // first call or a button selection change
        MYUCG->setFont(ucg_font_fub14_hr);
        // draw three buttons
        for (int i = 0; i < WCA_SelRight; i++) {
            Point p = {(int16_t)(but_geom[i].x + dw/2), but_geom[i].y};
            MYUCG->startBuffering(p.x - 1 , p.y - 1, 62, 33);
            MYUCG->setColor(COLOR_WHITE);
            MYUCG->drawRFrame(p.x, p.y, 60, 30, 4);
            if (event == (i+1)) {
                MYUCG->setColor(COLOR_HEADER);
                MYUCG->drawRBox(p.x + 3, p.y +3, 55, 24, 3);
            }
            MYUCG->setColor(COLOR_WHITE);
            int16_t wl = MYUCG->getStrWidth(but_labels[i]);
            MYUCG->setPrintPos(p.x + (60 - wl)/2, p.y + LINE_HEIGHT);
            MYUCG->print(but_labels[i]);
            MYUCG->finishBuffering();
        }
    }
    else {
        NmeaPrtcl *jumbo = static_cast<NmeaPrtcl*>(DEVMAN->getProtocol(DeviceId::JUMBO_DEV, ProtocolType::JUMBOCMD_P));
        ESP_LOGI(FNAME, "wipe %d", ab);
        int side = (ab == WCA_SelLeft) ? 1 : 0; // right == 0
        if (event == WCA_Press) {
            jumbo->sendJPShortPress(side);
        }
        else if (event == WCA_LongPress) {
            jumbo->sendJPHoldPressed(side);
        }
        else if (event == WCA_Release) {
            jumbo->sendJPRelease(side);
        }
    }
}

void WiperControl::callback(int par) {
    ESP_LOGI(FNAME, "WiperControl callback: side %d, %d", par>>16, par&0xFFFF);

    const int16_t dw = MYUCG->getDisplayWidth();
    int16_t percent = par & 0xFFFF;
    char buf[30];
    snprintf(buf, sizeof(buf), "% 4d%% ", percent);
    MYUCG->setFont(ucg_font_fub14_hr, true);
    menuPrintLn(buf, 4, ((par>>16) ? dw/4 : 3*dw/4) - 20);

    if (uiMonitor) {
        // classify bug wiper operation as an ongoing ui user interaction
        uiMonitor->pet();
    }
}

void WiperControl::rot(int count) {
    int ab_prev = getUserAttr();
    int ab_new = std::clamp(ab_prev + count, (int)WCA_SelLeft, (int)WCA_SelRight);
    if ( ab_new != ab_prev) {
        display(ab_new);
        setUserAttr(ab_new);
    }
}

void WiperControl::press() {
    if (getUserAttr() == WCA_SelExit) {
        exit();
    } else {
        display(WCA_Press);
    }
}

void WiperControl::longPress() {
    if (getUserAttr() == WCA_SelExit) {
        exit();
    } else {
        display(WCA_LongPress);
    }
}

void WiperControl::release() {
    display(WCA_Release);
}
