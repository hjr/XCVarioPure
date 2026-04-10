/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "Connection.h"

#include "comm/WifiApSta.h"
#include "comm/BTspp.h"
#include "comm/BTnus.h"
#include "comm/DeviceMgr.h"
#include "driver/time/AliveMonitor.h"
#include "Colors.h"
#include "AdaptUGC.h"
#include "setup/SetupNG.h"

#include "logdefnone.h"


extern AdaptUGC *MYUCG;


constexpr int16_t BTSIZE =  5;
constexpr int16_t BTW    = 15;
constexpr int16_t BTH    = 24;

void Connection::drawBT() {
    int16_t btx = _ref_x + 7;
    int16_t bty = _ref_y - 24 + (BTH / 2) + 6;
    ESP_LOGI(FNAME, "Connection::drawBT %d %d", btx, bty);
    if (_hash.bt_alive != ALIVE_OK)
        MYUCG->setColor(COLOR_MGREY);
    else
        MYUCG->setColor(COLOR_BLUE);  // blue

    MYUCG->drawRBox(btx - BTW / 2, bty - BTH / 2, BTW, BTH, BTW / 2 - 1);
    // inner symbol
    if (_hash.flarm_alive == ALIVE_OK)
        MYUCG->setColor(COLOR_GREEN);
    else
        MYUCG->setColor(COLOR_WHITE);
    MYUCG->drawTriangle(btx, bty, btx + BTSIZE, bty - BTSIZE, btx, bty - 2 * BTSIZE);
    MYUCG->drawTriangle(btx, bty, btx + BTSIZE, bty + BTSIZE, btx, bty + 2 * BTSIZE);
    MYUCG->drawLine(btx, bty, btx - BTSIZE, bty - BTSIZE);
    MYUCG->drawLine(btx, bty, btx - BTSIZE, bty + BTSIZE);
}

void Connection::drawWifi() {
    ESP_LOGI(FNAME, "Connection::drawWifi %d %d", _ref_x, _ref_y);
    if (_hash.wifi_alive != ALIVE_OK) {
        MYUCG->setColor(COLOR_MGREY);
    } else {
        MYUCG->setColor(COLOR_BLUE);
    }
    MYUCG->drawCircle(_ref_x, _ref_y, 9,  UCG_DRAW_UPPER_RIGHT);
    MYUCG->drawCircle(_ref_x, _ref_y, 10, UCG_DRAW_UPPER_RIGHT);
    MYUCG->drawCircle(_ref_x, _ref_y, 16, UCG_DRAW_UPPER_RIGHT);
    MYUCG->drawCircle(_ref_x, _ref_y, 17, UCG_DRAW_UPPER_RIGHT);
    if (_hash.flarm_alive == ALIVE_OK) { // fixme really
        MYUCG->setColor(COLOR_GREEN);
    }
    MYUCG->drawDisc(_ref_x, _ref_y, 3, UCG_DRAW_ALL);
}

void Connection::drawCable() {
    constexpr int16_t CANH = 8;
    constexpr int16_t CANW = 14;

    int16_t myx = (_side_by_side) ? _ref_x - 24 : _ref_x + 5;
    int16_t myy = (_side_by_side) ? _ref_y - CANH / 2 : _ref_y + 22;
    ESP_LOGI(FNAME, "Connection::drawCable %d %d", _hash.xcv_alive, xcv_alive.get());

    (_hash.xcv_alive == ALIVE_OK) ? MYUCG->setColor(COLOR_LBLUE) : MYUCG->setColor(COLOR_MGREY);
    // lower horizontal line
    if (_hash.mags_alive == ALIVE_OK) {
        MYUCG->setColor(COLOR_GREEN);
    }
    MYUCG->drawLine(myx - CANW / 2, myy + CANH / 2, myx + 3, myy + CANH / 2);
    MYUCG->drawLine(myx - CANW / 2, myy + CANH / 2 - 1, myx + 3, myy + CANH / 2 - 1);
    MYUCG->drawDisc(myx - CANW / 2, myy + CANH / 2, 2, UCG_DRAW_ALL);
    (_hash.mags_alive == ALIVE_OK) ? MYUCG->setColor(COLOR_LBLUE) : MYUCG->setColor(COLOR_MGREY);
    // Z diagonal line
    if (_hash.flarm_alive == ALIVE_OK) {
        MYUCG->setColor(COLOR_GREEN);
    }
    MYUCG->drawLine(myx + 2, myy + CANH / 2, myx - 4, myy - CANH / 2);
    MYUCG->drawLine(myx + 3, myy + CANH / 2 - 1, myx - 3, myy - CANH / 2 - 1);
    // upper horizontal line
    (_hash.xcv_alive == ALIVE_OK) ? MYUCG->setColor(COLOR_LBLUE) : MYUCG->setColor(COLOR_MGREY);
    MYUCG->drawLine(myx - 3, myy - CANH / 2, myx + CANW / 2, myy - CANH / 2);
    MYUCG->drawLine(myx - 3, myy - CANH / 2 - 1, myx + CANW / 2, myy - CANH / 2 - 1);
    MYUCG->drawDisc(myx + CANW / 2, myy - CANH / 2, 2, UCG_DRAW_ALL);
}

void Connection::draw()
{
    // create a hash of the connection status and see if anything needs to be redrawn
    conn_status_hash new_hash = {
        .wifi_alive = (DEVMAN->isIntf(WIFI_APSTA)) ? WIFI->isAlive() ? ALIVE_OK : ALIVE_TIMEOUT : ALIVE_NONE,
        .bt_alive = (DEVMAN->isIntf(BT_SPP) && BLUEspp) ? BLUEspp->isConnected() ? ALIVE_OK : ALIVE_TIMEOUT : 
                    (DEVMAN->isIntf(BT_LE) && BLUEnus) ? BLUEnus->isConnected() ? ALIVE_OK : ALIVE_TIMEOUT : ALIVE_NONE,
        .flarm_alive = (uint8_t)flarm_alive.get(),
        .can_up = SetupCommon::isWired(),
        .mags_alive = (uint8_t)mags_alive.get(),
        .xcv_alive = (uint8_t)xcv_alive.get()
    };
    ESP_LOGI(FNAME, "Connection::draw new hash %04x old hash %04x", new_hash.packed, _hash.packed);

    if (new_hash.packed == _hash.packed && !_dirty) {
        return; // nothing changed, skip drawing
    }

    // BT
    if (new_hash.bt_alive != ALIVE_NONE
        && (new_hash.bt_alive != _hash.bt_alive || new_hash.flarm_alive != _hash.flarm_alive || _dirty)) {
        _hash.bt_alive = new_hash.bt_alive;
        _hash.flarm_alive = new_hash.flarm_alive;
        drawBT();
    }
    // Wifi
    else if (new_hash.wifi_alive != ALIVE_NONE
        && (new_hash.wifi_alive != _hash.wifi_alive || new_hash.flarm_alive != _hash.flarm_alive || _dirty)) {
        _hash.wifi_alive = new_hash.wifi_alive;
        _hash.flarm_alive = new_hash.flarm_alive;
        drawWifi();
    }
    // Cable
    if (new_hash.can_up 
        && (new_hash.flarm_alive != _hash.flarm_alive || new_hash.xcv_alive != _hash.xcv_alive
            || new_hash.mags_alive != _hash.mags_alive || _dirty)) {
        _hash.packed = new_hash.packed;
        drawCable();
    }
    _hash.packed = new_hash.packed;
    _dirty = false;
}
