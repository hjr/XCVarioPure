/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "ScreenElement.h"


union conn_status_hash {
    struct {
        uint8_t wifi_alive  : 2;
        uint8_t bt_alive    : 2;
        uint8_t flarm_alive : 2;
        uint8_t can_up      : 1;
        uint8_t mags_alive  : 2;
        uint8_t xcv_alive   : 2;
        };
    uint16_t packed;
};

// all connection screen elements
class Connection : public ScreenElement
{
public:
    Connection(int16_t cx, int16_t cy, bool sbs=false) : ScreenElement(cx, cy), _side_by_side(sbs) {}
    // API
    void draw();

    // attributes
private:
    void drawBT();
    void drawWifi();
    void drawCable();
    bool _side_by_side = false;
    conn_status_hash _hash{.packed=0};
};