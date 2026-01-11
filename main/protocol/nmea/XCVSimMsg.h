/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "protocol/NMEA.h"

class XCVSimMsg  final : public NmeaPlugin
{
public:
    explicit XCVSimMsg(NmeaPrtcl &nr);
    virtual ~XCVSimMsg() = default;
    const ParserEntry* getPT() const override { return _pt; }

private:
    // Received messages
    static dl_action_t parse_Sens(NmeaPlugin *plg);
    
    static const ParserEntry _pt[];
};
