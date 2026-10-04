/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

template<typename T>
class SetupNG;

#include <string>
#include <cstdint>

enum DeviceId : uint8_t;

namespace XcvCaps
{

    // Capabilities resulting out of protocol negotiation
    enum XCV_CAP : uint32_t
    {
        GPS_CAP         = 1<<0,
        FLARM_CAP       = 1<<1,
        EXTWIND_CAP     = 1<<2,
        RADIOCTRL_CAP   = 1<<3,
        HEADING_CAP     = 1<<4,
        FLAPSENS_CAP    = 1<<5,
        GEARSENS_CAP    = 1<<6,
        AHRS_CAP        = 1<<7,
        TEMP_CAP        = 1<<8
    };

    // helper to handle the capability codes

    // add a capability
    void addToMine(int cap);

    // remove a capability
    void removeFromMine(int cap);
    
    // update my caps from connected devices
    void updateCapsFromDev(DeviceId did, bool add);

    // setup peer protocols based on caps
    void setupPeerProtos(int listen_port, int send_port);

    // some queries to check on capabilities
    bool isMyCap(int cap);
    bool isPeerCap(int cap);
    bool haveCap(int cap);
};

