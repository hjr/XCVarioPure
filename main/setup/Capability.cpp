/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "Capability.h"

#include "comm/InterfaceCtrl.h"
#include "comm/Devices.h"
#include "comm/DeviceMgr.h"
#include "setup/SetupNG.h"
#include "logdef.h"

#include <cstring>

// The XCV CAN peer capabilities query message.
//
// - Caps need to be know on early stage to configure the device properly. The caps need to cross sync from own to peer caps.
//      Thus the Xcv Sync message is not able to do that. A dedicated CAN message is used to propagate capabilities.
//      The suggested message is used from both sides, master and client, to propagate own caps as soon as possible
//      and on every change.
//   $PJPCAP, <my caps>*<CRC>\r\n
//
// OR, piggiback on the registration query/response message from master to client
//   $PJPREG, <token>, <protocol type>, <client caps>*<CRC>\r\n
//   $PJMACC, <token>, <drive id>, <master id>, <master caps>*<CRC>\r\n
//

void XcvCaps::addToMine(int cap)
{
    my_caps.set(my_caps.get() | cap);
    ESP_LOGI(FNAME, "Added capability: %x", cap);
}

void XcvCaps::removeFromMine(int cap)
{
    my_caps.set(my_caps.get() & (~cap));
    ESP_LOGI(FNAME, "Removed capability: %x", cap);
}

// translate devices to capabilities
void XcvCaps::updateCapsFromDev(DeviceId did, bool add)
{
    int cap = 0;
    switch ( did ) {
        case ANEMOI_DEV:
            cap = XcvCaps::EXTWIND_CAP;
            if ( ! add && wind_enable.get() == WA_EXTERNAL ) {
                // no external wind source selection possible any more, so set it to internal
                wind_enable.set(WA_BOTH);
            }
            break;
        case FLARM_DEV:
            cap = XcvCaps::FLARM_CAP | XcvCaps::GPS_CAP;
            break;
        case MAGSENS_DEV:
            cap = XcvCaps::HEADING_CAP;
            break;
        case RADIO_KRT2_DEV:
        case RADIO_ATR833_DEV:
            cap = XcvCaps::RADIOCTRL_CAP;
            break;
        case TEMPSENS_DEV:
            cap = XcvCaps::TEMP_CAP;
            break;
        case FLAP_SENS_DEV:
            cap = XcvCaps::FLAPSENS_CAP;
            break;
       default:
            break;
    }
    if ( cap != 0 ) {
        if ( add ) {
            addToMine(cap);
        } else {
            removeFromMine(cap);
        }
    }
}

// Setup devices on peer based on capabilities
// Those devices are then virtually connected to this XCVario via the CAN interface
void XcvCaps::setupPeerProtos(int listen_port, int send_port)
{
    // look for caps the peer has but not this device
    int capdiff  = peer_caps.get() & (~my_caps.get());

    // setup peer protocols based on extra peer caps
    if ( capdiff & XcvCaps::FLARM_CAP ) {
        ESP_LOGI(FNAME, "Peer has FLARM host capability, add virtual device on peer connection");
        DEVMAN->addDevice(FLARM_DEV, FLARM_P, listen_port, send_port, CAN_BUS, false);
    }
}

bool XcvCaps::isMyCap(int cap) {
    return my_caps.get() & cap;
}
bool XcvCaps::isPeerCap(int cap) {
    return peer_caps.get() & cap;
}
bool XcvCaps::haveCap(int cap) {
    return (my_caps.get() & cap || peer_caps.get() & cap);
}
