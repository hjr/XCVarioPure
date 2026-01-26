/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "OpenVarioMsg.h"
#include "protocol/nmea_util.h"
#include "comm/DataLink.h"
#include "comm/Messages.h"
#include "setup/SetupNG.h"
#include "logdefnone.h"


// The Open Vario protocol parser.
//
// Supported messages:


const ParserEntry OpenVarioMsg::_pt[] = {
    {}
};


void NmeaPrtcl::sendOpenVario(float baro, float dp)
{
    if ( _dl.isBinActive() ) {
        return; // no NMEA output in binary mode
    }
    Message* msg = newMessage();

    msg->buffer = "$POV,P,";
    char buffer[50];
    std::sprintf(buffer, "%0.1f", Units::pipe(baro, Units::hpa));
    msg->buffer += buffer;
    std::sprintf(buffer, ",Q,%0.1f", dp);
    msg->buffer += buffer;
    std::sprintf(buffer, ",E,%0.1f", te_vario.get());
    msg->buffer += buffer;
    if( OAT.getValid() ) {
        std::sprintf(buffer, ",T,%0.1f", Units::pipe(OAT.get(), Units::celsius));
        msg->buffer += buffer;
    }

    msg->buffer += "*" + NMEA::CheckSum(msg->buffer.c_str()) + "\r\n";
    ESP_LOGD(FNAME, "OpenVario %s", msg->buffer.c_str());
    DEV::Send(msg);
}


