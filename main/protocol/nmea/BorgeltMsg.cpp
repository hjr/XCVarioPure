/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "BorgeltMsg.h"
#include "protocol/nmea_util.h"
#include "comm/DataLink.h"
#include "comm/Messages.h"
#include "math/Floats.h"
#include "math/Units.h"
#include "setup/SetupNG.h"
#include "setup/CruiseMode.h"
#include "logdefnone.h"

#include <cmath>

// The Borgelt protocol parser.
//
// Supported messages:


const ParserEntry BorgeltMsg::_pt[] = {
    {}
};

/*
    Sentence has following format:
    $PBB50,AAA,BBB.B,C.C,DDDDD,EE,F.FF,G,HH*CHK crlf
    AAA = TAS 0 to 150 knots
    BBB.B = Vario, -10 to +15 knots, negative sign for sink
    C.C = MacCready 0 to 8.0 knots
    DDDDD = IAS squared 0 to 22500
    EE = bugs degradation, 0 = clean to 30 %
    F.FF = Ballast 1.00 to 1.60
    G = 0 in climb, 1 in cruise
    HH = Outside airtemp in degrees celcius ( may have leading negative sign )
    CHK = standard NMEA checksum
*/
void NmeaPrtcl::sendBorgelt()
{
    if ( _dl.isBinActive() ) {
        return; // no NMEA output in binary mode
    }
    Message* msg = newMessage();

    celsius_t temp = OAT.getValid() ? Units::pipe(OAT.get(), Units::celsius) : 0;
    knots_t iaskn = Units::pipe(ias.get(), Units::kts);

    msg->buffer = "$PBB50,";
    char buffer[50];
    std::sprintf(buffer, "%03d", fast_iroundf(Units::pipe(tas.get(), Units::kts)));
    msg->buffer += buffer;
    std::sprintf(buffer, ",%3.1f", Units::pipe(te_vario.get(), Units::kts));
    msg->buffer += buffer;
    std::sprintf(buffer, ",%1.1f", Units::pipe(MC.get(), Units::kts));
    msg->buffer += buffer;
    std::sprintf(buffer, ",%d", fast_iroundf(iaskn*iaskn));
    msg->buffer += buffer;
    std::sprintf(buffer, ",%d", int(bugs.get()));
    msg->buffer += buffer;
    std::sprintf(buffer, ",%1.2f", (ballast.get()+100.f)/100.f);
    msg->buffer += buffer;
    std::sprintf(buffer, ",%1d", !CRMOD.getCMode());
    msg->buffer += buffer;
    std::sprintf(buffer, ",%2d", fast_iroundf(temp));

    msg->buffer += "*" + NMEA::CheckSum(msg->buffer.c_str()) + "\r\n";
    ESP_LOGD(FNAME, "Borgelt %s", msg->buffer.c_str());
    DEV::Send(msg);
}


