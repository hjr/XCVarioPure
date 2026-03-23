/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "GarminMsg.h"

#include "Flarm.h"
#include "math/Units.h"
#include "setup/SetupNG.h"
#include "sensor.h"

#include "logdefnone.h"


// The Garmin protocol parser.
//
// Supported messages:
// PGRMZ,<Value>,F,2
// e.g.
// $PGRMZ,880,F,2*3A
//
dl_action_t GarminMsg::parsePGRMZ(NmeaPlugin *plg)
{
    if (alt_select.get() != ALT_EXTERNAL) {
        return DO_ROUTING;
    }
    ProtocolState *sm = plg->getNMEA().getSM();
    const std::vector<int> *word = &sm->_word_start;
    ESP_LOGI(FNAME, "parsePGRMZ");

    if (word->size() == 4 && sm->_frame.at(word->at(1)) == 'F') {
        int alt1013_ft = atoi(sm->_frame.c_str()+word->at(0));
        meter_t alt_external = Units::read(Units::foot, alt1013_ft);
        ESP_LOGI(FNAME, "PGRMZ %d: ALT(1013):%5.0f m", alt1013_ft, alt_external);
        statp.set(Units::calcPressureISA(alt_external));
    }
    return DO_ROUTING;
}



const ParserEntry GarminMsg::_pt[] = {
    { Key("GRMZ"), GarminMsg::parsePGRMZ },
    {}
};


