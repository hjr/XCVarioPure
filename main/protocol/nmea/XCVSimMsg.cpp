/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "XCVSimMsg.h"

#include "protocol/nmea_util.h"
#include "driver/time/Clock.h"
#include "comm/Messages.h"
#include "sensor/pressure/PressureSensor.h"
#include "sensor/press_diff/AirspeedSensor.h"
#include "sensor/temp/TempVSensor.h"
#include "sensor/imu/AccMPU6050.h"
#include "sensor/imu/GyroMPU6050.h"
#include "sensor/mag/MagVSensor.h"
#include "sensor/adc/FlapSens.h"
#include "math/Trigonometry.h"
#include "logdefnone.h"

#include <cstring>

// The XCV Sens raw sensor log messages to run the vario in simulation mode.

XCVSimMsg::XCVSimMsg(NmeaPrtcl &nr) :
    NmeaPlugin(nr, XCVSENS_P)
{
    // route diverse protocols further (e.g. Flarm to master-second-BT...)
    _nmeaRef.setDefaultAction(NOACTION);
}

//
// The Sens transmitter routine
//

// bool XCVSimMsg::sendSens()
// {
//     // Message* msg = _nmeaRef.newMessage();
//     // ...
//     // return DEV::Send(msg);
// }

//
// Sens receiver routines
//

//
// Example message:
//        gpsT, deltaT, baroP, teP, dynamicP, Temp, Az, Ay, Ax, Gx, Gy, Gz, MagX, MagY, MagZ
// $SENS;43799.060,179,990.646,990.562,0.000,13.43,0.1728,0.0805,0.9861,-0.2031,-0.0145,0.0053,19.6474,19.7357,-34.4881
//
dl_action_t XCVSimMsg::parse_Sens(NmeaPlugin *plg)
{
    ProtocolState *sm = plg->getNMEA().getSM();
    const std::vector<int> *word = &sm->_word_start;

    ESP_LOGD(FNAME,"parseSens %s", sm->_frame.c_str() );

    if ( word->size() < 12 ) {
        return NOACTION; // invalid SENS message
    }
    // int pos = word->at(2);
    int time = Clock::getMillis();
    float tmp = atof(sm->_frame.c_str() + word->at(2)) * 100.f; // convert to Pa
    baroSensor->pushAndPublish(tmp, time);

    tmp = atof(sm->_frame.c_str() + word->at(3)) * 100.f; // convert to Pa
    teSensor->pushAndPublish(tmp, time);

    tmp = atof(sm->_frame.c_str() + word->at(4));
    asSensor->pushAndPublish(tmp, time);

    tmp = atof(sm->_frame.c_str() + word->at(5)) + Units::C2K; // convert to Kelvin
    if ( oatSensor ) oatSensor->pushAndPublish(tmp, time);

    vector_f vtmp;
    vtmp.x = atof(sm->_frame.c_str() + word->at(6));
    vtmp.y = atof(sm->_frame.c_str() + word->at(7));
    vtmp.z = atof(sm->_frame.c_str() + word->at(8));
    if ( accSensor ) accSensor->pushAndPublish(vtmp, time);

    vtmp.x = deg2rad(atof(sm->_frame.c_str() + word->at(9)));
    vtmp.y = deg2rad(atof(sm->_frame.c_str() + word->at(10)));
    vtmp.z = deg2rad(atof(sm->_frame.c_str() + word->at(11)));
    if ( gyroSensor ) gyroSensor->pushAndPublish(vtmp, time);

    if ( word->size() >= 15 ) {
        vtmp.x = atof(sm->_frame.c_str() + word->at(12));
        vtmp.y = atof(sm->_frame.c_str() + word->at(13));
        vtmp.z = atof(sm->_frame.c_str() + word->at(14));
        if ( magSensor ) magSensor->pushAndPublish(vtmp, time);
    }

    return NOACTION; // never forward the simulation
}


dl_action_t XCVSimMsg::parseExcl_XCV(NmeaPlugin *plg)
{
    ProtocolState *sm = plg->getNMEA().getSM();
    const std::vector<int> *word = &sm->_word_start;

    ESP_LOGD(FNAME, "XCV: %d, %s", word->at(0), sm->_frame.c_str());
    int time = Clock::getMillis();

    const char *s = sm->_frame.c_str();
    const char *key = s + word->at(0);
    if (strncmp(key, "CA1", 3) == 0)
    {
        //          glider type, empty weight, crew weight, ballast, speed cal, QNH
        // !XCV,CA1,2305,229,88,0,0,102000.00*42
        ESP_LOGI(FNAME, "ConfigAll1");
        int value = atoi(s + word->at(1));
        glider_type.set(value);
        value = atoi(s + word->at(2));
        empty_weight.set(value);
        value = atoi(s + word->at(3));
        crew_weight.set(value);
        value = atoi(s + word->at(4)); // ballast
        ballast_kg.set(value);
        value = atoi(s + word->at(5)); // speed cal
        speedcal.set(value);
        float fvalue = atof(s + word->at(6)); // QNH
        QNH.set(fvalue);
    }
    else if (strncmp(key, "FLP", 3) == 0)
    {
        // flap (x10) setting changed 
        // !XCV,FLP,33*17
        float fvalue = atoi(s + word->at(1)) / 10.f;
        ESP_LOGI(FNAME, "New flap setting %f", fvalue);
        if (flapSensor) flapSensor->pushAndPublish(fvalue, time);
    }

    return NOACTION;
}

const ParserEntry XCVSimMsg::_pt[] = {
    {Key("SENS"), XCVSimMsg::parse_Sens},
    {Key("XCV"), XCVSimMsg::parseExcl_XCV},
    {}
};
