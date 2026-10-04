/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "XCVSyncMsg.h"

#include "protocol/nmea_util.h"
#include "comm/Messages.h"
#include "driver/time/Clock.h"
#include "sensor/pressure/PressureSensor.h"
#include "sensor/press_diff/AirspeedSensor.h"
#include "sensor/imu/AccMPU6050.h"
#include "sensor/imu/GyroMPU6050.h"
#include "setup/SetupNG.h"
#include "sensor.h"

#include "logdefnone.h"

#include <cstring>

// The XCV sync messages to synchronize a client vario.

XCVSyncMsg::XCVSyncMsg(NmeaPrtcl &nr, bool master, bool as) :
    NmeaPlugin(nr, XCVSYNC_P, as),
    _is_master(master)
{
    // for XCV clients: kick a sync request with the very first message received from the master
    _kick_sync = ! master; // master does not need to kick a sync, the client does
    // tell the only user of this
    SetupCommon::setSyncProto(this);
    // route diverse protocols further (e.g. Flarm to master-second-BT...)
    _nmeaRef.setDefaultAction(DO_ROUTING);
}

XCVSyncMsg::~XCVSyncMsg()
{
    SetupCommon::setSyncProto(nullptr);
}


//
// The sync transmitter routine
//

bool XCVSyncMsg::sendInitSyncRequest()
{
    // a XCV Secondary sends this message to the master to initialize the sync
    Message* msg = _nmeaRef.newMessage();
    _kick_sync = false; // only once
    msg->buffer.assign("!xcvR,Init\r\n");
    return DEV::Send(msg);
}

bool XCVSyncMsg::sendItem(const char *key, char type, void *value, uint8_t len)
{
    Message* msg = _nmeaRef.newMessage();

    ESP_LOGD(FNAME,"sendItem: %s", key );

    msg->buffer =  "!xcv";
    msg->buffer.push_back(type);
    msg->buffer.push_back(',');
    msg->buffer += key;
    msg->buffer.push_back(',');
    msg->buffer.push_back(len);
    msg->buffer.append(reinterpret_cast<const char*>(value), len);
    msg->appendCheckSum();
    return DEV::Send(msg);
}

bool XCVSyncMsg::sendCAPs(int caps)
{
    Message* msg = _nmeaRef.newMessage();

    ESP_LOGI(FNAME,"sendCAPs: %x", caps );
    msg->buffer = "$PJPCAP, ";
    msg->buffer += std::to_string(caps);
    msg->appendCheckSum();
    return DEV::Send(msg);
}

void XCVSyncMsg::sendSensors()
{
    // called with full 10Hz rate from sensor loop
    Message* msg = _nmeaRef.newMessage();

    kelvin_t temp = OAT.get();
    if (!OAT.getValid()) {
        temp = Units::isa_temperature(altitude.get());
        // ESP_LOGW(FNAME,"T invalid, using 15 deg");
    }

    msg->buffer =  "!xcvB,S,";
    // send by default: time, baro, diff, te_alt, accel, gyro -> 1 + 3 * 4 + 2 * 12;
    uint8_t len = 37;
    msg->buffer.push_back(len);
    // int now = Clock::getMillis();
    // msg->buffer.append(reinterpret_cast<const char*>(&now), 4);
    msg->buffer.append(reinterpret_cast<const char*>(baroSensor->getHeadPtr()), 4);
    msg->buffer.append(reinterpret_cast<const char*>(teSensor->getHeadPtr()), 4);
    msg->buffer.append(reinterpret_cast<const char*>(asSensor->getHeadPtr()), 4);
    if (imuSensor) {
        msg->buffer.append(reinterpret_cast<const char*>(accSensor->getHeadPtr()), 12);
        msg->buffer.append(reinterpret_cast<const char*>(gyroSensor->getHeadPtr()), 12);
    } else {
        vector_f dummy;
        msg->buffer.append(reinterpret_cast<const char*>(&dummy), 12);
        msg->buffer.append(reinterpret_cast<const char*>(&dummy), 12);
    }
    msg->appendCheckSum();
    DEV::Send(msg);
}

//
// sync receiver routines
//

dl_action_t XCVSyncMsg::parseExcl_xcvX(NmeaPlugin *plg)
{
    // example: !xcvX,key,<length byte><binary value>*CRC\r\n
    ProtocolState *sm = plg->getNMEA().getSM();
    const std::vector<int> *word = &sm->_word_start;

    ESP_LOGD(FNAME,"parse xcvX %s p0 %d", sm->_frame.c_str(), word->at(0));
    char type = sm->_frame[4];
    int pos = word->at(0);
    std::string key = NMEA::extractWord(sm->_frame, pos);
    const uint8_t* valptr = reinterpret_cast<const uint8_t*>(sm->_frame.c_str()) + word->at(1);
    ESP_LOGI(FNAME,"parsed NMEA: type=%c key=%s len=%d", type , key.c_str(), (int)*valptr);
    if ( type != 'B' ) {
        valptr += 1; // skip the length byte
        SetupCommon *item = SetupCommon::getMember(key.c_str());
        if ( item ) {
            if( type == 'F' ) {
                SetupNG<float> *mi = static_cast<SetupNG<float> *>(item);
                mi->set( *reinterpret_cast<const float*>(valptr), false );
            }
            else if( type == 'I' ) {
                SetupNG<int> *mi = static_cast<SetupNG<int> *>(item);
                mi->set( *reinterpret_cast<const int*>(valptr), false );
            }
            else if( type == 'V' ) {
                SetupNG<vector_f> *mi = static_cast<SetupNG<vector_f> *>(item);
                mi->set( *reinterpret_cast<const vector_f*>(valptr), false );
            }

            // Once
            if ( static_cast<XCVSyncMsg*>(plg)->isSyncInitPending() ) {
                static_cast<XCVSyncMsg*>(plg)->sendInitSyncRequest();
            }
        }
        else {
            ESP_LOGW(FNAME,"Setup item with key %s not found", key.c_str() );
        }
    }
    else {
        uint8_t len = *valptr++;
        if ( len != 37 ) return NOACTION;
        int now = Clock::getMillis();
        // do some clever check on the masters time 
        // int time = *(reinterpret_cast<const int*>(valptr));
        // valptr += sizeof(int);
        float tmp = *(reinterpret_cast<const float*>(valptr));
        baroSensor->pushAndPublish(tmp, now);
        valptr += sizeof(float);
        tmp = *(reinterpret_cast<const float*>(valptr));
        teSensor->pushAndPublish(tmp, now);
        valptr += sizeof(float);
        tmp = *(reinterpret_cast<const float*>(valptr));
        asSensor->pushAndPublish(tmp, now);
        valptr += sizeof(float);
        vector_f vtmp;
        vtmp = *(reinterpret_cast<const vector_f*>(valptr));
        if ( accSensor ) accSensor->pushAndPublish(vtmp, now);
        valptr += sizeof(vector_f);
        vtmp = *(reinterpret_cast<const vector_f*>(valptr));
        if ( gyroSensor ) gyroSensor->pushAndPublish(vtmp, now);
        valptr += sizeof(vector_f);
        if (ReadSensorsLoop) xTaskNotifyGive(ReadSensorsLoop); // kick the sensor loop
    }

    return NOACTION; // never forward the XCV internal blabla
}

dl_action_t XCVSyncMsg::parseExcl_xcvRequest(NmeaPlugin *plg)
{
    ProtocolState *sm = plg->getNMEA().getSM();
    const std::vector<int> *word = &sm->_word_start;
    const char *s = sm->_frame.c_str();
    const char *key = s + word->at(0);
    if ( strncmp(key, "Init", 4) == 0) {
        if ( static_cast<XCVSyncMsg*>(plg)->isMaster() ) {
            ESP_LOGI(FNAME, "Master received SyncInit request from client");
            startClientSync();
        }
    }
    return NOACTION;
}

dl_action_t XCVSyncMsg::parse_caps(NmeaPlugin *plg)
{
    // message e.g. "$PJPCAP, 123"
    ProtocolState *sm = plg->getNMEA().getSM();
    const std::vector<int> *word = &sm->_word_start;

    if ( word->size() < 1 ) {
        return NOACTION;
    }

    int pos = word->at(0);
    ESP_LOGI(FNAME,"parse_caps: %s pos %d", sm->_frame.c_str(), pos );
    int caps = std::stoi(NMEA::extractWord(sm->_frame, pos));
    peer_caps.set(caps);
    ESP_LOGI(FNAME, "XCV CAPS received %x", caps);

    return NOACTION;
}

const ParserEntry XCVSyncMsg::_pt[] = {
    {Key("xcvB"), ParserInfo(XCVSyncMsg::parseExcl_xcvX, MessageFormat::Binary)},
    {Key("xcvI"), ParserInfo(XCVSyncMsg::parseExcl_xcvX, MessageFormat::Binary)},
    {Key("xcvF"), ParserInfo(XCVSyncMsg::parseExcl_xcvX, MessageFormat::Binary)},
    {Key("xcvV"), ParserInfo(XCVSyncMsg::parseExcl_xcvX, MessageFormat::Binary)},
    {Key("xcvR"), ParserInfo(XCVSyncMsg::parseExcl_xcvRequest)},
    {Key("PCAP"), ParserInfo(XCVSyncMsg::parse_caps)},
    {}
};
