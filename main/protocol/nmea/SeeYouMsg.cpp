/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "SeeYouMsg.h"
#include "protocol/nmea_util.h"
#include "driver/time/Clock.h"
#include "comm/DataLink.h"
#include "comm/Messages.h"
#include "setup/SetupNG.h"
#include "setup/CruiseMode.h"
#include "sensor/imu/AccMPU6050.h"
#include "math/Floats.h"
#include "math/Units.h"
#include "logdefnone.h"

#include <string_view>

// The Naviter/SeeYou protocol parser.
//
// Supported messages:
// PLXV0,<Attribute>,R/W,<Value>*CS\r\n

static void send_attr(NmeaPrtcl &nmea, int attridx, float val);

const std::string_view ATTR_ITEM[] = { "MC", "BAL", "BUGS", "QNH", "ELEVATION" };
SetupNG<float>* const NVS_ITEM[] = { &MC, &ballast_kg, &bugs, &QNH, &airfield_elevation};

dl_action_t SeeYouMsg::gen_query(NmeaPlugin *plg)
{
    ESP_LOGI(FNAME, "SeeYou gen_query");
    // e.g. read message "$PLXV0,MC,W,1.2*CS"
    NmeaPrtcl &nmea = plg->getNMEA();
    ProtocolState *sm = nmea.getSM();
    const std::vector<int> *word = &sm->_word_start;

    if ( word->size() < 3 ) {
        return NOACTION;
    }

    int pos = sm->_word_start[0];
    std::string token = NMEA::extractWord(sm->_frame, pos);
    ESP_LOGI(FNAME, "SeeYou %d param %d, %s", word->size(), pos, token.c_str());
    int idx = 0;
    for (const std::string_view& s : ATTR_ITEM) {
        if ( token.compare(s) == 0 ) {
            break;
        }
        idx++;
    }
    if ( idx >= sizeof(ATTR_ITEM) / sizeof(ATTR_ITEM[0]) ) {
        return NOACTION;
    }
    // todo CONNECTION, NMEARATE

    pos = sm->_word_start[1];
    bool write = NMEA::extractWord(sm->_frame, pos).c_str()[0] == 'W';
    if ( write ) {
        token = NMEA::extractWord(sm->_frame, sm->_word_start[2]);
        float val = std::stof(token);
        if ( idx == 3 ) { // QNH
            val = Units::read(Units::hpa, val);
        }
        NVS_ITEM[idx]->setCheckRange(val);
        ESP_LOGI(FNAME, "SeeYou set value %.1f", NVS_ITEM[idx]->get());
    }
    else {
        send_attr(nmea, idx, NVS_ITEM[idx]->get());
    }

    return NOACTION;
}

const ParserEntry SeeYouMsg::_pt[] = {
    { Key("LXV0"), SeeYouMsg::gen_query },
    {}
};

// helper function to send attribute setting
void send_attr(NmeaPrtcl &nmea, int attridx, float val)

{
    // e.g. send message "$PLXV0,MC,W,1.2*CS\r\n"
    Message* msg = nmea.newMessage();

    msg->buffer = "$PLXV0,";
    msg->buffer += ATTR_ITEM[attridx];
    msg->buffer += ",W,";
    char tmp[20];
    if ( attridx == 3 ) { // QNH
        val = Units::pipe(val, Units::hpa);
    }
    std::sprintf(tmp, "%.1f", val);
    msg->buffer += tmp;
    msg->buffer += "*" + NMEA::CheckSum(msg->buffer.c_str()) + "\r\n";
    ESP_LOGD(FNAME, "SeeYou set attr %s", msg->buffer.c_str());
    DEV::Send(msg);
}

void NmeaPrtcl::sendSeeYouVal(float val, int idx)
{
    send_attr(*this, idx, val);
}


/*
    Fast sentence has following format:
    $PLXVF, <TIME>, <IMUaccelX>, <IMUaccelY>, <IMUaccelZ>, <VARIO>, <IAS>, <ALT>, <S2FMode>*CS\r\n
    TIME = time since epoch in seconds and millis  (float)
    IMUaccel* = G aspect (float)
    VARIO = m/s (float)
    IAS = km/h (float)
    ALT = m QNH (float)
    S2FMODE = 0:1; 0 == vario mode (int)
    CS = standard NMEA checksum
*/
void NmeaPrtcl::sendSeeYouF()
{
    if ( _dl.isBinActive() ) {
        return; // no NMEA output in binary mode
    }
    Message* msg = newMessage();

    msg->buffer = "$PLXVF,";
    char tmp[50];
    struct timeval tv;
    gettimeofday(&tv, NULL);
    std::sprintf(tmp, "%d.%03d,", (int)(tv.tv_sec - 315964800), (int)(tv.tv_usec / 1000));
    // int32_t ms_midnight = Clock::getMilisMidnightUTC();
    // std::sprintf(tmp, "%ld.%03ld,", ms_midnight / 1000, ms_midnight % 1000);
    msg->buffer += tmp;
    vector_f accel = {0,0,0};
    if ( accSensor ) {
        accel = accSensor->getHead();
    }
    std::sprintf(tmp, "%.1f,", accel.x); // Naviter expects "NED"
    msg->buffer += tmp;
    std::sprintf(tmp, "%.1f,", accel.y);
    msg->buffer += tmp;
    std::sprintf(tmp, "%.1f,", accel.z);
    msg->buffer += tmp;
    std::sprintf(tmp, "%.1f,", te_vario.get());
    msg->buffer += tmp;
    std::sprintf(tmp, "%.1f,", ias.get());
    msg->buffer += tmp;
    std::sprintf(tmp, "%.1f,", altitude.get());
    msg->buffer += tmp;
    std::sprintf(tmp, "%1d,", CRMOD.getCMode());
    msg->buffer += tmp;

    msg->buffer += "*" + NMEA::CheckSum(msg->buffer.c_str()) + "\r\n";
    ESP_LOGD(FNAME, "SeeYouF %s", msg->buffer.c_str());
    DEV::Send(msg);
}


/*
    Slow sentence has following format:
    $PLXVS, <OAT>, <S2FMode>, <VOLTAGE>, <PALT>*CS\r\n
    OAT = temperature in °C (float)
    S2FMODE = 0:1; 0 == vario mode (int)
    VOLTAGE = battery volts (float)
    ALT = m (float)
    CS = standard NMEA checksum
*/
void NmeaPrtcl::sendSeeYouS()
{
    if ( _dl.isBinActive() ) {
        return;
    }
    Message* msg = newMessage();

    msg->buffer = "$PLXVS,";
    char tmp[50];
    std::sprintf(tmp, "%.1f,", Units::pipe(OAT.get(), Units::celsius));
    msg->buffer += tmp;
    std::sprintf(tmp, "%1d,", CRMOD.getCMode());
    msg->buffer += tmp;
    std::sprintf(tmp, "%.1f,", battery_voltage.get());
    msg->buffer += tmp;
    std::sprintf(tmp, "%.1f,", altitude.get());
    msg->buffer += tmp;

    msg->buffer += "*" + NMEA::CheckSum(msg->buffer.c_str()) + "\r\n";
    ESP_LOGD(FNAME, "SeeYouS %s", msg->buffer.c_str());
    DEV::Send(msg);
}


// $LK8EX1,pressure,altitude,vario,temp,battery,*checksum
void NmeaPrtcl::sendLK8EX1()
{
    if ( _dl.isBinActive() ) {
        return; // no NMEA output in binary mode
    }
    Message* msg = newMessage();

    msg->buffer = "$LK8EX1,";
    char tmp[50];

    std::sprintf(tmp, "%d,99999,", fast_iroundf(statp.get())); // Pa
    msg->buffer += tmp;
    std::sprintf(tmp, "%d,", fast_iroundf(te_vario.get() * 100.)); // cm/sec
    msg->buffer += tmp;
    std::sprintf(tmp, "%.1f,", Units::pipe(OAT.get(), Units::celsius));
    msg->buffer += tmp;
    std::sprintf(tmp, "%.1f", battery_voltage.get());
    msg->buffer += tmp;

    msg->buffer += "*" + NMEA::CheckSum(msg->buffer.c_str()) + "\r\n";
    ESP_LOGD(FNAME, "SeeYouF %s", msg->buffer.c_str());
    DEV::Send(msg);
}