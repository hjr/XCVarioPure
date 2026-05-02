/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "FlarmMsg.h"
#include "protocol/FlarmBin.h"
#include "protocol/nmea/XCVSimMsg.h"
#include "driver/time/Clock.h"
#include "Flarm.h"
#include "screen/UiEvents.h"
#include "screen/DrawDisplay.h"
#include "sensor.h"
#include "sensor/SensorMgr.h"
#include "sensor/mag/MagVSensor.h"
#include "sensor/VarioFilter.h"
#include "screen/MessageBox.h"
#include "comm/DeviceMgr.h"
#include "setup/SetupNG.h"

#include "logdefnone.h"


static bool status_ok = false;

// The FLARM protocol parser.
//
// Supported messages:
// PFLAA,<AlarmLevel>,<RelativeNorth>,<RelativeEast>,<RelativeVertical>,<IDType>,<ID>,<Track>,<TurnRate>,<GroundSpeed>,<ClimbRate>,<AcftType>[,<NoTrack>[,<Source>,<RSSI>]]
// PFLAE,<QueryType>,<Severity>,<ErrorCode>[,<Message>]
// PFLAU,<RX>,<TX>,<GPS>,<Power>,<AlarmLevel>,<RelativeBearing>,<AlarmType>,<RelativeVertical>,<RelativeDistance>[,<ID>]
// PFLAX,A*2E

FlarmMsg::FlarmMsg(NmeaPrtcl &nr) :
    NmeaPlugin(nr, FLARM_P, false)
{
    _nmeaRef.setDefaultAction(DO_ROUTING);
}

// PFLAA,<AlarmLevel>,<RelativeNorth>,<RelativeEast>,<RelativeVertical>,<IDType>,<ID>,<Track>,<TurnRate>,<GroundSpeed>,<ClimbRate>,<AcftType>
// e.g.
// $PFLAA,0,-1234,1234,220,2,DD8F12,180,,30,-1.4,1*
dl_action_t FlarmMsg::parsePFLAA(NmeaPlugin *plg)
{
    ESP_LOGD(FNAME, "parsePFLAA");
    return DO_ROUTING;
}


// PFLAE,<QueryType>,<Severity>,<ErrorCode>[,<Message>]
//
// <QueryType> R = request FLARM to send status and error codes; other parameters should then be omitted
//         A = FLARM sends status (requested and spontaneous)
// <Severity> Decimal integer value. Range: from 0 to 3.
//         0 = no error, i.e. normal operation. Disregard other parameters.
//         1 = information only, i.e. normal operation
//         2 = functionality may be reduced
//         3 = fatal problem, device will not work
// <ErrorCode> Hexadecimal value. Range: from 0 to FFF. Error codes:
//         11 = Firmware expired (requires valid GPS information, i.e. will not be available in the first minute or so after power-on)
//         12 = Firmware update error
//         21 = Power (e.g. voltage < 8V)
//         22 = UI error
//         23 = Audio error
//         24 = ADC error
//         25 = SD card error
//         26 = USB error
//         27 = LED error
//         28 = EEPROM error
//         29 = General hardware error
//         2A = Transponder receiver Mode-C/S/ADS-B unserviceable
//         2B = EEPROM error
//         2C = GPIO error
//         31 = GPS communication
//         32 = Configuration of GPS module
//         33 = GPS antenna
//         41 = RF communication
//         42 = Another FLARM device with the same radio ID is being received. Alarms are suppressed for the relevant device.
//         43 = Wrong ICAO 24-bit address or radio ID
//         51 = Communication
//         61 = Flash memory
//         71 = Pressure sensor
//         81 = Obstacle database (e.g. incorrect file type)
//         82 = Obstacle database expired.
//         91 = Flight recorder
//         93 = Engine-noise recording not possible
//         94 = Range analyzer
//         A1 = Configuration error, e.g. while reading
//         flarmcfg.txt from SD/USB.
//         B1 = Invalid obstacle database license (e.g. wrong
//         serial number)
//         B2 = Invalid IGC feature license
//         B3 = Invalid AUD feature license
//         B4 = Invalid ENL feature license
//         B5 = Invalid RFB feature license
//         B6 = Invalid TIS feature license
//         100 = Generic error
//         101 = Flash File System error
//         110 = Failure updating firmware of external display
//         120 = Device is operated outside the designated region. The device does not work.
//         F1 = Other
// <Message> Field is omitted if data port version <7 or if DEVTYPE = Flarm04.
//         String. Maximum 40 ASCII characters. Textual description of the error in English. The field may be empty.

dl_action_t FlarmMsg::parsePFLAE(NmeaPlugin *plg)
{
    ESP_LOGD(FNAME, "parsePFLAE");
    ProtocolState *sm = plg->getNMEA().getSM();
    const std::vector<int> *word = &sm->_word_start;

    if ( word->size() == 3
        && sm->_frame.at(word->at(0)) == 'A'
        && sm->_frame.at(word->at(1)) == '0'
        && sm->_frame.at(word->at(2)) == '0' )
    {
        ESP_LOGI(FNAME, "got PFLAE");
    }
    return DO_ROUTING;
}


// PFLAU,<RX>,<TX>,<GPS>,<Power>,<AlarmLevel>,<RelativeBearing>,<AlarmType>,<RelativeVertical>,<HorizontalDistance>,<ID>
//  $PFLAU,3,1,2,1,2,-30,2,-32,755*FLARM is working properly and currently receives 3 other aircraft.
//  The most dangerous of these aircraft is at 11 o’clock, position 32m below and 755m away. It is a level 2 alarm
//
// <AcftType>
// 0 = unknown
// 1 = glider / motor glider
// 2 = tow / tug plane
// 3 = helicopter / rotorcraft
// 4 = skydiver
// 5 = drop plane for skydivers
// 6 = hang glider (hard)
// 7 = paraglider (soft)
// 8 = aircraft with reciprocating engine(s)
// 9 = aircraft with jet/turboprop engine(s)
// A = unknown
// B = balloon
// C = airship
// D = unmanned aerial vehicle (UAV)
// E = unknown
// F = static object
//
dl_action_t FlarmMsg::parsePFLAU(NmeaPlugin *plg)
{
    ESP_LOGI(FNAME,"parsePFLAU");
    ProtocolState *sm = plg->getNMEA().getSM();
    const std::vector<int> *word = &sm->_word_start;
    if ( word->size() < 9 ) {
        return NOACTION;
    }

    const char *s = sm->_frame.c_str();
    Flarm::RX          = atoi(s + word->at(0));
    Flarm::TX          = atoi(s + word->at(1));
    Flarm::GPS         = atoi(s + word->at(2));
    Flarm::Power       = atoi(s + word->at(3));
    Flarm::AlarmLevel  = atoi(s + word->at(4));
    int tmp = atoi(s + word->at(5));
    Flarm::RelativeBearing = Units::deg_to_rad(tmp);
    Flarm::RelativeVertical = atoi(s + word->at(7));
    Flarm::HorizontalDistance = atoi(s + word->at(8));
    Flarm::IcaoId = 0;
    if ( word->size() >= 10 ) {
        Flarm::IcaoId = atoi(s + word->at(9));
    }
    ESP_LOGI(FNAME,"RB: %f ALT:%d  DIST %d", Flarm::RelativeBearing, Flarm::RelativeVertical, Flarm::RelativeDistance);

    if ( Flarm::AlarmLevel >= flarm_warning.get() && ! Flarm::isConfirmed() ) {
        ESP_LOGI(FNAME,"FLARM ALARM LEVEL %d", Flarm::AlarmLevel);
        // Send a flarm event to update display
        int evt = ScreenEvent(ScreenEvent::FLARM_ALARM).raw;
        xQueueSend(uiEventQueue, &evt, 0);
    }

    if ( !status_ok && Flarm::GPS > 0 && Flarm::TX > 0 ) {
        ESP_LOGI(FNAME, "FLARM status OK %d,%d,%d,%d", Flarm::RX, Flarm::TX, Flarm::GPS, Flarm::Power);
        status_ok = true;
        MBOX->pushMessage(1, "FLARM status Okay");
    }
    return DO_ROUTING;
}


// $PFLAX,A*2E
//
// Note, if the Flarm switch to binary mode was accepted, Flarm will answer
// with $PFLAX,A*2E. In error case you will get as answer $PFLAX,A,<error>*
// and the Flarm stays in text mode.
dl_action_t FlarmMsg::parsePFLAX(NmeaPlugin *plg)
{
    ESP_LOGI(FNAME,"parsePFLAX A ----------------> switch to binary");
    NmeaPrtcl &nmea = plg->getNMEA();
    ProtocolState *sm = nmea.getSM();
    const std::vector<int> *word = &sm->_word_start;

    if ( word->size() == 2 ) {
        if ( *(sm->_frame.c_str() + word->at(0)) == 'A' ) {
            // this is the confirmation from flarm to go binary
            ESP_LOGI(FNAME,"confirmed");
            DataLink *host = DEVMAN->getFlarmBPInitiator();
            if ( host && host->getProtocol(FLARMBIN_P) && nmea.getDL()->getProtocol(FLARMBIN_P)) {
                // Host side
                FlarmBinary *hostfb = static_cast<FlarmBinary*>(host->getBinary());
                ESP_LOGI(FNAME, "Host side %d", hostfb->getDeviceId());
                // Device side
                FlarmBinary *devfb = static_cast<FlarmBinary*>(nmea.getDL()->goBIN());
                ESP_LOGI(FNAME, "Device side %d", devfb->getDeviceId());
                // Cross link them
                devfb->setPeer(hostfb);
                hostfb->setPeer(devfb);
            }
        }
        else if ( *(sm->_frame.c_str() + word->at(0)) == 'S' ) {
            if ( SetupCommon::isMaster() && !gflags.inSimulationMode ) {
                // XCV extension to switch to simulation mode
                ESP_LOGI(FNAME,"enter SIMULATION MODE");
                // replace the temp sensor with a virtual one
                DEVMAN->removeDevice(TEMPSENS_DEV);
                DEVMAN->addDevice(TEMPSENS_DEV, NO_ONE, 0, 0, NO_PHY);
                DEVMAN->removeDevice(MAGLEG_DEV); // drop input from a sensor
                SensorBase *mag = MagVSensor::createMagVSensor(); // add a mag sensor so that it is not part of the sensor doread loop
                SensorRegistry::registerSensor(mag);
                // disable real sensors
                SensorRegistry::enterSimMode();
                // add the XCVSimMsg NMEA plugin to the same data link / protocol instance
                nmea.addPlugin(new XCVSimMsg(nmea));
                gflags.inSimulationMode = true;
                MBOX->pushMessage(1, "Simulation Mode");
            }
            if ( gflags.inSimulationMode) {
                // time jump, reset clock
                if ( word->size() >= 2 ) {
                    int speed = atoi(sm->_frame.c_str() + word->at(1));
                    ESP_LOGI(FNAME,"Set SIM speed %d", speed);
                    Clock::setSimSpeed(speed);
                }
            }
            // always prepare vario for disruptive jump
            bmpVario.prepareForSimJump();
        }
    }

    return DO_ROUTING;
}


const ParserEntry FlarmMsg::_pt[] = {
    { Key("FLAA"), FlarmMsg::parsePFLAA },
    { Key("FLAE"), FlarmMsg::parsePFLAE },
    { Key("FLAU"), FlarmMsg::parsePFLAU },
    { Key("FLAX"), FlarmMsg::parsePFLAX },
    {}
};
