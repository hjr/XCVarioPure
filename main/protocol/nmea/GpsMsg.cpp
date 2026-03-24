/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "GpsMsg.h"
#include "Flarm.h"
#include "sensor/gps/GpsVSensor.h"
#include "wind/CircleWind.h"
#include "wind/WindCalcTask.h"
#include "driver/time/Clock.h"
#include "setup/SetupNG.h"
#include "math/Units.h"
#include "sensor.h"
#include "logdef.h"


#include <cmath>
#include <cstdlib>
#include <cstring>
#include <time.h>
#include <sys/time.h>




static float decodeNMEADegMin(const char* degmin, char hemi)
{
    if (!degmin || !*degmin)
        return NAN;

    float v = std::strtof(degmin, nullptr);

    int deg = static_cast<int>(v / 100.0f);
    float min = v - deg * 100.0f;

    float deg_dec = deg + min / 60.0f;

    if (hemi == 'S' || hemi == 'W')
        deg_dec = -deg_dec;

    return deg_dec;
}



// The GPS protocol parser.
//
// Supported messages:
// GPRMC,081836,A,3751.65,S,14507.36,E,000.0,360.0,130998,011.3,E*62
// GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh


// $GPRMC,081836,A,3751.65,S,14507.36,E,000.0,360.0,130998,011.3,E*62
// $GPRMC,225446,A,4916.45,N,12311.12,W,000.5,054.7,191194,020.3,E*68
// $GPRMC,201914.00,A,4857.58740,N,00856.94735,E,0.172,122.95,310321,,,A*6D
//
//            225446.00    Time of fix 22:54:46 UTC
//            A            Navigation receiver warning A = OK, V = warning
//            4916.45,N    Latitude 49 deg. 16.45 min North
//            12311.12,W   Longitude 123 deg. 11.12 min West
//            000.5        Speed over ground, Knots
//            054.7        Course Made Good, True
//            191194       Date of fix  19 November 1994
//            020.3,E      Magnetic variation 20.3 deg East
//  *68          mandatory checksum
dl_action_t GpsMsg::parseGPRMC(NmeaPlugin *plg)
{
    ProtocolState *sm = plg->getNMEA().getSM();
    const std::vector<int> *word = &sm->_word_start;
    if ( word->size() < 9 ) {
        return NOACTION;
    }
    ESP_LOGD(FNAME, "parseGPRMC");

    struct tm t;
    const char *s = sm->_frame.c_str();
    int valid_time_scan = sscanf( s + word->at(0), "%02d%02d%02d", &t.tm_hour, &t.tm_min, &t.tm_sec );
    char warn = *(s + word->at(1));
    float lat = decodeNMEADegMin( s + word->at(2), *(s + word->at(3)) );
    float lon = decodeNMEADegMin( s + word->at(4), *(s + word->at(5)) );
    float tmp;
    sscanf( s + word->at(6), "%f", &tmp );
    Flarm::gndSpeed = Units::knots_to_mps(tmp);
    sscanf( s + word->at(7), "%f", &tmp );
    Flarm::gndCourse = Units::deg_to_rad(tmp);
    int valid_date_scan = sscanf( s + word->at(8),"%02d%02d%02d", &t.tm_mday, &t.tm_mon, &t.tm_year );
    // ESP_LOGI(FNAME,"SC: %d/%d/%d %02d:%02d:%02d ", t.tm_year, t.tm_mon, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec );

    // ESP_LOGI(FNAME,"G: %s",_gprmc );
    // ESP_LOGI(FNAME,"parseGPRMC() GPS: %d, Speed: %3.1f mps, Track: %3.1f° warn:%c", Flarm::myGPS_OK, Flarm::gndSpeed, Units::rad_to_deg(Flarm::gndCourse), warn);
    // ESP_LOGI(FNAME,"GP%s, GPS_OK:%d warn:%c T:%s D:%s", gprmc+3, myGPS_OK, warn, time, date  );
    // set the GND speed
    gnd_speed.set(Flarm::gndSpeed);

    Flarm::myGPS_OK = (warn == 'A');
    if (Flarm::myGPS_OK) {
        if (!std::isnan(lat) && !std::isnan(lon)) {
           // valid fix
           gpsSensor->inject(lat, lon);
        }
        if ( BackgroundTaskQueue ) {
            ESP_LOGD(FNAME,"Track: %3.2f, GPRMC: %s", Flarm::gndCourse, sm->_frame.c_str() );
            circleWind->setNewSample(Vector(Flarm::gndCourse, Flarm::gndSpeed));

            CalkTaskJob job(CalkTaskJob::CALK_TASK_EVENT_NEW_GPSPOSE);
            xQueueSend(BackgroundTaskQueue, &job, 0);
        }
        ESP_LOGD(FNAME,"Valid GPS Time info received %d/%d.", valid_date_scan, valid_time_scan );
        if (valid_time_scan == 3 && valid_date_scan == 3)
        {
            t.tm_year += 2000 - 1900; // NMEA gives only two digit year
            t.tm_mon -= 1; // struct tm month is 0..11
            long int epoch_time = mktime(&t);
            if (!Clock::isValidUTC()) // set system time if not yet done
            {
                // just once, at first valid GPS time info
                ESP_LOGD(FNAME, "Start TimeSync");
                timeval epoch = {epoch_time, 0};
                settimeofday(&epoch, nullptr);
                Clock::setTimeUTC(epoch_time * 1000LL);
                ESP_LOGD(FNAME, "Finish Time Sync");
            }
            else if ( Clock::getUpdateAgeMs() > 60000 || gflags.inSimulationMode )  // update every minute
            {
                // update time offset
                Clock::updateTimeUTC(epoch_time * 1000LL);
            }
        }
    }
    return DO_ROUTING;
}


// eg. $GPGGA,121318.00,4857.58750,N,00856.95715,E,1,05,3.87,247.7,M,48.0,M,,*52
//
// hhmmss.ss = UTC of position
// llll.ll = latitude of position
// a = N or S
// yyyyy.yy = Longitude of position
// a = E or W
// x = GPS Quality indicator (0=no fix, 1=GPS fix, 2=Dif. GPS fix)
// xx = number of satellites in use
// x.x = horizontal dilution of precision
// x.x = Antenna altitude above mean-sea-level
// M = units of antenna altitude, meters
// x.x = Geoidal separation
// M = units of geoidal separation, meters
// x.x = Age of Differential GPS data (seconds)
// xxxx = Differential reference station ID
//
dl_action_t GpsMsg::parseGPGGA(NmeaPlugin *plg)
{
    ProtocolState *sm = plg->getNMEA().getSM();
    const std::vector<int> *word = &sm->_word_start;
    ESP_LOGD(FNAME, "parseGPGGA");

    if (word->size() > 13)
    {
        int numSat = atoi(sm->_frame.c_str() + word->at(6));
        ESP_LOGD(FNAME, "numSat=%d", numSat);
        if ((numSat != Flarm::_numSat) && (wind_enable.get() & WA_BOTH))
        {
            Flarm::_numSat = numSat;
            if (BackgroundTaskQueue) {
                CalkTaskJob job(CalkTaskJob::CALK_TASK_EVENT_NUMSAT);
                job.setDetail(numSat);
                xQueueSend(BackgroundTaskQueue, &job, 0);
            }
        }

        float alt = std::strtof(sm->_frame.c_str() + word->at(8), nullptr);
        // GPGGA altitude is defined in meters (MSL)
        // Unit field should be 'M', ignore otherwise
        if ((sm->_frame.c_str() + word->at(8))[0] == 'M') {
            gpsSensor->setExternalAltitude(alt);
        }

    }
    return DO_ROUTING;
}

const ParserEntry GpsMsg::_pt[] = {
    { Key("PRMC"), GpsMsg::parseGPRMC },
    { Key("PGGA"), GpsMsg::parseGPGGA },
    {}
};
