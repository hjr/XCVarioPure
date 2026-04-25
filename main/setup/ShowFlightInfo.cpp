/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "ShowFlightInfo.h"

#include "SensorMgr.h"
#include "comm/DeviceMgr.h"
#include "driver/time/Clock.h"
#include "AdaptUGC.h"
#include "logdef.h"

#include <time.h>

extern AdaptUGC *MYUCG;

void ShowFlightInfo::display(int mode) {
    ESP_LOGI(FNAME, "display() mode=%d", mode);
    if (mode == 0) {
        clear();
    }
    MYUCG->setFont(ucg_font_ncenR14_hr);
    menuPrintLn(_title.c_str(), 0);

    time_t t = Clock::getMillisUTC() / 1000;
    struct tm utc;
    gmtime_r(&t, &utc);

    char buffer[200];
    int i = 1;

    sprintf(buffer, "Date: %d-%d-%d %02d:%02d:%02d", utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday, utc.tm_hour, utc.tm_min, utc.tm_sec);
    menuPrintLn(buffer, i++);

    menuPrintLn("Press button to exit", i + 1, 5);

#ifdef DEBUG_AND_TEST
    DEVMAN->dumpMap();
    SensorRegistry::dump();
#endif
}
