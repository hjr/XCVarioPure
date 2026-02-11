/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ****************************************************************************

File: ShowCompassSettings.cpp

Class to display the compass status overview.

Author: Axel Pauli, March 2021

Last update: 2021-04-18

 ****************************************************************************/

#include "SetupMenuDisplay.h"

#include "setup/SetupNG.h"
#include "AdaptUGC.h"

extern AdaptUGC *MYUCG;

int show_compass_setting(SetupMenuDisplay *p, int mode) {
    if (!theCompass) {
        p->clear();
        MYUCG->setFont(ucg_font_ncenR14_hr);
        MYUCG->setPrintPos(1, 30);
        MYUCG->printf("No magnetic Sensor");
    }
    else {
        p->clear();
        MYUCG->setFont(ucg_font_ncenR14_hr);
        p->menuPrintLn(p->getTitle(), 0, 5);

        uint16_t y = 75;
        char buffer[32];

        MYUCG->setPrintPos(0, y);
        sprintf(buffer, "Sensor enabled: %s", (theCompass) ? "Yes" : "No");
        MYUCG->print(buffer);
        y += LINE_HEIGHT;

        MYUCG->setPrintPos(0, y);
        bitfield_compass state = calibration_bits.get();
        bitfield_compass target = {1, 1, 1, 1, 1, 1};
        bool all_green = false;
        if (state == target)
            all_green = true;
        sprintf(buffer, "Sensor calibrated: %s", (compass_calibrated.get() == 0 || !all_green) ? "No" : "Yes");
        MYUCG->print(buffer);
        y += LINE_HEIGHT;

        const char *soText = "Sensor overflow: ";
        int sotw = MYUCG->getStrWidth(soText);
        MYUCG->setPrintPos(0, y);
        MYUCG->print(soText);
        MYUCG->setPrintPos(sotw, y);
        MYUCG->print((theCompass->overflowFlag() == false) ? "No" : "Yes");
        y += LINE_HEIGHT;

        MYUCG->setPrintPos(0, y);
        sprintf(buffer, "Compass declination: %d°", static_cast<int>(compass_declination.get()));
        MYUCG->print(buffer);
        y += LINE_HEIGHT;

        MYUCG->setPrintPos(0, y);
        sprintf(buffer, "Display damping: %.02fs", (compass_damping.get()));
        MYUCG->print(buffer);
        y += LINE_HEIGHT;

        MYUCG->setPrintPos(0, y);
        sprintf(buffer, "NMEA mag heading: %s", (compass_nmea_hdm.get() == 0) ? "No" : "Yes");
        MYUCG->print(buffer);
        y += LINE_HEIGHT;

        MYUCG->setPrintPos(0, y);
        sprintf(buffer, "NMEA true heading: %s", (compass_nmea_hdt.get() == 0) ? "No" : "Yes");
        MYUCG->print(buffer);
        y += LINE_HEIGHT;
    }
    MYUCG->setPrintPos(5, 290);
    MYUCG->print("Press button to exit");
    return 0;
}
