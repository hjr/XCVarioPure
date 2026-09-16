/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "setup/SetupMenuDisplay.h"

#include "AdaptUGC.h"
#include "sensor.h"

#include <string>

extern AdaptUGC *MYUCG;

int show_boot_log(SetupMenuDisplay *p,int mode)
{
    if ( mode <= 1 ) { p->clear(); }

    const int line_height = 20;
    int ln = line_height;
    MYUCG->setFont(ucg_font_fub11_tr);

    size_t start = 0;
    while (start < logged_tests.size()) {
        size_t end = logged_tests.find('\n', start);
        if (end == std::string::npos) {
            end = logged_tests.size();
        }

        MYUCG->setPrintPos(0, ln);
        MYUCG->print(logged_tests.substr(start, end - start).c_str());

        ln += line_height;
        start = end + 1;
    }

    if ( mode == 0 ) {
        MYUCG->setPrintPos(20, ln+line_height);
        MYUCG->print("Press button to exit");
    }
    return 0;
}
