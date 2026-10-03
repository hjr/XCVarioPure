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
#include "logdefnone.h"

#include <string>

extern AdaptUGC *MYUCG;

int show_boot_log(SetupMenuDisplay *p,int mode)
{
    int updateln = p->getUserAttr();
    if ( mode < 1 ) {
        p->clear();
        updateln = 0;
    }
    ESP_LOGI(FNAME, "show_boot_log called with mode=%d, updateln=%d", mode, updateln);

    MYUCG->setFont(ucg_font_fub11_tr);

    const int LINE_HEIGHT = 20;
    int ln = 0;
    size_t start = 0;
    while (start < logged_tests.size()) {
        size_t end = logged_tests.find('\n', start);
        if (end == std::string::npos) {
            end = logged_tests.size();
        }
        ln += LINE_HEIGHT;

        if ( ln >= updateln ) {
            MYUCG->setPrintPos(0, ln);
            MYUCG->print(logged_tests.substr(start, end - start).c_str());
        }

        start = end + 1;
    }
    p->setUserAttr(ln);

    if ( mode == 0 ) {
        MYUCG->setPrintPos(20, ln+LINE_HEIGHT);
        MYUCG->print("Press button to exit");
    }
    return 0;
}
