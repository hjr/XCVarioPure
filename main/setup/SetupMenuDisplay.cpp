/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ****************************************************************************

File: SetupMenuDisplay.cpp

Generic class to display text in a menu item.

Author: Axel Pauli, February 2021

Last update: 2021-02-25

 ****************************************************************************/

#include "setup/SetupMenuDisplay.h"
#include "AdaptUGC.h"
#include "logdefnone.h"

#include <esp_system.h>

extern AdaptUGC *MYUCG;

SetupMenuDisplay::SetupMenuDisplay(const char *title, int (*action)(SetupMenuDisplay *p, int mode)) :
    MenuEntry(title)
{
    _action = action;
}

void SetupMenuDisplay::display(int mode) {
    if (_action != nullptr) {
        ESP_LOGI(FNAME, "SetupMenuDisplay::display mode %d", mode);
        MYUCG->setFont(ucg_font_fub14_hr, true);
        // Call user's callback
        (*_action)(this, mode);
    }
}

void SetupMenuDisplay::press() { exit(); }

void SetupMenuDisplay::longPress() { press(); }
