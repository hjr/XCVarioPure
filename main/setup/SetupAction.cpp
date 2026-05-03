/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "SetupAction.h"

#include "setup/SetupMenu.h"
#include "AdaptUGC.h"
#include "logdefnone.h"

extern AdaptUGC *MYUCG;


SetupAction::SetupAction(const char *title, int (*action)(SetupAction *), int code) :
    MenuEntry(title),
    _action(action),
    _code(code)
{
    ESP_LOGI(FNAME, "SetupAction( %s ) ", title);
}

void SetupAction::enter()
{
    // decide on return value wether this got hijacked
    if ( ! (*_action)(this) ) {
        // or control is handed straight back to the calling menu
        _parent->display(); // refresh parent menu, e.g. to update values after action
    }
}

