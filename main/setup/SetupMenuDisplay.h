/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ****************************************************************************

File: SetupMenuDisplay.h

Generic class to display text in a menu item.

Author: Axel Pauli, February 2021

Last update: 2021-02-25

 ****************************************************************************/

#pragma once

#include "setup/MenuEntry.h"


class SetupMenuDisplay: public MenuEntry
{
public:
    explicit SetupMenuDisplay(const char *title, int (*action)(SetupMenuDisplay *p, int mode) = nullptr);
    SetupMenuDisplay() = delete;
    virtual ~SetupMenuDisplay() = default;

    // Display the menu item, optionally with a mode parameter
    void display(int mode = 0) override;
    // In case some additional display logic is needed
    void setUserAttr(int attr) { _user_attr = attr; }
    int getUserAttr() const { return _user_attr; }

    const char *value() const override { return nullptr; }
    void rot(int count) override {}
    void press() override;
    void longPress() override;

private:
    // User's callback function
    int (*_action)(SetupMenuDisplay *p, int mode);
    int _user_attr = 0;
};
