/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "setup/MenuEntry.h"

// The action menu never attaches and has no ability to be entered, 
// but executes the action immediately and passes this pointer to properly exit it.

class SetupAction final : public MenuEntry {
   public:
    explicit SetupAction(const char* title, int (*action)(SetupAction*), int code);
    virtual ~SetupAction() = default;
    void enter() override;
    void display(int mode = 0) override {};
    const char* value() const override { return ""; };
    int getCode() const { return _code; }
    // Rotoary API
    void rot(int count) override {};
    void press() override {};
    void longPress() override {}

   private:
    int (*_action)(SetupAction* p);
    int _code;
};
