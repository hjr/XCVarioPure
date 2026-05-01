/*
 * OTA.h
 *
 *  Created on: Feb 24, 2019
 *      Author: iltis
 */
#pragma once

#include "setup/SetupMenuDisplay.h"
#include "driver/time/ClockIntf.h"


class OTA final : public SetupMenuDisplay, public Clock_I 
{
  public:
    OTA(bool do_dpp);
    ~OTA();
    void display(int) override;
    void press() override;
    bool tick() override;

  private:
  int _count = 900;
  bool _do_dpp;
};

