
#pragma once

#include "setup/SetupMenuDisplay.h"

class PressureSensor;
class AirspeedSensor;

class LeakTest : public SetupMenuDisplay {
  public:
    LeakTest(const char* title) : SetupMenuDisplay(title, nullptr) {}
    virtual ~LeakTest() = default;

    void display(int mode = 0) override;
};
