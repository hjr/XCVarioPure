
#pragma once

#include "setup/SetupMenuDisplay.h"

class PressureSensor;
class AirspeedSensor;

class LeakTest : public SetupMenuDisplay {
  public:
    static constexpr float LEAK_TEST_MAX_LOSS = 0.05f;

  public:
    LeakTest(const char* title) : SetupMenuDisplay(title, nullptr) {}
    virtual ~LeakTest() = default;

    void display(int mode = 0) override;
};
