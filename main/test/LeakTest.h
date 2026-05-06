
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
    const char *value() const override;

    void display(int mode = 0) override;
    static const char * getStatus();
  private:
    mutable std::string _result;
};
