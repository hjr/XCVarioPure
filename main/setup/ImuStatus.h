
#pragma once

#include "setup/SetupMenuDisplay.h"

class ImuStatus : public SetupMenuDisplay {
  public:
    ImuStatus(const char* title) : SetupMenuDisplay(title, nullptr) {}
    virtual ~ImuStatus() = default;

    void display(int mode = 0) override;
    void press() override;
  private:
    bool go_on = true;
};
