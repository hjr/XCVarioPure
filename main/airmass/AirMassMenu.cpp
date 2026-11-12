// AirMass entries of the setup menu

#include "AirMass.h"

#include "setup/primitive/SetupMenu.h"
#include "setup/primitive/SetupMenuSelect.h"
#include "setup/SetupCommon.h"
#include "setup/Capability.h"
#include "sensor.h"

// the estimator runs on the master, a second seat shows its values
static bool available()
{
    return SetupCommon::isMaster() ? airmass::supported() : XcvCaps::haveCap(XcvCaps::AHRS_CAP);
}

void airmass::menuTeMethod(SetupMenuSelect* method)
{
    if (!available()) {
        return;
    }
    method->setHelp("Probe, electronic or Air Mass compensation");
    method->addEntry("Air Mass", TE_AIRMASS);
}

void airmass::menuVario(SetupMenu* vario)
{
    if (available() && gflags.expert) {
        vario->addEntry(statusPage());
    }
}

void airmass::menuWind(SetupMenu* wind)
{
    if (!available()) {
        return;
    }
    SetupMenuSelect* navi = new SetupMenuSelect("Wind to Navi", RST_NONE, nullptr, &airmass_wind_nmea);
    navi->setHelp("Send the 30s average of the air mass wind to the navigation computer ($WIMWV, or Cambridge !w), XCSoar takes it as external wind");
    navi->mkEnable();
    wind->addEntry(navi);
}
