// AirMass entries of the setup menu

#include "AirMass.h"
#include "AirMassSensor.h"

#include "setup/primitive/SetupMenu.h"
#include "setup/primitive/SetupMenuSelect.h"
#include "setup/SetupCommon.h"
#include "setup/Capability.h"

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

static void createMenu(SetupMenu* top)
{
    SetupMenuSelect* amwind = new SetupMenuSelect("Wind to Navi", RST_NONE, nullptr, &airmass_wind_nmea);
    amwind->setHelp("Send the 30s average of the air mass wind to the navigation computer ($WIMWV, or Cambridge !w), XCSoar takes it as external wind");
    amwind->mkEnable();
    top->addEntry(amwind);
}

void airmass::menuVario(SetupMenu* vario)
{
    if (!available()) {
        return;
    }
    SetupMenu* amass = new SetupMenu("Air Mass", createMenu);
    amass->setHelp("Settings of the air mass estimator, fusing GNSS, IMU and air data. The Air Mass vario uses half the Vario "
                   "Damping, it does not take gusts for lift; Super-Netto applies to it as well");
    vario->addEntry(amass);
}
