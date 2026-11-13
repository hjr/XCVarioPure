// AirMass entries of the setup menu

#include "AirMass.h"

#include "setup/primitive/SetupMenu.h"
#include "setup/primitive/SetupMenuSelect.h"
#include "setup/primitive/SetupMenuValFloat.h"
#include "setup/SetupCommon.h"
#include "setup/Capability.h"
#include "sensor.h"

// the estimator runs on the master, a second seat shows its values
static bool available()
{
    return SetupCommon::isMaster() ? airmass::supported() : XcvCaps::haveCap(XcvCaps::AHRS_CAP);
}

void airmass::menuTeMethod(SetupMenuSelect* method, SetupMenuValFloat* adjustment)
{
    if (!available()) {
        return;
    }
    method->setHelp("Probe, electronic or Air Mass compensation");
    method->addEntry("Air Mass", TE_AIRMASS);
    adjustment->setHelp("Compensation factor, raise it if pull-ups show false climb, lower it if they show false sink");
}

void airmass::menuLogging(SetupMenuSelect* logging)
{
    if (available()) {
        logging->setHelp("R&D option, do not just enable!\n Collect data with NMEA logger in XCSoar. All Sensor Data includes the Air Mass "
                         "output and start for the replay on a PC (1.4 kB/s in all, use Wi-Fi or Bluetooth)");
    }
}

void airmass::menuVario(SetupMenu* vario)
{
    if (available()) {
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
