// AirMass entries of the setup menu

#include "AirMass.h"

#include "setup/primitive/SetupMenu.h"
#include "setup/primitive/SetupMenuSelect.h"
#include "setup/primitive/SetupMenuValFloat.h"
#include "setup/SetupCommon.h"
#include "setup/Capability.h"
#include "sensor.h"
#include "AirMassSensor.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

// the estimator runs on the master, a second seat shows its values
static bool available()
{
    return SetupCommon::isMaster() ? airmass::supported() : XcvCaps::haveCap(XcvCaps::AHRS_CAP);
}


void airmass::menuLogging(SetupMenuSelect* logging)
{
    if (available()) {
        logging->setHelp("R&D option, do not just enable!\n Collect data with NMEA logger in XCSoar. All Sensor Data includes the Air Mass "
                         "output and start for the replay on a PC (1.4 kB/s in all, use Wi-Fi or Bluetooth)");
    }
}

void airmass::menuEAdjust(MenuEntry* eadjust, const char* help)
{
    if (!eadjust) {
        return;
    }
    static char text[200];
    const float v = airmassSensor ? airmassSensor->teRecommended() : NAN;
    if (std::isfinite(v)) {
        snprintf(text, sizeof(text), "%s. Recommended value for this glider: %.0f%%.", help,
                 std::clamp(v, te_comp_adjust.getMin(), te_comp_adjust.getMax()));
        eadjust->setHelp(text);
    } else {
        eadjust->setHelp(help);
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
    navi->setHelp("Send the averaged air mass wind to the navigation computer ($WIMWV, or Cambridge !w), XCSoar takes it as external wind. $PXCVW has the average and the instant wind");
    navi->mkEnable();
    wind->addEntry(navi);
}

void airmass::menuAudio(SetupMenu* audio)
{
    // only the estimator sounds it
    if (!airmass::supported()) {
        return;
    }
    SetupMenuSelect* gust = new SetupMenuSelect("Gust Sound", RST_NONE, nullptr, &airmass_gust_sound);
    gust->setHelp("Wind noise and a message when the head wind changes suddenly in straight flight");
    gust->mkEnable();
    audio->addEntry(gust);
}
