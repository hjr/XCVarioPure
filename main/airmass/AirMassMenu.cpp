// AirMass entries of the setup menu

#include "AirMass.h"
#include "AirMassSensor.h"

#include "setup/primitive/SetupMenu.h"
#include "setup/primitive/SetupMenuSelect.h"
#include "setup/primitive/SetupMenuValFloat.h"
#include "setup/SetupCommon.h"
#include "setup/Capability.h"

// the estimator runs on the master, a second seat shows its values
static bool available()
{
    return SetupCommon::isMaster() ? airmass::supported() : XcvCaps::haveCap(XcvCaps::AHRS_CAP);
}

void airmass::menuVarioModes(SetupMenuSelect* mode)
{
    if (!available()) {
        return;
    }
    mode->setHelp("Vario indicator considers polar sink (=netto), or not (=gross), or net only in Cruise Mode. Air Mass shows the "
                  "vertical air movement from the GNSS/IMU estimator and uses it for speed to fly, falls back to netto without it. "
                  "Air Mass Gross adds the polar sink: the climb of the glider with the air mass filter, falls back to gross");
    mode->addEntry("Air Mass", static_cast<int>(e_vario_mode::VARIO_AIRMASS));
    mode->addEntry("Cruise-Air Mass", static_cast<int>(e_vario_mode::CRUISE_ONLY_AIRMASS));
    mode->addEntry("Air Mass Gross", static_cast<int>(e_vario_mode::VARIO_AIRMASS_GROSS));
}

static void createMenu(SetupMenu* top)
{
    SetupMenuSelect* amwind = new SetupMenuSelect("Wind to Navi", RST_NONE, nullptr, &airmass_wind_nmea);
    amwind->setHelp("Send the 30s average of the air mass wind to the navigation computer ($WIMWV, or Cambridge !w), XCSoar takes it as external wind");
    amwind->mkEnable();
    top->addEntry(amwind);

    if (SetupCommon::isMaster()) {
        SetupMenuValFloat* amc = new SetupMenuValFloat("Pull-up Comp.", "%", nullptr, &airmass_te_adjust, RST_NONE, false);
        amc->setHelp("Static port error compensation, same meaning as the TE eAdjustment: lower it if pull-ups show false climb, raise it if they show false sink");
        amc->setPrecision(1);
        top->addEntry(amc);
    }
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
