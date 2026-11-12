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

static int airmass_action(SetupMenuSelect* entry)
{
    if ((entry->getSelect() == 1 )) {
        airmass::startup();
    }
    else {
        airmass::shutdown();
    }
    return 0;
}

static void createMenu(SetupMenu* top)
{
    SetupMenuSelect* amwind = new SetupMenuSelect("Wind to Navi", RST_NONE, nullptr, &airmass_wind_nmea);
    amwind->setHelp("Send the 30s average of the air mass wind to the navigation computer ($WIMWV, or Cambridge !w), XCSoar takes it as external wind");
    amwind->mkEnable();
    top->addEntry(amwind);

    if (!SetupCommon::isMaster()) {
        return; // the estimator settings belong to the master
    }
    SetupMenuSelect* amena = new SetupMenuSelect("Estimator", RST_NONE, airmass_action, &airmass_enable);
    amena->setHelp("GNSS/IMU fusion for the vertical air mass movement (vario mode Air Mass) and instant wind. When disabled, the Air Mass vario modes show netto "
                   "and the estimator takes no memory. Changes restart the device");
    amena->mkEnable();
    top->addEntry(amena);

    SetupMenuValFloat* amv = new SetupMenuValFloat("Vario Agility", "", nullptr, &airmass_vario_resp, RST_NONE, false);
    amv->setHelp("Higher values follow vertical air movement faster, lower values give a calmer indication");
    amv->setPrecision(1);
    top->addEntry(amv);

    SetupMenuValFloat* amc = new SetupMenuValFloat("Pull-up Comp.", "%", nullptr, &airmass_te_adjust, RST_NONE, false);
    amc->setHelp("Static port error compensation, same meaning as the TE eAdjustment: lower it if pull-ups show false climb, raise it if they show false sink");
    amc->setPrecision(1);
    top->addEntry(amc);

    SetupMenuValFloat* amw = new SetupMenuValFloat("Wind Agility", "", nullptr, &airmass_wind_resp, RST_NONE, false);
    amw->setHelp("Higher values follow wind changes and gusts faster, lower values give a steadier instant wind");
    amw->setPrecision(1);
    top->addEntry(amw);

    SetupMenuSelect* amnmea = new SetupMenuSelect("NMEA Output", RST_NONE, nullptr, &airmass_nmea);
    amnmea->setHelp("Send the $PXCVA sentence with air mass vario, instant wind and learned calibration. Recording adds the estimator "
                    "inputs at 20Hz (about 3 kB/s, use Wi-Fi or Bluetooth): a flight logged by the navigation computer can then "
                    "be replayed and analysed on a PC");
    amnmea->addEntry("Disable", 0);
    amnmea->addEntry("Output", 1);
    amnmea->addEntry("Recording", 2);
    top->addEntry(amnmea);
}

void airmass::menuVario(SetupMenu* vario)
{
    if (!available()) {
        return;
    }
    SetupMenu* amass = new SetupMenu("Air Mass", createMenu);
    amass->setHelp("Settings of the air mass estimator, fusing GNSS, IMU and probes.");
    vario->addEntry(amass);
}
