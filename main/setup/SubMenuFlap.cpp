/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "SubMenuFlap.h"

#include "Flap.h"
#include "setup/SetupNG.h"
#include "setup/SetupMenu.h"
#include "setup/SetupMenuSelect.h"
#include "setup/SetupMenuChar.h"
#include "setup/SetupMenuValFloat.h"
#include "AdaptUGC.h"
#include "sensor.h"
#include "logdefnone.h"

#include <string>
#include <cstdint>

extern AdaptUGC *MYUCG;

static int new_level_label;
static SetupNG<mps_t> *new_level_speed = nullptr;
static int8_t new_level_complete;

static std::string flap_level_menu[Flap::MAX_NR_POS];
static std::string flap_level_buzz[Flap::MAX_NR_POS];


// Action Routines
static int select_flap_sens_pin(SetupMenuSelect *p)
{
    ESP_LOGI(FNAME, "select_flap_sens_pin");
    FLAP->configureADC();
    if (p->getSelect() == FLAP_SENSOR_ENABLE)
    {
        p->clear();
        ESP_LOGI(FNAME, "select_flap_sens_pin, have flap");
        if (FLAP->haveAdcSensor())
        {
            // ESP_LOGI(FNAME,"select_flap_sens_pin, have sensor");
            MYUCG->setPrintPos(5, 50);
            MYUCG->setFont(ucg_font_ncenR14_hr, true);
            MYUCG->printf("Check Sensor Reading,");
            MYUCG->setPrintPos(5, 80);
            MYUCG->printf("Press Button to exit");
            while (! Rotary->readSwitch(100))
            {
                // ESP_LOGI(FNAME,"SW wait loop");
                MYUCG->setPrintPos(5, 120);
                MYUCG->printf("Sensor: %d       ", FLAP->getSensorRaw());
            }
        }
        delay(800);
        p->clear();
        // add the flap sensor to my caps
        my_caps.set( my_caps.get() | XcvCaps::FLAPSENS_CAP );
    }
    else
    {
        ESP_LOGI(FNAME, "NO flap");
        // remove the flap sensor to my caps
        my_caps.set( my_caps.get() & ~XcvCaps::FLAPSENS_CAP );
    }
    p->getParent()->setDirty();
    p->getParent()->getParent()->setDirty();
    return 0;
}

static int wk_calib_level(SetupMenuSelect *p, int wk, Average<15> &filter)
{
    MYUCG->setPrintPos(1, 60);
    MYUCG->printf("Set Flap %s ", FLAP->getFL(wk)->label);
    int sensval = 0;
    int i = 0;
    while (! Rotary->readSwitch() && FLAP)
    {
        i++;
        sensval = filter((int)(FLAP->getSensorRaw()));
        if (!(i % 5))
        {
            MYUCG->setPrintPos(1, 140);
            MYUCG->printf("Sensor: %d      ", sensval);
        }
    }
    return sensval;
}

static int flap_cal_act(SetupMenuSelect *p)
{
    ESP_LOGI(FNAME, "WK calibration ( %d ) ", p->getSelect());
    if (!FLAP)
    {
        return 0;
    }
    if (!FLAP->haveAdcSensor())
    {
        p->clear();
        MYUCG->setPrintPos(1, 60);
        MYUCG->printf("No Sensor, Abort");
        delay(2000);
        ESP_LOGI(FNAME, "Abort calibration, no signal");
        return 0;
    }
    Average<15> filter;
    if (p->getSelect())
    {
        // do calibration
        p->clear();
        MYUCG->setPrintPos(1, 200);
        MYUCG->setFont(ucg_font_ncenR14_hr, true);
        MYUCG->printf("Press for next");
        MYUCG->setFont(ucg_font_fub25_hr, true);
        int sensval;
        for (int wk = 0; wk < FLAP->getNrPositions(); wk++)
        {
            sensval = wk_calib_level(p, wk, filter);
            FLAP->setSensCal(wk, sensval);
        }
        FLAP->modLevels();
        MYUCG->setPrintPos(1, 260);
        MYUCG->setFont(ucg_font_ncenR14_hr, true);
        MYUCG->printf("Saved");
        delay(800);
        p->clear();
    }
    p->setSelect(0);
    return 0;
}

void flap_menu_create_flap_sensor(SetupMenu *wkm) // dynamic!
{
    if ( wkm->getNrChilds() == 0 ) {
        wkm->setDynContent();
        SetupMenuSelect *wkes = new SetupMenuSelect("Flap Sensor", RST_NONE, select_flap_sens_pin, &flap_sensor);
        wkes->mkEnable();
        wkes->setHelp("A to this unit connected Flap sensor");
        wkm->addEntry(wkes);

        SetupMenuSelect *wkcal = new SetupMenuSelect("Sensor Calibration", RST_NONE, flap_cal_act);
        wkcal->addEntry("Cancel");
        wkcal->addEntry("Start");
        wkcal->setHelp("Calibrate the flap sensor to the configured levels. (Press button to proceed)");
        wkm->addEntry(wkcal);
    }
    SetupMenu *wkcal = static_cast<SetupMenu*>(wkm->getEntry(1));
    if ( flap_sensor.get() == FLAP_SENSOR_ENABLE ) {
        wkcal->unlock();
    } else {
        wkcal->lock();
    }
}


//////////////////////////////
// Add new level
static int select_level_action(SetupMenuChar *p)
{
    ESP_LOGI(FNAME,"action %s", p->value());
    std::strncpy((char*)&new_level_label, p->value(), 4);
    ((char*)&new_level_label)[3] = '\0';
    new_level_complete |= 0x1;
    if ( new_level_complete == 0x3 ) {
        p->getParent()->getEntry(2)->unlock();
    }
    return 0;
}
static int select_speed_action(SetupMenuValFloat *p)
{
    ESP_LOGI(FNAME,"action speed %.1f", p->get());
    new_level_complete |= 0x2;
    if ( new_level_complete == 0x3 ) {
        p->getParent()->getEntry(2)->unlock();
    }
    return 0;
}
static int create_level_action(SetupMenuSelect *p)
{
    if ( p->getSelect() == 1 ) {
        // Confirmed
        FlapLevel nl(new_level_speed->get(), new_level_label, 0);
        FLAP->addLevel(nl);
    }
    p->getParent()->getParent()->setDirty();
    p->setTerminateMenu();
    p->setSelect(0); // reset to cancel
    return 0;
}
static void flap_menu_add_level(SetupMenu *top) // dynamic!
{
    ESP_LOGI(FNAME,"Create new flap level");
    SetupMenuSelect *flab = static_cast<SetupMenuSelect*>(top->getEntry(0));
    if ( ! flab ) {
        top->setDynContent();

        // the label
        SetupMenuChar *flab = new SetupMenuChar( "Label", "Aa0+ ", 3, RST_NONE, select_level_action, "0");
        top->addEntry( flab );

        // the minimum speed
        SetupMenuValFloat *minspeed = new SetupMenuValFloat("Minimum Speed", "", select_speed_action, false, new_level_speed);
        minspeed->setHelp("Minimum speed for the flap level's speed band");
        minspeed->setPrecision(0);
        top->addEntry( minspeed );

        // confirmation
        SetupMenuSelect *confirm = new SetupMenuSelect("Create it", RST_NONE, create_level_action);
        confirm->mkConfirm();
        top->addEntry(confirm);
    }

    // confirmation
    SetupMenuSelect *confirm = static_cast<SetupMenuSelect*>(top->getEntry(2));
    confirm->lock();
    new_level_complete = 0;
    top->highlightFirst();
}

//////////////////////////////
// Level details
static int level_label_action(SetupMenuChar *p)
{
    ESP_LOGI(FNAME,"level_mod_action %s", p->value());
    FLAP->setLabel(p->getParent()->getContId(), p->value());
    FLAP->modLevels();
    p->getParent()->getParent()->setDirty();
    return 0;
}
static int level_speed_action(SetupMenuValFloat *p)
{
    ESP_LOGI(FNAME,"level_speed_action %.1f", p->getNVSVal());
    FLAP->setSpeed(p->getParent()->getContId(), p->getNVSVal());
    FLAP->modLevels();
    p->getParent()->getParent()->setDirty();
    return 0;
}
static int remove_level(SetupMenuSelect *p)
{
    if ( p->getSelect() == 1 ) {
        // level index to remove
        FLAP->removeLevel(p->getParent()->getContId());
        p->getParent()->getParent()->setDirty();
    }

    p->setTerminateMenu();
    p->setSelect(0);
    return 0;
}

static void one_flap_level(SetupMenu *top) // dynamic!
{
    int lid = top->getContId();

    if ( top->getNrChilds() == 0 ) {
        top->setDynContent();
        
        // the label
        int tmp = FLAP->getFL(lid)->label_int;
        SetupMenuChar *flab = new SetupMenuChar( "Label", "Aa0+ ", 3, RST_NONE, level_label_action, (char*)&tmp );
        top->addEntry( flab );

        // the minimum speed
        SetupMenuValFloat *minspeed = new SetupMenuValFloat("Minimum Speed", "", level_speed_action, false, FLAP->getSpeedNVS(lid) );
        minspeed->setHelp("Minimum speed for the flap level's speed band");
        minspeed->setPrecision(0);
        top->addEntry( minspeed );

        // remove level
        SetupMenuSelect *remove = new SetupMenuSelect("Remove level", RST_NONE, remove_level);
        remove->mkConfirm();
        top->addEntry(remove);
    }
}

void flap_levels_menu_create(SetupMenu* top) // dynamic!
{
    SetupMenuValFloat *flgnd = static_cast<SetupMenuValFloat*>(top->getEntry(0));
    if ( ! flgnd ) {
        if ( ! new_level_speed ) {
            new_level_speed = new SetupNG<kmh_t>("foo", 100.f, false, SYNC_NONE, VOLATILE, nullptr, quantity_t::QUANT_HSLEGACY, &polar_speed_limits);
        }
        top->setDynContent();
        flgnd = new SetupMenuValFloat("Takeoff Flap",".", nullptr, false, &flap_takeoff  );
        flgnd->setHelp("Flap position to be set for takeoff");
        top->addEntry( flgnd );
        SetupMenu *addlev = new SetupMenu("Add Level", flap_menu_add_level);
        addlev->setHelp("Get XCVario to know about your gliders flap levels");
        top->addEntry(addlev);
    }

    SetupMenu *addlev = static_cast<SetupMenu*>(top->getEntry(1));
    if ( FLAP->getNrPositions() < Flap::MAX_NR_POS ) {
        addlev->unlock();
        addlev->setBuzzword();
    } else {
        addlev->lock();
        addlev->setBuzzword("max reached");
    }

    MenuEntry *ent = top->getEntry(2);
    while ( ent ) {
        top->delEntry(ent);
        ent = top->getEntry(2);
    }

    // List all configured levels
    for ( int level=0; level < FLAP->getNrPositions(); level++ ) {
        const FlapLevel *lev = FLAP->getFL(level);
        flap_level_menu[level].assign(std::to_string(level));
        flap_level_menu[level] += ". Level ";
        flap_level_menu[level] += lev->label;
        SetupMenu *levmenu = new SetupMenu(flap_level_menu[level].c_str(), one_flap_level, level);
        ESP_LOGI(FNAME,"flap level %d - %s: %f", level, lev->label, lev->nvs_speed);
        flap_level_buzz[level] = std::to_string(static_cast<int>(SpeedUnit->apply(Units::kmh_to_mps(lev->nvs_speed)))) + " " + SpeedUnit->getName();
        levmenu->setBuzzword(flap_level_buzz[level].data());
        top->addEntry(levmenu);
    }
}


void free_flap_menu()
{
    for ( int i=0; i < Flap::MAX_NR_POS; i++) {
        flap_level_menu[i].clear();
        flap_level_menu[i].shrink_to_fit();
        flap_level_buzz[i].clear();
        flap_level_buzz[i].shrink_to_fit();
    }
    if ( new_level_speed ) {
        delete new_level_speed;
        new_level_speed = nullptr;
    }
}
