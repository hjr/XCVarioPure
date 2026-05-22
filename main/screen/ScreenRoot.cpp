/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "ScreenRoot.h"

#include "DrawDisplay.h"
#include "HorizonPage.h"
#include "UiEvents.h"
#include "setup/SubMenuAudio.h"
#include "setup/SubMenuDevices.h"
#include "setup/SubMenuFlap.h"
#include "setup/SubMenuImu.h"
#include "setup/SetupNG.h"
#include "IpsDisplay.h"
#include "driver/audio/ESPAudio.h"

#include "sensor.h"
#include "logdefnone.h"

// the global screen root
ScreenRoot *MenuRoot = nullptr;

// bit field of all configured screens
// set to zero for boot-up
static uint32_t all_screens;

ScreenRoot::ScreenRoot(IpsDisplay *display) :
    SetupMenu("Setup Root", nullptr),
    _ui_mon_wd(this)
{
    // set statics once
    _display = display;
    MenuEntry::grabDisplaySize();
    current_menu = this;
    ESP_LOGI(FNAME,"Init root menu");
    attach();
    if ( rot_default.get() == 0) {
        // volume needs rotary dynamics
        setRotDynamic(2.3f);
    }
}

ScreenRoot::~ScreenRoot()
{
    detach();
}

void ScreenRoot::display(int mode)
{
    ESP_LOGI(FNAME,"ScreenRoot display mode %d", mode);
}

// time-out on UI interaction while airborne and walking through setup
void ScreenRoot::barked()
{
    int exitMenu = ButtonEvent(ButtonEvent::ESCAPE).raw;
    xQueueSend(uiEventQueue, &exitMenu, 0);
}

void ScreenRoot::initScreens()
{
    all_screens = SCREEN_VARIO; // always
    if (screen_horizon.get()) {
        all_screens |= SCREEN_HORIZON;
    }
    if (screen_gmeter.get()) {
        all_screens |= SCREEN_GMETER;
        if (screen_gmeter.get() == SCREEN_PRIMARY) {
            all_screens = SCREEN_GMETER; // override and only show the g meter screen
            active_screen = SCREEN_GMETER;
        }
    }
    if (!menu_long_press.get()) {
        all_screens |= SCREEN_SETUP_MENU; // treat setup as a screen in this case
    }
}

void ScreenRoot::begin(MenuEntry *setup)
{
    ESP_LOGI(FNAME,"SetupMenu %s", _title.c_str());
    // root will always own only one child
    if ( ! _childs.empty() ) {
        ESP_LOGW(FNAME,"Found root menu not empty");
    }
    current_menu = this;

    // Given setup might be for QNH, or voltage adjustment
    if ( setup ) {
        addEntry(setup);
    } else {
        addEntry(SetupMenu::createTopSetup());
        if ( airborne.get() ) {
            // exit setup after timeout w/o user activity
            _ui_mon_wd.start(16000); // 16 seconds
            uiMonitor = &_ui_mon_wd;
        }
    }

    // Todo setup able to recurse
    if ( ! gflags.inSetup ) {
        gflags.inSetup = true;
        _childs.front()->enter();
    }
    SetupMenu::initGearWarning(); // Huh fixme
}

void ScreenRoot::pushTop(MenuEntry *menu)
{
    ESP_LOGI(FNAME,"Push Menu on top %s", menu->getTitle());
    gflags.inSetup = true;

    MenuEntry *sel = getSelected(); // may return a nullptr
    if ( sel && sel->isLeaf() ) {
        ESP_LOGW(FNAME,"Cannot push menu on leaf");
        sel->exit();
    }
    SetupMenu *parent = current_menu ? current_menu : this;
    menu->regParent(parent);
    ESP_LOGW(FNAME,"Push flarm screen hooked");
    menu->enter();
}

void ScreenRoot::exit(int levels)
{
    ESP_LOGI(FNAME,"End Setup Menu");
    if ( uiMonitor ) {
        uiMonitor->stop();
        uiMonitor = nullptr;
    }
    free_connected_devices_menu();
    free_audio_menu();
    free_flap_menu();
    free_imu_menu();

    // current menu status
    current = nullptr; // no current item any more
    // current_menu = this; // root is current menu again, set implicitely from the child exit() calls

    if (_restart) {
        reBoot();
    }
    SetupCommon::commitDirty(); // commit all dirty setup items

    if ( !_childs.empty() ) {
        delete _childs.front(); // the exited setup tree
        _childs.erase(_childs.begin());
    }

    if ( !_childs.empty() ) {
        ESP_LOGI(FNAME,"More menus to run");
        _childs.front()->enter(); // more menues to go back to
    }
    else {
        screens_init = INIT_DISPLAY_NULL; // set screen dirty
        gflags.inSetup = false;
        if ( rot_default.get() == 0) {
            setRotDynamic(2.5f); // only for volume control
        }
        initScreens(); // re-evaluate available screens
        if (active_screen == SCREEN_SETUP_MENU) {
            press(); // jump to next screen, e.g. to get out of setup menu if it is treated as a screen
        }
        else {
            screens_init = INIT_DISPLAY_NULL; // set screen dirty to force redraw
            BasePage::_DIRTY = true;
        }
    }
}

void ScreenRoot::rot(int count)
{
    // ESP_LOGI(FNAME,"root: rot");
    if (rot_default.get() == 1) {
        // MC Value
        MC.setCheckRange(MC.get() + 0.1 * count);
    }
    else {
        // Volume
        AUDIO->setVolume(audio_volume.get() + count);
    }
}

void ScreenRoot::press()
{
    ESP_LOGI(FNAME,"root press active_srceen %d (0x%x)", active_screen, (unsigned)all_screens);

    int current_screen = active_screen;

    // cycle through screens, incl. setup
    ESP_LOGI(FNAME, "Cycle screen from %d (%d)", current_screen, gflags.inSetup);
    do {
        if (active_screen < SCREEN_LIST_END) {
            active_screen <<= 1; // next screen
        }
        else {
            active_screen = SCREEN_VARIO; // back to first screen
        }
    } while ( ! (all_screens & active_screen) );
    ESP_LOGI(FNAME, "New active_screen: %x", active_screen);

    if ( current_screen != active_screen )
    {
        ESP_LOGI(FNAME, "Switching screen from %d to %d", current_screen, active_screen);
        screens_init = INIT_DISPLAY_NULL; // set screen dirty to force redraw
        BasePage::_DIRTY = true;
    }

    // only if menu long press is not set, jump into the setup befor going back to the first screen
    if (active_screen == SCREEN_SETUP_MENU) {
        begin();
    }
}

void ScreenRoot::longPress()
{
    // enter setup from any screen
    if (!gflags.inSetup) {
        begin();
    }
}
