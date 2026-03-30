/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "SetupRoot.h"

#include "DrawDisplay.h"
#include "HorizonPage.h"
#include "UiEvents.h"
#include "setup/SubMenuAudio.h"
#include "setup/SubMenuDevices.h"
#include "setup/SubMenuFlap.h"
#include "setup/SetupNG.h"
#include "IpsDisplay.h"
#include "driver/audio/ESPAudio.h"
#include "screen/element/ArrowIndicator.h"

#include "sensor.h"
#include "logdefnone.h"

// bit field of all configured screens
// set to zero for boot-up
static uint32_t all_screens;

SetupRoot::SetupRoot(IpsDisplay *display) :
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

SetupRoot::~SetupRoot()
{
    detach();
}

void SetupRoot::display(int mode)
{
    ESP_LOGI(FNAME,"SetupRoot display mode %d", mode);
}

// time-out on UI interaction while airborne and walking through setup
void SetupRoot::barked()
{
    int exitMenu = ButtonEvent(ButtonEvent::ESCAPE).raw;
    xQueueSend(uiEventQueue, &exitMenu, 0);
}

void SetupRoot::initScreens()
{
    all_screens = 0;
    if ( screen_gmeter.get() ) {
        all_screens |= SCREEN_GMETER;
    }
    // horizon only if AHRS license is valid
    if ( screen_horizon.get() && gflags.ahrsKeyValid ) {
        all_screens |= SCREEN_HORIZON;
    }
    all_screens |= SCREEN_VARIO; // always
}

void SetupRoot::begin(MenuEntry *setup)
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

void SetupRoot::pushTop(MenuEntry *menu)
{
    ESP_LOGI(FNAME,"Push Menu on top %s", menu->getTitle());
    gflags.inSetup = true;

    MenuEntry *sel = getSelected();
    if ( sel->isLeaf() ) {
        ESP_LOGW(FNAME,"Cannot push menu on leaf");
        sel->exit();
    }
    SetupMenu *parent = static_cast<SetupMenu*>(getSelected());
    menu->regParent(parent);
    ESP_LOGW(FNAME,"Push flarm screen hooked");
    menu->enter();
}

void SetupRoot::exit(int levels)
{
    ESP_LOGI(FNAME,"End Setup Menu");
    if ( uiMonitor ) {
        uiMonitor->stop();
        uiMonitor = nullptr;
    }
    free_connected_devices_menu();
    free_audio_menu();
    free_flap_menu();

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
        BasePage::_DIRTY = true;
    }
}

void SetupRoot::rot(int count)
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

void SetupRoot::press()
{
    ESP_LOGI(FNAME,"root press active_srceen %d (0x%x)", active_screen, (unsigned)all_screens);

    int current_screen = active_screen;

    // cycle through screens, incl. setup
    if (!gflags.inSetup)
    {
        while (active_screen < SCREEN_LIST_END)
        {
            active_screen <<= 1;
            if (all_screens & active_screen)
            {
                ESP_LOGI(FNAME, "New active_screen: %x", active_screen);
                break;
            }
        }
        if ( active_screen >= SCREEN_LIST_END ) {
            ESP_LOGI(FNAME, "select vario screen");
            active_screen = SCREEN_VARIO;
        }
    }
    if ( current_screen != active_screen )
    {
        ESP_LOGI(FNAME, "Switching screen from %d to %d", current_screen, active_screen);
        BasePage::_DIRTY = true;
    }
    // only if menu long press is not set, jump into the setup befor going back to the vario screen
    if (!menu_long_press.get() && active_screen == SCREEN_VARIO && !gflags.inSetup)
    {
        begin();
    }
}

void SetupRoot::longPress()
{
    // enter setup from any screen
    if (!gflags.inSetup) {
        begin();
    }
}
