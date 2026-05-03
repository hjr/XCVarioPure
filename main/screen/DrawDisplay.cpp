/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "DrawDisplay.h"

#include "CenterAid.h"
#include "IpsDisplay.h"
#include "UiEvents.h"
#include "ScreenRoot.h"
#include "MessageBox.h"
#include "BootUpScreen.h"
#include "FlarmScreen.h"
#include "HorizonPage.h"

#include "setup/SetupMenuValFloat.h"
#include "setup/SetupMenuSelect.h"
#include "setup/SetupMenuDisplay.h"
#include "setup/SetupMenu.h"
#include "setup/SubMenuGlider.h"
#include "setup/SetupNG.h"
#include "setup/CruiseMode.h"
#include "driver/gpio/ESPRotary.h"
#include "sensor/imu/AccMPU6050.h"
#include "driver/audio/ESPAudio.h"
#include "Flarm.h"
#include "S2F.h"
#include "sensor.h"
#include "driver/time/WatchDog.h"
#include "logdefnone.h"


// The context to serialize all display access.
QueueHandle_t uiEventQueue = nullptr;
bool ui_update_done = false;


void UiEventLoop(void *arg)
{
    ESPRotary &knob = *static_cast<ESPRotary *>(arg);

    xQueueReset(uiEventQueue);

    while (1)
    {
        // handle button events in this context
        ui_update_done = true; // accept an UI screen event again (flood protection)
        int eparam;
        if (xQueueReceive(uiEventQueue, &eparam, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            UiEvent event(eparam);
            uint8_t detail = event.getUDetail();
            ESP_LOGI(FNAME, "Event (%d) param %x", uxQueueMessagesWaiting(uiEventQueue), eparam);
            if (event.isButtonEvent())
            {
                // ESP_LOGI(FNAME, "Button event %x", detail);
                if (detail == ButtonEvent::SHORT_PRESS) {
                    knob.sendPress();
                }
                else if (detail == ButtonEvent::LONG_PRESS) {
                    knob.sendLongPress();
                }
                else if (detail == ButtonEvent::BUTTON_RELEASED) {
                    knob.sendRelease();
                }
                else if (detail == ButtonEvent::ESCAPE) {
                    knob.sendEscape();
                }
                if (uiMonitor) {
                    uiMonitor->pet(); // knob ui interaction happened
                }
            }
            else if (event.isRotaryEvent()) {
                // ESP_LOGI(FNAME, "Rotation step %d", event.getSDetail());
                knob.sendRot(event.getSDetail());
                if (uiMonitor) {
                    uiMonitor->pet(); // knob ui interaction happened
                }
            }
            else if (event.isScreenEvent()) {
                // ESP_LOGI(FNAME, "Screen event %d", detail);
                if (detail == ScreenEvent::MAIN_SCREEN) {
                    if (!gflags.inSetup) {
                        switch (MenuRoot->getActiveScreen()) {
                            case SCREEN_VARIO:
                                Display->drawDisplay();
                                break;
                            case SCREEN_GMETER:
                                assert(accSensor);
                                Display->drawLoadDisplay( accSensor->getGload() );
                                break;
                            case SCREEN_HORIZON:
                                HorizonPage::HORIZON()->draw( accSensor->getAHRSQuaternion() );
                                break;
                        }
                    }
                } else if (detail == ScreenEvent::MSG_BOX) {
                    if (MBOX->draw()) { // time triggered mbox update
                        // mbox finish, time to refresh the bottom line of the screen
                        Display->setBottomDirty();
                    }
                } else if ( detail == ScreenEvent::FLARM_ALARM ) {
                    if ( ! FLARMSCREEN ) {
                        ESP_LOGI(FNAME,"Flarm::alarmLevel: %d, flarm_warning.get() %d", Flarm::alarmLevel(), flarm_warning.get() );
                        MenuRoot->pushTop(FlarmScreen::create());
                    } else {
                        FLARMSCREEN->display(1);
                    }
                    if (uiMonitor) {
                        // classify flarm as ui interaction, because flarm screen could have been pushed on top of the UI stack
                        uiMonitor->pet();
                    }
                } else if ( detail == ScreenEvent::FLARM_ALARM_TIMEOUT ) {
                    if ( FLARMSCREEN ) {
                        FLARMSCREEN->remove();
                    }
                } else if (detail == ScreenEvent::BOOT_SCREEN) {
                    BootUpScreen::draw(); // time triggered boot screen update
                } else if (detail == ScreenEvent::QNH_ADJUST) {
                    MenuRoot->begin(SetupMenu::createQNHMenu());
                } else if (detail == ScreenEvent::BALLAST_CONFIRM) {
                    MenuRoot->begin(SetupMenu::createBallastMenu());
                } else if (detail == ScreenEvent::POLAR_CONFIG) {
                    MenuRoot->begin(createGliderSelectMenu());
                } else if (detail == ScreenEvent::FACTORY_CONFIG) {
                    MenuRoot->begin(SetupMenu::createFactorySetup());
                }
            }
            else if (event.isModeEvent()) {
                if (detail == ModeEvent::MODE_TOGGLE) {
                    CRMOD.setCMode(!CRMOD.getCMode());
                }
                else if (detail == ModeEvent::MODE_VARIO || detail == ModeEvent::MODE_S2F) {
                    CRMOD.setCMode(detail == ModeEvent::MODE_S2F);
                }
            }
            else {
                // ESP_LOGI(FNAME, "Unknown event %x", event);
            }
        }

        if (uxTaskGetStackHighWaterMark(NULL) < 512) {
            ESP_LOGW(FNAME, "Warning UiEventLoop stack low: %d bytes", uxTaskGetStackHighWaterMark(NULL));
        }
    }
}
