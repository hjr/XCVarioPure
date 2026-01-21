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
#include "SetupRoot.h"
#include "MessageBox.h"
#include "BootUpScreen.h"
#include "FlarmScreen.h"
#include "HorizonPage.h"

#include "setup/SetupMenuValFloat.h"
#include "setup/SetupMenuDisplay.h"
#include "setup/SetupMenu.h"
#include "setup/SubMenuGlider.h"
#include "setup/SetupNG.h"
#include "setup/CruiseMode.h"
#include "ESPRotary.h"
#include "sensor/imu/KalmanMPU6050.h"
#include "ESPAudio.h"
#include "Flarm.h"
#include "sensor.h"
#include "protocol/WatchDog.h"
#include "logdefnone.h"


// The context to serialize all display access.
QueueHandle_t uiEventQueue = nullptr;


void UiEventLoop(void *arg)
{
    ESPRotary &knob = *static_cast<ESPRotary *>(arg);
    int16_t stall_warning_active = 0;
    int16_t gload_warning_active = 0;
    bool gear_warning_active = false;

    xQueueReset(uiEventQueue);

    while (1)
    {
        // handle button events in this context
        int eparam;
        if (xQueueReceive(uiEventQueue, &eparam, pdMS_TO_TICKS(20)) == pdTRUE)
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
                                Display->drawDisplay(s2f_delta);
                                break;
                            case SCREEN_GMETER:
                                Display->drawLoadDisplay( IMU::getGliderAccelZ() );
                                break;
                            case SCREEN_HORIZON:
                                HorizonPage::HORIZON()->draw( IMU::getAHRSQuaternion() );
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
                        MenuRoot->push(FlarmScreen::create());
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
                } else if (detail == ScreenEvent::VOLT_ADJUST) {
                    MenuRoot->begin(SetupMenu::createVoltmeterAdjustMenu());
                } else if (detail == ScreenEvent::POLAR_CONFIG) {
                    MenuRoot->begin(createGliderSelectMenu());
                }
            }
            else if (event.isModeEvent()) {
                if (detail == ModeEvent::MODE_TOGGLE) {
                    VCMode.setCMode(!VCMode.getCMode());
                }
                else if (detail == ModeEvent::MODE_VARIO || detail == ModeEvent::MODE_S2F) {
                    VCMode.setCMode(detail == ModeEvent::MODE_S2F);
                }
            }
            else {
                // ESP_LOGI(FNAME, "Unknown event %x", event);
            }
        }

        if ( ! BootUpScreen::isActive() )
        {
            // Stall Warning fixme no need for this to be here, could be in sensor loop, no display context needed
            if (stall_warning.get() && screen_gmeter.get() != SCREEN_PRIMARY && airborne.get()) {
                // In aerobatics stall warning is contra productive, we concentrate on G-Load Display if permanent enabled
                float acceleration = IMU::getGliderAccelZ();
                if (acceleration < 0.3) {
                    acceleration = 0.3; // limit acceleration effect to minimum 0.3g
                }
                // accelerated and ballast(ed) stall speed
                float acc_stall = Speed2Fly.getStallSpeed() * sqrt(acceleration);
                if (ias.get() < acc_stall && ias.get() > acc_stall * 0.7 && airborne.get()) {
                    if (!stall_warning_active) {
                        MBOX->pushMessage(4, "! STALL !", 20); // 20 sec
                    }
                    if (stall_warning_active % 50 == 0) {
                        AUDIO->startSound(AUDIO_ALARM_STALL);
                    }
                    stall_warning_active++;
                }
                else if ( stall_warning_active ) {
                    stall_warning_active = 0;
                    MBOX->popMessage();
                }
            }
            // Gear Warning fixme no need for this to be here, could be in sensor loop, no display context needed
            if (gear_warning.get()) {
                int gw = 0;
                if (gear_warning.get() == GW_EXTERNAL) {
                    gw = gflags.gear_warn_external;
                } else {
                    gw = gpio_get_level(SetupMenu::getGearWarningIO());
                    if (gear_warning.get() == GW_FLAP_SENSOR_INV || gear_warning.get() == GW_S2_RS232_RX_INV) {
                        gw = !gw;
                    }
                }
                if (gw) {
                    if (!gear_warning_active && !stall_warning_active) {
                        AUDIO->startSound(AUDIO_ALARM_GEAR);
                        MBOX->pushMessage(4, "! GEAR !", 20);
                        gear_warning_active = true;
                    }
                } else {
                    gear_warning_active = false;
                }
            }

            // G-Load alarm when limits reached
            if (screen_gmeter.get() != SCREEN_OFF) {
                if (IMU::getGliderAccelZ() > gload_pos_limit.get() || IMU::getGliderAccelZ() < gload_neg_limit.get()) {
                    if (gload_warning_active % 10 == 0) {
                        AUDIO->startSound(AUDIO_ALARM_GLOAD);
                        gload_warning_active++;
                    }
                }
                else if (gload_warning_active) {
                    gload_warning_active = 0;
                }
            }

            // fixme no need for this to be here, could be in sensor loop, no display context needed
            if (theCenteraid) {
				theCenteraid->tick();
			}
		}
		if( uxTaskGetStackHighWaterMark(NULL) < 512  ) {
			ESP_LOGW(FNAME,"Warning UiEventLoop stack low: %d bytes", uxTaskGetStackHighWaterMark(NULL) );
		}
    }
}
