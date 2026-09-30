/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "WindCalcTask.h"
#include "CircleWind.h"
#include "StraightWind.h"
#include "protocol/NMEA.h"
#include "AverageVario.h"
#include "setup/SetupCommon.h"
#include "setup/SetupNG.h"
#include "logdefnone.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>


QueueHandle_t BackgroundTaskQueue = nullptr;
WindCalcTask *CalcTask = nullptr;

constexpr int CALC_MQ_SIZE = 10;

// background task to calculate wind with lowest prio
static void wind_calc_task(void *arg)
{
    BackgroundTaskQueue = xQueueCreate( CALC_MQ_SIZE, sizeof(CalkTaskJob) );
    ESP_LOGI(FNAME, "Background task queue created %p", BackgroundTaskQueue);
    CalkTaskJob job(CalkTaskJob::CALK_TASK_NONE);
    TickType_t timeout = pdMS_TO_TICKS(3000);

    while (true)
    {
        // sleep until the queue gives us something to do, or we have to handle the time-out
        if ( xQueueReceive(BackgroundTaskQueue, &job, timeout) == pdTRUE ) {
            if (job.raw == 0) {
                break; // termination signal
            }

            ESP_LOGI(FNAME, "job type %x", (unsigned)job.getJobTyp() );

            switch(job.getJobTyp()) {
            case CalkTaskJob::CALK_TASK_EVENT_NEW_GPSPOSE:
                if ( circleWind ) {
                    circleWind->setGpsStatus(true);
                    circleWind->newSample();
                }

                if ( straightWind ) {
                    straightWind->calculateWind();
                }
                break;
            case CalkTaskJob::CALK_TASK_EVENT_NUMSAT:
                if ( circleWind ) {
                    circleWind->newConstellation(job.getDetail());
                }
                break;
            case CalkTaskJob::CALK_TASK_SEND_SENS:
                if ( ToyNmeaPrtcl ) {
                    ToyNmeaPrtcl->sendSens();
                }
                break;
            case CalkTaskJob::CALK_TASK_THERMAL_STATS:
                AverageVario::recalcAvgClimb();
                break;
            case CalkTaskJob::CALK_TASK_TOY_FEED:
            {
                static uint8_t count = 0;
                if ( ToyNmeaPrtcl ) {

                    if (ahrs_rpyl_dataset.get())
                    {
                        ToyNmeaPrtcl->sendXcvRPYL();
                        ToyNmeaPrtcl->sendXcvAPENV1();
                    }
                    if (ahrs_raw_data.get()) {
                        ToyNmeaPrtcl->sendXcvAhrsRaw();
                    }

                    switch (ToyNmeaPrtcl->getProtocolId())
                    {
                    case BORGELT_P:
                        ToyNmeaPrtcl->sendBorgelt();
                        ToyNmeaPrtcl->sendXcvGeneric();
                        break;
                    case OPENVARIO_P:
                        ToyNmeaPrtcl->sendOpenVario();
                        break;
                    case CAMBRIDGE_P:
                        ToyNmeaPrtcl->sendCambridge();
                        break;
                    case XCVARIO_P:
                        ToyNmeaPrtcl->sendStdXCVario();
                        break;
                    case SEEYOU_P:
                        if ( !(count%5) ) ToyNmeaPrtcl->sendLK8EX1();
                        break;
                    default:
                        ESP_LOGE(FNAME, "Protocol %d not supported error", ToyNmeaPrtcl->getProtocolId());
                    }

                    // Some extra NMEA sentences
                    if( !(++count%5) ) {
                        if ( compass_nmea_hdm.get() ) {
                            ToyNmeaPrtcl->sendXCVNmeaHDM();
                        }
                        if ( compass_nmea_hdt.get() ) {
                            ToyNmeaPrtcl->sendXCVNmeaHDT();
                        }
                    }
                }
                break;
            }
            default:
                ESP_LOGE(FNAME, "Unknown job type %d", job.getJobTyp() );
                break;
            }
        }
        else {
            // time-out, no valid gps fix any more
            if ( circleWind ) {
                circleWind->setGpsStatus(false);
                if ( circleWind->isValid() ) {
                    ESP_LOGI(FNAME, "GPS timeout, invalidate synoptic Wind");
                    synoptic_wind.setInvalid();
                };
            }
            // if ( straightWind ) { straightWind->tick(); }
        }
    }
    vQueueDelete(BackgroundTaskQueue);
    BackgroundTaskQueue = nullptr;
    ESP_LOGW(FNAME, "Background task exiting");
    vTaskDelete(NULL);
}


// more or less just a guard to the background task
WindCalcTask::WindCalcTask()
{
    xTaskCreate(wind_calc_task, "background", 4000, NULL, 1, nullptr); // least priority
}


WindCalcTask::~WindCalcTask()
{
    xQueueSend( BackgroundTaskQueue, nullptr, 0 ); // stop task
}

void WindCalcTask::createWindResources()
{
    if ( SetupCommon::isMaster() ) {
        // straight wind
        bool needCircleWind = wind_enable.get() & WA_CIRCLING;
        if ( wind_enable.get() & WA_STRAIGHT) {
            needCircleWind = true;
            if ( ! straightWind ) {
                straightWind = new StraightWind();
                straightWind->begin();
#ifdef Wind_Test
                straightWind->test();
#endif
            }
        }
        else if ( ! (wind_enable.get() & WA_STRAIGHT) && straightWind ) {
            StraightWind *tmp = straightWind;
            straightWind = nullptr;
            delete tmp;
        }

        // circle wind
        if ( needCircleWind && ! circleWind ) {
            circleWind = new CircleWind();
        }
        else if ( ! needCircleWind && circleWind ) {
            CircleWind *tmp = circleWind;
            circleWind = nullptr;
            delete tmp;
        }
    }

    // always need the background task
    if ( ! CalcTask ) {
        CalcTask = new WindCalcTask();
    }

}
