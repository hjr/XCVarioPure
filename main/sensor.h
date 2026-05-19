#pragma once

#include "math/vector_3d_fwd.h"
#include "math/Units.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <esp_timer.h>
#include <string>

// Display 4 Wire SPI and Display CS
#define RESET_Display  GPIO_NUM_5       // Reset pin for Display
#define CS_Display     GPIO_NUM_13      // CS pin 13 is for Display
#define SPI_SCLK       GPIO_NUM_14      // SPI Clock pin 14
#define SPI_DC         GPIO_NUM_15      // SPI Data/Command pin 15
#define SPI_MOSI       GPIO_NUM_27      // SPI SDO Master Out Slave In pin
#define SPI_MISO       GPIO_NUM_32      // SPI SDI Master In Slave Out

union global_flags {
    struct {
        uint16_t inSetup : 1;
        uint16_t gear_warn_external : 1;
        uint16_t schedule_reboot : 1;
        uint16_t first_pure_run : 1;
        uint16_t inSimulationMode : 1;
        uint16_t isPro : 1;
        uint16_t expert : 1;
    };
    uint16_t raw;
};

class CANbus;
class SerialLine;
class AnalogInput;
class WatchDog_C;

extern global_flags gflags;
extern CANbus *CAN;
extern SerialLine *S1,*S2;

extern WatchDog_C *uiMonitor;
extern AnalogInput *BatVoltage;

extern std::string logged_tests;

extern AnalogInput *AnalogInWk;

extern meter_t alt_external;

extern SemaphoreHandle_t spiMutex;

extern vector_f gravity_vector;

inline void delay(uint32_t ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

void startClientSync();
