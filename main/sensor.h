#pragma once

#include "math/vector_3d_fwd.h"
#include "S2F.h"

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

#define GYRO_FS (mpud::GYRO_FS_250DPS)

union global_flags {
	struct {
		uint16_t inSetup :1;
		uint16_t haveIMU :1;
		uint16_t ahrsKeyValid :1;
		uint16_t standard_setting :1;
		uint16_t validTemperature :1 ;
		uint16_t mpu_pwm_initalized :1;
		uint16_t gear_warn_external :1;
		uint16_t schedule_reboot :1;
		uint16_t first_devices_run :1;
		uint16_t flaps_nvs_defined :1;
	};
	uint16_t raw;
};

class CANbus;
class SerialLine;
class Clock;
class ESPRotary;
class AnalogInput;
class PressureSensor;
class SetupRoot;
class WatchDog_C;
class BMPVario;
class AirspeedSensor;
namespace mpud {
    class MPU;
}
namespace i2cbus {
    class I2C;
}

extern S2F Speed2Fly;
extern BMPVario bmpVario;

extern global_flags gflags;
extern CANbus *CAN;
extern SerialLine *S1,*S2;
extern Clock *MY_CLOCK;
extern AirspeedSensor *asSensor;
extern PressureSensor *baroSensor;
extern SetupRoot *MenuRoot;
extern WatchDog_C *uiMonitor;
extern AnalogInput *BatVoltage;

extern std::string logged_tests;

extern float getTAS();

extern i2cbus::I2C& i2c;

extern AnalogInput *AnalogInWk;

extern float aTE;
extern float tas;
extern float aTES2F;
extern float as2f;
extern float s2f_delta;
extern float polar_sink;
extern float alt_external;
extern float wksensor;

extern int MyGliderPolarIndex;
extern float meanClimb;
extern float baroP;    // Static pressure
extern float dynamicP; // Pitot pressure

extern long unsigned int _gps_millis;

extern ESPRotary *Rotary;

extern SemaphoreHandle_t spiMutex;

extern vector_f gravity_vector;

#define NEED_VOLTAGE_ADJUST (abs(factory_volt_adjust.get() - 0.00815) < 0.00001)

extern float mpu_target_temp;

extern mpud::MPU MPU;

// Arduino.h remains
inline unsigned long millis() // fixme
{
    return (unsigned long) (esp_timer_get_time() / 1000ULL);
}

inline void delay(uint32_t ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

int sign(int num);

void startClientSync();
