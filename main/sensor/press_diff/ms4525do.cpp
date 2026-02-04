#include "ms4525do.h"

#include "setup/SetupNG.h"


#define MAX_AUTO_CORRECTED_OFFSET 50

// ABPMRR and MS4525DO share the same I2C identity and are nearly identical, but delivering mirrored results,
// more or lees given by their tube connector setup. (ABPMRR negativ on positive pressure diff)
// This is here taken into account by a flag that can be set in MS4525DO to behave like an ABPMRR.

constexpr char I2C_ADDRESS_MS4525DO   = 0x28;    /**< 7-bit address =0x28. 8-bit is 0x50. Depends on the order code (this is for code "I") */
 
// Register address
// #define ADDR_READ_MR            0x00    /* write to this address to start conversion */
 
// MS4525D sensor full scale range and units
// const int16_t MS4525FullScaleRange = 1;  // 1 psi
 
// MS4525D Sensor type (A or B) comment out the wrong type assignments
// Type A (10% to 90%)
const int16_t MS4525MinScaleCounts = 1638;
const int16_t MS4525FullScaleCounts = 14746;
  
const int16_t MS4525Span = MS4525FullScaleCounts - MS4525MinScaleCounts;
 
// MS4525D sensor differential pressure
// const int16_t MS4525ZeroCounts = (MS4525MinScaleCounts + MS4525FullScaleCounts) / 2;

MS4525DO::MS4525DO(bool is_abpmrr) : AsSensI2c(&i2c1, I2C_ADDRESS_MS4525DO), _is_abpmrr(is_abpmrr)
{
    changeConfig();
}

const char *MS4525DO::name() const {
    if ( PS_NONE == airspeed_sensor.get() ) {
        return "undefined";
    }
    if (_is_abpmrr) {
        return "ABPMRR";
    }
    else {
        return "MS4525DO";
    }
}

void MS4525DO::changeConfig() {
    _multiplier = (2.f * 6894.76 / MS4525Span) * ((100.0 + speedcal.get()) / 100.0);
    _multiplier *= (!_is_abpmrr) ? -1.0f : 1.0f;
}


void MS4525DO::setSubType(bool positive)
{
    // This will set the correct sensor type for the next XCV boot and
    // provide the proper zero offset tollerances
    _is_abpmrr = positive;
    changeConfig();
    if ( _is_abpmrr ) {
        airspeed_sensor.set( AirspeedSensor::PS_ABPMRR );
    }
    else {
        airspeed_sensor.set( AirspeedSensor::PS_TE4525 );
    }
}

bool MS4525DO::offsetPlausible(int32_t offset)
{
    if ( _is_abpmrr ) {
        constexpr int lower_val = 8192 - 200;
        constexpr int upper_val = 8192 + 200;
        return (offset > lower_val) && (offset < upper_val);
    }
    else {
        constexpr int lower_val = 7700;
        constexpr int upper_val = 8300;
        return (offset > lower_val) && (offset < upper_val);
    }
}

int MS4525DO::getMaxACOffset()
{
    return MAX_AUTO_CORRECTED_OFFSET;
}

float MS4525DO::getTemperature(void)
{
    float temp = t_dat;
    temp = temp / 10;          // now in deg F
    temp = (temp - 32) / 1.8f; // now in deg C
    return temp;
}
