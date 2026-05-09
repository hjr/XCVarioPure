#include "ImuStatus.h"

#include "screen/DrawDisplay.h"
#include "AdaptUGC.h"
#include "math/Units.h"
#include "sensor.h"
#include "logdefnone.h"
#include <cmath>
#include <algorithm>
#include <cmath>
#include "sensor/imu/AccMPU6050.h"
#include "sensor/imu/GyroMPU6050.h"
#include "math/Floats.h"

//
// This test checks for leaks in the pressure system by monitoring the stability of the 
// static and total pressure readings over time.
// It collects readings every 5 seconds for up to 1 minute and compares them against
// the initial baseline.
// If the readings deviate beyond defined thresholds, it indicates a potential leak.
//

void ImuStatus::display(int mode) {
    clear(true);
    menuPrintLn(_title.c_str(), 0);

    // we show the status if Accelerator, Gyro, Heating
    char buf[64];
    clear(true); // filled font true
    while(1){
    	int idx=1;
    	menuPrintLn("Acccelerator [g]:", idx++ );
    	if( Rotary->readSwitch(200) ) { break; } // exit by press
    	if( accSensor->getMpu().hasHeatCtlr() ){
    		const char * tstat;
    		switch( accSensor->getTempStatus() ){
    		case temp_status_t::MPU_T_LOCKED: tstat = "Locked"; break;
    		case temp_status_t::MPU_T_LOW:  tstat = "Too Low"; break;
    		case temp_status_t::MPU_T_HIGH: tstat = "Too High"; break;
    		case temp_status_t::MPU_T_UNKNOWN:
    		default:                        tstat = "Unknown"; break;
    		}
    		snprintf( buf, sizeof(buf),"  Heating %.1f°: %s    ", accSensor->getMpu().getTemperature(), tstat );
    	}else{
    		snprintf( buf, sizeof(buf),"No Heating");
    	}
    	menuPrintLn(buf,idx++);
    	const char *rest = accSensor->isResting() ? "  Resting" : " Moving  ";
    	menuPrintLn(rest,idx++);
    	vector_f attv = accSensor->getAttVector();
    	snprintf( buf, sizeof(buf),"  X:%1.3f Y:%1.3f Z:%1.3f   ", attv.x, attv.y, attv.z );
    	menuPrintLn(buf,idx++);
    	snprintf( buf, sizeof(buf),"  Pitch %.1f° Roll: %.1f°   ",  accSensor->getPitchDeg(), accSensor->getRollDeg() );
    	menuPrintLn(buf,idx++);
    	menuPrintLn("Gyro [°/s]:", idx++ );
    	rest = gyroSensor->isResting() ? "  Resting" : " Moving  ";
    	menuPrintLn(rest,idx++);
    	const vector_f& gyro = gyroSensor->getRef();
    	snprintf( buf, sizeof(buf),"  X:%1.2f Y:%1.2f Z:%1.2f     ", rad2deg(gyro.x), rad2deg(gyro.y), rad2deg(gyro.z) );
    	menuPrintLn(buf,idx++);
    }
}
