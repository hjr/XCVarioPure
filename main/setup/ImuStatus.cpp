#include "ImuStatus.h"

#include "sensor/imu/AccMPU6050.h"
#include "sensor/imu/GyroMPU6050.h"

//
//
//

void ImuStatus::press(){
	go_on = false;
}

void ImuStatus::display(int mode) {
    clear();
    menuPrintLn(_title.c_str(), 0);

    // we show the status if Accelerator, Gyro, Heating
    char buf[64];
    clear(); // filled font true
    while(go_on){
    	int idx=1;
    	menuPrintLn("Acccelerator [g]:", idx++ );
    	vTaskDelay(pdMS_TO_TICKS(200));
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
