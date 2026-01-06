
#include "sensor.h"

#include "Cipher.h"
#include "sensor/pressure/bme280_spi.h"
#include "sensor/pressure/spl06_007.h"
#include "ESP32NVS.h"
#include "sensor/press_diff/AirspeedSensor.h"
#include "sensor/SensorMgr.h"
#include "BMPVario.h"
#include "comm/BTspp.h"
#include "comm/BTnus.h"
#include "comm/OneWireBus.h"
#include "setup/SetupNG.h"
#include "setup/CruiseMode.h"
#include "ESPAudio.h"
#include "ESPRotary.h"
#include "AnalogInput.h"
#include "Atmosphere.h"
#include "IpsDisplay.h"
#include "S2F.h"
#include "Version.h"
#include "glider/Polars.h"
#include "Flarm.h"
#include "protocol/Clock.h"
#include "protocol/MagSensBin.h"
#include "protocol/NMEA.h"
#include "protocol/WatchDog.h"
#include "protocol/nmea/XCVSyncMsg.h"
#include "protocol/CANPeerCaps.h"
#include "screen/SetupRoot.h"
#include "screen/BootUpScreen.h"
#include "screen/MessageBox.h"
#include "screen/DrawDisplay.h"
#include "screen/UiEvents.h"

// #include "math/Quaternion.h"
// #include "math/Floats.h"
// #include "wmm/geomag.h"
#include "OTA.h"
#include "S2fSwitch.h"
#include "AverageVario.h"

#include "MPU.hpp"        // main file, provides the class itself
#include "mpu/math.hpp"   // math helper for dealing with MPU data
#include "mpu/types.hpp"  // MPU data types and definitions
#include "I2Cbus.hpp"
#include "KalmanMPU6050.h"
#include "LeakTest.h"
#include "Units.h"
#include "Flap.h"
#include "wind/WindCalcTask.h"
#include "comm/SerialLine.h"
#include "comm/OneWireBus.h"
#include "comm/CanBus.h"
#include "comm/DeviceMgr.h"
// #include "protocol/TestQuery.h"
#include "AdaptUGC.h"
#include "logdef.h"
#include "comm/Messages.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <coredump_to_server.h>
#include <driver/spi_master.h>
#include <esp_task_wdt.h>
#include <esp_task.h>
#include <soc/sens_reg.h> // needed for adc pin reset
#include <esp_sleep.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <esp_flash.h>
#include <esp_chip_info.h>
#include <driver/gpio.h>

#include <string>
#include <cstdio>
#include <cstring>






AirspeedSensor *asSensor = nullptr;
PressureSensor *baroSensor = nullptr;
PressureSensor *teSensor = nullptr;

SemaphoreHandle_t spiMutex=NULL;

S2F Speed2Fly;
int MyGliderPolarIndex; // Todo make private in S2F?

AnalogInput *BatVoltage = nullptr;

AdaptUGC *MYUCG = 0;  // ( SPI_DC, CS_Display, RESET_Display );
SetupRoot  *MenuRoot = nullptr;
WatchDog_C *uiMonitor = nullptr;

// Gyro and acceleration sensor
MPU_t MPU;         // create an object

// Magnetic sensor / compass
SerialLine *S1 = NULL;
SerialLine *S2 = NULL;
Clock *MY_CLOCK = nullptr;

// boot log
std::string logged_tests;

// global variables
float baroP=0; // barometric pressure
static float xcvTemp=15.0;
static unsigned long _millis = 0;
unsigned long _gps_millis = 0;


float dynamicP; // Pitot

// global color variables for adaptable display variant
uint8_t g_col_background; // black
uint8_t g_col_highlight;
uint8_t g_col_header_r;
uint8_t g_col_header_g;
uint8_t g_col_header_b;
uint8_t g_col_header_light_r;
uint8_t g_col_header_light_g;
uint8_t g_col_header_light_b;
uint8_t gyro_flash_savings=0;

// boot with flasg "inSetup":=true and release the screen for other purpouse by setting it false.
global_flags gflags = {};

int   ccp = 60;
float tas = 0;
float aTE = 0;
float alt_external;
float netto = 0;
float as2f = 0;
float s2f_delta = 0;
float polar_sink = 0;

float mpu_target_temp=45.0;

const constexpr char passed_text[] = "PASSED\n";
const constexpr char failed_text[] = "FAILED\n";
const constexpr char notfound_text[] = "NOT FOUND\n";

int IRAM_ATTR sign(int num) {
    return (num > 0) - (num < 0);
}

float getTAS() { return tas; }


static void grabMPU()
{
	// Automatically trac the gyro bias
	static int32_t cur_gyro_bias[3];
	const int MAXDRIFT         = 2;    // °/s maximum drift that is automatically compensated on ground
	const int NUM_GYRO_SAMPLES = 3000; // 10 per second -> 5 minutes, so T has been settled after power on
	static uint16_t num_gyro_samples = 0;

	// Read the IMU registers and check the output
	if( IMU::MPU6050Read() == ESP_OK )
	{
		// Do the gyro auto bias
		vector_f gyroDPS = IMU::getGliderGyro();
		// ESP_LOGI(FNAME,"Gyro:\t%4f\t%4f\t%4f", gyroDPS.a, gyroDPS.b, gyroDPS.c);
		// vector_f accl = IMU::getGliderAccel();
		// if (compass != nullptr) {
		// 	ESP_LOGI(FNAME,"Accl:\t%4f\t%4f\t%4f\tL%.2f Gyro:\t%4f\t%4f\t%4f Mag:\t%4f\t%4f\t%4f", accl.a, accl.b, accl.c, accl.get_norm(),
		// 		gyroDPS.a, gyroDPS.b, gyroDPS.c,
		// 		compass->rawX(), compass->rawY(), compass->rawZ());
		// }

		float GS=0; // Autoleveling Gyro feature only with GPS and GS close to zero to avoid triggering at push back taxi with zero AS
		bool gpsOK = Flarm::getGPSknots( GS );
		// ESP_LOGI(FNAME,"GS=%.3f %d", GS, gpsOK );
		if( gpsOK && GS < 2 && ias.get() < 5 ){  // GPS status, groundspeed and airspeed regarded for still stand
			// check low rotation on all 3 axes = on ground
			if( abs( gyroDPS.x ) < MAXDRIFT && abs( gyroDPS.y ) < MAXDRIFT && abs( gyroDPS.z ) < MAXDRIFT ) {
				num_gyro_samples++;
				cur_gyro_bias[0] = IMU::getRawGyroX();
				cur_gyro_bias[1] = IMU::getRawGyroY();
				cur_gyro_bias[2] = IMU::getRawGyroZ();
				if( num_gyro_samples > NUM_GYRO_SAMPLES ) { // every 5 minute (3000 samples) recalculate offset
					mpud::raw_axes_t gb;
					mpud::raw_axes_t gbo = MPU.getGyroOffset();
					for(int i=0; i<3; i++){
						gb[i]  = gbo[i] -(( (cur_gyro_bias)[i]/(NUM_GYRO_SAMPLES*4)) ); // translate to 1000 DPS
						cur_gyro_bias[i] = 0;
					}
					ESP_LOGI(FNAME,"New gyro offset X/Y/Z: OLD:%d/%d/%d NEW:%d/%d/%d", gbo.x, gbo.y, gbo.z, gb.x, gb.y, gb.z );
					if( (abs( gbo.x-gb.x ) > 0) || (abs( gbo.y-gb.y ) > 0) || (abs( gbo.z-gb.z ) > 0)  ){  // any delta is directly set in RAM
						ESP_LOGI(FNAME,"Set new gyro offset X/Y/Z: OLD:%d/%d/%d NEW:%d/%d/%d", gbo.x, gbo.y, gbo.z, gb.x, gb.y, gb.z );
						MPU.setGyroOffset( gb );
					}
					// if we have temperature control, we check if control is locked, otherwise we have no idea but anyway takeover better offset
					if( (HAS_MPU_TEMP_CONTROL && (MPU.getSiliconTempStatus() == MPU_T_LOCKED)) || !HAS_MPU_TEMP_CONTROL ){
						if( (abs( gbo.x-gb.x ) > 1 || abs( gbo.y-gb.y ) > 1 || abs( gbo.z-gb.z ) > 1) && gyro_flash_savings<5 ){ // Set only changes > 1 in Flash and only 5 times per boot
							gyro_bias.set( gb );
							ESP_LOGI(FNAME,"Store the new offset also in Flash, store number: %d", gyro_flash_savings );
							gyro_flash_savings++;
						}
					}
					num_gyro_samples = 0;
				}
			}
		}
		IMU::Process();
	}

}

static void toyFeed(int count) // Called at 5Hz from clientLoop or sensorloop
{
    if (ToyNmeaPrtcl)
    {
        if (ahrs_rpyl_dataset.get())
        {
            ToyNmeaPrtcl->sendXcvRPYL(IMU::getRoll(), IMU::getPitch(), IMU::getYaw(), IMU::getGliderAccelZ());
            ToyNmeaPrtcl->sendXcvAPENV1(ias.get(), altitude.get(), te_vario.get());
        }
        switch (ToyNmeaPrtcl->getProtocolId())
        {
        case BORGELT_P:
            ToyNmeaPrtcl->sendBorgelt(te_vario.get(), OAT.get(), ias.get(), tas, MC.get(), bugs.get(), ballast.get(), VCMode.getCMode(), gflags.validTemperature);
            ToyNmeaPrtcl->sendXcvGeneric(te_vario.get(), altitude_isa.get(), tas);
            break;
        case OPENVARIO_P:
            ToyNmeaPrtcl->sendOpenVario(baroP, dynamicP, te_vario.get(), OAT.get(), gflags.validTemperature);
            break;
        case CAMBRIDGE_P:
            ToyNmeaPrtcl->sendCambridge(te_vario.get(), tas, MC.get(), bugs.get(), altitude.get());
            break;
        case XCVARIO_P:
            ToyNmeaPrtcl->sendStdXCVario(baroP, dynamicP, VCMode.getCMode());
            break;
        case SEEYOU_P:
            ToyNmeaPrtcl->sendSeeYouF(IMU::getGliderAccelX(), IMU::getGliderAccelY(), IMU::getGliderAccelZ(), te_vario.get(), ias.get(), altitude.get(), VCMode.getCMode());
            if ( !(count%10) ) ToyNmeaPrtcl->sendSeeYouS(OAT.get(), VCMode.getCMode(), battery_voltage.get(), altitude.get());
            break;
        default:
            ESP_LOGE(FNAME, "Protocol %d not supported error", ToyNmeaPrtcl->getProtocolId());
        }
    }
}

static void commonThingsFirst()
{
    if (gflags.haveIMU)
    {
        grabMPU();
    }
}
static void commonThingsLast(int count)
{
    if (IMU::getGliderAccelZ() > gload_pos_max.get()) {
        gload_pos_max.set(IMU::getGliderAccelZ());
    }
    else if (IMU::getGliderAccelZ() < gload_neg_max.get()) {
        gload_neg_max.set(IMU::getGliderAccelZ());
    }

    // Need to be done for client and main vario
    polar_sink = Speed2Fly.sink(ias.get());
    te_netto.set(te_vario.get() - polar_sink);
    as2f = Speed2Fly.speed(te_netto.get(), !VCMode.getCMode());

    s2f_ideal.set(std::roundf(as2f));
    // low pass damping
    s2f_delta = s2f_delta + ((as2f - ias.get()) - s2f_delta) * (1 / (s2f_delay.get() * 10));
    // ESP_LOGI( FNAME, "te: %f, polar_sink: %f, netto %f, s2f: %f  delta: %f", aTES2F, polar_sink, te_netto.get(), as2f, s2f_delta );

    if (OneWIRE) {
        // read one wire sensors
        OneWIRE->groupUpdate(Clock::getMillis());
    }

    if (gflags.haveIMU && HAS_MPU_TEMP_CONTROL) {
        // ESP_LOGI(FNAME,"MPU temp control; T=%.2f", MPU.getTemperature() );
        MPU.temp_control(count, xcvTemp);
    }

    AUDIO->updateTone();
    const int screenEvent = ScreenEvent(ScreenEvent::MAIN_SCREEN).raw;
    xQueueSend(uiEventQueue, &screenEvent, 0);
}
static void commonThings5Secs()
{
    SetupCommon::commitDirty(); // very important, flash NVS settings permanently

    ESP_LOGI(FNAME, "Free Heap: %d bytes", heap_caps_get_free_size(MALLOC_CAP_8BIT));
    if (uxTaskGetStackHighWaterMark(NULL) < 512)
    {
        ESP_LOGW(FNAME, "Warning %s task stack low: %d bytes", pcTaskGetName(NULL), uxTaskGetStackHighWaterMark(NULL));
    }
    if (heap_caps_get_free_size(MALLOC_CAP_8BIT) < 20000)
    {
        ESP_LOGW(FNAME, "Warning heap_caps_get_free_size getting low: %d", heap_caps_get_free_size(MALLOC_CAP_8BIT));
    }
    extern MessagePool MP;
    ESP_LOGI(FNAME, "MPool in-use:%d, acq-fails: %d", MP.nrUsed(), MP.nrAcqFails());

    struct timeval tv;
    gettimeofday(&tv, NULL);
    ESP_LOGI(FNAME, "TofDay %d.%03ds", (int)(tv.tv_sec % (60 * 60 * 24)), (int)(tv.tv_usec / 1000));

    // static char buf[2048];
    // vTaskGetRunTimeStats(buf);
    // std::printf("Task runtime stats:\n%s\n", buf);

    // DeviceManager* dm = DeviceManager::Instance();
    // static_cast<TestQuery*>(dm->getProtocol( TEST_DEV2, TEST_P ))->sendTestQuery();  // all 5 seconds on burst
}

void clientLoop(void *pvParameters)
{
	int count = 0;
	esp_task_wdt_add(NULL);

	while (true)
	{
		TickType_t xLastWakeTime = xTaskGetTickCount();
		count++; // 10 Hz

        commonThingsFirst();
		aTE += (te_vario.get() - aTE)* (1/(10*vario_av_delay.get()));

		if( !(count%2) )
		{
			double tmpalt = altitude.get(); // get pressure from altitude
			if( (fl_auto_transition.get() == 1) && ((int)( Units::meters2FL( altitude.get() )) + (int)(gflags.standard_setting) > transition_alt.get() ) ) {
				ESP_LOGI(FNAME,"Above transition altitude");
				baroP = Atmosphere::calcPressureISA(tmpalt); // above transition altitude
			}
			else {
				baroP = Atmosphere::calcPressure( QNH.get(), tmpalt);
			}
			dynamicP = Atmosphere::kmh2pascal(ias.get());
			tas = Atmosphere::TAS2( ias.get(), altitude.get(), OAT.get() );
			if( IMU::getGliderAccelZ() > gload_pos_max.get() ){
				gload_pos_max.set( IMU::getGliderAccelZ() );
			}else if( IMU::getGliderAccelZ() < gload_neg_max.get() ){
				gload_neg_max.set( IMU::getGliderAccelZ() );
			}

			if( uxTaskGetStackHighWaterMark(NULL) < 512 ) {
				ESP_LOGW(FNAME,"Warning client task stack low: %d bytes", uxTaskGetStackHighWaterMark(NULL) );
			}
        }

        commonThingsLast(count);
        if (!(count % 2))
        {
            toyFeed(count);
            if (true)
            { // todo need a mag_hdm.valid() flag
                if (ToyNmeaPrtcl)
                {
                    if (compass_nmea_hdm.get())
                    {
                        ToyNmeaPrtcl->sendXCVNmeaHDM(mag_hdm.get());
                    }

                    if (compass_nmea_hdt.get())
                    {
                        ToyNmeaPrtcl->sendXCVNmeaHDT(mag_hdt.get());
                    }
                }
            }
        }
        if (!(count % 50)) { commonThings5Secs(); }

        esp_task_wdt_reset();
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
	}
}

static int client_sync_dataIdx = 10000;
void startClientSync()
{
	// Start the client sync in a moment
	client_sync_dataIdx = 0;
}

void readSensors(void *pvParameters)
{

	esp_task_wdt_add(NULL);
    int count = 0;
	int16_t landed = 0; // airborne detection counter
    uint32_t spartse_time;

	while (1)
	{
		TickType_t xLastWakeTime = xTaskGetTickCount();
		count++;   // 10x per second

        // pick the time
        spartse_time = Clock::getMillis();

        // loop over all sensors
        for (SensorEntry *e = SensorRegistry::begin(); e != SensorRegistry::end(); ++e)
        {
            if ( ! e->isActive() ) break;
            if ( e->dutycycle && ! (count%e->dutycycle) ) {
                e->sensor->update(spartse_time);
            }
        }

        float T=OAT.get(); // fixme
		if( !gflags.validTemperature ) {
			T= 15 - ( (altitude.get()/100) * 0.65 );
			// ESP_LOGW(FNAME,"T invalid, using 15 deg");
		}
		// ESP_LOGI(FNAME,"Start");
        commonThingsFirst();

		dynamicP = asSensor->getHead();
        baroP = baroSensor->getHead();  	// Baro Pressure
		float teP = teSensor->getHead();    // TE Pressure

        // ESP_LOGI(FNAME,"TE, Delta: %d - log%d", (int)(millis() - _millis));
        if( logging.get() ){
			char log[ProtocolItf::MAX_LEN];
			_millis=millis();
			struct timeval tv;
			gettimeofday(&tv, NULL);
			sprintf( log, "$SENS;");
			int pos = strlen(log);
			long delta = _millis - _gps_millis;
			if( delta < 0 )
				delta += 1000;
			sprintf( log+pos, "%d.%03d,%ld,%.3f,%.3f,%.3f,%.2f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f", (int)(tv.tv_sec%(60*60*24)), (int)(tv.tv_usec / 1000), delta, baroP, teP, dynamicP, T, IMU::getGliderAccelX(), IMU::getGliderAccelY(), IMU::getGliderAccelZ(),
					IMU::getGliderNogateGyroX(), IMU::getGliderNogateGyroY(), IMU::getGliderNogateGyroZ() );
			if( theCompass ){
				pos=strlen(log);
				sprintf( log+pos,",%.4f,%.4f,%.4f", theCompass->rawX(), theCompass->rawY(), theCompass->rawZ());
			}
			pos=strlen(log);
			sprintf( log+pos, "\n");
			const NmeaPrtcl *prtcl = DEVMAN->getNMEA(NAVI_DEV); // Todo preliminary solution ..
			if ( prtcl ) {
				prtcl->sendXCV(log);
			}
		}

		// TE vario calculation
		float tasraw =  Atmosphere::TAS( ias.get() , baroP, T);  // True airspeed in km/h
		float te = bmpVario.readTE( tasraw, teP );   // TE value caclulation
		te_vario.set( te );
		

		// float iasraw = Atmosphere::pascal2kmh( dynamicP );
		// if( baroP != 0 )
		// 	tasraw =  Atmosphere::TAS( iasraw , baroP, T);  // True airspeed in km/h

		// // ESP_LOGI("FNAME","P: %f  IAS:%f", dynamicP, iasraw );

		// static float new_ias = 0;
		// new_ias = ias.get() + (iasraw - ias.get())*0.25;
		// if( (int( ias.get()+0.5 ) != int( new_ias+0.5 ) ) || !(count%20) ){
		// 	ias.set( new_ias );  // low pass filter
		// }
		if( airspeed_max.get() < ias.get() ){
			airspeed_max.set( ias.get() );
		}
		// // ESP_LOGI("FNAME","P: %f  IAS:%f IASF: %d", dynamicP, iasraw, ias );
		// tas += (tasraw-tas)*0.25;       // low pass filter
		// // ESP_LOGI(FNAME,"IAS=%f, T=%f, TAS=%f baroP=%f", ias, T, tas, baroP );

		// // Slip angle estimation
		// float as = tas/3.6;                  // tas in m/s
		// const float K = 4000 * 180/M_PI;      // airplane constant and Ay correction factor
		// if( tas > 25.0 ){
		// 	slip_angle.set(slip_angle.get() + ((IMU::getGliderAccelY()*K / (as*as)) - slip_angle.get())*0.12);   // with atan(x) = x for small x
		// 	// ESP_LOGI(FNAME,"AS: %f m/s, CURSL: %f°, SLIP: %f", as, IMU::getGliderAccelY()*K / (as*as), slip_angle.get() );
		// }

		// ESP_LOGI(FNAME,"count %d ccp %d", count, ccp );
		if( !(count % ccp) ) {
			ESP_LOGI(FNAME,"count %d ccp %d", count, ccp );
			AverageVario::recalcAvgClimb();
		}
		if (FLAP && FLAP->haveAdcSensor()) { FLAP->progress(); }

		// ESP_LOGI(FNAME,"Baro Pressure: %4.3f", baroP );
		// float altSTD = 0;
		// if( Flarm::validExtAlt() && alt_select.get() == AS_EXTERNAL )
		// 	altSTD = alt_external;
		// else
		// 	altSTD = Atmosphere::calcAltitudeISA( baroP );
		// float new_alt = 0;
		// if( alt_select.get() == AS_TE_SENSOR ) // TE
		// 	new_alt = bmpVario.readAVGalt();
		// else if( alt_select.get() == AS_BARO_SENSOR  || alt_select.get() == AS_EXTERNAL ){ // Baro or external
		// 	if(  alt_unit.get() == ALT_UNIT_FL ) { // FL, always standard
		// 		new_alt = altSTD;
		// 		gflags.standard_setting = true;
		// 		// ESP_LOGI(FNAME,"au: %d", alt_unit.get() );
		// 	}else if( (fl_auto_transition.get() == 1) && ((int)Units::meters2FL( altSTD ) + (int)(gflags.standard_setting) > transition_alt.get() ) ) { // above transition altitude
		// 		new_alt = altSTD;
		// 		gflags.standard_setting = true;
		// 		// ESP_LOGI(FNAME,"auto:%d alts:%f ss:%d ta:%f", fl_auto_transition.get(), altSTD, gflags.standard_setting, transition_alt.get() );
		// 	}
		// 	else {
		// 		if( Flarm::validExtAlt() && alt_select.get() == AS_EXTERNAL )
		// 			new_alt = altSTD + ( QNH.get()- 1013.25)*8.2296;  // correct altitude according to ISA model = 27ft / hPa
		// 		else
		// 			new_alt = Atmosphere::calcAltitude( QNH.get(), baroP );
		// 		gflags.standard_setting = false;
		// 		// ESP_LOGI(FNAME,"QNH %f baro: %f alt: %f SS:%d", QNH.get(), baroP, alt, gflags.standard_setting  );
		// 	}
		// }
		// // if( (int( new_alt +0.5 ) != int( altitude.get() +0.5 )) || !(count%20) ){
		// 	// ESP_LOGI(FNAME,"New Altitude: %.1f", new_alt );
		 	// altitude.set( new_alt );
		// // }

		aTE = bmpVario.readAVGTE();

		if( (count % 2) == 0 ){
			if ( ! airborne.get() && (ias.get() >  Speed2Fly.getStallSpeed() + 7) ) {
				airborne.set(true);
			}
			else if ( airborne.get() && (ias.get() <  5) ) {
				if ( landed++ > 100 ) { // ias < 5 km/h for 10 seconds
					airborne.set(false);
				}
			}
			else {
				landed = 0;
			}
			
			toyFeed(count);
		}

		if( theCompass ){
			// ESP_LOGI(FNAME,"Compass, have sensor=%d  hdm=%d", compass->haveSensor(),  compass_nmea_hdt.get());
			if( ! theCompass->calibrationIsRunning() ) {
				// Trigger heading reading and low pass filtering. That job must be
				// done periodically.
				bool ok;
				float heading = theCompass->getGyroHeading( &ok );
				if(ok){
					if( (int)heading != (int)mag_hdm.get() && !(count%10) ){
						mag_hdm.set( heading );
					}
					if( !(count%5) && compass_nmea_hdm.get() == true ) {
						if ( ToyNmeaPrtcl ) {
							ToyNmeaPrtcl->sendXCVNmeaHDM(heading);
						}
					}
				}
				else{
					if( mag_hdm.get() != -1 )
						mag_hdm.set( -1 );
				}
				float theading = theCompass->filteredTrueHeading( &ok );
				if(ok){
					if( (int)theading != (int)mag_hdt.get() && !(count%10) ){
						mag_hdt.set( theading );
					}
					if( !(count%5) && ( compass_nmea_hdt.get() == true )  ) {
						if ( ToyNmeaPrtcl ) {
							ToyNmeaPrtcl->sendXCVNmeaHDT(heading);
						}
					}
				}
				else{
					if( mag_hdt.get() != -1 )
						mag_hdt.set( -1 );
				}
			}
		}

		// Check on new clients connecting
		if ( client_sync_dataIdx < SetupCommon::numEntries() ) {
			while( client_sync_dataIdx < SetupCommon::numEntries() ) {
				if ( SetupCommon::syncEntry(client_sync_dataIdx++) ) {
					break; // Hit entry to actually sync and send data
				}
			}
			if ( client_sync_dataIdx >= SetupCommon::numEntries() ) {
				ESP_LOGI(FNAME,"Client sync complete");
			}
		}


        // battery voltage update
        if ( !(count%10) ) {
            battery_voltage.set(BatVoltage->get());
            if (theCompass) {
               theCompass->ageIncr();
            }

			bool ok=false;
			float xt = baroSensor->readTemperature(ok);
			if( ok ) {
				xcvTemp = xt;
			}

			// Check auto s2f mode filter every second
			if (S2FSWITCH) {
				S2FSWITCH->checkCruiseMode();
			}
		}

        commonThingsLast(count);
        if ((count % 50) == 0) { commonThings5Secs(); }

		esp_task_wdt_reset();
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
	}
}


static esp_err_t _coredump_to_server_begin_cb(void * priv)
{
	std::printf("================= CORE DUMP START =================\r\n");
	return ESP_OK;
}

static esp_err_t _coredump_to_server_end_cb(void * priv)
{
	std::printf("================= CORE DUMP END ===================\r\n");
	return ESP_OK;
}

static esp_err_t _coredump_to_server_write_cb(void * priv, char const * const str)
{
	std::printf("%s\r\n", str);
	return ESP_OK;
}

void register_coredump() {
	coredump_to_server_config_t coredump_cfg = {
			.start = _coredump_to_server_begin_cb,
			.end = _coredump_to_server_end_cb,
			.write = _coredump_to_server_write_cb,
			.priv = NULL,
	};
	if( coredump_to_server(&coredump_cfg) != ESP_OK ){  // Dump to console and do not clear (will done after fetched from Webserver)
		ESP_LOGI( FNAME, "+++ All green, no coredump found in FLASH +++");
	}
}


////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
// Sensor board init method. Herein all functions that make the XCVario are launched and tested.
void system_startup(void *args){

	bool selftestPassed=true;
	ESP_LOGI(FNAME, "startup on core %d", xPortGetCoreID());

	esp_wifi_set_mode(WIFI_MODE_NULL);
	spiMutex = xSemaphoreCreateMutex();
	ESP_LOGI( FNAME, "Log level set globally to INFO %d; Max Prio: %d Wifi: %d",  ESP_LOG_INFO, configMAX_PRIORITIES, ESP_TASKD_EVENT_PRIO-5 );
	esp_chip_info_t chip_info;
	esp_chip_info(&chip_info);
	ESP_LOGI( FNAME,"This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
			chip_info.cores,
			(chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
					(chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");
	ESP_LOGI( FNAME,"Silicon revision %d, ", chip_info.revision);
	uint32_t flash_size = 0;
	ESP_ERROR_CHECK(esp_flash_get_size(NULL, &flash_size));
	ESP_LOGI( FNAME,"%dMB %s flash\n", (int)flash_size / (1024 * 1024),
			(chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
	ESP_LOGI(FNAME, "QNH.get() %.1f hPa", QNH.get() );
	// register_coredump();
	MyGliderPolarIndex = Polars::findMyGlider(glider_type.get());

	AverageVario::begin();

	BatVoltage = new AnalogInput((22.0+1.2)/1200, ADC_CHANNEL_7); // created allways, but only used on master XCV
	BatVoltage->begin(ADC_ATTEN_DB_0);  // for battery voltage
	BatVoltage->setAdjust(factory_volt_adjust.get());
	ccp = (int)(core_climb_period.get()*10);
	spi_bus_config_t buscfg = {
		.mosi_io_num = SPI_MOSI,
		.miso_io_num = SPI_MISO,
		.sclk_io_num = SPI_SCLK,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.data4_io_num = -1,
		.data5_io_num = -1,
		.data6_io_num = -1,
		.data7_io_num = -1,
		.data_io_default_level = false,
		.max_transfer_sz = 8192,
		.flags = 0,
		.isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO,
		.intr_flags = ESP_INTR_FLAG_IRAM
	};
	ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));

	MYUCG = new AdaptUGC();
	MYUCG->begin();
	Display = new IpsDisplay( MYUCG );
	Display->begin();
	Display->bootDisplay();
	MenuRoot = new SetupRoot(Display); // the root setup menu, screens still disabled

	Version V;
	std::string ver( " Ver.: " );
	ver += V.version();
	char hw[24];
	sprintf( hw,", XCV-%d", hardwareRevision.get()+18);  // plus 18, e.g. 2 = XCVario-20
	std::string hwrev( hw );
	ver += hwrev;
	Display->writeText(1, ver.c_str() );
	logged_tests.assign(ver);
	logged_tests += "\n";

    // Start UI task responsible to manage screens and display. Needed to habe the boot screen and message box working
    xTaskCreate(&UiEventLoop, "UIloop", 6144, Rotary, 4, NULL); // increase stack by 1K

    BootUpScreen *boot_screen = BootUpScreen::create();
    MessageBox::createMessageBox();
    if (gflags.schedule_reboot) {
        MBOX->pushMessage(3, "Detecting XCV hardware");
    }

    // Check if we shall enter OTA update mode
    if (software_update.get() || Rotary->readBootupStatus()) {
        software_update.set(0); // only one shot, then boot normal

        if (hardwareRevision.get() >= XCVARIO_22) {
            // Give CAN MagSens a chance for an update
            CANbus::createCAN();
            CAN->begin();
            DEVMAN->addDevice(CANREGISTRAR_DEV, REGISTRATION_P, CAN_REG_PORT, CAN_REG_PORT, CAN_BUS);
            DEVMAN->addDevice(MAGSENS_DEV, MAGSENSBIN_P, MagSensBin::LEGACY_MAGSTREAM_ID, 0, CAN_BUS); // fixme
        }
        delete boot_screen; // screen now belongs to OTA
        MenuRoot->begin(new OTA());
        return;
    }

    if( hardwareRevision.get() >= XCVARIO_21 )
	{
		gflags.haveIMU = true;
        // add AHRS to my caps
        CANPeerCaps::addCapability(XcvCaps::AHRS_CAP);
		mpu_target_temp = mpu_temperature.get();
		ESP_LOGI( FNAME,"MPU initialize");
		MPU.initialize();  // this will initialize the chip and set default configurations
		MPU.setSampleRate(50);  // in (Hz)
		MPU.setAccelFullScale(mpud::ACCEL_FS_8G);
		MPU.setGyroFullScale( GYRO_FS );
		MPU.setDigitalLowPassFilter(mpud::DLPF_5HZ);  // smoother data
		mpud::raw_axes_t gb = gyro_bias.get();
		mpud::raw_axes_t ab = accl_bias.get();
		if( gb.isZero() && ab.isZero() ) {
			ESP_LOGI( FNAME,"MPU computeOffsets");
			MPU.computeOffsets( &ab, &gb );  // returns Offsets in 16G scale
			accl_bias.set( ab );
			gyro_bias.set( gb );
			MPU.setGyroOffset(gb);
			ESP_LOGI( FNAME,"MPU new offsets accl:%d/%d/%d gyro:%d/%d/%d ZERO:%d", ab.x, ab.y, ab.z, gb.x,gb.y,gb.z, gb.isZero() );
		}else{
			MPU.setAccelOffset(ab);
			MPU.setGyroOffset(gb);
		}
		mpud::raw_axes_t accelRaw;
		mpud::float_axes_t accelG;
		float samples = 0;
		delay(200);
		for( auto i=0; i<10; i++ ){
			esp_err_t err = MPU.acceleration(&accelRaw);  // fetch raw data from the registers
			if( err != ESP_OK ) {
				ESP_LOGE(FNAME, "AHRS acceleration I2C read error");
				continue;
			}
			samples++;
			accelG += mpud::accelGravity(accelRaw, mpud::ACCEL_FS_8G);  // raw data to gravity
			ESP_LOGI( FNAME,"MPU %.2f", accelG[0] );
			delay( 10 );
		}
		accelG /= samples;
		float accel = std::sqrtf(accelG[0]*accelG[0]+accelG[1]*accelG[1]+accelG[2]*accelG[2]);
		char ahrs[10];
		sprintf(ahrs, "%.2f", accel);
		logged_tests += "MPU6050 AHRS (" + std::string(ahrs) + "g): ";
		if ( accel >0.8 && accel < 1.1 ) {
			logged_tests += passed_text;
		} else {
			logged_tests += failed_text;
		}
		IMU::init();
		if ( IMU::MPU6050Read() == ESP_OK) {
			IMU::Process();
		}
		ESP_LOGI( FNAME,"MPU current offsets accl:%d/%d/%d gyro:%d/%d/%d ZERO:%d", ab.x, ab.y, ab.z, gb.x,gb.y,gb.z, gb.isZero() );
	}

    // Show configured glider polar
    bool glider_polar_configured = glider_type.get() != glider_type.getDefault() 
            || ! S2F::isPolarEqualTo(MyGliderPolarIndex);
    if (glider_polar_configured) {
        ver.assign("  >> ");
        ver += Polars::getPolarName(MyGliderPolarIndex);
        MBOX->pushMessage(1, ver.c_str());
    }

    // Create serial interfaces
    S1 = new SerialLine((uart_port_t)1, GPIO_NUM_16, GPIO_NUM_17);
    logged_tests += "Serial Interface: ";
    bool ser_test = true;
    if ( ! S1->selfTest() ) {
        ser_test = false;
        logged_tests += "S1 ";
    }
    if (hardwareRevision.get() >= XCVARIO_21) {
        S2 = new SerialLine((uart_port_t)2, GPIO_NUM_18, GPIO_NUM_4);
        if ( ! S2->selfTest() ) {
            ser_test = false;
            logged_tests += "S2 ";
        }
    }
    if ( ser_test ) {
        logged_tests += passed_text;
    } else {
        MBOX->pushMessage(1, "Serial port: Fail");
        logged_tests += failed_text;
        selftestPassed = false;
        ESP_LOGE(FNAME, "Error: Serial Interface failed");
    }

    // Create CAN based on known HW revision (not the very first boot)
    if (hardwareRevision.get() >= XCVARIO_22) {
        ESP_LOGI(FNAME, "NOW add/test CAN");
        CANbus::createCAN();
        logged_tests += "CAN Interface: ";
        if (CAN->selfTest()) {
            logged_tests += passed_text;
        } else {
            MBOX->pushMessage(1, "CAN bus: Fail");
            logged_tests += failed_text;
            ESP_LOGE(FNAME, "Error: CAN Interface failed");
        }
    }

    // DEVMAN serialization, read in all configured devices.
    DEVMAN->reserectFromNvs();
    if (gflags.first_devices_run) {
        DEVMAN->introduceDevices(); // create a flarm etc.
    }
    if (CAN) {
        // just allways, it respects the XCV role setting
        DEVMAN->addDevice(CANREGISTRAR_DEV, REGISTRATION_P, CAN_REG_PORT, CAN_REG_PORT, CAN_BUS);
    }

    // Always check on one wire devices
    if (DEVMAN->addDevice(TEMPSENS_DEV, NO_ONE, 0, 0, OW_BUS))
    {
        CANPeerCaps::addCapability(XcvCaps::TEMP_CAP);
    }

    ESP_LOGI(FNAME, "Wirelss-ID: %s", SetupCommon::getID());
    std::string wireless_id;
    if (DEVMAN->isIntf(BT_SPP) || DEVMAN->isIntf(BT_LE))
    {
        ESP_LOGI(FNAME, "Use BT");
        wireless_id.assign("BT Id: ");
    }
    else if (DEVMAN->isIntf(WIFI_APSTA))
    {
        ESP_LOGI(FNAME, "Use WiFi");
        wireless_id.assign("WLAN: ");
    }
    if (!gflags.schedule_reboot && custom_wireless_id.get().id[0] == '\0')
    {
        custom_wireless_id.set(SetupCommon::getDefaultID()); // Default ID created from MAC address CRC
    }
    if (wireless_id.length() > 0)
    {
        wireless_id += SetupCommon::getID();
        MBOX->pushMessage(1, wireless_id.c_str());
    }

    {
        Cipher crypt;
        gflags.ahrsKeyValid = crypt.checkKeyAHRS();
        ESP_LOGI(FNAME, "AHRS key valid=%d", gflags.ahrsKeyValid);
    }
    boot_screen->finish(0);

    // Configure airspeed sensor
    asSensor = AirspeedSensor::autoSetup();
    logged_tests += "AS " + std::string(asSensor->name()) +  " offset: ";
    if (asSensor)
    {
        ESP_LOGI(FNAME, "AS Speed sensor %s self test PASSED", asSensor->name());
        bool as_ok = asSensor->setup();
        float p = asSensor->doRead();
        if (!std::isnan(p) && p > 60.f) {
            dynamicP = p;
        }
        ias.set(Atmosphere::pascal2kmh(dynamicP));

        // Initialize the airborne status
        airborne.set(ias.get() > Speed2Fly.getStallSpeed());

        ESP_LOGI(FNAME, "Aispeed sensor current speed=%f", ias.get());
        if (!as_ok && (ias.get() < 50))
        {
            MBOX->pushMessage(2, "AS Sensor: NEED ZERO");
            logged_tests += failed_text;
            selftestPassed = false;
        }
        else
        {
            logged_tests += passed_text;
            boot_screen->finish(1);
        }
    }
    else
    {
        ESP_LOGE(FNAME, "Error with air speed pressure sensor, no working sensor found!");
        MBOX->pushMessage(2, "AS Sensor: NOT FOUND");
        logged_tests += notfound_text;
        selftestPassed = false;
        asSensor = nullptr;
    }

    // Configure pressure sensors
    ESP_LOGI(FNAME, "Absolute pressure sensors init, detect type of sensor type..");
    logged_tests += "Baro Sensor: ";
    baroSensor = PressureSensor::autoSetup(SensorId::STATIC_PRESSURE);
    bool batest = true;
    float ba_t, ba_p, te_t, te_p;
    if (baroSensor) {
        if (!baroSensor->selfTest(ba_t, ba_p)) {
            ESP_LOGE(FNAME, "HW Error: Self test Barometric Pressure Sensor failed!");
            MBOX->pushMessage(2, "Baro Sensor: NOT FOUND");
            selftestPassed = false;
            batest = false;
            logged_tests += notfound_text;
        } else {
            ESP_LOGI(FNAME, "Baro Sensor test OK, T=%f P=%f", ba_t, ba_p);
            logged_tests += passed_text;
        }
    }

    logged_tests += "TE Sensor: ";
    teSensor = PressureSensor::autoSetup(SensorId::TE_PRESSURE);
    bool tetest = true;
    if (teSensor) {
        if (!teSensor->selfTest(te_t, te_p)) {
            ESP_LOGE(FNAME, "HW Error: Self test TE Pressure Sensor failed!");
            MBOX->pushMessage(2, "TE Sensor: NOT FOUND");
            selftestPassed = false;
            tetest = false;
            logged_tests += notfound_text;
        } else {
            ESP_LOGI(FNAME, "TE Sensor test OK,   T=%f P=%f", te_t, te_p);
            logged_tests += passed_text;
        }
    }
    if (tetest && batest) {
        ESP_LOGI(FNAME, "Both absolute pressure sensor TESTs SUCCEEDED, now test deltas");
        logged_tests += "TE/Baro Sens. T d. <4'C: ";
        if ((abs(ba_t - te_t) > 4.0) && (ias.get() < 50)) {  // each sensor has deviations, and new PCB has more heat sources
            selftestPassed = false;
            ESP_LOGE(FNAME, "Severe T delta > 4 °C between Baro and TE sensor: °C %f", abs(ba_t - te_t));
            MBOX->pushMessage(1, "TE/Baro Temp: Unequal");
            logged_tests += failed_text;
        } else {
            ESP_LOGI(FNAME, "Abs p sensors temp. delta test PASSED, delta: %f °C", abs(ba_t - te_t));
            logged_tests += passed_text;
        }
        float delta = 2.5;  // in factory we test at normal temperature, so temperature change is ignored.
        if (abs(factory_volt_adjust.get() - 0.00815) < 0.00001) {
            delta += 1.8;  // plus 1.5 Pa per Kelvin, for 60K T range = 90 Pa or 0.9 hPa per Sensor, for both there is 2.5 plus 1.8 hPa to
                           // consider
        }
        logged_tests += "TE/Baro Sens. P d. <2hPa: ";
        if ((abs(ba_p - te_p) > delta) && (ias.get() < 50)) {
            selftestPassed = false;
            ESP_LOGI(FNAME, "Abs p sensors deviation delta > 2.5 hPa between Baro and TE sensor: %f", abs(ba_p - te_p));
            MBOX->pushMessage(1, "TE/Baro P: Unequal");
            logged_tests += failed_text;
        } else {
            ESP_LOGI(FNAME, "AbsP sensor deta test PASSED, D: %f hPa", abs(ba_p - te_p));
            logged_tests += passed_text;
        }
        boot_screen->finish(2);
    } else {
        ESP_LOGI(FNAME, "Absolute pressure sensor TESTs failed");
    }

    AUDIO->applySetup();
    if (audio_mute_gen.get() != AUDIO_OFF) {
        ESP_LOGI(FNAME, "Audio begin");
        logged_tests += "Digi. Audio Poti test: ";
        if (AUDIO->isUp() && AUDIO->isPotiUp()) {
            ESP_LOGI(FNAME, "Digital potentiometer test PASSED");
            logged_tests += passed_text;
        } else {
            ESP_LOGE(FNAME, "Error: Digital potentiomenter selftest failed");
            MBOX->pushMessage(1, "Digital Poti: Failure");
            selftestPassed = false;
            logged_tests += failed_text;
        }
    }

    bmpVario.begin(teSensor, baroSensor, &Speed2Fly);
    bmpVario.setup();

    // 2021 series 3, or 2022 model with new digital poti CAT5171 also features CAN bus
    // do not move the check unless you know the sequence of HW detection
    if (!CAN && AUDIO->haveCAT5171()) {
        // fixme, shouldnt be the HW increased to 22 based on audio poti??
        // check on CAN available, if 100% reliable this would only be a one shot need.
        ESP_LOGI(FNAME, "probing CAN");
        CANbus::createCAN();
        if (CAN->selfTest()) {
            // series 2023 has fixed slope control, prior slope bit for AHRS temperature control
            if (CAN->hasSlopeSupport()) {
                if (hardwareRevision.get() < XCVARIO_22)
                    hardwareRevision.set(XCVARIO_22);  // XCV-22, CAN but no AHRS temperature control
            } else {
                ESP_LOGI(FNAME, "CAN Bus selftest without RS control OK: set hardwareRevision (XCV-23)");
                if (hardwareRevision.get() < XCVARIO_23)
                    hardwareRevision.set(XCVARIO_23);  // XCV-23, including AHRS temperature control
            }
        }
        delete CAN;
        CAN = nullptr;
    }

    if( gflags.haveIMU ){
		if( MPU.whoAmI() == 0x12 ){
			if( hardwareRevision.get() < XCVARIO_25){
				hardwareRevision.set(XCVARIO_25);  // XCV-25: ICL20602
				ESP_LOGI(FNAME,"MPU ICM-20602 -> hardwareRevision (XCV-25)");
			}
		}
	}

	float bat = BatVoltage->get(false);
	logged_tests += "Battery Voltage Sensor: ";
	if( bat < 1 || bat > 28.0 ){
		ESP_LOGE(FNAME,"Error: Battery voltage metering out of bounds, act value=%f", bat );
		MBOX->pushMessage(1, "Bat Meter: Fail");
		logged_tests += failed_text;
		selftestPassed = false;
	}
	else{
		ESP_LOGI(FNAME,"Battery voltage metering test PASSED, act value=%f", bat );
		logged_tests += passed_text;
	}

	// magnetic sensor / compass selftest
	if( theCompass ) {
		logged_tests += "Compass test: ";
		theCompass->begin();
		ESP_LOGI( FNAME, "Magnetic sensor enabled: initialize");
		esp_err_t err = theCompass->selfTest();
		if( err == ESP_OK )		{
			// Activate working of magnetic sensor
			ESP_LOGI( FNAME, "Magnetic sensor selftest: OKAY");
			logged_tests += passed_text;
		}
		else{
			ESP_LOGI( FNAME, "Magnetic sensor selftest: FAILED");
			MBOX->pushMessage(1, "Compass: FAILED");
			logged_tests += failed_text;
			selftestPassed = false;
		}
		theCompass->start();  // start task
	}

	// hardware components now got all detected
	if ( gflags.schedule_reboot ) {
		boot_screen->finish(3);
		SetupCommon::commitDirty();
		sleep(3);
		esp_restart();
	}

	Speed2Fly.begin();
	Version myVersion;
	ESP_LOGI(FNAME,"Program Version %s", myVersion.version() );
	ESP_LOGI(FNAME,"\n\n%s", logged_tests.c_str());
	if( !selftestPassed )
	{
		ESP_LOGI(FNAME,"\n\n\nSelftest failed, see above LOG for Problems\n\n\n");
		MBOX->pushMessage(2, "Selftest FAILED", ScreenMsg::CONFIRM);
        if ( ! airborne.get() ) { AUDIO->startSound(AUDIO_FAIL_SOUND); }
	}
	else{
		ESP_LOGI(FNAME,"\n\n\n*****  Selftest PASSED  ********\n\n\n");
		boot_screen->finish(3); // signal self tests passed
        if ( ! airborne.get()) { AUDIO->startSound(AUDIO_CHECK_SOUND); }
	}

	if( Rotary->readBootupStatus() )
	{
		LeakTest::start( baroSensor, teSensor, asSensor );
	}

	// Set QNH from setup Airfiled elevation, when ! Second && ! airborn
	if( ! SetupCommon::isClient() && ! airborne.get() ) {

		// remove logo
		sleep(1);
		BootUpScreen::terminate();

		ESP_LOGI(FNAME,"Master Mode: QNH Autosetup, IAS=%3f (<50 km/h)", ias.get() );
		// QNH autosetup
		float ae = airfield_elevation.get();
		ESP_LOGI(FNAME, "Airfield Elevation = %4.1f m", ae);
		if (ae > NO_ELEVATION) {
            float baroP = baroSensor->doRead();
            if (Flarm::validExtAlt() && alt_select.get() == AS_EXTERNAL) {
                // correct altitude according to ISA model = 27ft / hPa
                ae = alt_external + (QNH.get() - 1013.25f) * 8.2296f;
            }

            float qnh_best = Atmosphere::calcQNHPressure(baroP, ae);
            QNH.set(qnh_best);
            ESP_LOGI(FNAME, "Auto QNH (direkt) = %4.2f hPa", qnh_best);
        }
        Display->clear();

        // Some more checks on vario configuration
        int screenEvent;
        if (NEED_VOLTAGE_ADJUST) {
            // factory use case
            ESP_LOGI(FNAME, "Do Factory Voltmeter adj");
            screenEvent = ScreenEvent(ScreenEvent::VOLT_ADJUST).raw;
            xQueueSend(uiEventQueue, &screenEvent, 0);
        }

        // airfield use case
        // Glider polar set?
        ESP_LOGI(FNAME, "Check glider polar configuration %d, unchanged %d", glider_type.get(), S2F::isPolarEqualTo(MyGliderPolarIndex));
        if ( ! glider_polar_configured ) {
            screenEvent = ScreenEvent(ScreenEvent::POLAR_CONFIG).raw;
            xQueueSend(uiEventQueue, &screenEvent, 0);
        }
        // QNH adjust screen, always
        screenEvent = ScreenEvent(ScreenEvent::QNH_ADJUST).raw;
        xQueueSend(uiEventQueue, &screenEvent, 0);
        if ( ballast_kg.get() > 0 ) {
            // ballast set when boot-up, get a user confirmation
            screenEvent = ScreenEvent(ScreenEvent::BALLAST_CONFIRM).raw;
            xQueueSend(uiEventQueue, &screenEvent, 0);
        }

    }
	else {
		if ( SetupCommon::isClient() ) {
			bool already_connected = false;
			Device *dev = DEVMAN->getDevice(XCVARIOFIRST_DEV);
			if ( dev ) {
				NmeaPlugin *plg = dev->_link->getNmeaPlugin(XCVSYNC_P);
				if ( plg ) {
					already_connected = static_cast<XCVSyncMsg*>(plg)->syncStarted();
				}
			}
			if ( ! already_connected ) {
				// just sit, wait, show a little message
				MBOX->pushMessage(1, "Waiting for XCV Master");
				ESP_LOGI(FNAME,"Client Mode: Wait for Master XCVario %p", dev);
			}
		}

		BootUpScreen::terminate();
		Display->clear();
	}
	Rotary->begin();


	// Wind calculation
	WindCalcTask::createWindResources();

	// Init the vario screens
	SetupRoot::initScreens();

	if ( flapbox_enable.get() ) {
		FLAP = Flap::theFlap(); // check on FLAP pointer further on
	}
	if( hardwareRevision.get() != XCVARIO_20 ){
		gpio_pullup_en( GPIO_NUM_34 );
		if( gflags.haveIMU && HAS_MPU_TEMP_CONTROL && !gflags.mpu_pwm_initalized  )
		{
			// series 2023 does not have slope support on CAN bus but MPU temperature control
			MPU.pwm_init();
			gflags.mpu_pwm_initalized = true;
		}
	}

	// enter normal operation
	if( SetupCommon::isClient() ){
		xTaskCreate(&clientLoop, "clientLoop", 4096, NULL, 11, NULL);
	}
	else {
		xTaskCreate(&readSensors, "readSensors", 5120, NULL, 12, NULL);
	}

	VCMode.updateCache(); // correct initialization
    AUDIO->initVarioVoice();
}

// #include <xtensa/core-macros.h>  // for XTHAL_GET_CCOUNT

// uint32_t IRAM_ATTR cycle_count() {
// 	float a, b = rand() % 400 - 200;
// 	int idx = b*2;
// 	a = deg2rad(b);
// 	int16_t b1 = 50;
//     uint32_t start = XTHAL_GET_CCOUNT();
//     // asm volatile("add a4, a1, a2");
// 	idx = ((std::signbit(idx) ? -1 : 1));
// 	// b1 = fast_iroundf(b);
// 	uint32_t end = XTHAL_GET_CCOUNT();
// 	// for(b=0.; b<400.; b+=My_PIf)
// 	ESP_LOGI(FNAME,"CMP %f %d", b, b1);
//     return end - start;
// }


extern "C" void  app_main(void)
{
	// global log level
	esp_log_level_set("*", ESP_LOG_INFO);

	ESP_LOGI(FNAME, "app main on core %d", xPortGetCoreID());

	// Mute audio
	AUDIO = new Audio();

	// Access to the non volatile setup
	DeviceManager::Instance(); // Create a blank DM, because on a cleard flash initSetup starts to access it.
	ESP32NVS::CreateInstance(); // NVS is needed for the SetupCommon::initSetup() to work, and to query nvs var existance
	// Check on the existance of some nvs variables
	if ( ! ahrs_licence.exists() ) {
		Cipher crypt;
		crypt.initTest();
	}
    // check on legacy nvs variables to detect a XCVpro update
    if (!flarm_devsetup.exists()) {
        ESP_LOGI(FNAME, "Init devices");
        gflags.first_devices_run = true;
    }
    if (wk_speed_0.exists()) {
        gflags.flaps_nvs_defined = true;
    }
    ESP_LOGI(FNAME,"Now init all Setup elements");
	SetupCommon::initSetup();

	// ESP_LOGI(FNAME,"Measure add %ucount", (unsigned int)cycle_count());

	// Instance to a simple esp timer based clock
	MY_CLOCK = new Clock();

	// Figure HW revision first
	if( hardwareRevision.get() == HW_UNKNOWN ){  // per default we assume there is XCV-20
		ESP_LOGI( FNAME, "Hardware Revision unknown, set revision 2 (XCV-20)");
		hardwareRevision.set(XCVARIO_20);
		// Schedule a reboot after hardware revision got clarified
		gflags.schedule_reboot = true;
	}

	// start i2c bus 1
	ESP_LOGI( FNAME, "Now setup I2C bus IO 21/22");
	i2c1.begin(GPIO_NUM_21, GPIO_NUM_22, 100000 );

	// probe on MPU
	MPU.setBus(i2c1);  // set communication bus, for SPI -> pass 'hspi'
	MPU.setAddr(mpud::MPU_I2CADDRESS_AD0_LOW);  // set address or handle, for SPI -> pass 'mpu_spi_handle'
	if( (MPU.reset() == ESP_OK) && hardwareRevision.get() < XCVARIO_21 ){
		ESP_LOGI( FNAME,"MPU avail, increase hardware revision to 3 (XCV-21)");
		hardwareRevision.set(XCVARIO_21);  // there is MPU6050 gyro and acceleration sensor, at least we got an XCV-21
	}
	ESP_LOGI( FNAME,"Hardware revision %d", hardwareRevision.get());
	MPU.clearpwm(); // Stop MPU heating

	// Init ui and screen UiEventLoop task recources
	uiEventQueue = xQueueCreate(10, sizeof(int));

	// Init of rotary
	if( hardwareRevision.get() == XCVARIO_20 ){
		Rotary = new ESPRotary( GPIO_NUM_4, GPIO_NUM_2, GPIO_NUM_0); // XCV-20 uses GPIO_2 for Rotary
	}
	else {
		Rotary = new ESPRotary( GPIO_NUM_39, GPIO_NUM_36, GPIO_NUM_0);
	}

#ifdef Math_Test // Todo need more unit test code
    for (float v = 1.1; v>0.9; ) {
        if ( floatEqualFast(v, 1.0f) ) {
            ESP_LOGI(FNAME,"equal test %f == 1.0f", v);
            break;
        }
        v -= 0.000001;
    }
#endif
#ifdef Quaternionen_Test
		Quaternion::quaternionen_test();
#endif
#ifdef WMM_Test
		WMM_Model::geomag_test();
#endif
	system_startup( 0 );

	Rotary->updateRotDir();   // Update Rotary direction after XCVario hardware has been detected
	vTaskDelete( NULL );
}
