
#include "SubMenuCompassWind.h"

#include "comm/Devices.h"
#include "setup/SetupMenu.h"
#include "setup/SetupMenuSelect.h"
#include "setup/SetupMenuValFloat.h"
#include "sensor/mag/MagVSensor.h"

#include "ShowCirclingWind.h"
#include "ShowStraightWind.h"
#include "comm/DeviceMgr.h"
#include "setup/SetupNG.h"
#include "wind/WindCalcTask.h"
#include "math/vector_3d.h"
#include "AdaptUGC.h"
#include "Colors.h"
#include "logdefnone.h"

#include <cstdlib>

extern AdaptUGC *MYUCG;


static float tesla=0;
static bool showSensorRawData(SetupMenuSelect *p)
{
	const vector_f &raw = *magSensor->getHeadPtr();
	ESP_LOGI( FNAME, "showSensorRawData() %.2f %.2f %.2f", raw.x, raw.y, raw.z );
	char buf[100];
	MYUCG->setColor( COLOR_WHITE );
	sprintf(buf, "X = %.2f  ", raw.x);
	p->menuPrintLn(buf, 3);
	sprintf(buf, "Y = %.2f  ", raw.y);
	p->menuPrintLn(buf, 4);
	sprintf(buf, "Z = %.2f  ", raw.z);
	p->menuPrintLn(buf, 5);
	float t = raw.get_norm()/150.0;
	if( abs(t-tesla) > 5 )
		tesla += (t-tesla)*0.2;
	else if ( abs(t-tesla) > 1 )
		tesla += (t-tesla)*0.07;
	else
		tesla += (t-tesla)*0.01;
	sprintf(buf, "Mag. field H = %.1f uT  ", tesla );
	p->menuPrintLn(buf, 6);
	return true;
}

static inline int16_t absmax_sign(int16_t a, int16_t b)
{
    int16_t m = (abs(a) > abs(b)) ? abs(a) : abs(b);
    return (a < 0) ? -m : m;
}


/** Method for receiving intermediate calibration results. */
static void calibrationReport(const CompassCalibrationData &data, bool print)
{
	// X
	if( data.bits.xmax_green && data.bits.xmin_green ) {
		MYUCG->setColor( COLOR_GREEN );
	} else {
		MYUCG->setColor( COLOR_WHITE );
	}
	MYUCG->setPrintPos( 1, 60 );
	MYUCG->printf( "X-Scale=%3.1f  ", data.scale.x * 100 );
	MYUCG->setPrintPos( 160, 60 );
	MYUCG->printf( "(%.1f)  ", data.sample.x * 100.f/32768.f );
	MYUCG->setPrintPos( 1, 135 );
	MYUCG->printf( "X-Bias=%3.1f  ", data.bias.x * 100.f/32768.f );

	// Y
	if( data.bits.ymax_green && data.bits.ymin_green ) {
		MYUCG->setColor( COLOR_GREEN );
	} else {
		MYUCG->setColor( COLOR_WHITE );
	}
	MYUCG->setPrintPos( 1, 85 );
	MYUCG->printf( "Y-Scale=%3.1f  ", data.scale.y * 100);
	MYUCG->setPrintPos( 160, 85 );
	MYUCG->printf( "(%.1f)  ", data.sample.y * 100.f/32768.f );
	MYUCG->setPrintPos( 1, 160 );
	MYUCG->printf( "Y-Bias=%3.1f  ", data.bias.y * 100.f/32768.f );

	// Z
	if( data.bits.zmax_green && data.bits.zmin_green ) {
		MYUCG->setColor( COLOR_GREEN );
	} else {
		MYUCG->setColor( COLOR_WHITE );
	}
	MYUCG->setPrintPos( 1, 110 );
	MYUCG->printf( "Z-Scale=%3.1f  ", data.scale.z * 100 );
	MYUCG->setPrintPos( 160, 110 );
	MYUCG->printf( "(%.1f)  ", data.sample.z * 100.f/32768.f );
	MYUCG->setPrintPos( 1, 185 );
	MYUCG->printf( "Z-Bias=%3.1f  ", data.bias.z * 100.f/32768.f );

	if( !print ){
		const int16_t X = 180;
		const int16_t Y = 155;
		vector_i16 peak;

		peak.x = int16_t(data.sample.x*114.f/32768);
		peak.y = int16_t(data.sample.y*160.f/32768);
		peak.z = int16_t(data.sample.z*160.f/32768);

		// remove old circles
		static vector_i16 old = { 0,0,0 };
		MYUCG->setColor( COLOR_BLACK );
		MYUCG->drawCircle( X+old.x, Y-old.x, 2 );
		MYUCG->drawCircle( X+old.y, Y, 2 );
		MYUCG->drawCircle( X, Y+old.z,2 );

		// draw lines 3 pixel longer than max(old, new) to overdraw old circles
		vector_i16 axes;
		axes.x = absmax_sign(peak.x, old.x);
		axes.y = absmax_sign(peak.y, old.y);
		axes.z = absmax_sign(peak.z, old.z);

		// draw mag X alias the glider nose
		MYUCG->setColor( COLOR_RED );
		if ( peak.x > 0 && data.bits.xmax_green) {
			MYUCG->setColor( COLOR_GREEN );
		}
		else if ( peak.x < 0 && data.bits.xmin_green) {
			MYUCG->setColor( COLOR_GREEN );
		}
		MYUCG->drawLine( X, Y, X+axes.x+3, Y-axes.x-3);    // 45 degree

		// draw mag Y alias right wing
		MYUCG->setColor( COLOR_RED );
		if ( peak.y > 0 && data.bits.ymax_green ) {
			MYUCG->setColor( COLOR_GREEN );
		}
		else if ( peak.y < 0 && data.bits.ymin_green ) {
			MYUCG->setColor( COLOR_GREEN );
		}
		MYUCG->drawLine( X, Y, X+axes.y+3, Y );

		// draw mag Z alias down
		MYUCG->setColor( COLOR_RED );
		if( peak.z > 0 && data.bits.zmax_green ) {
			MYUCG->setColor( COLOR_GREEN );
		}
		else if ( peak.z < 0 && data.bits.zmin_green ) {
			MYUCG->setColor( COLOR_GREEN );
		}
		MYUCG->drawLine( X, Y, X, Y+axes.z+3);

		MYUCG->setColor( COLOR_WHITE );
		MYUCG->drawCircle( X+peak.x, Y-peak.x, 2 );
		MYUCG->drawCircle( X+peak.y, Y, 2);
		MYUCG->drawCircle( X, Y+peak.z, 2 );
		old = peak;
	}
}

static int compassSensorCalibrateAction(SetupMenuSelect *p) {
	ESP_LOGI(FNAME,"compassSensorCalibrateAction()");
	if( p->getSelect() == 0 ) {
		// Cancel is requested
		return 0;
	}

	p->clear();
	switch(p->getSelect()) {
	case 1: // Start
		MYUCG->setFont( ucg_font_ncenR14_hr, true );
		MYUCG->setPrintPos( 1, 30 );
		MYUCG->printf( "Calibration started" );
		MYUCG->setPrintPos( 1, 220 );
		MYUCG->printf( "Now rotate sensor until" );
		MYUCG->setPrintPos( 1, 245 );
		MYUCG->printf( "all numbers are green" );
		MYUCG->setPrintPos( 1, 270 );
		MYUCG->printf( "Press button to abort" );
		magSensor->calibrate( calibrationReport, false);
		MYUCG->setPrintPos( 1, 30 );
		MYUCG->printf( "Calibration finished !" );
		MYUCG->setPrintPos( 1, 250 );
		vTaskDelay(pdMS_TO_TICKS(3000));
		p->clear();
		magSensor->calibrate( calibrationReport, true );
		MYUCG->setPrintPos( 1, 270 );
		MYUCG->setColor( COLOR_WHITE );
		MYUCG->printf( "Press button to finish" );
		while( ! Rotary->readSwitch(100) ) ;
		break;
	case 2: // Show
		magSensor->calibrate( calibrationReport, true );
		MYUCG->setColor( COLOR_WHITE );
		MYUCG->setPrintPos( 1, 270 );
		MYUCG->printf( "Press button to continue" );
		while( ! Rotary->readSwitch(100) ) ;
		break;
	case 3: // Show Raw Data
		while( ! Rotary->readSwitch(100) ) {
			showSensorRawData(p);
		}
		break;
	default:
		ESP_LOGI(FNAME,"Unknown compass sensor calibration action: %d", p->getSelect());
	}
	p->setSelect(0);
	return 0;
}

int windResourcesAction(SetupMenuSelect *p) {
    ESP_LOGI(FNAME, "Enable/Disable Wind");
    WindCalcTask::createWindResources();
    return 0;
}

// static void options_menu_create_compasswind_compass_dev(SetupMenu *top) {
// 	const char *skydirs[8] = { "0°", "45°", "90°", "135°", "180°", "225°",
// 			"270°", "315°" };
// 	for (int i = 0; i < 8; i++) {
// 		SetupMenuSelect *sms = new SetupMenuSelect("Direction", RST_NONE, compassDeviationAction);
// 		sms->setHelp("Push button to start deviation action");
// 		sms->addEntry(skydirs[i]);
// 		top->addEntry(sms);
// 	}
// }

void options_menu_create_compass_calib(SetupMenu *top) {
	SetupMenuSelect *compSensorCal = new SetupMenuSelect("MagSens Calib.", RST_NONE, compassSensorCalibrateAction);
	compSensorCal->addEntry("Cancel");
	compSensorCal->addEntry("Start");
	compSensorCal->addEntry("Show");
	compSensorCal->addEntry("Show Raw Data");
	compSensorCal->setHelp("Calibrate Magnetic Sensor, mandatory for proper operation");
	top->addEntry(compSensorCal);

	// Fixme replace by WMM
	SetupMenuValFloat *cd = new SetupMenuValFloat("Setup Declination", "°", nullptr, false, &compass_declination);
	cd->setHelp("Set compass declination in degrees");
	top->addEntry(cd);

	// SetupMenuSelect *devMenuA = new SetupMenuSelect("AutoDeviation", RST_NONE, nullptr, &compass_dev_auto);
	// devMenuA->setHelp("Automatic adaptive deviation and precise airspeed evaluation method using data from circling wind");
	// devMenuA->addEntry("Disable");
	// devMenuA->addEntry("Enable");
	// top->addEntry(devMenuA);

	// SetupMenu *devMenu = new SetupMenu("Setup Deviations", options_menu_create_compasswind_compass_dev);
	// devMenu->setHelp("Compass Deviations");
	// top->addEntry(devMenu);

	// Show comapss deviations
	// SetupMenuDisplay *smd = new SetupMenuDisplay("Show Deviations", display_deviations_action);
	// top->addEntry(smd);

	// SetupMenuSelect *sms = new SetupMenuSelect("Reset Deviations ", RST_NONE, compassResetDeviationAction);
	// sms->setHelp("Reset all deviation data to zero");
	// sms->addEntry("Cancel");
	// sms->addEntry("Reset");
	// top->addEntry(sms);

	SetupMenuSelect *nmeaHdm = new SetupMenuSelect("Magnetic Heading", RST_NONE, nullptr, &compass_nmea_hdm);
	nmeaHdm->mkEnable();
	nmeaHdm->setHelp("Enable/disable NMEA '$HCHDM' sentence generation for magnetic heading");
	top->addEntry(nmeaHdm);

	SetupMenuSelect *nmeaHdt = new SetupMenuSelect("True Heading", RST_NONE, nullptr, &compass_nmea_hdt);
	nmeaHdt->mkEnable();
	nmeaHdt->setHelp("Enable/disable NMEA '$HCHDT' sentence generation for true heading");
	top->addEntry(nmeaHdt);

	// Show compass settings
	// SetupMenuDisplay *scs = new SetupMenuDisplay("Show Settings", show_compass_setting);
	// top->addEntry(scs);
}

static void options_menu_create_compasswind_straightwind_filters(SetupMenu *top) {
	SetupMenuValFloat *smgsm = new SetupMenuValFloat("Airspeed Lowpass", "", nullptr, false, &wind_as_filter);
	smgsm->setPrecision(3);
	top->addEntry(smgsm);
	smgsm->setHelp(
			"Lowpass filter factor (per sec) for airspeed estimation from AS/Compass and GPS tracks");

	SetupMenuValFloat *devlp = new SetupMenuValFloat("Deviation Lowpass", "", nullptr, false, &wind_dev_filter);
	devlp->setPrecision(3);
	top->addEntry(devlp);
	devlp->setHelp(
			"Lowpass filter factor (per sec) for deviation table correction from AS/Compass and GPS tracks");

	SetupMenuValFloat *smgps = new SetupMenuValFloat("GPS Lowpass", "sec", nullptr, false, &wind_gps_lowpass);
	smgps->setPrecision(1);
	top->addEntry(smgps);
	smgps->setHelp(
			"Lowpass filter factor for GPS track and speed, to correlate with Compass latency");

	SetupMenuValFloat *wlpf = new SetupMenuValFloat("Averager", "", nullptr, false, &wind_filter_lowpass);
	wlpf->setPrecision(0);
	top->addEntry(wlpf);
	wlpf->setHelp("Number of measurements (seconds) averaged in straight flight live wind estimation");
}

static void options_menu_create_compasswind_straightwind_limits(SetupMenu *top) {
	SetupMenuValFloat *smdev = new SetupMenuValFloat("Deviation Limit", "°", nullptr, false, &wind_max_deviation);
	smdev->setHelp(
			"Maximum deviation change accepted when derived from AS/Compass and GPS tracks");
	top->addEntry(smdev);

	SetupMenuValFloat *smslip = new SetupMenuValFloat("Sideslip Limit", "°", nullptr, false, &swind_sideslip_lim);
	smslip->setPrecision(1);
	top->addEntry(smslip);
	smslip->setHelp(
			"Maximum side slip estimation in ° accepted in straight wind calculation");

	SetupMenuValFloat *smcourse = new SetupMenuValFloat("Course Limit", "°", nullptr, false, &wind_straight_course_tolerance);
	smcourse->setPrecision(1);
	top->addEntry(smcourse);
	smcourse->setHelp(
			"Maximum delta angle in ° per second accepted for straight wind calculation");

	SetupMenuValFloat *aslim = new SetupMenuValFloat("AS Delta Limit", "km/h", nullptr, false, &wind_straight_speed_tolerance);
	aslim->setPrecision(0);
	top->addEntry(aslim);
	aslim->setHelp(
			"Maximum delta in airspeed estimation from wind and GPS during straight flight accepted for straight wind calculation");
}

static void options_menu_create_compasswind_straightwind(SetupMenu *top) {
	SetupMenu *strWindFM = new SetupMenu("Filters", options_menu_create_compasswind_straightwind_filters);
	top->addEntry(strWindFM);
	SetupMenu *strWindLM = new SetupMenu("Limits", options_menu_create_compasswind_straightwind_limits);
	top->addEntry(strWindLM);
	ShowStraightWind *ssw = new ShowStraightWind("Straight Wind Status");
	top->addEntry(ssw);
}

void options_menu_create_compasswind_circlingwind(SetupMenu *top) {
	SetupMenuValFloat *cirwd = new SetupMenuValFloat("Max Delta", "°", nullptr, false, &max_circle_wind_diff);
	top->addEntry(cirwd);
	cirwd->setHelp(
			"Maximum accepted value for heading error in circling wind calculation");

	SetupMenuValFloat *cirlp = new SetupMenuValFloat("Averager", "", nullptr, false, &circle_wind_lowpass);
	cirlp->setPrecision(0);
	top->addEntry(cirlp);
	cirlp->setHelp(
			"Number of circles used for circling wind averager. A value of 1 means no average");

	SetupMenuValFloat *cirwsd = new SetupMenuValFloat("Max Speed Delta", "km/h", nullptr, false, &max_circle_wind_delta_speed);
	top->addEntry(cirwsd);
	cirwsd->setPrecision(1);
	cirwsd->setHelp(
			"Maximum wind speed delta from last measurement accepted for circling wind calculation");

	SetupMenuValFloat *cirwdd = new SetupMenuValFloat("Max Dir Delta", "°", nullptr, false, &max_circle_wind_delta_deg);
	top->addEntry(cirwdd);
	cirwdd->setPrecision(1);
	cirwdd->setHelp(
			"Maximum wind direction delta from last measurement accepted for circling wind calculation");

	// Show Circling Wind Status
	ShowCirclingWind *scw = new ShowCirclingWind("Circling Wind Status");
	top->addEntry(scw);
}

#ifdef DEBUG_AND_TEST
void options_menu_create_wind(SetupMenu *top) {
	// Wind speed observation window
	SetupMenuSelect *windcal = new SetupMenuSelect("Wind Calculation", RST_NONE, windResourcesAction, &wind_enable);
	windcal->addEntry("Disable", WA_OFF);
	windcal->addEntry("Straight", WA_STRAIGHT);
	windcal->addEntry("Circling", WA_CIRCLING);
	windcal->addEntry("Both", WA_BOTH);
	windcal->addEntry("External", WA_EXTERNAL);
	windcal->setHelp("Enable Wind calculation for straight flight (needs compass), circling, both or external source");
	top->addEntry(windcal);

	SetupMenu *strWindM = new SetupMenu("Straight Wind", options_menu_create_compasswind_straightwind);
	top->addEntry(strWindM);
	strWindM->setHelp("Straight flight wind calculation needs compass module active");

	SetupMenu *cirWindM = new SetupMenu("Circling Wind", options_menu_create_compasswind_circlingwind);
	top->addEntry(cirWindM);
}
#endif
