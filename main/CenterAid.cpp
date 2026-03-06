/*
 * CenterAid.cpp
 *
 *  Created on: Jan 23, 2022
 *      Author: Eckhard Völlm
 */

#include "CenterAid.h"

#include "math/Units.h"
#include "screen/element/PolarGauge.h"
#include "protocol/Clock.h"
#include "math/Trigonometry.h"
#include "Flarm.h"
#include "setup/SetupNG.h"
#include "AdaptUGC.h"
#include "Colors.h"
#include "sensor/imu/AccMPU6050.h"
#include "logdefnone.h"

#include <algorithm>


constexpr rad_t CA_STEP = Units::deg_to_rad(360/CA_NUM_DIRS); // 15
constexpr rad_t CA_STEP_2 = CA_STEP/2.f;  // 7.5
constexpr int MAX_DISK_RAD = 6;
constexpr int PEAK_STORAGE = 120;
constexpr int DRAW_SCALE = PEAK_STORAGE/MAX_DISK_RAD;

extern AdaptUGC *MYUCG;
CenterAid  *theCenteraid = nullptr;

CenterAid *CenterAid::create(PolarGauge &g)
{
    if ( ! theCenteraid ) {
        // create the singleton
        theCenteraid = new CenterAid(g);
    }
    return theCenteraid;
}
void CenterAid::remove()
{
    if ( theCenteraid ) {
        // remove the singleton
        CenterAid *tmp = theCenteraid;
        theCenteraid = nullptr;
        delete tmp;
    }
}

CenterAid::CenterAid(PolarGauge &g) :
    _gauge(g),
	_glider_on_top(true)
{
	for( int i=0; i<CA_NUM_DIRS; i++ ){
		thermals[i] = 0;
		drawn_thermals[i] = 0;
	}
	idir = 0;
	agedir = 0;
	flightmode = undefined;
	turn_left = 0;
	turn_right = 0;
	fly_straight = 0;
	cur_heading = 0;
	peak_value = 1.0; // we start with expectation of 1 m/s thermals
	gyro_last = 0.0;
	scale = 1.0;
	gps_heading = 0.0;
	last_rts = 0.0;
}

void CenterAid::drawThermal(int tn, int idir, bool draw_red) {
    // ESP_LOGI(FNAME,"drawThermal, tn: %d, idir: %d, ds: %d", tn, idir, draw_red );
    if (idir >= CA_NUM_DIRS || idir < 0) {
        ESP_LOGE(FNAME, "index out of range: %d", idir);
        return;
    }
    int ddir = idir;
    if (!_glider_on_top) {
        if (flightmode == circlingR) {
            ddir = (idir + 3 * CA_NUM_DIRS / 4) % CA_NUM_DIRS;  // move reference to the left
        } else if (flightmode == circlingL) {
            ddir = (idir + CA_NUM_DIRS / 4) % CA_NUM_DIRS;  // move reference to the right
        }
    }
    int16_t cy = _gauge._ref_y - fast_cos_rad(ddir * CA_STEP) * _gauge._radius;
    int16_t cx = _gauge._ref_x + fast_sin_rad(ddir * CA_STEP) * _gauge._radius;

    if (idir != 0) {
        int td = drawn_thermals[idir];
        if (td && (tn < td)) {
            MYUCG->setColor(COLOR_BLACK);
            // ESP_LOGI(FNAME,"erase TH, dir:%d, TE:%d", idir, td );
            MYUCG->drawDisc(cx, cy, td / DRAW_SCALE, UCG_DRAW_ALL);  // 120 / 20 = 6
        }
        if (tn > 0) {
            // ESP_LOGI(FNAME,"draw TH, dir:%d, TE:%d", idir, td );
            if (draw_red)  // for max climb
                MYUCG->setColor(COLOR_RED);
            else
                MYUCG->setColor(COLOR_GREEN);
            MYUCG->drawDisc(cx, cy, tn / DRAW_SCALE, UCG_DRAW_ALL);
        }
        drawn_thermals[idir] = tn;
    }
}

//  x Cxy + (-A,-W)
//  .  .
//  .     .
//  .         .
//  .  Cxy      x Cxy + (B,0)
//  .         .
//  .     .
//  .  .
//  x Cxy + (-2,5)
//
void CenterAid::drawGlider() {
    constexpr int16_t A = -3;
    constexpr int16_t B = 5;
    constexpr int16_t W = 9;
    const int16_t triangle[3][6] = { { A, -W, A, W, B, 0}, {-A, -W, -A, W, -B, 0}, {-W, B, W, B, 0, -A} }; // left, right, tail
    int ddir = 0;
    if (!_glider_on_top) {
        if (flightmode == circlingR) {
            ddir = (3 * CA_NUM_DIRS / 4) % CA_NUM_DIRS;
        } else if (flightmode == circlingL) {
            ddir = (CA_NUM_DIRS / 4) % CA_NUM_DIRS;
        }
    }
    int16_t cy = _gauge._ref_y - fast_cos_rad(ddir * CA_STEP) * _gauge._radius;
    int16_t cx = _gauge._ref_x + fast_sin_rad(ddir * CA_STEP) * _gauge._radius;
    MYUCG->setColor(COLOR_BLACK);
    int16_t *trptr = (int16_t*)triangle[2];
    if (_glider_on_top) {
        MYUCG->drawBox(cx-B, cy-W, 2*B, 2*W);
        if ( flightmode == circlingR )
            trptr = (int16_t*)triangle[0];
        else if (flightmode == circlingL) {
            trptr = (int16_t*)triangle[1];
        }
    } else {
        MYUCG->drawBox(cx-W, cy-B, 2*W, 2*B);
    }
    MYUCG->setColor(COLOR_LBBLUE);
    MYUCG->drawTriangle(cx + trptr[0], cy + trptr[1], cx + trptr[2], cy + trptr[3], cx + trptr[4], cy + trptr[5]);
}

int CenterAid::maxClimbIndex(){
	int max=0;
	int max_index = -1;
	for( int i=0; i<CA_NUM_DIRS; i++ ){
		if( max < thermals[i] ){
			max = thermals[i];
			max_index = i;
		}
	}
	return max_index;
}


void CenterAid::checkThermal(){
	// ESP_LOGI(FNAME,"checkThermal");
	idir = (int)(cur_heading * ((float)CA_NUM_DIRS / PI2f)) % CA_NUM_DIRS;
	mps_t th = te_vario.get();
	th = th > 5.0 ? 5.0 : th;  // limit to 5 m/s to avoid peak value excess
	if( th > peak_value  )
		peak_value += (th - peak_value)*0.1;  // a bit low passing to catch values out of the row
	if( peak_value > 1.0 )                // don't go below 1 m/s this is maximum sensitivity
		peak_value = peak_value * 0.999;  // Peak value aging 0.1% per 100 mS or 1% per second
	scale = PEAK_STORAGE/peak_value;      // scale orients itself at measured peak values
	int ti = std::clamp((int)(th*scale), -126, 127); // positive limit of 8 bit integer type
	ESP_LOGI(FNAME,"newThermal dir:%d, TE:%.2f Peak:%.2f TI:%d", idir, th, peak_value, ti  );
	addThermal( ti  );  // 1 m/s = 10; 5 m/s = 50; -10 m/s = -100
}

void CenterAid::drawCenterAid(){
	// ESP_LOGI(FNAME,"drawCenterAid");
    if (_dirty) {
        drawGlider();
        _dirty = false;
    }
	int maxIndex = maxClimbIndex();
	for( int i=0; i<CA_NUM_DIRS; i++ ){
		int d = (i+idir) % CA_NUM_DIRS;
		// ESP_LOGI(FNAME,"dir:%d TE:%d", d, thermals[d] );
		bool red = d != maxIndex ? false : true;
		drawThermal( thermals[d], i, red );
	}
}


// add one thermal and draw thermal
void CenterAid::addThermal( int teval ){
	// ESP_LOGI(FNAME,"addThermal %.1f (%d), TE:%d", cur_heading, idir, teval );
	if( idir >= CA_NUM_DIRS || idir < 0 ){
		ESP_LOGE(FNAME,"index out of range: %d", idir );
	}else{
		thermals[ idir ] = teval;
	}
}

void CenterAid::ageThermal(){
	// ESP_LOGI(FNAME,"age: dir %d, TH: %d, FM: %d", agedir, thermals[agedir], flightmode );
	float lambda = 0.75; // age faster in straight flight: we leaf quickly the place of lift
	if( flightmode == circlingL ){
		agedir--;
		if( agedir < 0 )
			agedir += CA_NUM_DIRS;
		lambda = 0.95;
	}
	else if( flightmode == circlingR ){
		agedir++;
		lambda = 0.95;
	}
	else{
		agedir++;
	}
	if( agedir < 0 )
		agedir += CA_NUM_DIRS;
	agedir = agedir %CA_NUM_DIRS;
	if( agedir >= CA_NUM_DIRS || agedir < 0 ){
		ESP_LOGE(FNAME,"index out of range: %d", agedir );
		return;
	}
	if( thermals[agedir] != 0 ){
		thermals[agedir] = (int8_t)( (float)(thermals[agedir])*lambda);
	}
}


void CenterAid::calcFlightMode( rad_t headingDiff ){
	// ESP_LOGI(FNAME,"calcFlightMode cur_head: %.1f hdiff:%.1f gs:%.0f GPSS:%d MH:%.1f FM:%d", cur_heading, headingDiff, tas.get(), Flarm::gpsStatus(), mag_hdt.get(), flightmode  );
	if( ! airborne.get() ){
		if( flightmode != undefined ) {
			flightmode = undefined;
			ESP_LOGI(FNAME,"New fm: undefined, no AS");
		}
	}
	else{
		if( headingDiff > Units::deg_to_rad(4.f) ){
            if( turn_right < 4 )
                turn_right++;
            fly_straight = 0;
            if( flightmode != circlingR && turn_right > 2 ) {
                flightmode = circlingR;
                ESP_LOGI(FNAME,"New fm: circle right");
                drawGlider();
            }
		}
		else if( headingDiff < -Units::deg_to_rad(4.f) ) {
			if( turn_left < 4 )
				turn_left++;
			fly_straight = 0;
			if( flightmode != circlingL && turn_left > 2 ){
				flightmode = circlingL;
				ESP_LOGI(FNAME,"New fm: circle left");
				drawGlider();
			}
		}
		else{
			turn_left = turn_right = 0;
			if( fly_straight < 4 )
				fly_straight++;
			if( flightmode != straight && fly_straight > 2 ) {
				flightmode = straight;
				ESP_LOGI(FNAME,"New fm: straight");
			}
		}
	}
}

// 2 Hz called from sensor loop
// > tick : a 10 Hz counter
void CenterAid::tick(int tick){
    rad_t new_heading = -1.0;
    if( theCompass ) { // this is the best source for a heading, use this when avail
        new_heading = Units::deg_to_rad(mag_hdt.get()); // fixme
    }
    if( new_heading < 0.f )  {         // fall back to GPS course and fuse gps heading with gyro
        rad_t gyro_footing = accSensor ? accSensor->getGyroFooting() : 0.f;
        // ESP_LOGI(FNAME,"COD %f", gyro_footing );
        if( false ) { // Flarm::gpsStatus() ){
            if( gyro_last == 0 ){
                gyro_last = gyro_footing;
            }
            rad_t gpshead = Flarm::getGndCourse();
            rad_t gyro = gyro_footing;
            rad_t gyro_delta =  gyro - gyro_last;
            gyro_last = gyro;
            rad_t diff = Vector::angleDiff( gpshead, gps_heading );
            gps_heading += (diff*0.01 + gyro_delta*1.07);
            new_heading = Vector::normalizeDeg( gps_heading );
            // ESP_LOGI(FNAME,"GPS OK TC:%f gdY:%f fused:%f diff:%f", gpshead, gyro_delta, new_heading, diff );
        }else{     // trust as last resort just only gyro for Center Aid
            new_heading = gyro_footing;
            // ESP_LOGI(FNAME,"Gyro yaw %f", new_heading);
        }
        ESP_LOGI(FNAME,"NH %f", new_heading );
    }
    rad_t diff = Vector::angleDiff( new_heading, cur_heading );
    // ESP_LOGI(FNAME,"new heading %.1f diff:%.1f", new_heading, diff );
    if( new_heading != cur_heading ){
        uint32_t rts = Clock::getMillis();
        float dt = (float)(rts - last_rts)/1000.0f;
        last_rts = rts;
        calcFlightMode( diff/dt ); // we calculate own flight mode here
        cur_heading = new_heading;
    }
    checkThermal();

    if (!(tick % 10)) {
        ageThermal();  // 0.2 s per thermal = 4.8 seconds all 24 thermals aged by 0.1 m/s
    }
}
