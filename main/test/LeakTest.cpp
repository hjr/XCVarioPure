#include "LeakTest.h"

#include "sensor/press_diff/AirspeedSensor.h"
#include "sensor/pressure/PressureSensor.h"
#include "screen/DrawDisplay.h"
#include "AdaptUGC.h"
#include "math/Units.h"
#include "sensor.h"
#include "logdefnone.h"

#include <cmath>

#define LOOPS 150

extern AdaptUGC *MYUCG;

//
// This test checks for leaks in the pressure system by monitoring the stability of the 
// static and total pressure readings over time.
// It collects readings every 5 seconds for up to 2 minutes and compares them against 
// the initial baseline.
// If the readings deviate beyond defined thresholds, it indicates a potential leak.
//

void LeakTest::display(int mode) {
	ESP_LOGI(FNAME, "Starting Leak test");

	// Display initialization
	clear();
	MYUCG->setFont(ucg_font_ncenR14_hr, true);
    menuPrintLn(_title.c_str(), 0);

	// Constants for thresholds
	constexpr float ST_THRESHOLD   = 0.05f;   // %
	constexpr float TE_THRESHOLD   = 0.05f;   // %
	constexpr float SPEED_THRESHOLD = 1.0f;  // %
	constexpr float MIN_SPEED      = 10.0f;  // Pa
	constexpr int   MAX_ITER       = 12;     // 12 * 5s = 60s
	constexpr float INV_LOOPS      = 1.0f / LOOPS;

	// Baseline values
	float sba = 0.0f, ste = 0.0f, sspeed = 0.0f;
	bool failed = false;

	for (int i = 0; i < MAX_ITER; i++) {
		float ba = 0.0f, te = 0.0f, speed = 0.0f;
		int loops_run = 0;

		// Collect readings
		for (int j = 0; j < LOOPS; j++) {
			if (asSensor) {
				float s;
				if (asSensor->doRead(s) && s>5.f) {
					speed += s;
					loops_run++;
				}
			}
			float tmp;
			if ( baroSensor->doRead(tmp) ) {
				ba += Units::pa_to_hpa(tmp);
			}
			if ( teSensor->doRead(tmp) ) {
				te += Units::pa_to_hpa(tmp);
			}
			vTaskDelay(pdMS_TO_TICKS(33));
		}

		ba *= INV_LOOPS;
		te *= INV_LOOPS;
		if (loops_run > 0) speed /= loops_run;

		// Initialize baselines
		if (i == 0) {
			sba = ba;
			ste = te;
			sspeed = speed;
			ESP_LOGI(FNAME, "Baseline -> ST %.2f  TE %.2f  PI %.2f", ba, te, speed);
		}

		// Display sensor values
		char buf[40];
		snprintf(buf, sizeof(buf), "ST P: %3.2f hPa", ba);
		menuPrintLn(buf, 2);
		snprintf(buf, sizeof(buf), "TE P: %3.2f hPa", te);
		menuPrintLn(buf, 3);
		snprintf(buf, sizeof(buf), "PI P: %3.2f Pa", speed);
		menuPrintLn(buf, 4);

		if (i >= 1) {
			float bad = 100.0f * (ba - sba) / sba;
			float ted = 100.0f * (te - ste) / ste;
			float speedd = (fabs(sspeed) > 1e-6f) ?
					(100.0f * (speed - sspeed) / sspeed) : 0.0f;

			snprintf(buf, sizeof(buf), "ST delta: %2.3f %%", bad);
			menuPrintLn(buf, 5);
			ESP_LOGI(FNAME, "%s", buf);

			snprintf(buf, sizeof(buf), "TE delta: %2.3f %%", ted);
			menuPrintLn(buf, 6);
			ESP_LOGI(FNAME, "%s", buf);

			snprintf(buf, sizeof(buf), "PI delta: %2.2f %%", speedd);
			menuPrintLn(buf, 7);
			ESP_LOGI(FNAME, "%s", buf);

			// Stopping condition
			if ((fabs(bad) > ST_THRESHOLD) ||
					(fabs(ted) > TE_THRESHOLD) ||
					((sspeed > MIN_SPEED) && (fabs(speedd) > SPEED_THRESHOLD))) {
				failed = true;
				break;
			}
		}

		snprintf(buf, sizeof(buf), "Seconds: %d", (i * 5) + 5);
		menuPrintLn(buf, 8);
	}

	// Final result
	if (failed) {
		menuPrintLn("Test FAILED", 9);
		ESP_LOGI(FNAME, "FAILED");
	} else {
		menuPrintLn("Test PASSED", 9);
		ESP_LOGI(FNAME, "PASSED");
		gflags.leak_test_passed = 1;
	}

	// flush event queue to avoid false button press detection after test
	xQueueReset(uiEventQueue);
}

