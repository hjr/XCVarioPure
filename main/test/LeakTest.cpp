#include "LeakTest.h"

#include "sensor/press_diff/AirspeedSensor.h"
#include "sensor/pressure/PressureSensor.h"
#include "screen/DrawDisplay.h"
#include "AdaptUGC.h"
#include "math/Units.h"
#include "sensor.h"
#include "logdefnone.h"
#include <cmath>
#include <algorithm>

#include <cmath>

#define LOOPS 150

extern AdaptUGC *MYUCG;

//
// This test checks for leaks in the pressure system by monitoring the stability of the 
// static and total pressure readings over time.
// It collects readings every 5 seconds for up to 1 minute and compares them against
// the initial baseline.
// If the readings deviate beyond defined thresholds, it indicates a potential leak.
//

void LeakTest::display(int mode) {
    ESP_LOGI(FNAME, "Starting Leak test");

    clear();
    MYUCG->setFont(ucg_font_ncenR14_hr, true);
    menuPrintLn(_title.c_str(), 0);

    constexpr float SPEED_THRESHOLD = 1.0f;    // %
    constexpr float MIN_SPEED       = 10.0f;   // Pa
    constexpr int   MAX_ITER        = 12;      // 5s 
    constexpr int   BASELINE_ITERS  = 3;

    float sba = 0.0f, ste = 0.0f, sspeed = 0.0f;
    float baro_delta_f = 0.0f, te_delta_f = 0.0f, speedd_f = 0.0f;

    bool failed = false;
    int fail_count = 0;
    float leak_loss = 0.0f;

    for (int i = 0; i < MAX_ITER; i++) {

        // --- Collect samples over fixed time window ---
        float ba_buf[200], te_buf[200], sp_buf[200];
        int n_ba = 0, n_te = 0, n_sp = 0;

        TickType_t start = xTaskGetTickCount();
        while (xTaskGetTickCount() - start < pdMS_TO_TICKS(5000)) {

            float ba_tmp, te_tmp;

            bool b_ok = baroSensor->doRead(ba_tmp);
            bool t_ok = teSensor->doRead(te_tmp);

            if (b_ok && n_ba < 200)
                ba_buf[n_ba++] = Units::pa_to_hpa(ba_tmp);

            if (t_ok && n_te < 200)
                te_buf[n_te++] = Units::pa_to_hpa(te_tmp);

            if (asSensor) {
                float s;
                if (asSensor->doRead(s) && s > 5.f && n_sp < 200)
                    sp_buf[n_sp++] = s;
            }

            vTaskDelay(pdMS_TO_TICKS(20));
        }

        // --- Helper: trimmed mean ---
        auto trimmed_mean = [](float *buf, int n) -> float {
            if (n < 5) return 0.0f;
            std::sort(buf, buf + n);
            int start = n * 0.1f;
            int end   = n * 0.9f;
            float sum = 0.0f;
            for (int k = start; k < end; k++)
                sum += buf[k];
            return sum / (end - start);
        };

        float ba = trimmed_mean(ba_buf, n_ba);
        float te = trimmed_mean(te_buf, n_te);
        float speed = (n_sp > 10) ? trimmed_mean(sp_buf, n_sp) : sspeed;

        // --- Build baseline ---
        if (i < BASELINE_ITERS) {
            sba += ba;
            ste += te;
            sspeed += speed;

            if (i == BASELINE_ITERS - 1) {
                sba /= BASELINE_ITERS;
                ste /= BASELINE_ITERS;
                sspeed /= BASELINE_ITERS;
                ESP_LOGI(FNAME, "Baseline -> ST %.2f  TE %.2f  PI %.2f", sba, ste, sspeed);
            }
        }

        // --- Display values ---
        char buf[50];
        snprintf(buf, sizeof(buf), "ST P: %3.2f hPa", ba);
        menuPrintLn(buf, 2);
        snprintf(buf, sizeof(buf), "TE P: %3.2f hPa", te);
        menuPrintLn(buf, 3);
        snprintf(buf, sizeof(buf), "PI P: %3.2f Pa", speed);
        menuPrintLn(buf, 4);

        if (i >= BASELINE_ITERS) {

            float baro_delta = (sba > 1e-3f) ? 100.0f * (ba - sba) / sba : 0.0f;
            float te_delta = (ste > 1e-3f) ? 100.0f * (te - ste) / ste : 0.0f;
            float speedd = (sspeed > 1e-3f) ? 100.0f * (speed - sspeed) / sspeed : 0.0f;

            // Low-pass filtering
            baro_delta_f = 0.7f * baro_delta_f + 0.3f * baro_delta;
            te_delta_f = 0.7f * te_delta_f + 0.3f * te_delta;
            speedd_f = 0.7f * speedd_f + 0.3f * speedd;

            snprintf(buf, sizeof(buf), "ST delta: %2.3f %%", baro_delta_f);
            menuPrintLn(buf, 5);

            snprintf(buf, sizeof(buf), "TE delta: %2.3f %%", te_delta_f);
            menuPrintLn(buf, 6);

            snprintf(buf, sizeof(buf), "PI delta: %2.2f %%", speedd_f);
            menuPrintLn(buf, 7);

            // --- Fail logic with persistence ---
            bool trigger =
                (fabs(baro_delta_f) > LEAK_TEST_MAX_LOSS) ||
                (fabs(te_delta_f) > LEAK_TEST_MAX_LOSS) ||
                ((sspeed > MIN_SPEED) && (fabs(speedd_f) > SPEED_THRESHOLD));

            if (trigger)
                fail_count++;
            else
                fail_count = 0;

            if (fail_count >= 3) {
                failed = true;
                break;
            }

            if( fabs(baro_delta) > leak_loss ) leak_loss = fabs(baro_delta);
            if( fabs(te_delta) > leak_loss ) leak_loss = fabs(te_delta);
        }

        snprintf(buf, sizeof(buf), "Seconds: %d", (i + 1) * 5);
        menuPrintLn(buf, 8);
    }

    leak_test_loss.set(leak_loss);

    // --- Result ---
    if (failed) {
        menuPrintLn("Test FAILED", 9);
        ESP_LOGI(FNAME, "FAILED");
    } else {
        menuPrintLn("Test PASSED", 9);
        ESP_LOGI(FNAME, "PASSED");
        gflags.leak_test_passed = 1;
    }

    xQueueReset(uiEventQueue);
}

