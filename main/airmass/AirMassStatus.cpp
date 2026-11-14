#include "AirMass.h"
#include "AirMassSensor.h"

#include "setup/primitive/SetupMenuDisplay.h"
#include "setup/SetupCommon.h"
#include "screen/UiEvents.h"
#include "screen/DrawDisplay.h"
#include "sensor.h"
#include "driver/time/Clock.h"
#include "driver/time/ClockIntf.h"
#include "driver/time/WatchDog.h"
#include "wind/Wind.h"
#include "math/Units.h"
#include "Atmosphere.h"
#include "AdaptUGC.h"
#include "Colors.h"

#include <cstdio>
#include <cstring>

extern AdaptUGC *MYUCG;

namespace {

class StatusPage final : public SetupMenuDisplay, public Clock_I
{
public:
    StatusPage() : SetupMenuDisplay("Air Mass Status"), Clock_I(50)
    {
        setNeverInline();
    }
    ~StatusPage() override { hide(); }

    void display(int mode = 0) override;
    void exit(int ups = 1) override;
    bool tick() override;
    void draw();

    static StatusPage* shown;

private:
    void hide();
    void line(int ln, const char* text);

    static constexpr int ROWS = 16;
    static constexpr int LANDSCAPE_ROWS = 9; // title and 8 lines, no room for the exit hint
    char _drawn[ROWS][48] = {};
    int16_t _end[ROWS] = {}; // where the text of the row ends on the screen
};

StatusPage* StatusPage::shown = nullptr;

void StatusPage::hide()
{
    if (shown == this) {
        Clock::stop(this);
        shown = nullptr;
    }
}

void StatusPage::display(int)
{
    clear();
    printTitle();
    memset(_drawn, 0, sizeof(_drawn));
    memset(_end, 0, sizeof(_end));
    draw();
    if (maxLines() > LANDSCAPE_ROWS) {
        menuPrintLn("Press button to exit", maxLines() - 1, 5);
    }
    if (!shown) {
        shown = this;
        Clock::start(this);
    }
}

void StatusPage::exit(int ups)
{
    hide();
    MenuEntry::exit(ups);
}

// runs in the timer task, drawing belongs to the UI task
bool StatusPage::tick()
{
    int evt = ScreenEvent(ScreenEvent::AIRMASS_STATUS).raw;
    xQueueSend(uiEventQueue, &evt, 0);
    return false;
}

void StatusPage::line(int ln, const char* text)
{
    if (ln >= ROWS || !strncmp(_drawn[ln], text, sizeof(_drawn[ln]) - 1)) {
        return;
    }
    snprintf(_drawn[ln], sizeof(_drawn[ln]), "%s", text);
    // the text covers the one before with its background, clearing the row first would flicker
    MYUCG->setFont(ucg_font_fub14_hr, true);
    MYUCG->setColor(COLOR_WHITE);
    menuPrintLn(text, ln, 5);
    const int16_t end = 5 + MYUCG->getStrWidth(text);
    if (end < _end[ln]) {
        // what is left of a longer text: the row from the ascent to the descent of its text, menuClearLn would cut
        // the descenders of the row above
        MYUCG->setColor(COLOR_BLACK);
        MYUCG->drawBox(end, ln * LINE_HEIGHT + 6, _end[ln] - end, LINE_HEIGHT - 1);
        MYUCG->setColor(COLOR_WHITE);
    }
    _end[ln] = end;
}

int windText(char* buf, size_t size, const char* label, int raw)
{
    const WindData w(static_cast<uint32_t>(raw));
    if (w.isUsable()) {
        return snprintf(buf, size, "%s %d°/%.0f %s", label, w.getDeg(), SpeedUnit->apply(w.getVal()), SpeedUnit->getName());
    }
    return snprintf(buf, size, "%s --", label);
}

void StatusPage::draw()
{
    char buf[64];
    int ln = 1;
    const int st = airmass_status.get();
    const AirMassEKF* ekf = airmassSensor && airmassSensor->getEKF().isRunning() ? &airmassSensor->getEKF() : nullptr;

    const char* state = "On ground";
    if (st & AirMassEKF::ST_ALIGNED) {
        state = "Running";
    } else if (ekf || st) {
        state = "Aligning";
    }
    snprintf(buf, sizeof(buf), "%s:%s%s%s", state, (st & AirMassEKF::ST_VARIO_VALID) ? " vario" : "",
             (st & AirMassEKF::ST_WIND_VALID) ? " wind" : (st & AirMassEKF::ST_WIND_DEGRADED) ? " wind?" : "",
             (st & AirMassEKF::ST_GNSS_FRESH) ? " gnss" : "");
    line(ln++, buf);

    if (st & AirMassEKF::ST_VARIO_VALID) {
        snprintf(buf, sizeof(buf), "Vario: %+.1f %s", VarioUnit->apply(airmass_vario.get()), VarioUnit->getName());
    } else {
        snprintf(buf, sizeof(buf), "Vario: --");
    }
    line(ln++, buf);
    const int len = windText(buf, sizeof(buf), "Wind:", inst_wind.get());
    if (dwidth > dheight) {
        windText(buf + len, sizeof(buf) - len, ", avg", synoptic_wind.get());
    } else {
        line(ln++, buf);
        windText(buf, sizeof(buf), "Avg wind:", synoptic_wind.get());
    }
    line(ln++, buf);

    if (!SetupCommon::isMaster()) {
        return;
    }
    if (ekf) {
        snprintf(buf, sizeof(buf), "TAS scale: %.3f +-%.3f", ekf->kTas(), ekf->sigma(AirMassEKF::KTAS));
        line(ln++, buf);
        snprintf(buf, sizeof(buf), "Pitch: %.2f° +-%.2f", Units::rad_to_deg(ekf->pitchOffset()),
                 Units::rad_to_deg(ekf->sigma(AirMassEKF::THB)));
        line(ln++, buf);
        snprintf(buf, sizeof(buf), "GNSS lag: %.0f +-%.0f ms", ekf->gnssLatency() * 1000.f, ekf->sigma(AirMassEKF::TAU) * 1000.f);
        line(ln++, buf);
        const uint32_t age = ekf->gnssAgeMs();
        snprintf(buf, sizeof(buf), "Fix %d ms, rejected %u", age == UINT32_MAX ? -1 : static_cast<int>(age),
                 static_cast<unsigned>(ekf->rejectedGnss()));
        line(ln++, buf);
        snprintf(buf, sizeof(buf), "Bias: %.2f°/s %.2fm/s2", Units::rad_to_deg(ekf->yawRateBias()), ekf->accBias());
        line(ln++, buf);
    } else {
        snprintf(buf, sizeof(buf), "TAS scale: %.3f", airmass_k_tas.get());
        line(ln++, buf);
        const int learned = airmass::learnedFlapPositions();
        if (learned >= 0) {
            snprintf(buf, sizeof(buf), "Pitch: %d flap pos.", learned);
        } else {
            snprintf(buf, sizeof(buf), "Pitch: %.2f°", Units::rad_to_deg(airmass_pitch_ofs.get()));
        }
        line(ln++, buf);
        snprintf(buf, sizeof(buf), "GNSS lag: %.0f ms", airmass_latency.get() * 1000.f);
        line(ln++, buf);
        while (ln < maxLines() - (maxLines() > LANDSCAPE_ROWS)) {
            line(ln++, "");
        }
    }
}

} // namespace

MenuEntry* airmass::statusPage()
{
    return new StatusPage();
}

void airmass::statusUpdate()
{
    if (StatusPage::shown) {
        StatusPage::shown->draw();
        // a page to watch in flight: without it the setup closes after a while without the knob
        if (uiMonitor) {
            uiMonitor->pet();
        }
    }
}
