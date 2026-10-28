#pragma once

#include <cstdint>

// --------------------------------------------------------------------------
// Skin system — 24-slot color scheme for the XCVario Pure display
// --------------------------------------------------------------------------

struct skin_rgb_t {
    uint8_t r, g, b;  // inverted values as required by ILI9341 (pass directly to setColor)
};

struct skin_t {
    // -- Structural / background
    skin_rgb_t bg;       // outer background
    skin_rgb_t flaeche;  // instrument face / area
    // -- Text hierarchy (Primary → Secondary → Beiwerk)
    skin_rgb_t haupt;    // primary readings: Vario, AVG, Flap reading, S2F tip
    skin_rgb_t neben;    // secondary readings: Fahrt, Höhe, Wind, Batterie
    skin_rgb_t s2fpf;    // S2F passive (off-optimal) arrows
    skin_rgb_t label;    // labels / units (Beiwerk: Einheiten, Größen)
    // -- Gauge elements
    skin_rgb_t kante;    // needle edge / border
    skin_rgb_t skalz;    // scale numbers
    skin_rgb_t zeiger;   // vario needle
    // -- State colours
    skin_rgb_t steigen;  // avg-climb rising  (green)
    skin_rgb_t optimal;  // S2F optimal speed (dark green)
    skin_rgb_t warnung;  // warning
    skin_rgb_t alarm;    // alarm / avg-climb falling
    skin_rgb_t info;     // info / avg-climb stable / neutral figure
    // -- Misc elements
    skin_rgb_t windsch;  // wind-shadow indicator
    skin_rgb_t bootlogo; // boot-screen logo colour
    // -- G-meter
    skin_rgb_t gzeiger;  // G-meter needle
    skin_rgb_t gwarn;    // G-meter warning zone
    skin_rgb_t glimit;   // G-meter limit zone
    // -- Setup menu
    skin_rgb_t mtext;    // menu text
    skin_rgb_t mzweig;   // menu branch / tree item
    skin_rgb_t mwert;    // menu value
    // -- Horizon (AHRS)
    skin_rgb_t himmel;   // sky
    skin_rgb_t erde;     // earth / ground
};

// Active skin — pointer set at startup from NVS (see Skins.cpp / IpsDisplay.cpp)
extern const skin_t* g_skin;

// --------------------------------------------------------------------------
// Colour macros — all resolve through the active skin pointer
// --------------------------------------------------------------------------

// Core (previously g_col_background / g_col_highlight)
#define COLOR_BLACK        g_skin->bg.r,      g_skin->bg.g,      g_skin->bg.b
#define COLOR_WHITE        g_skin->haupt.r,   g_skin->haupt.g,   g_skin->haupt.b

// Header / secondary (previously g_col_header_xxx)
#define COLOR_HEADER       g_skin->label.r,   g_skin->label.g,   g_skin->label.b
#define COLOR_HEADER_LIGHT g_skin->neben.r,   g_skin->neben.g,   g_skin->neben.b

// State indicators
#define COLOR_GREEN        g_skin->steigen.r, g_skin->steigen.g, g_skin->steigen.b
#define COLOR_DGREEN       g_skin->optimal.r, g_skin->optimal.g, g_skin->optimal.b
#define COLOR_RED          g_skin->alarm.r,   g_skin->alarm.g,   g_skin->alarm.b
#define COLOR_ORANGE       g_skin->zeiger.r,  g_skin->zeiger.g,  g_skin->zeiger.b
#define COLOR_YELLOW       g_skin->warnung.r, g_skin->warnung.g, g_skin->warnung.b
#define LIGHT_GREEN        g_skin->warnung.r, g_skin->warnung.g, g_skin->warnung.b

// Gauge face / structure
#define DARK_GREY          g_skin->flaeche.r, g_skin->flaeche.g, g_skin->flaeche.b
#define DARK_DGREY         g_skin->bg.r,      g_skin->bg.g,      g_skin->bg.b
#define COLOR_MGREY        g_skin->neben.r,   g_skin->neben.g,   g_skin->neben.b
#define COLOR_LGREY        g_skin->label.r,   g_skin->label.g,   g_skin->label.b
#define COLOR_WGREY        g_skin->s2fpf.r,   g_skin->s2fpf.g,   g_skin->s2fpf.b

// Blue variants
#define COLOR_BLUE         g_skin->info.r,    g_skin->info.g,    g_skin->info.b
#define COLOR_LBLUE        g_skin->windsch.r, g_skin->windsch.g, g_skin->windsch.b
#define COLOR_SKYBLUE      g_skin->himmel.r,  g_skin->himmel.g,  g_skin->himmel.b
#define COLOR_BBLUE        g_skin->windsch.r, g_skin->windsch.g, g_skin->windsch.b
#define COLOR_LBBLUE       g_skin->mzweig.r,  g_skin->mzweig.g,  g_skin->mzweig.b

// Misc
#define COLOR_EARTH        g_skin->erde.r,     g_skin->erde.g,     g_skin->erde.b
#define COLOR_FIGURE       g_skin->info.r,     g_skin->info.g,     g_skin->info.b
#define COLOR_MARINE       g_skin->bootlogo.r, g_skin->bootlogo.g, g_skin->bootlogo.b

// -- New macros (no old equivalent; used in updated drawing code) ----------
#define COLOR_KANTE        g_skin->kante.r,    g_skin->kante.g,    g_skin->kante.b
#define COLOR_SKALZ        g_skin->skalz.r,    g_skin->skalz.g,    g_skin->skalz.b
#define COLOR_S2FPF        g_skin->s2fpf.r,    g_skin->s2fpf.g,    g_skin->s2fpf.b
#define COLOR_BOOTLOGO     g_skin->bootlogo.r, g_skin->bootlogo.g, g_skin->bootlogo.b
#define COLOR_GZEIGER      g_skin->gzeiger.r,  g_skin->gzeiger.g,  g_skin->gzeiger.b
#define COLOR_GWARN        g_skin->gwarn.r,    g_skin->gwarn.g,    g_skin->gwarn.b
#define COLOR_GLIMIT       g_skin->glimit.r,   g_skin->glimit.g,   g_skin->glimit.b
#define COLOR_MTEXT        g_skin->mtext.r,    g_skin->mtext.g,    g_skin->mtext.b
#define COLOR_MZWEIG       g_skin->mzweig.r,   g_skin->mzweig.g,   g_skin->mzweig.b
#define COLOR_MWERT        g_skin->mwert.r,    g_skin->mwert.g,    g_skin->mwert.b

// --------------------------------------------------------------------------
// Display dimensions (unchanged)
// --------------------------------------------------------------------------
extern int16_t DISPLAY_H;
extern int16_t DISPLAY_W;
