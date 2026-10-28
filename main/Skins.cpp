// Skins.cpp — 9 built-in colour presets for the XCVario Pure display
//
// All RGB values are INVERTED as required by the ILI9341 driver running in
// ILI9341_COLOR_18_BIT mode:  stored = (255-R, 255-G, 255-B).
// Pass them directly to ucg->setColor(r, g, b).
//
// Original (non-inverted) hex values are noted in each comment so the
// xcvario-farbschema-demo.html tool stays the authoritative reference.

#include "Skins.h"
#include "AdaptUGC.h"

extern AdaptUGC *MYUCG;

// -- Definition of the global skin pointer --------------------------------
const skin_t* g_skin = nullptr;   // set by skin_apply() at startup

// -- Preset definitions ----------------------------------------------------

static const skin_t skin_cross_country = {
    .bg         = {255,255,255},  // #000000
    .flaeche    = {239,219,197},  // #10243A
    .haupt      = {  0,  0,  0},  // #FFFFFF
    .neben      = { 97, 52,  0},  // #9ECBFF
    .s2fpf      = { 97, 52,  0},  // #9ECBFF
    .label      = {113,100, 87},  // #8E9BA8
    .kante      = {179,205,239},  // #4C3210
    .skalz      = {113,100, 87},  // #8E9BA8
    .zeiger     = { 18, 99,206},  // #ED9C31
    .steigen    = {214, 92,  0},  // #29A3FF
    .optimal    = {231,163, 87},  // #185CA8
    .warnung    = {  0, 43,255},  // #FFD400
    .alarm      = { 47,235,215},  // #D01428
    .info       = { 54, 41, 28},  // #C9D6E3
    .windsch    = {228,197,163},  // #1B3A5C
    .bootlogo   = {214, 92,  0},  // #29A3FF
    .gzeiger    = {  0,  0,  0},  // #FFFFFF
    .gwarn      = {  0, 43,255},  // #FFD400
    .glimit     = { 47,235,215},  // #D01428
    .mtext      = {  0,  0,  0},  // #FFFFFF
    .mzweig     = { 54, 41, 28},  // #C9D6E3
    .mwert      = {113,100, 87},  // #8E9BA8
    .himmel     = {167,101, 45},  // #589AD2
    .erde       = {204,215,240},  // #33280F
};  // Cross Country

static const skin_t skin_grass_strip = {
    .bg         = {255,255,255},  // #000000
    .flaeche    = {229,229,229},  // #1A1A1A
    .haupt      = {  0,  0,  0},  // #FFFFFF
    .neben      = { 95, 95, 95},  // #A0A0A0
    .s2fpf      = { 95, 95, 95},  // #A0A0A0
    .label      = { 94, 77, 62},  // #A1B2C1
    .kante      = {173,204,252},  // #523303
    .skalz      = { 94, 77, 62},  // #A1B2C1
    .zeiger     = {  0, 96,245},  // #FF9F0A
    .steigen    = {255, 61,229},  // #00C21A
    .optimal    = {255,143,240},  // #00700F
    .warnung    = {  0, 79,255},  // #FFB000
    .alarm      = {  0,223,223},  // #FF2020
    .info       = {211,138,  2},  // #2C75FD
    .windsch    = {255,190, 70},  // #0041B9
    .bootlogo   = {255,190, 70},  // #0041B9
    .gzeiger    = {  0, 96,245},  // #FF9F0A
    .gwarn      = {  0, 79,255},  // #FFB000
    .glimit     = {  0,223,223},  // #FF2020
    .mtext      = {  0,  0,  0},  // #FFFFFF
    .mzweig     = {211,138,  2},  // #2C75FD
    .mwert      = { 94, 77, 62},  // #A1B2C1
    .himmel     = {190,130,  0},  // #417DFF
    .erde       = {245,245,245},  // #0A0A0A
};  // Grass Strip

static const skin_t skin_energy_line = {
    .bg         = {255,255,255},  // #000000
    .flaeche    = {240,229,189},  // #0F1A42
    .haupt      = { 30, 19,  0},  // #E1ECFF
    .neben      = {109, 71,  0},  // #92B8FF
    .s2fpf      = {109, 71,  0},  // #92B8FF
    .label      = {100,133, 14},  // #9B7AF1
    .kante      = {179,173,213},  // #4C522A
    .skalz      = { 97, 97, 97},  // #9E9E9E
    .zeiger     = { 19,  0,125},  // #ECFF82
    .steigen    = { 73,211,  0},  // #B62CFF
    .optimal    = {207,119,255},  // #308800
    .warnung    = {  0, 73,211},  // #FFB62C
    .alarm      = {  0,211,138},  // #FF2C75
    .info       = {211,138,  0},  // #2C75FF
    .windsch    = {255,187, 58},  // #0044C5
    .bootlogo   = {135,181,  0},  // #784AFF
    .gzeiger    = {211, 32,  0},  // #2CDFFF
    .gwarn      = {  0, 73,211},  // #FFB62C
    .glimit     = {  0,179,212},  // #FF4C2B
    .mtext      = { 30, 19,  0},  // #E1ECFF
    .mzweig     = {138, 69,  0},  // #75BAFF
    .mwert      = { 44, 70,  0},  // #D3B9FF
    .himmel     = {235,156, 10},  // #1463F5
    .erde       = {205,216,235},  // #322714
};  // Energy Line

static const skin_t skin_vintage = {
    .bg         = {255,255,255},  // #000000
    .flaeche    = {225,225,229},  // #1E1E1A
    .haupt      = { 10, 15, 35},  // #F5F0DC
    .neben      = { 55, 65,105},  // #C8BE96
    .s2fpf      = { 55, 65,105},  // #C8BE96
    .label      = {113,120,151},  // #8E8768
    .kante      = {191,194,207},  // #403D30
    .skalz      = { 66, 76,117},  // #BDB38A
    .zeiger     = {  0,  0,  0},  // #FFFFFF
    .steigen    = {164, 59,149},  // #5BC46A
    .optimal    = {209,116,176},  // #2E8B4F
    .warnung    = { 31, 90,209},  // #E0A52E
    .alarm      = { 55,215,225},  // #C8281E
    .info       = { 99, 71, 55},  // #9CB8C8
    .windsch    = {213,197,189},  // #2A3A42
    .bootlogo   = { 55, 65,105},  // #C8BE96
    .gzeiger    = { 15, 23, 51},  // #F0E8CC
    .gwarn      = { 31, 90,209},  // #E0A52E
    .glimit     = { 55,215,225},  // #C8281E
    .mtext      = { 10, 15, 35},  // #F5F0DC
    .mzweig     = { 99, 71, 55},  // #9CB8C8
    .mwert      = {113,120,151},  // #8E8768
    .himmel     = {160,112, 71},  // #5F8FB8
    .erde       = {198,216,240},  // #39270F
};  // Vintage

static const skin_t skin_luftfahrt = {
    .bg         = {245,239,199},  // #0A1038
    .flaeche    = {233,222,163},  // #16215C
    .haupt      = {  0,  0,  0},  // #FFFFFF
    .neben      = { 96, 79, 47},  // #9FB0D0
    .s2fpf      = { 96, 79, 47},  // #9FB0D0
    .label      = {145,127, 87},  // #6E80A8
    .kante      = {172,193,214},  // #533E29
    .skalz      = {  0, 76,191},  // #FFB340
    .zeiger     = {  0,  0,  0},  // #FFFFFF
    .steigen    = {  0, 55,185},  // #FFC846
    .optimal    = {134,134,134},  // #797979
    .warnung    = {  0,117,225},  // #FF8A1E
    .alarm      = { 31,197,209},  // #E03A2E
    .info       = {112, 87, 39},  // #8FA8D8
    .windsch    = {227,210,121},  // #1C2D86
    .bootlogo   = { 18, 94,245},  // #EDA10A
    .gzeiger    = {  0,  0,  0},  // #FFFFFF
    .gwarn      = {  0,117,225},  // #FF8A1E
    .glimit     = { 31,197,209},  // #E03A2E
    .mtext      = {  0,  0,  0},  // #FFFFFF
    .mzweig     = {112, 87, 39},  // #8FA8D8
    .mwert      = {145,127, 87},  // #6E80A8
    .himmel     = {251,105,  0},  // #0496FF
    .erde       = {101,184,239},  // #9A4710
};  // Luftfahrt

static const skin_t skin_rieti_rossa = {
    .bg         = {255,255,255},  // #000000
    .flaeche    = {227,227,227},  // #1C1C1C
    .haupt      = {  0,  0,  0},  // #FFFFFF
    .neben      = { 55, 55, 55},  // #C8C8C8
    .s2fpf      = { 55, 55, 55},  // #C8C8C8
    .label      = {117,117,117},  // #8A8A8A
    .kante      = {173,186,255},  // #524500
    .skalz      = { 95, 95, 95},  // #A0A0A0
    .zeiger     = {  0, 39,255},  // #FFD800
    .steigen    = { 39,248,225},  // #D8071E
    .optimal    = {255, 95,195},  // #00A03C
    .warnung    = {  0, 45,255},  // #FFD200
    .alarm      = {  0,148,255},  // #FF6B00
    .info       = { 87, 75, 55},  // #A8B4C8
    .windsch    = {165,235,223},  // #5A1420
    .bootlogo   = {  0, 45,255},  // #FFD200
    .gzeiger    = {  0,  0,  0},  // #FFFFFF
    .gwarn      = {  0, 45,255},  // #FFD200
    .glimit     = {  0,148,255},  // #FF6B00
    .mtext      = {  0,  0,  0},  // #FFFFFF
    .mzweig     = { 87, 75, 55},  // #A8B4C8
    .mwert      = {117,117,117},  // #8A8A8A
    .himmel     = {181,128, 75},  // #4A7FB4
    .erde       = {197,213,229},  // #3A2A1A
};  // Rieti Rossa

static const skin_t skin_gold_badge = {
    .bg         = {251,234,242},  // #04150D
    .flaeche    = {241,213,229},  // #0E2A1A
    .haupt      = { 18, 40,101},  // #EDD79A
    .neben      = { 38, 88,190},  // #D9A741
    .s2fpf      = { 38, 88,190},  // #D9A741
    .label      = { 87,133,215},  // #A87A28
    .kante      = {143,182,229},  // #70491A
    .skalz      = { 16, 52,148},  // #EFCB6B
    .zeiger     = {  0,  9, 29},  // #FFF6E2
    .steigen    = {  4, 24, 94},  // #FBE7A1
    .optimal    = {225,133,181},  // #1E7A4A
    .warnung    = { 23,149,239},  // #E86A10
    .alarm      = { 65,211,222},  // #BE2C21
    .info       = {128, 79,101},  // #7FB09A
    .windsch    = {179,207,243},  // #4C300C
    .bootlogo   = { 16, 52,148},  // #EFCB6B
    .gzeiger    = {  4, 16, 41},  // #FBEFD6
    .gwarn      = { 23,149,239},  // #E86A10
    .glimit     = { 65,211,222},  // #BE2C21
    .mtext      = {  4, 16, 41},  // #FBEFD6
    .mzweig     = {128, 79,101},  // #7FB09A
    .mwert      = { 38, 88,190},  // #D9A741
    .himmel     = {193,133, 97},  // #3E7A9E
    .erde       = {204,215,229},  // #33281A
};  // Gold Badge

static const skin_t skin_vol_a_voile = {
    .bg         = {247,241,209},  // #080E2E
    .flaeche    = {235,219,167},  // #142458
    .haupt      = {  0,  0,  0},  // #FFFFFF
    .neben      = {145,100, 23},  // #6E9BE8
    .s2fpf      = {145,100, 23},  // #6E9BE8
    .label      = {119,103, 75},  // #8898B4
    .kante      = {171,225,207},  // #541E30
    .skalz      = { 87, 71, 31},  // #A8B8E0
    .zeiger     = {  9,190,202},  // #F64135
    .steigen    = {128, 71,  0},  // #7FB8FF
    .optimal    = {244,144, 47},  // #0B6FD0
    .warnung    = {  0, 89,209},  // #FFA62E
    .alarm      = { 35,245,245},  // #DC0A0A
    .info       = {112, 75, 15},  // #8FB4F0
    .windsch    = {225,203,119},  // #1E3488
    .bootlogo   = {  9,190,202},  // #F64135
    .gzeiger    = {  0,  0,  0},  // #FFFFFF
    .gwarn      = {  0, 89,209},  // #FFA62E
    .glimit     = { 35,245,245},  // #DC0A0A
    .mtext      = {  0,  0,  0},  // #FFFFFF
    .mzweig     = {112, 75, 15},  // #8FB4F0
    .mwert      = {117,103, 63},  // #8A98C0
    .himmel     = {251,105,  0},  // #0496FF
    .erde       = {101,184,239},  // #9A4710
};  // Vol a Voile

static const skin_t skin_benalla = {
    .bg         = {227,235,207},  // #1C1430
    .flaeche    = {203,215,179},  // #34284C
    .haupt      = { 11, 19, 11},  // #F4ECF4
    .neben      = { 55, 71, 31},  // #C8B8E0
    .s2fpf      = { 55, 71, 31},  // #C8B8E0
    .label      = {111,123, 79},  // #9084B0
    .kante      = {167,191,179},  // #58404C
    .skalz      = { 39, 59, 35},  // #D8C4DC
    .zeiger     = { 31, 87,111},  // #E0A890
    .steigen    = { 11,115, 83},  // #F48CAC
    .optimal    = {191, 79, 95},  // #40B0A0
    .warnung    = { 15, 71,159},  // #F0B860
    .alarm      = { 39,207,163},  // #D8305C
    .info       = { 99, 75, 23},  // #9CB4E8
    .windsch    = {179,207,143},  // #4C3070
    .bootlogo   = { 11,115, 83},  // #F48CAC
    .gzeiger    = { 11, 19, 11},  // #F4ECF4
    .gwarn      = { 15, 71,159},  // #F0B860
    .glimit     = { 39,207,163},  // #D8305C
    .mtext      = { 11, 19, 11},  // #F4ECF4
    .mzweig     = { 99, 75, 23},  // #9CB4E8
    .mwert      = { 55, 71, 31},  // #C8B8E0
    .himmel     = {167,147, 63},  // #586CC0
    .erde       = {179,207,191},  // #4C3040
};  // Benalla

// -- Preset table (index == NVS color_scheme value) ------------------------

const skin_t* const SKIN_PRESETS[SKIN_COUNT] = {
    &skin_cross_country,  // 0
    &skin_grass_strip,    // 1
    &skin_energy_line,    // 2
    &skin_vintage,        // 3
    &skin_luftfahrt,      // 4
    &skin_rieti_rossa,    // 5
    &skin_gold_badge,     // 6
    &skin_vol_a_voile,    // 7
    &skin_benalla,        // 8
};

const char* const SKIN_NAME[SKIN_COUNT] = {
    "Cross Country",
    "Grass Strip",
    "Energy Line",
    "Vintage",
    "Luftfahrt",
    "Rieti Rossa",
    "Gold Badge",
    "Vol a Voile",
    "Benalla",
};

// -- Apply a skin by index -------------------------------------------------

void skin_apply(int index) {
    if (index < 0 || index >= SKIN_COUNT)
        index = 0;
    g_skin = SKIN_PRESETS[index];
    MYUCG->setColor(1, COLOR_BLACK);
}
