// VarioKlar - sparsamer Varioton "Klar" für den XCVario (ESP32, 8-Bit-DAC), Fassung 2 (Stand 16.10.2026)
//
// Ein Tonerzeuger für Vario und Sollfahrt:
//  - Vario: Klar-Kurven bis +10 m/s, Atem beim Sinken, Ruhezone um Netto 0, "Gleichstand leiser".
//  - Sollfahrt: dieselben Kurven mit 10 km/h = 1 Einheit, wahlweise piepsend oder durchgehend
//    (Teppich mit ruhigem Doppelpieps). Totzone mit weichem Rand und Pause beim Seitenwechsel.
//  - Fahrt: leiser beim Langsamfliegen, Sinkton heller bei Fahrt. Eckwerte nach der eingestellten Polare.
//
// Aufteilung der Arbeit:
//  - update()  läuft im Audio-Task (Gleitkomma erlaubt), etwa 10x pro Sekunde mit dem neuen Wert.
//              Rechnet Kurven, Hüllkurvenform, Lautstärke und eine fertige Wellenform (257 Werte) für die
//              aktuelle Tonhöhe und Klangfarbe und übergibt sie per Doppelpuffer an den Interrupt.
//  - render()  läuft im DMA-Interrupt, nur Ganzzahlen. Pro Abtastwert: Phasenakkumulator, eine Tabellen-
//              lesung mit linearer Interpolation, Hüllkurvenrampe. Alle 64 Abtastwerte (~1 ms) rechnet
//              chunk() Hüllkurve, Doppelpieps-Lücke, Tonhöhengleiten, Trend und Lautstärken neu.
//
// Speicher: ca. 1,3 kB RAM (2 Wellenformen à 514 Byte + Zustand), ca. 7 kB Flash (Tabellen).
#pragma once
#include <cstdint>
#include <atomic>
#if __has_include(<esp_attr.h>)
#include <esp_attr.h>     // IRAM_ATTR, DRAM_ATTR (ESP-IDF)
#endif

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

// DAC-Aussteuerung: 1 = volle ±127 Stufen (6 dB mehr Abstand zum Rundungsrauschen), 0 = ±64 wie 2.0.7.
// Vor dem Umstellen prüfen, ob der Verstärkereingang den doppelten Pegel verträgt.
#ifndef VK_DAC_FULL
#define VK_DAC_FULL 1
#endif

class VarioKlar {
public:
    static constexpr int SAMPLE_RATE = 66150;    // muss zu ESPAudio.cpp passen
    static constexpr int CHUNK_BITS = 6;
    static constexpr int CHUNK = 1 << CHUNK_BITS; // Abtastwerte je Steuerschritt
    static constexpr int WT_LEN = 256;           // Wellenform, Zweierpotenz
    static constexpr int DAC_AMPL = VK_DAC_FULL ? 127 : 64;

    enum class Mode : uint8_t {
        Vario = 0,      // Wert = Vario in m/s
        S2fBeep = 1,    // Sollfahrt, zu schnell piepst wie Steigen (Chopping an)
        S2fCont = 2     // Sollfahrt, zu schnell durchgehend mit ruhigem Doppelpieps (Chopping aus)
    };

    void init();

    // Audio-Task, etwa 10x pro Sekunde.
    //  v       Vario in m/s, im Sollfahrt-Modus Abweichung in 10 km/h (+ = zu schnell), wie audio_value in ESPAudio
    //  dt      Sekunden seit dem letzten Aufruf
    //  on      false blendet weich aus (Lautstärke 0, Sinkton stumm, Vario-Totzone)
    //  mode    Vario / Sollfahrt piepsend / Sollfahrt durchgehend
    //  netto   Netto-Vario (te_netto), für die Ruhezone
    //  calm    true, wenn die Ruhezone gelten soll (Kreisflugmodus)
    //  iasKmh  angezeigte Fahrt in km/h (0 = unbekannt: wie langsam)
    void update(float v, float dt, bool on, Mode mode, float netto, bool calm, float iasKmh);

    // Polare (drei Punkte wie im Setup: km/h und m/s, Sinken negativ). Passt die Fahrt-Eckwerte an:
    // langsame Flugzeuge (Ka 8, ASK 13) werden früher voll laut. Einmal beim Start und nach jeder Änderung aufrufen.
    void setPolar(float v1, float w1, float v2, float w2, float v3, float w3);
    float polarFactor() const { return polF_; }

    // Ruhezone: attDb Absenkung in der Mitte, width Netto-Abstand (m/s), bei dem sie ganz aufhört,
    // tDown90 / tUp90 Sekunden bis 90 % leiser bzw. wieder lauter. attDb = 0 schaltet sie ab.
    void setCalmZone(float attDb, float width, float tDown90, float tUp90);
    // Fahrt: quietDb Absenkung beim Langsamfliegen, brightSt Helligkeit des Sinktons in Halbtönen (0 = aus)
    void setSpeedAdapt(float quietDb, float brightSt) { spdQuietDb_ = quietDb; brightSt_ = brightSt; }
    // Gleichstand leiser: depthDb Absenkung (0 = aus), startS Sekunden gleichen Werts bis zum Beginn
    void setHabituation(float depthDb, float startS) { habDepthDb_ = depthDb; habStartS_ = startS; }

    // DMA-Interrupt, nur Ganzzahlen. Schreibt n Abtastwerte als DAC-Bytes (127 ± DAC_AMPL) nach p, p+stride, ...
    void IRAM_ATTR render(uint8_t *p, int n, int stride);

    // DMA-Interrupt: einzelner Abtastwert (±DAC_AMPL), falls man lieber pro Abtastwert mischt.
    inline int32_t IRAM_ATTR next() {
        if (left_ == 0) chunk();
        left_--;
        return sample(ph_, inc_, env_, denv_, err_, wt_);
    }

    // true, solange der Ton ganz ausgeblendet ist (dann darf der Verstärker aus)
    bool silent() const { return envEnd_ == 0 && gate_ == 0; }

private:
    static constexpr int OUT_SHIFT = VK_DAC_FULL ? 23 : 24;
    struct Ctl {                  // vom Task an den Interrupt übergeben
        int32_t  lp;              // Ziel-Tonhöhe, log2(f / F_REF) in Q16
        uint32_t binc;            // Pieps-Phase je CHUNK
        uint32_t d16;             // Tonanteil (Q16 der Periode)
        uint32_t invE, invEr, invD; // 2^32 / Anstieg, Abfall, Tonanteil (jeweils Q16)
        int32_t  floor15;         // Grundpegel zwischen den Pieps (Atem, Teppich), Q15
        int32_t  trend;           // Trend-Gleiten, Q16 Oktaven
        int32_t  glideA;          // Glättung der Tonhöhe je CHUNK, Q16
        int32_t  gateTarget;      // 0 oder 32767
        int32_t  rzTarget;        // Ruhezone bzw. Sollfahrt-Totzone: Ziel-Verstärkung Q15
        int32_t  rzAUp, rzADn;    // Glättung je CHUNK beim Lauter- / Leiserwerden, Q16
        int32_t  volTarget;       // Fahrt x Gleichstand: Ziel-Verstärkung Q15
        uint8_t  sub;             // 1 = Doppelpieps (Lücke in der Mitte des Pieps)
        uint8_t  snap;            // 1 = Verstärkungen sofort auf das Ziel (Start)
    };
    void IRAM_ATTR chunk();
    void buildWave(int16_t *w, const float *a);
    static inline int32_t IRAM_ATTR sample(uint32_t &ph, uint32_t inc, int32_t &env, int32_t denv, int32_t &err, const int16_t *wt) {
        ph += inc;
        const uint32_t i = ph >> 24;
        const int32_t fr = (int32_t)((ph >> 16) & 0xFF);
        const int32_t a = wt[i], b = wt[i + 1];
        const int32_t s = a + (((b - a) * fr) >> 8);       // Q15
        env += denv;                                        // Q22
        const int32_t x = s * (env >> 7) + err;             // Q30 (2^30 = 128 bzw. 64 DAC-Stufen)
        const int32_t y = (x + (1 << (OUT_SHIFT - 1))) >> OUT_SHIFT;   // auf DAC-Stufen runden
        err = x - y * (1 << OUT_SHIFT);                     // Rundungsfehler in den nächsten Wert: schiebt
        return y;                                           // das 8-Bit-Rauschen über 10 kHz (Rauschformung)
    }

    // Interrupt-Zustand
    uint32_t ph_ = 0, inc_ = 0, bph_ = 0;
    int32_t  env_ = 0, denv_ = 0, envEnd_ = 0, lp_ = 0, gate_ = 0;
    int32_t  left_ = 0;             // Abtastwerte bis zum nächsten Steuerschritt
    int32_t  err_ = 0;              // Rauschformung (Fehlerrückführung 1. Ordnung)
    int32_t  rz_ = 32767 << 8;      // Ruhezone / Totzone: aktuelle Verstärkung Q23
    int32_t  vg_ = 32767 << 8;      // Fahrt x Gleichstand: aktuelle Verstärkung Q23
    const int16_t *wt_ = nullptr;
    const Ctl *cp_ = nullptr;      // aktiver Steuersatz (wird vom Task nicht beschrieben)

    // Übergabe Task -> Interrupt
    Ctl ctl_[2]{};
    int16_t wbuf_[2][WT_LEN + 1]{};
    std::atomic<uint8_t> cur_{0};
    std::atomic<bool> fresh_{false};

    // Task-Zustand
    float vs_ = 0.f, vsPrev_ = 0.f, dvs_ = 0.f;
    float aLast_[16]{};
    bool started_ = false, snap_ = true;
    Mode mode_ = Mode::Vario;
    uint32_t incRef_ = 0;
    float rzAtt_ = 0.f, rzWid_ = 1.f;
    int32_t rzAUp_ = 65536, rzADn_ = 65536, s2fAUp_ = 65536, s2fADn_ = 65536;
    // Sollfahrt-Totzone: Seite (−1, 0, +1), Stille seit dem Verlassen der Seite, restliche Pause
    int8_t zSide_ = 0; float zQuiet_ = 0.f, zGap_ = 0.f;
    // Fahrt
    float polF_ = 1.f, spdQuietDb_ = 0.f, brightSt_ = 0.f, spdG_ = 1.f, bright_ = 0.f;
    // Gleichstand
    float habDepthDb_ = 0.f, habStartS_ = 60.f, habAvg_ = 0.f, habRef_ = 0.f, habT_ = 0.f, habG_ = 1.f;
    bool habInit_ = false;
};
