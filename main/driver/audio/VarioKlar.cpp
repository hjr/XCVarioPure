#include "VarioKlar.h"
#ifndef DRAM_ATTR
#define DRAM_ATTR
#endif
#include "VarioKlarTables.h"
#include <cmath>
#include <cstring>
#include <algorithm>


namespace {
constexpr float F_REF = 62.5f;   // Bezugston für die log-Tonhöhe (alle Töne liegen darüber)
constexpr float PI_F = 3.14159265f;
// Spitzenpegel der Hüllkurve: bei voller DAC-Aussteuerung 126/128, damit Spitze + Rundung nie über ±127 gehen
constexpr int32_t ENV_TOP = VK_DAC_FULL ? 32256 : 32767;
// Glättung von Fahrt x Gleichstand im Interrupt: Zeitkonstante 50 ms (glättet die 10-Hz-Schritte des Tasks)
constexpr int32_t VG_A = (int32_t)(0.0192f * 65536.f);   // 1 - exp(-64 / (66150 * 0.05))

inline __attribute__((always_inline)) int32_t rc(uint32_t x)          // x: 0..65535  ->  0..32767
{
    const uint32_t i = x >> 10, f = x & 1023;
    return vk::RC_Q15[i] + (((vk::RC_Q15[i + 1] - vk::RC_Q15[i]) * (int32_t)f) >> 10);
}
inline float ss(float x) { x = std::min(1.f, std::max(0.f, x)); return x * x * (3.f - 2.f * x); }
inline float cl01(float x) { return std::min(1.f, std::max(0.f, x)); }
inline float lerpLut(const uint16_t *t, int i, float f) { return t[i] + (t[i + 1] - t[i]) * f; }
inline float lerpLut(const uint8_t *t, int i, float f) { return t[i] + (t[i + 1] - t[i]) * f; }
int32_t coefT90(float t90)    // Glättung je CHUNK für "90 % nach t90 Sekunden" (t90 = 2,3 Zeitkonstanten)
{
    if (t90 < 0.002f) return 65536;
    const float tau = t90 / 2.302585f;
    return (int32_t)std::max(1.f, (1.f - std::exp(-(float)VarioKlar::CHUNK / (VarioKlar::SAMPLE_RATE * tau))) * 65536.f);
}
} // namespace

// Klar (Ton aus Fassung 2): die unveränderten Tabellen aus VarioKlarTables.h. Ohne eigene Hüllkurventabelle und ohne Pieps-Form,
// damit rechnet update() genau wie Fassung 2.
const VkVoice VK_VOICE_KLAR = {
    "Klar", vk::HARM_CLIMB, vk::HARM_SINK, vk::HARM_GLANZ, vk::C_HZ_Q2, vk::C_RATE_Q10, vk::C2_RATE_Q10,
    vk::C_FLOOR_Q8, vk::C_DUTY_Q8, vk::C_GAIN_Q7, vk::C2_FLOOR_Q8, vk::C2_DUTY_Q8, vk::C2_GAIN_Q7, nullptr,
    vk::ATT_S, vk::ATT_S_CONT, vk::REL_RATIO, vk::TREND_ST, vk::GL_START, vk::GL_MAX, vk::B_IN,
    0.f, 0.f, 0.f, 1.f, true, false, 1.f, (uint8_t)vk::NF, (uint8_t)vk::NH, (uint16_t)vk::LUT_N };

bool VarioKlar::setVoice(const VkVoice *voice)
{
    if (!voice || voice->nf != vk::NF || voice->nh != vk::NH || voice->lutN != vk::LUT_N) return false;
    if (voice == voice_) { pending_ = nullptr; return true; }
    if (!started_ || gate_ == 0) applyVoice(voice);   // vor dem ersten Wert oder bei stummem Ton: sofort
    else pending_ = voice;                              // sonst kurz ausblenden (update), dann wechseln: kein Knacks
    return true;
}

void VarioKlar::applyVoice(const VkVoice *voice)
{
    voice_ = voice; pending_ = nullptr; fadeOut_ = false;
    std::memset(aLast_, 0, sizeof(aLast_)); waveE_ = 0.f;    // Wellenform beim nächsten update() neu
}

void VarioKlar::init()
{
    using namespace vk;
    incRef_ = (uint32_t)(F_REF / SAMPLE_RATE * 4294967296.0);
    cur_.store(0); fresh_.store(false);
    ph_ = bph_ = 0; env_ = denv_ = envEnd_ = gate_ = err_ = 0; left_ = 0; rz_ = vg_ = 32767 << 8;
    setCalmZone(RZ_ATT_DB, RZ_WIDTH, RZ_T_DOWN, RZ_T_UP);
    s2fADn_ = coefT90(S2F_T_DOWN); s2fAUp_ = coefT90(S2F_T_UP);
    setSpeedAdapt(SPD_QUIET_DB, BRIGHT_ST);
    setHabituation(HAB_DEPTH_DB, HAB_START_S);
    started_ = false; waveE_ = 0.f; pending_ = nullptr; fadeOut_ = false;
    update(0.f, 0.1f, false, Mode::Vario, 0.f, false, 0.f);   // Startzustand in beide Puffer
    ctl_[0] = ctl_[1]; std::memcpy(wbuf_[0], wbuf_[1], sizeof(wbuf_[0]));
    fresh_.store(false);
    cp_ = &ctl_[0]; wt_ = wbuf_[0]; lp_ = ctl_[0].lp;
    started_ = false; snap_ = true;     // der erste echte Wert setzt Glättungen und Verstärkungen direkt
    zSide_ = 0; zQuiet_ = zGap_ = 0.f; habInit_ = false; habG_ = 1.f;
}

// ------------------------------------------------------------------ Audio-Task
void VarioKlar::setCalmZone(float attDb, float width, float tDown90, float tUp90)
{
    rzAtt_ = std::max(0.f, attDb); rzWid_ = std::max(0.05f, width);
    rzADn_ = coefT90(tDown90); rzAUp_ = coefT90(tUp90);
}

void VarioKlar::setPolar(float v1, float w1, float v2, float w2, float v3, float w3)
{
    // Parabel durch die drei Polarpunkte, Fahrt bei 1,5 m/s Sinken. Bezug: DG 800S/18 (159 km/h).
    // Nur nach unten (langsame Flugzeuge), höchstens bis POLAR_F_MIN. Ungültige Polare: Faktor 1.
    using namespace vk;
    float f = 1.f;
    const float d12 = v1 - v2, d13 = v1 - v3, d23 = v2 - v3;
    if (std::fabs(d12) > 1.f && std::fabs(d13) > 1.f && std::fabs(d23) > 1.f) {
        const float l1 = w1 / (d12 * d13), l2 = w2 / (-d12 * d23), l3 = w3 / (d13 * d23);
        const float a2 = l1 + l2 + l3, a1 = -(l1 * (v2 + v3) + l2 * (v1 + v3) + l3 * (v1 + v2));
        const float a0 = l1 * v2 * v3 + l2 * v1 * v3 + l3 * v1 * v2 + 1.5f;
        const float D = a1 * a1 - 4.f * a2 * a0;
        if (a2 < 0.f && D >= 0.f) {
            const float r1 = (-a1 + std::sqrt(D)) / (2.f * a2), r2 = (-a1 - std::sqrt(D)) / (2.f * a2);
            const float v15 = std::max(r1, r2);
            if (v15 > 40.f && v15 < 400.f) f = std::min(1.f, std::max(POLAR_F_MIN, v15 / POLAR_V15_REF));
        }
    }
    polF_ = f;
}

void VarioKlar::update(float v, float dt, bool on, Mode mode, float netto, bool calm, float iasKmh)
{
    using namespace vk;
    // Tonwechsel: ein Aufruf lang ausblenden (25 ms), beim nächsten mit dem neuen Ton weich einblenden
    if (pending_) {
        if (fadeOut_ || !on) applyVoice(pending_);
        else { fadeOut_ = true; on = false; }
    }
    dt = std::min(0.5f, std::max(0.01f, dt));
    const bool s2f = mode != Mode::Vario, cont = mode == Mode::S2fCont;
    if (mode != mode_) { zSide_ = 0; zQuiet_ = zGap_ = 0.f; habInit_ = false; habG_ = 1.f; mode_ = mode; }
    v = s2f ? std::min(6.f, std::max(-6.f, v)) : std::min(10.f, std::max(-6.f, v));

    // ---- Zustand, der bei jedem Aufruf weiterläuft
    if (!started_) { vs_ = vsPrev_ = v; dvs_ = 0.f; started_ = true; }
    vs_ += (1.f - std::exp(-dt / VS_TAU)) * (v - vs_);
    dvs_ += (1.f - std::exp(-dt / DVS_TAU)) * ((vs_ - vsPrev_) / dt - dvs_);
    vsPrev_ = vs_;

    // Fahrt: langsam leiser, schnell heller (Eckwerte x Polarfaktor), 1 s geglättet
    {
        const float f = polF_, ias = std::max(0.f, iasKmh);
        const float quiet = voice_->slowQuiet ? spdQuietDb_ : 0.f;   // OptClassic: nicht leiser beim Langsamfliegen
        const float spdT = std::pow(10.f, -quiet * cl01((SPD_V_HI * f - ias) / ((SPD_V_HI - SPD_V_LO) * f)) / 20.f);
        const float brT = brightSt_ * cl01((ias - BRIGHT_V_LO * f) / ((BRIGHT_V_HI - BRIGHT_V_LO) * f));
        const float a = snap_ ? 1.f : 1.f - std::exp(-dt / 1.0f);
        spdG_ += (spdT - spdG_) * a; bright_ += (brT - bright_) * a;
    }
    // Gleichstand leiser (nur Vario): 5-s-Mittel bleibt nahe am Bezug -> nach HAB_START_S langsam leiser
    if (!s2f && habDepthDb_ > 0.f) {
        if (!habInit_) { habAvg_ = habRef_ = v; habT_ = 0.f; habInit_ = true; }
        habAvg_ += (1.f - std::exp(-dt / HAB_AVG_S)) * (v - habAvg_);
        if (std::fabs(habAvg_ - habRef_) > HAB_THR) { habRef_ = habAvg_; habT_ = 0.f; } else habT_ += dt;
        const float tg = std::pow(10.f, -habDepthDb_ * cl01((habT_ - habStartS_) / HAB_RAMP_S) / 20.f);
        habG_ = tg < habG_ ? tg : habG_ + (tg - habG_) * (1.f - std::exp(-dt / HAB_UP_TAU));
    } else { habG_ = 1.f; habInit_ = false; }

    // Zone: im Vario-Modus Ruhezone um Netto 0, in der Sollfahrt Totzone mit Pause beim Seitenwechsel
    float g = 1.f;
    if (s2f) {
        const float a = std::fabs(v), x = std::max(0.f, a - S2F_INNER) / std::max(S2F_OUTER - S2F_INNER, 0.01f);
        if (x < 1.f) g = 0.5f - 0.5f * std::cos(PI_F * x);                 // innen still, weicher Rand
        const int8_t side = a > S2F_INNER ? (v > 0.f ? 1 : -1) : 0;
        if (side == 0) zQuiet_ += dt;
        else { if (zSide_ != 0 && side != zSide_ && zQuiet_ < S2F_GAP) zGap_ = S2F_GAP - zQuiet_; zSide_ = side; zQuiet_ = 0.f; }
        if (zGap_ > 0.f) { zGap_ -= dt; g = 0.f; }
    } else if (calm && rzAtt_ > 0.f) {
        const float x = std::fabs(netto) / rzWid_;                          // halber Kosinus bis |Netto| = Breite
        if (x < 1.f) g = std::pow(10.f, -rzAtt_ * 0.5f * (1.f + std::cos(PI_F * x)) / 20.f);
    }

    if (fresh_.load(std::memory_order_acquire)) return;   // Interrupt hat den letzten Satz noch nicht geholt

    // ---- Kurven des gewählten Tons (0,1-Tabelle, linear interpoliert); Satz 1 für Sollfahrt durchgehend
    const VkVoice &V = *voice_;
    const uint16_t *tR = cont ? V.rate2 : V.rate;
    const uint8_t *tF = cont ? V.floor2 : V.floor, *tD = cont ? V.duty2 : V.duty, *tG = cont ? V.gain2 : V.gain;
    float x = (v - LUT_V0) / LUT_DV; int i = (int)x; if (i > LUT_N - 2) i = LUT_N - 2; if (i < 0) i = 0; const float fr = x - i;
    const float hz = lerpLut(V.hz, i, fr) * 0.25f;
    const float rate = lerpLut(tR, i, fr) * (1.f / 1024.f);
    const float flo = lerpLut(tF, i, fr) * (1.f / 256.f);
    const float duty = lerpLut(tD, i, fr) * (1.f / 256.f);
    const float gain = lerpLut(tG, i, fr) * (1.f / 128.f);

    // Hüllkurvenform: beim Sinken weicher Atem (Anschwellen bIn, Abschwellen Rest), beim Steigen Pieps.
    // Übergang Atem -> Pieps aus der Tabelle des Tons (env) oder wie Klar zwischen -0,3 und +0,4 m/s.
    const float t01 = (!cont && V.env) ? lerpLut(V.env, i, fr) * (1.f / 255.f) : ss((v + 0.3f) / 0.7f);
    float e = (1.f - t01) * V.bIn + t01 * std::min((cont ? V.attSCont : V.attS) * rate, 0.2f);
    float er = (1.f - t01) * (1.f - V.bIn) + t01 * std::min(e * V.relRatio, 0.35f);
    // Pieps-Form des Tons (Kosinus hinein / hinaus in festen Zeiten), weich eingeblendet: ab halbem Übergang und ab shV0 m/s
    if (!cont && V.shAttS > 0.f) {
        const float wSh = ss((t01 - 0.25f) / 0.5f) * ss((v - V.shV0) / (V.shV1 - V.shV0));
        if (wSh > 0.f) {
            e += (std::min(V.shAttS * rate, 0.45f) - e) * wSh;
            er += (std::min(V.shRelS * rate, 0.45f) - er) * wSh;
        }
    }
    const float d = std::min(duty, 1.f - er);
    const float trend = cont ? 0.f : std::min(TREND_MAX_ST, std::max(0.f, dvs_ * V.trendSt)) / 12.f;   // Oktaven

    // Klangfarbe: Obertonsatz der Tonhöhe, zwischen Sinken / Steigen / Glanz gemischt.
    // Bei Fahrt bekommt der Sinkanteil den Satz einer tieferen Tonhöhe (mehr Obertöne, gleiche Tonhöhe).
    const float mix = ss((vs_ + 1.5f) / 1.5f), gl = V.glMax * ss((vs_ - V.glStart) / GL_SPAN);
    float pos = (std::log2(hz / F_LO) - bright_ * (1.f - mix) / 12.f) / F_OCT * (NF - 1);
    pos = std::min((float)(NF - 1), std::max(0.f, pos));
    int t = (int)pos; if (t > NF - 2) t = NF - 2; const float tf = pos - t;
    const float gH = gain * V.harmGain;
    const float wS = gH * (1.f - mix), wC = gH * mix * (1.f - gl), wG = gH * mix * gl;
    float a[NH]; float dmax = 0.f, amax = 1e-6f;
    for (int k = 0; k < NH; k++) {
        const int i0 = t * NH + k, i1 = i0 + NH;
        a[k] = wS * (V.harmSink[i0] + (V.harmSink[i1] - V.harmSink[i0]) * tf)
             + wC * (V.harmClimb[i0] + (V.harmClimb[i1] - V.harmClimb[i0]) * tf)
             + wG * (V.harmGlanz[i0] + (V.harmGlanz[i1] - V.harmGlanz[i0]) * tf);
        dmax = std::max(dmax, std::fabs(a[k] - aLast_[k])); amax = std::max(amax, std::fabs(a[k]));
    }
    const uint8_t back = cur_.load(std::memory_order_relaxed) ^ 1;
    // Pegelausgleich beim Wellenformtausch (gainGlide): Die neue Wellenform trägt den neuen Pegel. Der Interrupt übernimmt sie mit
    // dem Verhältnis alt / neu in der Verstärkung vg_ und gleitet dann in 50 ms auf das Ziel: keine Pegelsprünge im 10-Hz-Takt bei Böen.
    float gRatio = 1.f;
    if (dmax > 0.002f * amax) {
        buildWave(wbuf_[back], a); std::memcpy(aLast_, a, sizeof(a));
        if (V.gainGlide) {   // Verhältnis über die Energie der Obertöne (Pegel und Klangwechsel zusammen)
            float E = 0.f; for (int k = 0; k < NH; k++) E += a[k] * a[k];
            if (waveE_ > 0.f && E > 0.f && !snap_) gRatio = std::min(4.f, std::max(0.25f, std::sqrt(waveE_ / E)));
            waveE_ = E;
        }
    }
    else std::memcpy(wbuf_[back], wbuf_[back ^ 1], sizeof(wbuf_[0]));   // unverändert übernehmen

    // Steuerwerte für den Interrupt
    Ctl &c = ctl_[back];
    c.lp = (int32_t)std::lround(std::log2(hz / F_REF) * 65536.f);
    c.binc = (uint32_t)(rate / SAMPLE_RATE * CHUNK * 4294967296.0);
    c.d16 = (uint32_t)(d * 65536.f);
    c.invEr = (uint32_t)std::min(4294967295.0, 65536.0 * 65536.0 / std::max(er * 65536.f, 1.f));
    c.invD = (uint32_t)std::min(4294967295.0, 65536.0 * 65536.0 / std::max(d * 65536.f, 1.f));
    c.invE = (uint32_t)std::min(4294967295.0, 65536.0 * 65536.0 / std::max(e * 65536.f, 1.f));
    c.floor15 = (int32_t)(flo * 32767.f);
    c.trend = (int32_t)(trend * 65536.f);
    c.glideA = (int32_t)((1.f - std::exp(-(float)CHUNK / (SAMPLE_RATE * GLIDE_S))) * 65536.f);
    c.gateTarget = on ? 32767 : 0;
    c.rzTarget = (int32_t)(g * 32767.f);
    c.rzAUp = s2f ? s2fAUp_ : rzAUp_; c.rzADn = s2f ? s2fADn_ : rzADn_;
    c.volTarget = (int32_t)(std::min(1.f, spdG_ * habG_) * 32767.f);
    c.gRatio = (uint32_t)(gRatio * 65536.f);
    c.sub = cont && t01 > 0.5f ? 1 : 0;
    c.snap = snap_ ? 1 : 0;
    snap_ = false;
    fresh_.store(true, std::memory_order_release);
}

void VarioKlar::buildWave(int16_t *w, const float *a)
{
    using namespace vk;
    for (int j = 0; j < WT_LEN; j++) {
        float acc = 0.f;
        for (int k = 0; k < NH; k++) acc += a[k] * SIN_Q15[((k + 1) * j + HPHASE[k]) & (WT_LEN - 1)];
        int s = (int)std::lround(acc * (1.f / 32768.f));
        w[j] = (int16_t)std::min(32767, std::max(-32767, s));
    }
    w[WT_LEN] = w[0];
}

// ------------------------------------------------------------------ DMA-Interrupt (nur Ganzzahlen)
void IRAM_ATTR VarioKlar::render(uint8_t *p, int n, int stride)
{
    while (n > 0) {
        if (left_ == 0) chunk();
        int m = left_ < n ? left_ : n;
        left_ -= m; n -= m;
        uint32_t ph = ph_; const uint32_t inc = inc_;
        int32_t env = env_, err = err_; const int32_t denv = denv_;
        const int16_t *wt = wt_;
        for (; m > 0; m--) { *p = (uint8_t)(127 + sample(ph, inc, env, denv, err, wt)); p += stride; }
        ph_ = ph; env_ = env; err_ = err;
    }
}

void IRAM_ATTR VarioKlar::chunk()
{
    left_ = CHUNK;
    if (fresh_.load(std::memory_order_acquire)) {          // neuen Satz vom Task übernehmen
        const uint8_t n = cur_.load(std::memory_order_relaxed) ^ 1;
        cp_ = &ctl_[n]; wt_ = wbuf_[n];
        cur_.store(n, std::memory_order_relaxed);
        fresh_.store(false, std::memory_order_release);
        if (cp_->snap) { rz_ = cp_->rzTarget << 8; vg_ = cp_->volTarget << 8; }   // Start: kein Ein- oder Ausblenden
        else if (cp_->gRatio != 65536) vg_ = (int32_t)(((int64_t)vg_ * cp_->gRatio) >> 16);   // Pegel der neuen Wellenform ausgleichen
    }
    const Ctl &c_ = *cp_;
    // Tonhöhe weich nachführen (log-Bereich)
    lp_ += (int32_t)(((int64_t)(c_.lp - lp_) * c_.glideA) >> 16);
    // Pieps-Phase am Ende dieses Abschnitts
    bph_ += c_.binc;
    const uint32_t p16 = bph_ >> 16;
    int32_t pu, r16;
    if (p16 < c_.d16) {
        pu = rc((uint32_t)std::min<uint64_t>(65535, ((uint64_t)p16 * c_.invE) >> 16));
        r16 = (int32_t)std::min<uint64_t>(65535, ((uint64_t)p16 * c_.invD) >> 16);
        if (c_.sub) {                                       // Doppelpieps: Lücke in der Mitte des Pieps
            const int32_t x16 = r16 > 32768 ? r16 - 32768 : 32768 - r16;   // Abstand zur Mitte, Q16
            if (x16 <= 5243) pu = 0;                                        // ±8 % still
            else if (x16 < 10486) pu = (pu * rc((uint32_t)(((x16 - 5243) * 25) >> 1))) >> 15;   // bis ±16 % weich
        }
    } else {
        pu = 32767 - rc((uint32_t)std::min<uint64_t>(65535, ((uint64_t)(p16 - c_.d16) * c_.invEr) >> 16));
        r16 = 65535;
    }
    // Ein-/Ausblenden (~25 ms)
    if (gate_ < c_.gateTarget) gate_ = std::min(c_.gateTarget, gate_ + 1311);
    else if (gate_ > c_.gateTarget) gate_ = std::max(c_.gateTarget, gate_ - 1311);
    // Ruhezone / Totzone: eigene Zeiten für lauter und leiser (Q23, eine 64-Bit-Multiplikation je Abschnitt)
    const int32_t rzT = c_.rzTarget << 8;
    rz_ += (int32_t)(((int64_t)(rzT - rz_) * (rzT > rz_ ? c_.rzAUp : c_.rzADn)) >> 16);
    // Fahrt x Gleichstand: 50 ms Glättung
    vg_ += (int32_t)(((int64_t)((c_.volTarget << 8) - vg_) * VG_A) >> 16);
    int32_t env15 = c_.floor15 + (((32767 - c_.floor15) * pu) >> 15);
    env15 = (env15 * gate_) >> 15;
    env15 = (env15 * (rz_ >> 8)) >> 15;
    env15 = (env15 * std::min(vg_ >> 8, (int32_t)65535)) >> 15;   // vg_ kann nach einem Pegelausgleich kurz über 1 liegen (Produkt bleibt in 32 Bit)
    if (env15 > 32767) env15 = 32767;                       // nie über Vollausschlag
    env15 = (env15 * ENV_TOP) >> 15;
    // Hüllkurve linear über den Abschnitt
    env_ = envEnd_;
    envEnd_ = env15 << 7;
    denv_ = (envEnd_ - env_) >> CHUNK_BITS;
    // Phaseninkrement aus log-Tonhöhe + Trend
    const int32_t lpe = lp_ + ((c_.trend * r16) >> 16);
    const uint32_t oct = (uint32_t)lpe >> 16, f6 = ((uint32_t)lpe >> 10) & 63, f10 = (uint32_t)lpe & 1023;
    const uint32_t e0 = vk::EXP2_Q30[f6], e1 = vk::EXP2_Q30[f6 + 1];
    const uint32_t ex = e0 + ((((e1 - e0) >> 4) * f10) >> 6);
    inc_ = (uint32_t)(((uint64_t)(incRef_ << oct) * ex) >> 30);   // incRef_ << 5 passt in 32 Bit
}
