/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "SubMenuAudio.h"

#include "driver/audio/ESPAudio.h"
#include "setup/primitive/SetupMenu.h"
#include "setup/primitive/SetupMenuSelect.h"
#include "setup/primitive/SetupMenuValFloat.h"
#include "setup/primitive/SetupMenu.h"
#include "SetupNG.h"
#include "math/Floats.h"
#include "Atmosphere.h"
#include "logdef.h"

#include <algorithm>
#include <cstdint>
#include <string>
#include <string_view>

extern int unitChangeS(SetupMenuSelect* p);

static void audio_menu_create_tonestyle(SetupMenu *top);
static void audio_menu_create_deadband(SetupMenu *top);
static void audio_menu_create_adaptive(SetupMenu *top);

const std::array<std::string_view, 2> adaptive_mode = { "Off", "On" };
static const char *adaptiveName() { return adaptive_mode[audio_adaptive.get() != AUD_ADAPT_OFF ? 1 : 0].data(); }

static std::string range0;
static std::string range1;
static std::string range2;

// audio profiles
struct AUDIO_PROFILE
{
    bool isEqual(const AUDIO_PROFILE &ap) const;
    const char* getName() const { return name.data(); }
    std::string_view name;
    // profile attributes
    float center;
    float maxvariation;
    bool  dual_tone;
    int   chopping;
    int   harmonics;
    float exponent;
    int   voice;      // e_audio_voice
    int   sink;       // e_audio_sink_tone
};
bool AUDIO_PROFILE::isEqual(const AUDIO_PROFILE &ap) const
{
    if (!floatEqualFast(center, ap.center)) return false;
    if (!floatEqualFast(maxvariation, ap.maxvariation)) return false;
    if (dual_tone != ap.dual_tone) return false;
    if (chopping != ap.chopping) return false;
    if (harmonics != ap.harmonics) return false;
    if (!floatEqualFast(exponent, ap.exponent)) return false;
    if (voice != ap.voice) return false;
    if (sink != ap.sink) return false;
    return true;
}
constexpr int NUM_AUDIO_PRFLS = 8;
constexpr int PUR_PRFL = 0;
const std::array<AUDIO_PROFILE, NUM_AUDIO_PRFLS> audio_prfls = {{ 
    // Pur: tone from its own tables, the other values are not used by it
    {"Pur",        520., 1.8, false, BOTH_CHOP, AUD_HARM_HIGH, .7,  AUD_VOICE_PUR,     AUD_SINK_BREATHING },
    {"XCVario",    500., 2.,  false, BOTH_CHOP, AUD_HARM_HIGH, 1.,  AUD_VOICE_PROFILE, AUD_SINK_STEADY },
    {"eXtended",   500., 3.,  false, BOTH_CHOP, AUD_HARM_HIGH, 1.2, AUD_VOICE_PROFILE, AUD_SINK_STEADY },
    {"Ilec",       500., 3.,  true,  BOTH_CHOP, AUD_HARM_MED,  1.2, AUD_VOICE_PROFILE, AUD_SINK_STEADY },
    {"Westerboer", 500., 1.2, false, BOTH_CHOP, AUD_HARM_MED,  1.,  AUD_VOICE_PROFILE, AUD_SINK_STEADY },
    {"Zander",     600., 2.,  false, BOTH_CHOP, AUD_HARM_HIGH, 1.,  AUD_VOICE_PROFILE, AUD_SINK_STEADY },
    {"Borgelt",    500., 1.4, false, BOTH_CHOP, AUD_HARM_LOW, 0.8,  AUD_VOICE_PROFILE, AUD_SINK_STEADY },
    {"Custom",     500., 2.,  false, BOTH_CHOP, AUD_HARM_OFF, 1.,   AUD_VOICE_PROFILE, AUD_SINK_STEADY } }};

static int getAudioProfile()
{
    AUDIO_PROFILE ap = { "", center_freq.get(), tone_var.get(), dual_tone.get() != 0, chopping_mode.get(), audio_harmonics.get(), audio_factor.get(),
        audio_voice.get(), audio_sink_tone.get() };
    int i = 0;
    for (; i < NUM_AUDIO_PRFLS-1; i++) {
        if (audio_prfls[i].isEqual(ap)) {
            break;
        }
    }
    ESP_LOGI(FNAME,"audio profile %d: %s", i, audio_prfls[i].getName() );
    return i;
}
static int applyAudioProfile(SetupMenuSelect *p)
{
    if ( p->getSelect() < NUM_AUDIO_PRFLS-1 ) {
        int8_t profile_idx = p->getSelect();
        // apply profile
        const AUDIO_PROFILE &ap = audio_prfls[profile_idx];
        center_freq.set(ap.center);
        tone_var.set(ap.maxvariation);
        dual_tone.set(ap.dual_tone ? 1 : 0);
        chopping_mode.set(ap.chopping);
        audio_harmonics.set(ap.harmonics);
        audio_factor.set(ap.exponent);
        audio_sink_tone.set(ap.sink);
        audio_voice.set(ap.voice);
        AUDIO->applySetup();
        p->getParent()->setDirty();
    }
    else if ( audio_voice.get() != AUD_VOICE_PROFILE ) {
        // custom: always the tone from the settings
        audio_voice.set(AUD_VOICE_PROFILE);
        AUDIO->applySetup();
        p->getParent()->setDirty();
    }
    return 0;
}

static int update_range_entry(SetupMenuValFloat *p)
{
    ESP_LOGI(FNAME,"update_rentry vu:%s", VarioUnit->getName() );
    range0.assign("Fixed (5  ");
    range0 += VarioUnit->getName();
    range0 += ")";
    range1.assign("Fixed (10 ");
    range1 += VarioUnit->getName();
    range1 += ")";
    range2.assign("Variable (");
    range2 += std::to_string((int)(scale_range.get()));
    range2 += " ";
    range2 += VarioUnit->getName();
    range2 += ")";
    return 0;
}
int update_range_entry_s(SetupMenuSelect *p)
{
    unitChangeS(p);
    return update_range_entry(nullptr);
}

int audio_setup_enable(SetupMenuSelect *p) {
    AUDIO->applySetup();
    if ( p->getValue() == AUDIO_ON ) {
        AUDIO->startSound(AUDIO_CHECK_SOUND);
    }
    return 0;
}
static int show_vol_dflt(SetupMenuValFloat *p) {
    AUDIO->setVolume(p->getNVSVal());
    return 0;
}

void audio_menu_create_tonestyle(SetupMenu *top) { // dynamic!

    int8_t profile_idx = getAudioProfile();
    // update buzzword
    top->setBuzzword(audio_prfls[profile_idx].getName());

    if (top->getNrChilds() == 0)
    {
        top->setDynContent();

        SetupMenuSelect *prfl = new SetupMenuSelect("Preset scheme", RST_NONE, applyAudioProfile);
        prfl->setHelp("Choose one of the predefined audio profiles to preset audio parameters");
        for (const auto &p : audio_prfls) {
            prfl->addEntry(p.getName());
        }
        top->addEntry(prfl);

        SetupMenuValFloat *cf = new SetupMenuValFloat("CenterFreq", "Hz", set_parent_dirty, &center_freq);
        cf->setHelp("Center frequency for Audio at zero Vario or zero S2F delta");
        top->addEntry(cf);

        SetupMenuValFloat *oc = new SetupMenuValFloat("Tone Variation", "fold", set_parent_dirty, &tone_var);
        oc->setHelp("Maximum tone frequency variation");
        top->addEntry(oc);

        SetupMenuSelect *dt = new SetupMenuSelect("Dual Tone", RST_NONE, set_parent_dirty, &dual_tone);
        dt->setHelp("Select dual tone modue aka ilec sound (di/da/di), or single tone (di/di/di) mode");
        dt->mkEnable();
        top->addEntry(dt);

        SetupMenuSelect *tch = new SetupMenuSelect("Chopping", RST_NONE, set_parent_dirty, &chopping_mode);
        tch->setHelp("Select tone chopping option on positive values for Vario and or S2F");
        tch->addEntry("Disabled");      // 0
        tch->addEntry("Vario only");    // 1
        tch->addEntry("S2F only");      // 2
        tch->addEntry("Vario and S2F"); // 3  default
        top->addEntry(tch);

        SetupMenuSelect *tharm = new SetupMenuSelect("Harmonics", RST_NONE, set_parent_dirty, &audio_harmonics);
        tharm->setHelp("Add harmonics to the vario tone, making it a bit more crispy");
        tharm->addEntry("None", AUD_HARM_OFF);
        tharm->addEntry("Some", AUD_HARM_LOW);
        tharm->addEntry("More", AUD_HARM_MED);
        tharm->addEntry("Sparky", AUD_HARM_HIGH);
        top->addEntry(tharm);

        SetupMenuValFloat *afac = new SetupMenuValFloat("Audio Response", "", set_parent_dirty, &audio_factor);
        afac->setHelp("How the audio frequency responds to the climb rate: < 1 for logarithmic, and > 1 for exponential, response");
        top->addEntry(afac);

        SetupMenuSelect *sink = new SetupMenuSelect("Sink Tone", RST_NONE, set_parent_dirty, &audio_sink_tone);
        sink->setHelp("Steady tone in sink, or a slow breathing tone that swells and fades");
        sink->addEntry("Steady", AUD_SINK_STEADY);
        sink->addEntry("Breathing", AUD_SINK_BREATHING);
        top->addEntry(sink);
    }

    // update selected profile
    SetupMenuSelect *prfl = static_cast<SetupMenuSelect*>(top->getEntry(0));
    prfl->setSelect(profile_idx);
    ESP_LOGI(FNAME,"set selection %d", profile_idx);

    // the Pur tone comes from its own tables: its values cannot be modified
    for (int i = 1; i < top->getNrChilds(); i++) {
        MenuEntry *me = top->getEntry(i);
        if ( profile_idx == PUR_PRFL ) { me->lock(); }
        else { me->unlock(); }
    }
}

static int update_adaptive_buzz(SetupMenuSelect *p) {
    p->getParent()->setBuzzword(adaptiveName());
    return 0;
}

void audio_menu_create_adaptive(SetupMenu *top) {
    SetupMenuSelect *mode = new SetupMenuSelect("Mode", RST_NONE, update_adaptive_buzz, &audio_adaptive);
    mode->setHelp("Quieter on a long steady value and when flying slowly, brighter sink tone at speed, pitch lift on increasing climb, "
        "soft S2F dead band edges with a pause when the command changes side");
    for (const auto &m : adaptive_mode) {
        mode->addEntry(m.data());
    }
    top->addEntry(mode);

    SetupMenuValFloat *sdb = new SetupMenuValFloat("Steady Value", "dB", nullptr, &audio_steady_db);
    sdb->setHelp("Level reduction when the vario stays the same for more than a minute, 0 switches it off");
    top->addEntry(sdb);

    SetupMenuValFloat *ldb = new SetupMenuValFloat("Slow Flight", "dB", nullptr, &audio_slow_db);
    ldb->setHelp("Level reduction when flying slowly, corner speeds follow the polar, 0 switches it off");
    top->addEntry(ldb);
}

void audio_menu_create_deadband(SetupMenu *top) {
    update_range_entry(0);
    SetupMenuSelect *audio_range_sm = new SetupMenuSelect("Range", RST_NONE, set_parent_dirty, &audio_range);
    audio_range_sm->addEntry(range0.c_str());
    audio_range_sm->addEntry(range1.c_str());
    audio_range_sm->addEntry(range2.c_str());
    audio_range_sm->setHelp("Audio range: fixed, or variable according to current Vario display range setting");
    top->addEntry(audio_range_sm);

    SetupMenuValFloat *dbv = new SetupMenuValFloat("Vario", "", set_parent_dirty, &deadband);
    dbv->setHelp("Vario dead band, symmetric around zero");
    top->addEntry(dbv);

    SetupMenuSelect *dbd = new SetupMenuSelect("Vario Depth", RST_NONE, set_parent_dirty, &deadband_depth);
    dbd->setHelp("Silent inside the vario dead band, or while circling only quieter: fading from the edge to the given level at zero");
    dbd->addEntry("Silent", AUD_DB_SILENT);
    dbd->addEntry("-6 dB", 6);
    dbd->addEntry("-12 dB", 12);
    dbd->addEntry("-18 dB", 18);
    top->addEntry(dbd);

    SetupMenuValFloat *dbs = new SetupMenuValFloat("S2F", "", set_parent_dirty, &s2f_deadband);
    dbs->setHelp("S2F dead band, symmetric around zero");
    top->addEntry(dbs);
}


void audio_menu_create(SetupMenu *audio) {
    if (audio->getNrChilds() == 0)
    {
        SetupMenuSelect *asida = new SetupMenuSelect("V-Tone at Sink", RST_NONE, set_parent_dirty, &audio_mute_sink);
        asida->setHelp("Select whether vario audio will be on while in sink");
        asida->addEntry(ENABLE_MODE[0].data(), 1);
        asida->addEntry(ENABLE_MODE[1].data(), 0); // inverted logic
        audio->addEntry(asida);

        SetupMenuValFloat *dv = new SetupMenuValFloat("Default Volume", "%", show_vol_dflt, &default_volume, RST_NONE, true);
        dv->setHelp("Default volume for Audio when device is switched on");
        audio->addEntry(dv);

        SetupMenuValFloat *flarmv = new SetupMenuValFloat("Alarm Vol. Raise", "%", nullptr, &alarm_volraise);
        flarmv->setHelp("Audio volume raise for alarms and warnings (min. up to 60%)");
        audio->addEntry(flarmv);

        SetupMenuSelect* stawaen = new SetupMenuSelect("Stall Warning", RST_NONE, nullptr, &stall_warning);
        stawaen->setHelp("Alarm sound when speed goes below configured stall speed");
        stawaen->mkEnable();
        audio->addEntry(stawaen);

        SetupMenuSelect *amspvol = new SetupMenuSelect("Split Volume", RST_NONE, nullptr, &audio_split_vol);
        amspvol->setHelp("Independent audio volumes for Cruise and Vario mode");
        amspvol->mkEnable();
        audio->addEntry(amspvol);

        SetupMenuSelect *abnm = new SetupMenuSelect("Cruise Audio", RST_NONE, nullptr, &cruise_audio_mode);
        abnm->setHelp("Select either S2F command or Vario tone while cruising");
        abnm->addEntry("Speed2Fly");
        abnm->addEntry("Vario");
        audio->addEntry(abnm);

        int8_t pidx = getAudioProfile();
        SetupMenu *astyle = new SetupMenu("Tone Style", audio_menu_create_tonestyle);
        astyle->setBuzzword(audio_prfls[pidx].getName());
        astyle->setHelp("Refine XCV audio by preset tone styles, or a customized parameter set");
        audio->addEntry(astyle);

        SetupMenu *adapt = new SetupMenu("Adaptive Tone", audio_menu_create_adaptive);
        adapt->setBuzzword(adaptiveName());
        adapt->setHelp("Vario tone adapts to the flight situation, works with every tone style");
        audio->addEntry(adapt);

        SetupMenu *db = new SetupMenu("Range & Deadbands", audio_menu_create_deadband);
        audio->addEntry(db);
        db->setHelp("Dead band limits within which audio remains silent.  1 m/s equals roughly 200 fpm or 2 knots");
    }
}


void free_audio_menu()
{
    range0.clear();
    range0.shrink_to_fit();
    range1.clear();
    range1.shrink_to_fit();
    range2.clear();
    range2.shrink_to_fit();
}
