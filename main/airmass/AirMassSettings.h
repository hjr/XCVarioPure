#pragma once

// AirMass settings and black board items. Defined here rather than in SetupNG.cpp, so the feature stays in its
// own directory; the keys (NVS, sync) are the same.

#include "setup/SetupNG.h"

extern SetupNG<mps_t>  		airmass_vario; // vertical air mass movement, up positive
extern SetupNG<int>  		airmass_status; // AirMassEKF status bits
extern SetupNG<float> 		airmass_k_tas;
extern SetupNG<float> 		airmass_pitch_ofs;
extern SetupNG<float> 		airmass_latency;
extern SetupNG<float> 		airmass_latency_sigma;
extern SetupNG<float> 		airmass_k_aoa;
extern SetupNG<float> 		airmass_flap_ofs_0; // learned pitch/AoA offset per flap lever position
extern SetupNG<float> 		airmass_flap_ofs_1;
extern SetupNG<float> 		airmass_flap_ofs_2;
extern SetupNG<float> 		airmass_flap_ofs_3;
extern SetupNG<float> 		airmass_flap_ofs_4;
extern SetupNG<float> 		airmass_flap_ofs_5;
extern SetupNG<float> 		airmass_flap_ofs_6;
extern SetupNG<int> 		airmass_sig_pitch;
extern SetupNG<int> 		airmass_sig_speed;
extern SetupNG<int> 		airmass_wind_nmea;
extern SetupNG<int> 		airmass_gust_sound;
extern SetupNG<float> 		airmass_te_set; // learned TE compensation [%], 0: none yet
extern SetupNG<float> 		airmass_te_wt;  // its regression weight
extern SetupNG<float> 		airmass_te_h;   // the hours of flight behind it
extern SetupNG<float> 		airmass_te_age; // the learned age of the GNSS altitude [s]
extern SetupNG<int> 		airmass_te_fl;  // the flights behind it
extern SetupNG<int> 		airmass_te_sig; // installation it was learned with
