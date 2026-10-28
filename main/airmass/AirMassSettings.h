#pragma once

// AirMass settings and black board items. Defined here rather than in SetupNG.cpp, so the feature stays in its
// own directory; the keys (NVS, sync) are the same.

#include "setup/SetupNG.h"

extern SetupNG<mps_t>  		airmass_vario; // vertical air mass movement, up positive
extern SetupNG<int>  		airmass_inst_wind; // int alias WindData, instant wind of the air mass estimator
extern SetupNG<int>  		airmass_avg_wind; // int alias WindData, long term average of the air mass estimator
extern SetupNG<int>  		airmass_status; // AirMassEKF status bits
extern SetupNG<int> 		airmass_enable;
extern SetupNG<float> 		airmass_wind_resp;
extern SetupNG<float> 		airmass_vario_resp;
extern SetupNG<float> 		airmass_te_adjust;
extern SetupNG<float> 		airmass_k_tas;
extern SetupNG<float> 		airmass_pitch_ofs;
extern SetupNG<float> 		airmass_lat_ofs;
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
extern SetupNG<int> 		airmass_nmea;
extern SetupNG<int> 		airmass_wind_nmea;
