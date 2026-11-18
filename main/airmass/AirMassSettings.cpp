#include "AirMassSettings.h"

SetupNG<mps_t>  		airmass_vario( "AM_VARIO", 0.0, 0, SYNC_FROM_MASTER, VOLATILE ); // vertical air mass movement
SetupNG<int>  			airmass_status( "AM_STAT", 0, 0, SYNC_FROM_MASTER, VOLATILE );
SetupNG<float> 			airmass_k_tas("AM_KTAS", 1.0, USER_RST, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, nullptr, true); // learned
SetupNG<float> 			airmass_pitch_ofs("AM_PITCH", 0.0, USER_RST, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, nullptr, true); // learned
SetupNG<float> 			airmass_latency("AM_LAT", 0.75, USER_RST, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, nullptr, true); // learned GNSS latency [s]
SetupNG<float> 			airmass_latency_sigma("AM_LATSIG", 0.3, USER_RST, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, nullptr, true); // its uncertainty [s], 0.3: not learned yet
// learned pitch/AoA offset per flap lever position [rad], 1.0: not learned yet
SetupNG<float> 			airmass_flap_ofs_0("AM_FOFS0", 1.0, USER_RST, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, nullptr, true);
SetupNG<float> 			airmass_flap_ofs_1("AM_FOFS1", 1.0, USER_RST, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, nullptr, true);
SetupNG<float> 			airmass_flap_ofs_2("AM_FOFS2", 1.0, USER_RST, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, nullptr, true);
SetupNG<float> 			airmass_flap_ofs_3("AM_FOFS3", 1.0, USER_RST, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, nullptr, true);
SetupNG<float> 			airmass_flap_ofs_4("AM_FOFS4", 1.0, USER_RST, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, nullptr, true);
SetupNG<float> 			airmass_flap_ofs_5("AM_FOFS5", 1.0, USER_RST, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, nullptr, true);
SetupNG<float> 			airmass_flap_ofs_6("AM_FOFS6", 1.0, USER_RST, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, nullptr, true);
SetupNG<float> 			airmass_k_aoa("AM_KAOA", 1.0, USER_RST, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, nullptr, true); // learned angle of attack scale
SetupNG<int> 			airmass_sig_pitch("AM_SIGP", 0, USER_RST, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, nullptr, true); // installation the pitch offset was learned with
SetupNG<int> 			airmass_sig_speed("AM_SIGS", 0, USER_RST, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, nullptr, true); // installation the airspeed scale was learned with
SetupNG<int> 			airmass_wind_nmea("AM_WNMEA", 0, USER_RST);
SetupNG<int> 			airmass_gust_sound("AM_GUST", 1, USER_RST);
