#include "AirMassSettings.h"
#include "AirMassSensor.h"

static void change_airmass()
{
    if (airmassSensor) {
        airmassSensor->configChanged();
    }
}

SetupNG<mps_t>  		airmass_vario( "AM_VARIO", 0.0, 0, SYNC_FROM_MASTER, VOLATILE ); // vertical air mass movement
SetupNG<int>  			airmass_inst_wind( "AM_IWIND", 0xffff, 0, SYNC_FROM_MASTER, VOLATILE ); // WindData alias
SetupNG<int>  			airmass_avg_wind( "AM_AWIND", 0xffff, 0, SYNC_FROM_MASTER, VOLATILE ); // WindData alias
SetupNG<int>  			airmass_status( "AM_STAT", 0, 0, SYNC_FROM_MASTER, VOLATILE );
SetupNG<int> 			airmass_enable("AM_ENA", 1, USER_RST, SYNC_NONE, PERSISTENT, change_airmass);
SetupNG<float> 			airmass_wind_resp("AM_WRESP", 1.0, USER_RST, SYNC_NONE, PERSISTENT, change_airmass, quantity_t::QUANT_NONE, LIMITS(0.2, 5.0, 0.1));
SetupNG<float> 			airmass_vario_resp("AM_VRESP", 1.0, USER_RST, SYNC_NONE, PERSISTENT, change_airmass, quantity_t::QUANT_NONE, LIMITS(0.2, 5.0, 0.1));
SetupNG<float> 			airmass_te_adjust("AM_TEADJ", 100.f, USER_RST, SYNC_NONE, PERSISTENT, change_airmass, quantity_t::QUANT_NONE, LIMITS(80, 120, 0.1));
SetupNG<float> 			airmass_k_tas("AM_KTAS", 1.0, USER_RST, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, nullptr, true); // learned
SetupNG<float> 			airmass_pitch_ofs("AM_PITCH", 0.0, USER_RST, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, nullptr, true); // learned
SetupNG<float> 			airmass_lat_ofs("AM_LATOFS", 0.0, USER_RST, SYNC_NONE, PERSISTENT, nullptr, quantity_t::QUANT_NONE, nullptr, true); // learned GNSS latency correction [s]
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
SetupNG<int> 			airmass_nmea("AM_NMEA", 0, USER_RST);
SetupNG<int> 			airmass_wind_nmea("AM_WNMEA", 0, USER_RST);
