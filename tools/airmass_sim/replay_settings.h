#pragma once
// Estimator settings by name (AirMassEKF::Config), for experiments: the replay (airmass_replay --set, the browser
// replay's settings field) and the simulator (--set cfg.<name>=value).

#include "AirMassEKF.h"

#include <cstdlib>
#include <string>
#include <utility>
#include <vector>

namespace airmass::replay {

using Settings = std::vector<std::pair<std::string, float>>;

// estimator settings for experiments (name=value, AirMassEKF::Config), applied after every setup; false on an unknown name
inline bool applySettings(AirMassEKF& ekf, const Settings& settings, std::string* unknown = nullptr) {
    AirMassEKF::Config c = ekf.config();
    const std::pair<const char*, float*> known[] = {
        {"sigma_pitch", &c.sigma_pitch}, {"sigma_energy", &c.sigma_energy}, {"energy_tau", &c.energy_tau}, {"q_wind_v", &c.q_wind_v},
        {"q_wind_h", &c.q_wind_h}, {"q_thb", &c.q_thb}, {"q_kcl", &c.q_kcl}, {"q_tau", &c.q_tau}, {"q_tau_start", &c.q_tau_start}, {"tau_start_time", &c.tau_start_time}, {"q_psi", &c.q_psi}, {"lag_static", &c.lag_static}, {"lag_pitot", &c.lag_pitot},
        {"sigma_air", &c.sigma_air}, {"gust_acc", &c.gust_acc}, {"kinetic_tau", &c.kinetic_tau}, {"r_baro", &c.r_baro},
        {"wind_work", &c.wind_work}, {"gate", &c.gate}, {"energy_load_weight", &c.energy_load_weight},
        {"energy_gust_weight", &c.energy_gust_weight}, {"anchor_tau", &c.anchor_tau},
        {"kcl_min", &c.kcl_min}, {"kcl_max", &c.kcl_max}, {"energy_gate", &c.energy_gate},
        {"wind_work_hp", &c.wind_work_hp}, {"blend_hp", &c.blend_hp}, {"polar_calibrated", &c.polar_calibrated},
        {"blend_bank", &c.blend_bank}, {"blend_bank_width", &c.blend_bank_width}, {"circling_tau", &c.circling_tau}, {"circling_blend_tau", &c.circling_blend_tau},
        {"degraded_time", &c.degraded_time}, {"degraded_factor", &c.degraded_factor},
    };
    for (const auto& s : settings) {
        bool found = false;
        for (const auto& k : known) {
            if (s.first == k.first) {
                *k.second = s.second;
                found = true;
            }
        }
        if (!found) {
            if (unknown) {
                *unknown = s.first;
            }
            return false;
        }
    }
    ekf.setConfig(c);
    return true;
}

// "name=value" pairs separated by ',' or ';' (spaces allowed) appended to out; false and the bad item in error
inline bool parseSettings(const std::string& text, Settings& out, std::string* error = nullptr) {
    size_t pos = 0;
    while (pos < text.size()) {
        size_t end = text.find_first_of(",;", pos);
        if (end == std::string::npos) {
            end = text.size();
        }
        std::string item = text.substr(pos, end - pos);
        pos = end + 1;
        item.erase(0, item.find_first_not_of(" \t\r\n"));
        item.erase(item.find_last_not_of(" \t\r\n") + 1);
        if (item.empty()) {
            continue;
        }
        const size_t eq = item.find('=');
        if (eq == std::string::npos) {
            if (error) {
                *error = "a setting needs name=value: " + item;
            }
            return false;
        }
        out.emplace_back(item.substr(0, eq), std::strtof(item.c_str() + eq + 1, nullptr));
    }
    return true;
}

} // namespace airmass::replay
