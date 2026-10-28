#!/bin/bash
# Sensitivity sweep: the Dutch day for each variant of the simulation assumptions, averaged over 6 noise seeds.
# Run inside the ESP-IDF Docker image after tools/airmass_sim/run.sh built build_sim:
#   docker run --rm --entrypoint "" -v "$PWD":/project -w /project espressif/idf:release-v5.4 bash tools/airmass_sim/sweep.sh
# Columns: AirMass/TE netto for cruise rms, false lift per hour, thermal entry rms and max, circling rms,
# climb reading per thermal, display lag; the averaged wind error; the learned airspeed scale error; failed runs.
B=build_sim/airmass_sim
run() {
  name=$1; shift
  for s in 1 2 3 4 5 6; do
    out=$($B nl --seed $s "$@" 2>&1)
    echo "$out" | tr "\n" " " | grep -oE "cruise rms [0-9.]+ / [0-9.]+|false lift events [0-9]+ / [0-9]+|thermal entry rms [0-9.]+ / [0-9.]+ \(max [0-9.]+ / [0-9.]+\)|circling rms [0-9.]+ / [0-9.]+|thermals\): rms [0-9.]+ / [0-9.]+|display lag [0-9.]+ / [0-9.]+|30s average [0-9.]+|restarts [0-9]+|k_tas [0-9.]+ \(err [-0-9.]+\)" | sed "s/[^0-9. -]//g" | tr "\n" " "
    echo "$out" | grep -q "FAILED" && echo " FAIL" || echo " ok"
  done | awk -v n="$name" '{c++; for(i=1;i<=NF;i++) if ($i ~ /^-?[0-9.]+$/) s[i]+=$i; if ($0 ~ /FAIL/) f++}
    END {printf "%-22s cruise %.2f/%.2f  falselift %.1f/%.1f  entry %.2f/%.2f max %.2f/%.2f  circ %.2f/%.2f  climb %.2f/%.2f  lag %.1f/%.1f  wind %.2f  ktas_err %+.3f  fails %d/%d\n", n, s[4]/c,s[5]/c,s[6]/c,s[7]/c,s[8]/c,s[9]/c,s[10]/c,s[11]/c,s[12]/c,s[13]/c,s[14]/c,s[15]/c,s[16]/c,s[17]/c,s[19]/c,s[2]/c,f+0,c}'
}
run baseline
run "airspeed -10%"   --set k_tas=1.15
run "airspeed +10%"   --set k_tas=0.95
run "polar +15% sink" --set polar_err=1.15
run "polar -10% sink" --set polar_err=0.9
run "mass +15%"       --set mass_err=0.87
run "mass -15%"       --set mass_err=1.15
run "IMU mount 3 deg" --set mount=0.052
run "IMU mount -2 deg" --set mount=-0.035
run "gyro bias x5"    --set gyro_bias=0.01
run "acc bias x6"     --set acc_bias=0.03
run "noisy airspeed"  --set tas_noise=0.4
run "other glider"    --set cla_true=4.8
run "wind 43 km/h"    --set wind=12
run "wind 11 km/h"    --set wind=3
run "rough air x2"    --set turb=2
run "weak thermals"   --set lift=0.6
run "strong thermals" --set lift=1.6
run "dolphin flying"  --set dolphin=1
run "GPS out 5 min"   --set gps_out=600 --set gps_out_len=300
run "combined worst"  --set k_tas=1.15 --set polar_err=1.15 --set mount=0.052 --set gyro_bias=0.01 --set tas_noise=0.4 --set turb=2
