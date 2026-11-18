# Host build of the AirMass EKF with a synthetic flight simulation.
# Run through tools/airmass_sim/run.sh (Docker, espressif/idf image).
cmake_minimum_required(VERSION 3.16)
project(airmass_sim CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(REPO ${CMAKE_CURRENT_SOURCE_DIR}/../..)

# the device AHRS math (quaternions) comes from the firmware sources
set(UTIL_MATH ${REPO}/components/util/math)
set(AHRS_MATH_SOURCES
    ${UTIL_MATH}/Quaternion.cpp ${UTIL_MATH}/vector_3d.cpp
    ${UTIL_MATH}/Trigonometry.cpp ${UTIL_MATH}/Floats.cpp ${UTIL_MATH}/Units.cpp)
# the host builds of the device code: its include paths, and the host stand-ins for the ESP-IDF headers
set(HOST_INCLUDES ${REPO}/main/airmass ${REPO}/components/util ${REPO}/main ${CMAKE_CURRENT_SOURCE_DIR}/stub)
set(HOST_OPTIONS -Wall -Wextra -fsingle-precision-constant -include ${CMAKE_CURRENT_SOURCE_DIR}/stub/host_compat.h)

add_executable(airmass_sim
    sim.cpp
    ${REPO}/main/airmass/AirMassEKF.cpp
    ${UTIL_MATH}/Quaternion.cpp
    ${UTIL_MATH}/vector_3d.cpp
    ${UTIL_MATH}/Trigonometry.cpp
    ${UTIL_MATH}/Floats.cpp
    ${UTIL_MATH}/Units.cpp
    ${REPO}/main/protocol/nmea_util.cpp
)
target_include_directories(airmass_sim PRIVATE
    ${REPO}/main/airmass
    ${REPO}/components/util
    ${CMAKE_CURRENT_SOURCE_DIR}/stub
    ${REPO}/main
)
target_compile_options(airmass_sim PRIVATE -Wall -Wextra -Wno-unused-parameter -fsingle-precision-constant
    -include ${CMAKE_CURRENT_SOURCE_DIR}/stub/host_compat.h)

# conversion of the raw device measurements into the estimator input, flap offsets
add_executable(input_test input_test.cpp ${REPO}/main/airmass/AirMassEKF.cpp)
target_include_directories(input_test PRIVATE ${REPO}/main/airmass ${REPO}/components/util)
target_compile_options(input_test PRIVATE ${HOST_OPTIONS})

# replay of recorded flights (main/airmass/AirMassRecord.h) through the same pipeline
add_executable(airmass_replay replay.cpp ${REPO}/main/airmass/AirMassEKF.cpp ${REPO}/main/protocol/nmea_util.cpp ${AHRS_MATH_SOURCES})
target_include_directories(airmass_replay PRIVATE ${HOST_INCLUDES})
target_compile_options(airmass_replay PRIVATE ${HOST_OPTIONS})

# the GNSS/INS reference (ins_reference.h) on replayed $SENS flights (sens2rec.py, airmass_replay)
add_executable(ref_replay ref_replay.cpp)
target_compile_options(ref_replay PRIVATE -Wall -Wextra -fsingle-precision-constant)

enable_testing()
add_executable(robustness_test robustness_test.cpp ${REPO}/main/airmass/AirMassEKF.cpp ${REPO}/main/protocol/nmea_util.cpp)
target_include_directories(robustness_test PRIVATE ${HOST_INCLUDES})
target_compile_options(robustness_test PRIVATE ${HOST_OPTIONS})
add_test(NAME sensor_gnss_robustness COMMAND robustness_test)
add_executable(mahony_test mahony_test.cpp ${REPO}/main/airmass/AirMassEKF.cpp ${REPO}/main/protocol/nmea_util.cpp ${AHRS_MATH_SOURCES})
target_include_directories(mahony_test PRIVATE ${HOST_INCLUDES})
target_compile_options(mahony_test PRIVATE ${HOST_OPTIONS})
add_test(NAME mahony_ahrs COMMAND mahony_test)
add_executable(replay_start_test replay_start_test.cpp ${REPO}/main/airmass/AirMassEKF.cpp ${REPO}/main/protocol/nmea_util.cpp ${AHRS_MATH_SOURCES})
target_include_directories(replay_start_test PRIVATE ${HOST_INCLUDES})
target_compile_options(replay_start_test PRIVATE ${HOST_OPTIONS})
add_test(NAME replay_airborne_start COMMAND replay_start_test)
add_executable(sampling_test sampling_test.cpp ${REPO}/main/airmass/AirMassEKF.cpp ${REPO}/main/protocol/nmea_util.cpp ${AHRS_MATH_SOURCES})
target_include_directories(sampling_test PRIVATE ${HOST_INCLUDES})
target_compile_options(sampling_test PRIVATE ${HOST_OPTIONS})
add_test(NAME sensor_freshness_20hz COMMAND sampling_test)
# the in place recording formatter writes the same bytes as the former std::string one
add_executable(record_format_test record_format_test.cpp ${REPO}/main/airmass/AirMassEKF.cpp ${REPO}/main/protocol/nmea_util.cpp ${AHRS_MATH_SOURCES})
target_include_directories(record_format_test PRIVATE ${HOST_INCLUDES})
target_compile_options(record_format_test PRIVATE ${HOST_OPTIONS})
add_test(NAME record_format COMMAND record_format_test)
add_test(NAME input_conversion COMMAND input_test)
foreach(scenario straight thermal gust gust6 pullup slip nl endurance)
    add_test(NAME ${scenario} COMMAND airmass_sim ${scenario})
endforeach()

# true GNSS latency differs from the nominal one, the filter has to learn it
foreach(scenario straight endurance)
    foreach(err -300 300)
        add_test(NAME ${scenario}_lat${err} COMMAND airmass_sim ${scenario} --lat-err ${err})
    endforeach()
endforeach()
# in a single steady circle the latency error is indistinguishable from a heading offset (turn rate * error),
# the remaining heading error after the roll-out costs wind accuracy until the next turn rate change
foreach(err -300 300)
    add_test(NAME thermal_lat${err} COMMAND airmass_sim thermal --lat-err ${err} --max-wind-rms 1.2)
endforeach()

# static port error of the installation, compensated with the matching pull-up compensation
add_test(NAME pullup_static108 COMMAND airmass_sim pullup --static-adj 108 --comp 108)
add_test(NAME straight_static92 COMMAND airmass_sim straight --static-adj 92 --comp 92)

# ASW 20: flaps shift the fuselage angle of attack by degrees, the estimator learns an offset per lever position
foreach(scenario nl pullup thermal)
    add_test(NAME ${scenario}_asw20 COMMAND airmass_sim ${scenario} --set flaps=1)
endforeach()
# no flap sensor: the estimator takes the flap box recommendation, the pilot flies their own schedule or follows the box
add_test(NAME nl_asw20_flapbox COMMAND airmass_sim nl --set flaps=1 --set flap_aid=2)
add_test(NAME nl_asw20_flapbox_pilot COMMAND airmass_sim nl --set flaps=1 --set flap_aid=2 --set flap_pilot=1)
# a lift curve bending towards the stall: the flapped glider circles where the AoA model still holds
add_test(NAME nl_asw20_bent COMMAND airmass_sim nl --set flaps=1 --set aoa_nonlin=0.25)

# a large gyro bias (in flight drift, the device calibrates the gyro at rest): the pitch gyro bias enters the heading
# rate in turns with sin(roll), the body z bias with cos(roll), the heading must not drift off after circling; the
# attitude takes the body z bias from the estimator, unlearned it tilts the attitude in every circle (climb reading 0.13)
add_test(NAME nl_gyro_bias COMMAND airmass_sim nl --set gyro_bias=0.01 --max-climb-err 0.06)

# compass aided alignment: the wind gets valid early in the first turn (78s without compass)
add_test(NAME straight_compass COMMAND airmass_sim straight --compass 2 --max-wind-valid 70)

# record a flight like the device's "NMEA Output" Recording, the replay must reproduce the recorded $PXCVA output:
# a flapped glider with compass, so every sentence type is in the log
add_test(NAME record_nl COMMAND airmass_sim nl --set flaps=1 --compass 2 --record ${CMAKE_CURRENT_BINARY_DIR}/nl_record.nmea)
set_tests_properties(record_nl PROPERTIES FIXTURES_SETUP nl_record)
add_test(NAME replay_nl COMMAND airmass_replay ${CMAKE_CURRENT_BINARY_DIR}/nl_record.nmea --check 0.02)
set_tests_properties(replay_nl PROPERTIES FIXTURES_REQUIRED nl_record)

# an IMU rolled 3° against the wings: its lateral force is learned, not taken for sideslip (the wind never got valid in
# thermal before)
foreach(scenario nl thermal)
    add_test(NAME ${scenario}_mount_roll3 COMMAND airmass_sim ${scenario} --set mount_roll=3)
endforeach()

# the IMU reference pitched 8° in flight (pitch adjustment): the estimator turns its body axes and the flap offsets,
# keeps heading and wind; the replay of the recorded $PXCVO must follow
add_test(NAME record_mount_step COMMAND airmass_sim nl --set flaps=1 --set mount_step=-8 --set mount_step_t=1800
         --record ${CMAKE_CURRENT_BINARY_DIR}/mount_step_record.nmea)
set_tests_properties(record_mount_step PROPERTIES FIXTURES_SETUP mount_step_record)
add_test(NAME replay_mount_step COMMAND airmass_replay ${CMAKE_CURRENT_BINARY_DIR}/mount_step_record.nmea --check 0.02)
set_tests_properties(replay_mount_step PROPERTIES FIXTURES_REQUIRED mount_step_record)

# The same physical observations and truth, with prediction-only ticks between 10Hz measurements.
foreach(scenario straight thermal gust nl)
    add_test(NAME ${scenario}_predict20 COMMAND airmass_sim ${scenario} --set predict_hz=20)
endforeach()
add_test(NAME record_predict20 COMMAND airmass_sim nl --set predict_hz=20 --compass 2 --record ${CMAKE_CURRENT_BINARY_DIR}/predict20_record.nmea)
set_tests_properties(record_predict20 PROPERTIES FIXTURES_SETUP predict20_record)
add_test(NAME replay_predict20 COMMAND airmass_replay ${CMAKE_CURRENT_BINARY_DIR}/predict20_record.nmea --check 0.02)
set_tests_properties(replay_predict20 PROPERTIES FIXTURES_REQUIRED predict20_record)

# Regression for flap offsets in the delayed GNSS history.
add_executable(flap_history_test flap_history_test.cpp ${REPO}/main/airmass/AirMassEKF.cpp)
target_include_directories(flap_history_test PRIVATE ${REPO}/main/airmass ${REPO}/components/util)
target_compile_options(flap_history_test PRIVATE ${HOST_OPTIONS})
add_test(NAME flap_history_geometry COMMAND flap_history_test)
add_executable(gust_test gust_test.cpp)
target_include_directories(gust_test PRIVATE ${REPO}/main/airmass)
target_compile_options(gust_test PRIVATE ${HOST_OPTIONS})
add_test(NAME gust_detector COMMAND gust_test)
