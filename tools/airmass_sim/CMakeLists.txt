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
add_executable(airmass_sim
    sim.cpp
    ${REPO}/main/airmass/AirMassEKF.cpp
    ${UTIL_MATH}/Quaternion.cpp
    ${UTIL_MATH}/vector_3d.cpp
    ${UTIL_MATH}/Trigonometry.cpp
    ${UTIL_MATH}/Floats.cpp
    ${UTIL_MATH}/Units.cpp
)
target_include_directories(airmass_sim PRIVATE
    ${REPO}/main/airmass
    ${REPO}/components/util
    ${CMAKE_CURRENT_SOURCE_DIR}/stub
)
target_compile_options(airmass_sim PRIVATE -Wall -Wextra -Wno-unused-parameter -fsingle-precision-constant
    -include ${CMAKE_CURRENT_SOURCE_DIR}/stub/host_compat.h)

# conversion of the raw device measurements into the estimator input, flap offsets
add_executable(input_test input_test.cpp ${REPO}/main/airmass/AirMassEKF.cpp)
target_include_directories(input_test PRIVATE ${REPO}/main/airmass ${REPO}/components/util)
target_compile_options(input_test PRIVATE -Wall -Wextra -fsingle-precision-constant
    -include ${CMAKE_CURRENT_SOURCE_DIR}/stub/host_compat.h)

enable_testing()
add_test(NAME input_conversion COMMAND input_test)
foreach(scenario straight thermal gust pullup slip nl endurance)
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

# compass aided alignment: the wind gets valid early in the first turn (78s without compass)
add_test(NAME straight_compass COMMAND airmass_sim straight --compass 2 --max-wind-valid 70)
