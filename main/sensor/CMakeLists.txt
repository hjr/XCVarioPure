
file(GLOB SUB_SRCS "*.cpp" "adc/*.cpp" "gps/*.cpp" "imu/*.cpp" "mag/*.cpp" "press_diff/*.cpp" "pressure/*.cpp" "temp/*.cpp")

set_source_files_properties(
    imu/AccMPU6050.cpp
    SensorBase.cpp
    PROPERTIES COMPILE_OPTIONS "-O2"
)

target_sources(${COMPONENT_LIB} PRIVATE ${SUB_SRCS})
target_include_directories(${COMPONENT_LIB} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
