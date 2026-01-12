
file(GLOB SUB_SRCS "*.cpp" "adc/*.cpp" "gps/*.cpp" "imu/*.cpp" "press_diff/*.cpp" "pressure/*.cpp" "temp/*.cpp")

target_sources(${COMPONENT_LIB} PRIVATE ${SUB_SRCS})
target_include_directories(${COMPONENT_LIB} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
