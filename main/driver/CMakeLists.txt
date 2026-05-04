
file(GLOB SUB_SRCS "*.cpp" "audio/*.cpp" "gpio/*.cpp" "time/*.cpp" "web_ota/*.cpp")

set_source_files_properties(
    audio/ESPAudio.cpp
    PROPERTIES COMPILE_OPTIONS "-O2"
)

target_sources(${COMPONENT_LIB} PRIVATE ${SUB_SRCS})
target_include_directories(${COMPONENT_LIB} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
