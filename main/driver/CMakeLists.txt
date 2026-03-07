
file(GLOB SUB_SRCS "*.cpp" "audio/*.cpp" "gpio/*.cpp")

target_sources(${COMPONENT_LIB} PRIVATE ${SUB_SRCS})
target_include_directories(${COMPONENT_LIB} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
