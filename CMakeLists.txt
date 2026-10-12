# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.11)

# (Not part of the boilerplate)
# This example uses an extra component 
#set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/examples/sensor/components)


if(DEFINED ENV{BUILD_TYPE})
    set(CMAKE_BUILD_TYPE $ENV{BUILD_TYPE} CACHE STRING "" FORCE)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    message(STATUS "Building in Release mode")
    set(SDKCONFIG_DEFAULTS "sdkconfig.defaults;sdkconfig.defaults.release")
    add_compile_definitions(ALL_LOGS_DISABLED=1)
    add_compile_definitions(LOG_LOCAL_LEVEL=ESP_LOG_NONE)
    add_compile_definitions(COMPILER_OPTIMIZATION_ASSERTIONS_SILENT)
elseif(CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    message(STATUS "Building in Release-Developer mode")
    set(SDKCONFIG_DEFAULTS "sdkconfig.defaults")
    add_compile_definitions(ALL_LOGS_DISABLED=1)
    add_compile_definitions(LOG_LOCAL_LEVEL=ESP_LOG_NONE)
    add_compile_definitions(COMPILER_OPTIMIZATION_ASSERTIONS_SILENT)
    add_compile_definitions(DEBUG_AND_TEST=1)
elseif(CMAKE_BUILD_TYPE STREQUAL "RelWithLogInfo")
    message(STATUS "Building in Release-Logging mode")
    set(SDKCONFIG_DEFAULTS "sdkconfig.defaults")
    add_compile_definitions(COMPILER_OPTIMIZATION_ASSERTIONS_SILENT)
else()
    message(STATUS "Building in Debug mode")
    set(SDKCONFIG_DEFAULTS "sdkconfig.defaults")
    add_compile_definitions(DEBUG_AND_TEST=1)
endif()
message(STATUS "CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(xcvario_pure)

include (${project_dir}/html/CMakeLists.txt)
