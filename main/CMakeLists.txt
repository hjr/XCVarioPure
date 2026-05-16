idf_component_register(SRC_DIRS "."
                       INCLUDE_DIRS "."
                       EMBED_TXTFILES ${project_dir}/server_certs/ca_cert.pem
                       REQUIRES soc driver nvs_flash esp_adc esp_driver_dac esp_wifi wpa_supplicant esp_system esp_http_server esp_http_client json esp_https_ota bt I2Cbus MPUdriver ESP32-coredump eglib qrcodegen simplex glider onewire_bus util)


# global compiler settings
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error=format)

add_subdirectory("comm")
add_subdirectory("driver")
add_subdirectory("protocol")
add_subdirectory("screen")
add_subdirectory("sensor")
add_subdirectory("setup")
add_subdirectory("test")
add_subdirectory("wind")
add_subdirectory("wmm")

if(DEFINED ENV{BUILD_TYPE})
    set(CMAKE_BUILD_TYPE $ENV{BUILD_TYPE} CACHE STRING "" FORCE)
endif()


if(CMAKE_BUILD_TYPE STREQUAL "Release")
    message(STATUS "Building in Release mode")
    add_compile_definitions(ALL_LOGS_DISABLED=1)
    add_compile_definitions(LOG_LOCAL_LEVEL=ESP_LOG_NONE)
    add_compile_definitions(COMPILER_OPTIMIZATION_ASSERTIONS_SILENT)
elseif(CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    message(STATUS "Building in Release-Developer mode")
    add_compile_definitions(ALL_LOGS_DISABLED=1)
    add_compile_definitions(LOG_LOCAL_LEVEL=ESP_LOG_NONE)
    add_compile_definitions(COMPILER_OPTIMIZATION_ASSERTIONS_SILENT)
    add_compile_definitions(DEBUG_AND_TEST=1)
elseif(CMAKE_BUILD_TYPE STREQUAL "RelWithLogInfo")
    message(STATUS "Building in Release-Logging mode")
    add_compile_definitions(COMPILER_OPTIMIZATION_ASSERTIONS_SILENT)
else()
    message(STATUS "Building in Debug mode")
    add_compile_definitions(DEBUG_AND_TEST=1)
endif()

# a firmware version string
string(TIMESTAMP FW_VERSION "%y%m%d-%H%M")

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/version.h.in
    ${CMAKE_CURRENT_BINARY_DIR}/version.h
    @ONLY
)

include_directories(
    ${CMAKE_CURRENT_BINARY_DIR}
)

