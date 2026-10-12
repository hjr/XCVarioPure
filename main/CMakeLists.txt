idf_component_register(SRC_DIRS "."
                       INCLUDE_DIRS "."
                       EMBED_TXTFILES ${project_dir}/server_certs/ca_cert.pem
                       REQUIRES soc driver nvs_flash esp_adc esp_driver_dac esp_wifi wpa_supplicant esp_driver_i2c esp_system esp_http_server esp_https_ota bt MPUdriver ESP32-coredump eglib qrcodegen simplex glider onewire_bus util)


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


# a firmware version string
if(NOT DEFINED FW_VERSION)
    execute_process(
        COMMAND git describe --tags --dirty
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE FW_VERSION
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
endif()

message(STATUS "FW_VERSION='${FW_VERSION}'")

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/version.h.in
    ${CMAKE_CURRENT_BINARY_DIR}/version.h
    @ONLY
)

target_include_directories(${COMPONENT_LIB}
    PRIVATE ${CMAKE_CURRENT_BINARY_DIR}
)
