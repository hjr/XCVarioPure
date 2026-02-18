idf_component_register(SRC_DIRS "."
                       INCLUDE_DIRS "."
                       EMBED_TXTFILES ${project_dir}/server_certs/ca_cert.pem
                       REQUIRES soc driver nvs_flash esp_adc esp_driver_dac esp_wifi esp_system esp_http_server esp_https_ota bt I2Cbus MPUdriver ESP32-coredump eglib qrcodegen simplex glider onewire_bus )

add_subdirectory("comm")
add_subdirectory("math")
add_subdirectory("protocol")
add_subdirectory("screen")
add_subdirectory("sensor")
add_subdirectory("setup")
add_subdirectory("wind")
add_subdirectory("wmm")

if(DEFINED ENV{BUILD_TYPE})
    set(CMAKE_BUILD_TYPE $ENV{BUILD_TYPE} CACHE STRING "" FORCE)
endif()


if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_definitions(CONFIG_LOG_DISABLED=1)
else()
    add_compile_definitions(DEBUG_AND_TEST=1)
endif()




