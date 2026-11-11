idf_component_register(SRCS "hc33_main.c" "../common/camera_protocol.c"
    INCLUDE_DIRS "../common"
    PRIV_REQUIRES halow esp_wifi esp_event esp_netif nvs_flash esp_http_server json
                  esp_driver_gpio esp_timer)
get_target_property(shims_sources shims SOURCES)
list(FILTER shims_sources EXCLUDE REGEX "mmhal_os\\.c$")
set_property(TARGET shims PROPERTY SOURCES "${shims_sources}")
target_sources(shims PRIVATE "${CMAKE_CURRENT_LIST_DIR}/mmhal_os_ps_override.c")
target_link_options(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=mmdrv_host_get_beacon")
target_include_directories(${COMPONENT_LIB} PRIVATE
    "../platforms/esp-halow/halow/components/mm-iot-sdk/framework/morselib/src"
    "../platforms/esp-halow/halow/components/mm-iot-sdk/framework/morselib/include")
