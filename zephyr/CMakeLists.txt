cmake_minimum_required(VERSION 3.20.0)
set(MORSE_ZEPHYR_MODULE "${CMAKE_CURRENT_LIST_DIR}/../platforms/mm-iot-zephyr")
list(APPEND ZEPHYR_EXTRA_MODULES "${MORSE_ZEPHYR_MODULE}")
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(edgez_camera_node)
zephyr_include_directories("${CMAKE_CURRENT_LIST_DIR}/../common"
    "${CMAKE_CURRENT_LIST_DIR}/../nrf54"
    "${MORSE_ZEPHYR_MODULE}/components/morsemicro/libmorse_interface/include"
    "${MORSE_ZEPHYR_MODULE}/submodules/mm-iot-sdk/framework/src/mmregdb"
    "${MORSE_ZEPHYR_MODULE}/submodules/edgez-rf-api/include"
    "${MORSE_ZEPHYR_MODULE}/submodules/edgez-rf-morsemicro/include"
    "${MORSE_ZEPHYR_MODULE}/submodules/edgez-halow-mesh-prebuilt/include"
    "${MORSE_ZEPHYR_MODULE}/submodules/mm-iot-sdk/framework/morselib/include"
    "${MORSE_ZEPHYR_MODULE}/submodules/mm-iot-sdk/framework/morselib/src"
    "${MORSE_ZEPHYR_MODULE}/drivers/wifi/morsemicro")
target_sources(app PRIVATE "${CMAKE_CURRENT_LIST_DIR}/../common/camera_protocol.c"
    "${MORSE_ZEPHYR_MODULE}/submodules/edgez-rf-morsemicro/src/edgez_morse_channel.c")
target_link_libraries(app PRIVATE libmorse_interface edgez_umac_core
    edgez_mesh_security edgez_rf_morsemicro libmorse)

zephyr_compile_definitions(MMOSAL_NOASSERT MMOSAL_NO_DEBUGLOG)
