cmake_minimum_required(VERSION 3.16)
# This example deliberately consumes the public, protected SDK archives.
unset(ENV{EDGEZ_HALOW_MESH_SOURCE_ROOT})
list(APPEND EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/platforms/esp-halow/halow")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
set(PROJECT_VER "0.1.0")
project(edgez_camera_example)
