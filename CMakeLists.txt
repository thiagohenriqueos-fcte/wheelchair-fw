cmake_minimum_required(VERSION 3.16)

set(IDF_TARGET "esp32s3" CACHE STRING "ESP-IDF target")
set(PROJECT_VER "0.4.0")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(wheelchair_esp32s3_firmware)
