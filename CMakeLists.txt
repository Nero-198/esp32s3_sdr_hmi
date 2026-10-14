cmake_minimum_required(VERSION 3.22)
# esp-dsp comes pinned with ESP-SDR (tools/fetch_esp_sdr.sh).
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/third_party/esp-sdr/components/esp-dsp")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32s3_sdr_hmi)
