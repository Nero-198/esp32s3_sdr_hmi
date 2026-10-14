# ESP32-S3 snapshot receiver from ESP-SDR (GPL-3.0), fetched by
# tools/fetch_esp_sdr.sh. Mirrors the esp32s3 part of upstream main/CMakeLists.txt
# except the serial transport (serial_shim.c) and the continuous ring capture
# (ring_stub.c), which cannot run alongside the RGB LCD.
set(up "${CMAKE_CURRENT_LIST_DIR}/../../third_party/esp-sdr")
if(NOT EXISTS "${up}/main/targets/esp32s3/receiver.c")
  message(FATAL_ERROR "ESP-SDR missing: run tools/fetch_esp_sdr.sh")
endif()
set(um "${up}/main")

idf_component_register(
  SRCS "esp_sdr_s3.c" "serial_shim.c" "ring_stub.c"
       "${um}/common/burst_gpio.c" "${um}/common/burst_version.c"
       "${um}/common/spectrum_stats.c" "${um}/common/rx_recalibration.c"
       "${um}/common/rx_calibration_state.c"
  INCLUDE_DIRS "include"
  PRIV_INCLUDE_DIRS "." "${um}" "${um}/common" "${um}/targets/esp32s3" "${um}/diagnostics"
  REQUIRES esp_wifi esp_phy esp_hw_support esp_timer nvs_flash esp_event
           esp_driver_gpio esp_driver_usb_serial_jtag)

idf_build_get_property(version_python PYTHON)
set(version_header "${CMAKE_CURRENT_BINARY_DIR}/generated/build_version.h")
add_custom_target(esp_sdr_s3_build_version
  COMMAND "${version_python}" "${up}/tools/generate_version.py"
          --root "${up}" --profile esp32s3 --output "${version_header}"
  BYPRODUCTS "${version_header}" VERBATIM)
add_dependencies(${COMPONENT_LIB} esp_sdr_s3_build_version)
target_include_directories(${COMPONENT_LIB} PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/generated")

# Same link hooks as upstream for esp32s3.
target_link_options(${COMPONENT_LIB} INTERFACE
  "-T${um}/targets/esp32s3/sram_guard.ld"
  "-Wl,--wrap=chip_v7_set_chan_ana"
  "-Wl,--wrap=set_rx_gain_cal_dc")
# Upstream code is not ours to restyle; keep its warnings out of the build.
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-function -Wno-format)
