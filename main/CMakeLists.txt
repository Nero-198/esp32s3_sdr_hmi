idf_component_register(SRCS "main.c" "display.c" "touch_gt911.c" "lvgl_port.c" "lv_mem_psram.c" "sdr_task.c" "ui.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_lcd esp_driver_gpio esp_driver_i2c esp_timer esp_sdr_s3 sdr_dsp lvgl)

# lv_mem_psram.c is only referenced from liblvgl, which links after main.
foreach(sym lv_mem_init lv_mem_deinit lv_mem_add_pool lv_mem_remove_pool lv_malloc_core
            lv_realloc_core lv_free_core lv_mem_monitor_core lv_mem_test_core)
  target_link_libraries(${COMPONENT_LIB} INTERFACE "-u ${sym}")
endforeach()
