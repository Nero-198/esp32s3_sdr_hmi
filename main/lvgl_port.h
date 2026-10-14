#pragma once

#include "esp_err.h"
#include "esp_lcd_panel_ops.h"

/* Attach LVGL to the double-buffered RGB panel (direct mode, buffer swap on
 * frame completion, so no tearing) and the GT911 touch, call build_ui() under
 * the LVGL lock, then run lv_timer_handler() in a task pinned to core 0. */
esp_err_t lvgl_port_start(esp_lcd_panel_handle_t panel, void (*build_ui)(void));

/* Guard LVGL calls made from other tasks. */
void lvgl_port_lock(void);
void lvgl_port_unlock(void);
