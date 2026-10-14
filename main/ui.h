#pragma once

/* Build the analyzer screen. Call once under the LVGL lock after
 * sdr_task_start(); the UI then polls the SDR task from an LVGL timer. */
void ui_build(void);

#include <stdint.h>

/* Paint spectrum and waterfall into the RGB565 frame buffer LVGL is about to
 * render (row stride in pixels). Called by lvgl_port on LV_EVENT_RENDER_START. */
void ui_render(uint16_t *fb, int stride);
