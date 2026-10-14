#pragma once

#include <stdint.h>

#include "esp_err.h"
#include "esp_lcd_panel_ops.h"

/* Bring up the RGB panel with two RGB565 frame buffers in PSRAM (LVGL renders
 * into one while the other is scanned out) and switch the backlight on. */
esp_err_t display_init(esp_lcd_panel_handle_t *panel);

static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
  return (uint16_t)(((r & 0xf8) << 8) | ((g & 0xfc) << 3) | (b >> 3));
}
