/* ESP32-S3 HMI 2.4 GHz spectrum analyzer.
 *
 * Core 1: sdr_task (ESP-SDR snapshots -> stitched dBFS traces).
 * Core 0: LVGL (double-buffered RGB panel, GT911 touch, ui.c).
 * The receiver is initialised first so Wi-Fi gets its internal RAM before the
 * LCD bounce buffers and LVGL take theirs.
 */
#include <stdio.h>

#include "display.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_sdr_s3.h"
#include "lvgl_port.h"
#include "sdr_task.h"
#include "touch_gt911.h"
#include "ui.h"

#define GAIN_CODE 50

static const char *TAG = "app";

void app_main(void) {
  ESP_ERROR_CHECK(esp_sdr_s3_init());
  char reply[64];
  char cmd[24];
  snprintf(cmd, sizeof(cmd), "GAIN MANUAL %d", GAIN_CODE);
  if (!esp_sdr_s3_command(cmd, reply, sizeof(reply))) ESP_LOGW(TAG, "%s -> %s", cmd, reply);
  ESP_ERROR_CHECK(sdr_task_start());

  esp_lcd_panel_handle_t panel;
  ESP_ERROR_CHECK(display_init(&panel));
  char id[5];
  if (touch_gt911_init(id) != ESP_OK) ESP_LOGE(TAG, "GT911 not responding; touch disabled");
  lvgl_port_set_render_hook(ui_render);
  ESP_ERROR_CHECK(lvgl_port_start(panel, ui_build));
  ESP_LOGI(TAG, "running, internal heap free %u (min %u)",
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
           (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL));
}
