/* Adapter around the unmodified ESP-SDR S3 receiver.
 *
 * The upstream receiver keeps its state and helpers static, so it is compiled
 * as part of this translation unit. Its app_main is renamed and never called;
 * esp_sdr_s3_init() repeats the radio part of it without the USB/serial
 * transport. Replies that upstream would send to the host land in reply_buf
 * through the burst_serial shim in serial_shim.c.
 */
#define app_main esp_sdr_upstream_app_main
#include "targets/esp32s3/receiver.c"
#undef app_main

#include "esp_check.h"
#include "esp_sdr_s3.h"
#include "serial_shim.h"

esp_err_t esp_sdr_s3_init(void) {
  esp_err_t e = nvs_flash_init();
  if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_RETURN_ON_ERROR(nvs_flash_erase(), "esp_sdr", "nvs erase");
    e = nvs_flash_init();
  }
  ESP_RETURN_ON_ERROR(e, "esp_sdr", "nvs");
  e = esp_event_loop_create_default();
  if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) return e;
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), "esp_sdr", "wifi init");
  ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), "esp_sdr", "storage");
  ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_NULL), "esp_sdr", "mode");
  ESP_RETURN_ON_ERROR(esp_wifi_start(), "esp_sdr", "start");
  ESP_RETURN_ON_ERROR(esp_wifi_set_ps(WIFI_PS_NONE), "esp_sdr", "ps");
  ESP_RETURN_ON_ERROR(esp_wifi_set_promiscuous(true), "esp_sdr", "promiscuous");
  ESP_RETURN_ON_ERROR(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE), "esp_sdr", "channel");
  prepare_rx();
  return ESP_OK;
}

bool esp_sdr_s3_command(const char *line, char *reply_out, size_t reply_size) {
  char buf[128];
  snprintf(buf, sizeof(buf), "%s", line);
  serial_shim_begin();
  handle_command(buf);
  const char *r = serial_shim_reply();
  if (reply_out && reply_size) {
    size_t n = strcspn(r, "\n");
    if (n >= reply_size) n = reply_size - 1;
    memcpy(reply_out, r, n);
    reply_out[n] = '\0';
  }
  return strncmp(r, "ERR", 3) != 0;
}

bool esp_sdr_s3_capture(unsigned n_samples, esp_sdr_s3_rate_t rate,
                        const int8_t **iq, uint32_t *capture_us) {
  char cmd[32], r[96];
  snprintf(cmd, sizeof(cmd), "CAP16 %u %u", n_samples, (unsigned)rate);
  if (!esp_sdr_s3_command(cmd, r, sizeof(r))) return false;
  unsigned n = 0, us = 0, crc = 0;
  if (sscanf(r, "DATA %u %x %u", &n, &crc, &us) != 3 || n != n_samples) return false;
  *iq = (const int8_t *)IQ_BUFFER;
  if (capture_us) *capture_us = us;
  return true;
}

double esp_sdr_s3_rate_hz(esp_sdr_s3_rate_t rate) {
  switch (rate) {
    case ESP_SDR_S3_RATE_80M: return 80e6;
    case ESP_SDR_S3_RATE_40M: return 40e6;
    case ESP_SDR_S3_RATE_16M: return 16e6;
  }
  return 0.0;
}
