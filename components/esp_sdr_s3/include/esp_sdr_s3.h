/* ESP32-S3 snapshot I/Q receiver built from ESP-SDR (GPL-3.0).
 *
 * Wraps the upstream S3 burst receiver without modifying it: commands go
 * through the upstream command handler, so tuning, gain and capture behave
 * exactly as with the stock firmware. Only the snapshot (CAP16) path is used;
 * the continuous ring modes need interrupts masked and are never started.
 *
 * Not thread-safe: call everything from one task. Wi-Fi is owned by this
 * module after esp_sdr_s3_init().
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Upstream rate indices. */
typedef enum {
  ESP_SDR_S3_RATE_80M = 0,
  ESP_SDR_S3_RATE_40M = 1,
  ESP_SDR_S3_RATE_16M = 6,
} esp_sdr_s3_rate_t;

#define ESP_SDR_S3_MAX_SAMPLES 16380u

/* Initialise NVS, Wi-Fi (promiscuous, no association) and the RX path. */
esp_err_t esp_sdr_s3_init(void);

/* Run one upstream protocol command (e.g. "FREQ 2437", "BANDWIDTH 69",
 * "GAIN MANUAL 40"). The text reply (without newline) is copied to reply if
 * non-NULL. Returns true when the reply does not start with "ERR". */
bool esp_sdr_s3_command(const char *line, char *reply, size_t reply_size);

/* Capture n_samples (256..16380) complex samples. On success *iq points to
 * 2 * n_samples signed 8-bit values (I, Q interleaved) that stay valid until
 * the next command, and *capture_us holds the RF capture time. */
bool esp_sdr_s3_capture(unsigned n_samples, esp_sdr_s3_rate_t rate,
                        const int8_t **iq, uint32_t *capture_us);

double esp_sdr_s3_rate_hz(esp_sdr_s3_rate_t rate);

/* Reuse the RX DC calibration for LOs within window_mhz of the last one
 * (refreshed every 30 s); 0 = upstream behaviour, recalibrate on every LO
 * change. Saves ~30 ms per retune when sweeping. */
void esp_sdr_s3_set_dc_reuse(unsigned window_mhz);

#ifdef __cplusplus
}
#endif
