/* Acquisition task: sweeps or zooms with ESP-SDR snapshots on core 1 and
 * publishes one stitched dBFS trace per pass. The UI reads the newest trace
 * and changes the mode; neither side blocks the other for long. */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#define SDR_COLS 680

#define SDR_SWEEP_START_HZ 2400e6
#define SDR_SWEEP_STOP_HZ 2500e6
#define SDR_ZOOM_SPAN_HZ 24e6

typedef enum { SDR_MODE_SWEEP, SDR_MODE_ZOOM } sdr_mode_t;

typedef struct {
  sdr_mode_t mode;
  double zoom_centre_hz; /* used in SDR_MODE_ZOOM */
} sdr_settings_t;

typedef struct {
  uint32_t seq;             /* increments per published trace */
  sdr_mode_t mode;
  double f_start_hz, f_stop_hz;
  float dbfs[SDR_COLS];
  uint32_t pass_us;         /* acquisition + DSP time of this trace */
  uint32_t failures;        /* capture failures since start */
} sdr_frame_t;

/* esp_sdr_s3_init() must have succeeded. */
esp_err_t sdr_task_start(void);
void sdr_task_set(const sdr_settings_t *s);
/* Copies the newest trace if its seq differs from last_seq. */
bool sdr_task_get(uint32_t last_seq, sdr_frame_t *out);
