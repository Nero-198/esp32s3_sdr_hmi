#include "sdr_task.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_sdr_s3.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdr_dsp.h"

#define SDR_TASK_CORE 1
#define SDR_TASK_PRIO 5
#define SDR_TASK_STACK (6 * 1024)

#define FFT_N 1024
#define USABLE 0.85
#define MAX_STEPS 4
/* Zoom tunes the LO this far above the window centre so the DC/LO residue
 * (filled over +-2 bins) does not sit under the marker. */
#define ZOOM_LO_OFFSET_HZ 5e6
/* Reuse RX DC calibration across the sweep LOs (2425/2475 MHz): retune drops
 * from ~33 ms to ~4.5 ms with no measurable change in DC level or noise
 * floor (A/B on hardware, README). */
#define DC_REUSE_WINDOW_MHZ 60

typedef struct {
  esp_sdr_s3_rate_t rate;
  unsigned bandwidth_mhz;
} rf_profile_t;

static const rf_profile_t SWEEP_RF = {ESP_SDR_S3_RATE_80M, 69};
static const rf_profile_t ZOOM_RF = {ESP_SDR_S3_RATE_40M, 34};

static const char *TAG = "sdr";
static SemaphoreHandle_t mutex;
static sdr_settings_t settings = {.mode = SDR_MODE_SWEEP};
static sdr_frame_t published;

static float spec[FFT_N], work[2 * FFT_N], acc[FFT_N];
static sdr_frame_t building;
static bool filled[SDR_COLS];


static void command(const char *fmt, unsigned v) {
  char c[32], r[64];
  snprintf(c, sizeof(c), fmt, v);
  if (!esp_sdr_s3_command(c, r, sizeof(r))) ESP_LOGW(TAG, "%s -> %s", c, r);
}

static void sdr_task(void *arg) {
  sdr_spectrum_cfg_t cfg = {.fft_n = FFT_N, .dc_bins = 2};
  unsigned lo_mhz = 0, bandwidth = 0;
  uint32_t failures = 0, seq = 0;
  for (;;) {
    xSemaphoreTake(mutex, portMAX_DELAY);
    sdr_settings_t s = settings;
    xSemaphoreGive(mutex);

    const rf_profile_t *rf = s.mode == SDR_MODE_ZOOM ? &ZOOM_RF : &SWEEP_RF;
    double fs = esp_sdr_s3_rate_hz(rf->rate);
    double centres[MAX_STEPS];
    size_t steps;
    building.mode = s.mode;
    if (s.mode == SDR_MODE_ZOOM) {
      building.f_start_hz = s.zoom_centre_hz - SDR_ZOOM_SPAN_HZ / 2;
      building.f_stop_hz = s.zoom_centre_hz + SDR_ZOOM_SPAN_HZ / 2;
      centres[0] = s.zoom_centre_hz + ZOOM_LO_OFFSET_HZ;
      steps = 1;
    } else {
      building.f_start_hz = SDR_SWEEP_START_HZ;
      building.f_stop_hz = SDR_SWEEP_STOP_HZ;
      steps = sdr_sweep_plan(SDR_SWEEP_START_HZ, SDR_SWEEP_STOP_HZ, fs * USABLE, centres,
                             MAX_STEPS);
      if (steps > MAX_STEPS) steps = MAX_STEPS;
    }
    if (bandwidth != rf->bandwidth_mhz) {
      bandwidth = rf->bandwidth_mhz;
      command("BANDWIDTH %u", bandwidth);
    }

    int64_t t0 = esp_timer_get_time();
    sdr_trace_t t = {building.f_start_hz, building.f_stop_hz, SDR_COLS, building.dbfs, filled};
    sdr_trace_clear(&t);
    for (size_t i = 0; i < steps; i++) {
      unsigned mhz = (unsigned)lround(centres[i] / 1e6);
      if (mhz != lo_mhz) {
        lo_mhz = mhz;
        command("FREQ %u", mhz);
      }
      const int8_t *iq;
      if (!esp_sdr_s3_capture(ESP_SDR_S3_MAX_SAMPLES, rf->rate, &iq, NULL)) {
        failures++;
        continue;
      }
      sdr_spectrum_dbfs(iq, ESP_SDR_S3_MAX_SAMPLES, &cfg, work, acc, spec);
      sdr_trace_merge(&t, spec, FFT_N, (double)mhz * 1e6, fs, USABLE);
    }
    building.pass_us = (uint32_t)(esp_timer_get_time() - t0);
    building.failures = failures;
    building.seq = ++seq;

    xSemaphoreTake(mutex, portMAX_DELAY);
    published = building;
    xSemaphoreGive(mutex);
    vTaskDelay(1);
  }
}

/* The target FFT is esp-dsp, not the host-tested C FFT: confirm bin order,
 * sign and dBFS scaling once at boot with a full-scale tone 100 bins above DC. */
static bool fft_self_test(void) {
  static int8_t iq[2 * FFT_N];
  for (int i = 0; i < FFT_N; i++) {
    double ph = 2.0 * M_PI * 100.0 * i / FFT_N;
    iq[2 * i] = (int8_t)lrint(127.0 * cos(ph));
    iq[2 * i + 1] = (int8_t)lrint(127.0 * sin(ph));
  }
  sdr_spectrum_cfg_t cfg = {.fft_n = FFT_N, .dc_bins = 0};
  if (sdr_spectrum_dbfs(iq, FFT_N, &cfg, work, acc, spec) != 1) return false;
  size_t pk = 0;
  for (size_t k = 1; k < FFT_N; k++)
    if (spec[k] > spec[pk]) pk = k;
  ESP_LOGI(TAG, "FFT self-test: peak bin %u (want %u), %.2f dBFS (want 0)", (unsigned)pk,
           FFT_N / 2 + 100, spec[pk]);
  return pk == FFT_N / 2 + 100 && fabsf(spec[pk]) < 0.3f;
}

esp_err_t sdr_task_start(void) {
  esp_sdr_s3_set_dc_reuse(DC_REUSE_WINDOW_MHZ);
  if (!fft_self_test()) {
    ESP_LOGE(TAG, "FFT self-test failed");
    return ESP_FAIL;
  }
  mutex = xSemaphoreCreateMutex();
  if (!mutex) return ESP_ERR_NO_MEM;
  return xTaskCreatePinnedToCore(sdr_task, "sdr", SDR_TASK_STACK, NULL, SDR_TASK_PRIO, NULL,
                                 SDR_TASK_CORE) == pdPASS
             ? ESP_OK
             : ESP_ERR_NO_MEM;
}

void sdr_task_set(const sdr_settings_t *s) {
  xSemaphoreTake(mutex, portMAX_DELAY);
  settings = *s;
  xSemaphoreGive(mutex);
}

bool sdr_task_get(uint32_t last_seq, sdr_frame_t *out) {
  bool fresh = false;
  xSemaphoreTake(mutex, portMAX_DELAY);
  if (published.seq != last_seq) {
    *out = published;
    fresh = true;
  }
  xSemaphoreGive(mutex);
  return fresh;
}
