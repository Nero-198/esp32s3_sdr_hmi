/* Host unit tests for components/sdr_dsp. Build and run:
 *   cmake -S tests/host -B build-host && cmake --build build-host && ctest --test-dir build-host
 */
#include "sdr_dsp.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures;

#define CHECK(cond)                                                       \
  do {                                                                    \
    if (!(cond)) {                                                        \
      fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
      failures++;                                                         \
    }                                                                     \
  } while (0)
#define CHECK_NEAR(a, b, tol) CHECK(fabs((double)(a) - (double)(b)) <= (tol))

#define PI 3.14159265358979323846

static void make_tone(int8_t *iq, size_t n, double cycles_per_sample,
                      double amp) {
  for (size_t i = 0; i < n; i++) {
    double ph = 2.0 * PI * cycles_per_sample * (double)i;
    iq[2 * i] = (int8_t)lrint(amp * cos(ph));
    iq[2 * i + 1] = (int8_t)lrint(amp * sin(ph));
  }
}

static size_t argmax(const float *v, size_t n) {
  size_t m = 0;
  for (size_t i = 1; i < n; i++)
    if (v[i] > v[m]) m = i;
  return m;
}

static void test_fft_impulse_and_tone(void) {
  float buf[2 * 16] = {0};
  buf[0] = 1.0f;
  CHECK(sdr_fft(buf, 16));
  for (int k = 0; k < 16; k++) {
    CHECK_NEAR(buf[2 * k], 1.0, 1e-6);
    CHECK_NEAR(buf[2 * k + 1], 0.0, 1e-6);
  }

  float tone[2 * 64];
  for (int i = 0; i < 64; i++) {
    tone[2 * i] = (float)cos(2 * PI * 5 * i / 64.0);
    tone[2 * i + 1] = (float)sin(2 * PI * 5 * i / 64.0);
  }
  CHECK(sdr_fft(tone, 64));
  CHECK_NEAR(tone[2 * 5], 64.0, 1e-3);
  CHECK_NEAR(tone[2 * 6], 0.0, 1e-3);

  CHECK(!sdr_fft(buf, 12));
  CHECK(!sdr_fft(buf, SDR_FFT_MAX_N * 2));
}

static void test_spectrum_full_scale_tone_is_0dbfs(void) {
  enum { N = 1024, SAMPLES = 4 * N };
  static int8_t iq[2 * SAMPLES];
  static float work[2 * N], acc[N], out[N];
  /* +100 bins above DC, exactly on a bin centre. */
  make_tone(iq, SAMPLES, 100.0 / N, 127.0);
  sdr_spectrum_cfg_t cfg = {.fft_n = N, .dc_bins = 0};
  CHECK(sdr_spectrum_dbfs(iq, SAMPLES, &cfg, work, acc, out) == 4);
  size_t pk = argmax(out, N);
  CHECK(pk == N / 2 + 100);
  CHECK_NEAR(out[pk], 0.0, 0.2);

  /* Negative frequency lands below the centre; -20 dB amplitude reads -20. */
  make_tone(iq, SAMPLES, -50.0 / N, 12.7);
  CHECK(sdr_spectrum_dbfs(iq, SAMPLES, &cfg, work, acc, out) == 4);
  pk = argmax(out, N);
  CHECK(pk == N / 2 - 50);
  CHECK_NEAR(out[pk], -20.0, 0.5);
}

static void test_spectrum_dc_fill_and_bad_args(void) {
  enum { N = 256 };
  static int8_t iq[2 * N];
  static float work[2 * N], acc[N], out[N];
  for (size_t i = 0; i < N; i++) {
    iq[2 * i] = (int8_t)(40 + (int)(i % 7) - 3); /* strong DC + small ripple */
    iq[2 * i + 1] = (int8_t)((int)(i % 5) - 2);
  }
  sdr_spectrum_cfg_t cfg = {.fft_n = N, .dc_bins = 0};
  CHECK(sdr_spectrum_dbfs(iq, N, &cfg, work, acc, out) == 1);
  float dc_raw = out[N / 2];
  cfg.dc_bins = 2;
  CHECK(sdr_spectrum_dbfs(iq, N, &cfg, work, acc, out) == 1);
  CHECK(out[N / 2] < dc_raw - 20.0f);
  /* Interpolated bins lie between the neighbours used. */
  float lo = out[N / 2 - 3], hi = out[N / 2 + 3];
  float mn = lo < hi ? lo : hi, mx = lo < hi ? hi : lo;
  CHECK(out[N / 2] >= mn - 1e-3f && out[N / 2] <= mx + 1e-3f);

  CHECK(sdr_spectrum_dbfs(iq, N - 1, &cfg, work, acc, out) == 0);
  cfg.fft_n = 100;
  CHECK(sdr_spectrum_dbfs(iq, N, &cfg, work, acc, out) == 0);
}

static void test_sweep_plan_covers_span(void) {
  double c[8];
  size_t n = sdr_sweep_plan(2400e6, 2500e6, 30e6, c, 8);
  CHECK(n == 4);
  CHECK_NEAR(c[0], 2412.5e6, 1);
  CHECK_NEAR(c[3], 2487.5e6, 1);
  CHECK(sdr_sweep_plan(2400e6, 2500e6, 200e6, c, 8) == 1);
  CHECK_NEAR(c[0], 2450e6, 1);
  CHECK(sdr_sweep_plan(2400e6, 2500e6, 0, c, 8) == 0);
  /* Reports the needed count even when the buffer is too small. */
  CHECK(sdr_sweep_plan(2400e6, 2500e6, 10e6, c, 2) == 10);
}

static void test_trace_merge_stitches_segments(void) {
  enum { FFT = 512, COLS = 681 };
  static float spec[FFT], dbfs[COLS];
  static bool filled[COLS];
  sdr_trace_t t = {2400e6, 2500e6, COLS, dbfs, filled};
  sdr_trace_clear(&t);

  /* Flat -80 dBFS with one -30 dBFS spike at +5 MHz from each LO. */
  const double fs = 40e6;
  double centres[4];
  size_t steps = sdr_sweep_plan(2400e6, 2500e6, fs * 0.75, centres, 4);
  CHECK(steps == 4);
  for (size_t s = 0; s < steps; s++) {
    for (size_t k = 0; k < FFT; k++) spec[k] = -80.0f;
    spec[FFT / 2 + (size_t)(5e6 / (fs / FFT))] = -30.0f;
    sdr_trace_merge(&t, spec, FFT, centres[s], fs, 0.75);
  }
  size_t unfilled = 0;
  for (size_t i = 0; i < COLS; i++) unfilled += !filled[i];
  CHECK(unfilled == 0);

  /* Columns are ~147 kHz wide, bins ~78 kHz: the max-hold must keep spikes. */
  for (size_t s = 0; s < steps; s++) {
    size_t idx;
    CHECK(sdr_trace_index(&t, centres[s] + 5e6, &idx));
    CHECK_NEAR(dbfs[idx], -30.0, 1e-3);
  }
  size_t idx;
  CHECK(sdr_trace_index(&t, 2401e6, &idx));
  CHECK_NEAR(dbfs[idx], -80.0, 1e-3);
  CHECK(!sdr_trace_index(&t, 2300e6, &idx));
  CHECK_NEAR(sdr_trace_freq(&t, 340), 2450e6, 1);
}

static void test_trace_merge_interpolates_when_zoomed(void) {
  enum { FFT = 64, COLS = 201 };
  static float spec[FFT], dbfs[COLS];
  static bool filled[COLS];
  for (size_t k = 0; k < FFT; k++) spec[k] = (float)k; /* ramp */
  /* 2 MHz span over 64 bins of 1 MHz: ~10 kHz columns -> interpolation. */
  sdr_trace_t t = {2449e6, 2451e6, COLS, dbfs, filled};
  sdr_trace_clear(&t);
  sdr_trace_merge(&t, spec, FFT, 2450e6, 64e6, 1.0);
  size_t idx;
  CHECK(sdr_trace_index(&t, 2450.5e6, &idx));
  CHECK_NEAR(dbfs[idx], 32.5, 1e-3);
}

static void test_post_average_and_peak(void) {
  float avg[2], peak[2];
  sdr_post_t p = {.n_points = 2, .avg = avg, .peak = peak, .avg_len = 4};
  sdr_post_reset(&p);
  const float a[2] = {-60, -80}, b[2] = {-40, -100};
  sdr_post_update(&p, a, 0.1f);
  CHECK_NEAR(avg[0], -60, 1e-4);
  sdr_post_update(&p, b, 0.1f);
  CHECK_NEAR(avg[0], -50, 1e-4);
  CHECK_NEAR(avg[1], -90, 1e-4);
  /* Max hold follows raw input, not the average. */
  CHECK_NEAR(peak[0], -40, 1e-4);
  CHECK_NEAR(peak[1], -80, 1e-4);
  sdr_post_peak_reset(&p);
  sdr_post_update(&p, b, 0.1f);
  CHECK_NEAR(peak[1], -100, 1e-4);

  /* avg_len 1 follows the input exactly. */
  p.avg_len = 1;
  sdr_post_reset(&p);
  sdr_post_update(&p, a, 0.1f);
  sdr_post_update(&p, b, 0.1f);
  CHECK_NEAR(avg[0], -40, 1e-4);
}

static void test_post_peak_decay(void) {
  float avg[1], peak[1];
  sdr_post_t p = {.n_points = 1, .avg = avg, .peak = peak, .avg_len = 1,
                  .peak_decay_db_s = 10.0f};
  sdr_post_reset(&p);
  const float burst[1] = {-30}, quiet[1] = {-80};
  sdr_post_update(&p, burst, 0.0f);
  sdr_post_update(&p, quiet, 0.5f);
  CHECK_NEAR(peak[0], -35, 1e-4);
  for (int i = 0; i < 10; i++) sdr_post_update(&p, quiet, 1.0f);
  CHECK_NEAR(peak[0], -80, 1e-4); /* never decays below the live trace */

  /* 0 dB/s holds forever. */
  p.peak_decay_db_s = 0.0f;
  sdr_post_update(&p, burst, 0.1f);
  for (int i = 0; i < 10; i++) sdr_post_update(&p, quiet, 1.0f);
  CHECK_NEAR(peak[0], -30, 1e-4);
}

static void test_range_auto_and_fixed(void) {
  enum { N = 500 };
  float v[N];
  for (int i = 0; i < N; i++) v[i] = -90.0f + (float)(i % 3);
  v[123] = -35.0f;
  sdr_range_t r = {.automatic = true};
  sdr_range_update(&r, v, N, 0.1f); /* first update jumps */
  CHECK_NEAR(r.bottom_dbfs, -100.0, 1.0);
  CHECK_NEAR(r.top_dbfs, -25.0, 1e-3);

  /* Quiet trace: minimum span applies. Smoothing moves 10 % per call. */
  for (int i = 0; i < N; i++) v[i] = -90.0f;
  sdr_range_update(&r, v, N, 0.1f);
  CHECK_NEAR(r.top_dbfs, -25.0 + 0.1 * (-60.0 + 25.0), 1e-3);

  sdr_range_t fixed = {.automatic = false, .top_dbfs = 0, .bottom_dbfs = -100};
  sdr_range_update(&fixed, v, N, 1.0f);
  CHECK(fixed.top_dbfs == 0 && fixed.bottom_dbfs == -100);
}

static void test_axis_helpers(void) {
  CHECK_NEAR(sdr_wifi_channel_hz(1), 2412e6, 1);
  CHECK_NEAR(sdr_wifi_channel_hz(13), 2472e6, 1);
  CHECK_NEAR(sdr_wifi_channel_hz(14), 2484e6, 1);
  CHECK(sdr_wifi_channel_hz(0) == 0.0);
  CHECK(sdr_wifi_channel_at(2437.4e6) == 6);
  CHECK(sdr_wifi_channel_at(2478.5e6) == 0);
  CHECK(sdr_wifi_channel_at(2483e6) == 14);
  CHECK_NEAR(sdr_tick_step_hz(100e6, 10), 10e6, 1);
  CHECK_NEAR(sdr_tick_step_hz(100e6, 8), 20e6, 1);
  CHECK_NEAR(sdr_tick_step_hz(20e6, 8), 5e6, 1);
}

static void test_peak_index_skips_unfilled(void) {
  float v[4] = {-50, -10, -30, -20};
  bool f[4] = {true, false, true, true};
  size_t idx;
  CHECK(sdr_peak_index(v, f, 4, &idx) && idx == 3);
  CHECK(sdr_peak_index(v, NULL, 4, &idx) && idx == 1);
  bool none[4] = {false};
  CHECK(!sdr_peak_index(v, none, 4, &idx));
}

int main(void) {
  test_fft_impulse_and_tone();
  test_spectrum_full_scale_tone_is_0dbfs();
  test_spectrum_dc_fill_and_bad_args();
  test_sweep_plan_covers_span();
  test_trace_merge_stitches_segments();
  test_trace_merge_interpolates_when_zoomed();
  test_post_average_and_peak();
  test_post_peak_decay();
  test_range_auto_and_fixed();
  test_axis_helpers();
  test_peak_index_skips_unfilled();
  if (failures) {
    fprintf(stderr, "%d check(s) failed\n", failures);
    return 1;
  }
  printf("all sdr_dsp tests passed\n");
  return 0;
}
