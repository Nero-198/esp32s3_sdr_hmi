#include "sdr_dsp.h"

#include <math.h>

#ifdef ESP_PLATFORM
#include "dsps_fft2r.h"
#endif

#define SDR_PI 3.14159265358979323846
/* Amplitude of a full-scale complex tone on signed 8-bit samples. */
#define FULL_SCALE 127.0f

#ifndef ESP_PLATFORM
static float twiddle_cos[SDR_FFT_MAX_N / 2];
static float twiddle_sin[SDR_FFT_MAX_N / 2];
static bool twiddle_ready;
#endif

static float window[SDR_FFT_MAX_N];
static float window_sum;
static size_t window_n;

static bool is_pow2(size_t n) { return n >= 2 && (n & (n - 1)) == 0; }

#ifndef ESP_PLATFORM
static void twiddle_init(void) {
  if (twiddle_ready) return;
  for (size_t k = 0; k < SDR_FFT_MAX_N / 2; k++) {
    double a = -2.0 * SDR_PI * (double)k / SDR_FFT_MAX_N;
    twiddle_cos[k] = (float)cos(a);
    twiddle_sin[k] = (float)sin(a);
  }
  twiddle_ready = true;
}
#endif

bool sdr_fft(float *buf, size_t n) {
  if (!buf || !is_pow2(n) || n > SDR_FFT_MAX_N) return false;
#ifdef ESP_PLATFORM
  /* esp-dsp radix-2 (SIMD on ESP32-S3), same forward e^-j convention. */
  static bool dsp_ready;
  if (!dsp_ready) {
    if (dsps_fft2r_init_fc32(NULL, SDR_FFT_MAX_N) != ESP_OK) return false;
    dsp_ready = true;
  }
  dsps_fft2r_fc32(buf, (int)n);
  dsps_bit_rev_fc32(buf, (int)n);
  return true;
#else
  twiddle_init();

  for (size_t i = 1, j = 0; i < n; i++) {
    size_t bit = n >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j |= bit;
    if (i < j) {
      float tr = buf[2 * i], ti = buf[2 * i + 1];
      buf[2 * i] = buf[2 * j];
      buf[2 * i + 1] = buf[2 * j + 1];
      buf[2 * j] = tr;
      buf[2 * j + 1] = ti;
    }
  }

  for (size_t len = 2; len <= n; len <<= 1) {
    size_t half = len >> 1;
    size_t stride = SDR_FFT_MAX_N / len;
    for (size_t i = 0; i < n; i += len) {
      for (size_t k = 0; k < half; k++) {
        float wr = twiddle_cos[k * stride], wi = twiddle_sin[k * stride];
        float *a = &buf[2 * (i + k)];
        float *b = &buf[2 * (i + k + half)];
        float tr = b[0] * wr - b[1] * wi;
        float ti = b[0] * wi + b[1] * wr;
        b[0] = a[0] - tr;
        b[1] = a[1] - ti;
        a[0] += tr;
        a[1] += ti;
      }
    }
  }
  return true;
#endif
}

static void window_init(size_t n) {
  if (window_n == n) return;
  window_sum = 0.0f;
  for (size_t i = 0; i < n; i++) {
    window[i] = (float)(0.5 - 0.5 * cos(2.0 * SDR_PI * (double)i / (double)n));
    window_sum += window[i];
  }
  window_n = n;
}

static void fill_dc(float *out, size_t n, size_t dc_bins) {
  size_t c = n / 2;
  if (dc_bins == 0 || dc_bins + 1 > c - 1) return;
  size_t lo = c - dc_bins - 1, hi = c + dc_bins + 1;
  float a = out[lo], b = out[hi];
  for (size_t k = lo + 1; k < hi; k++)
    out[k] = a + (b - a) * (float)(k - lo) / (float)(hi - lo);
}

size_t sdr_spectrum_dbfs(const int8_t *iq, size_t n_samples,
                         const sdr_spectrum_cfg_t *cfg,
                         float *work, float *acc, float *out_dbfs) {
  if (!iq || !cfg || !work || !acc || !out_dbfs) return 0;
  size_t n = cfg->fft_n;
  if (!is_pow2(n) || n > SDR_FFT_MAX_N || n_samples < n) return 0;
  window_init(n);

  for (size_t k = 0; k < n; k++) acc[k] = 0.0f;
  size_t segments = n_samples / n;
  for (size_t s = 0; s < segments; s++) {
    const int8_t *seg = iq + 2 * s * n;
    for (size_t i = 0; i < n; i++) {
      work[2 * i] = (float)seg[2 * i] * window[i];
      work[2 * i + 1] = (float)seg[2 * i + 1] * window[i];
    }
    sdr_fft(work, n);
    for (size_t k = 0; k < n; k++)
      acc[k] += work[2 * k] * work[2 * k] + work[2 * k + 1] * work[2 * k + 1];
  }

  float ref = FULL_SCALE * window_sum;
  float scale = 1.0f / ((float)segments * ref * ref);
  for (size_t k = 0; k < n; k++) {
    size_t src = (k + n / 2) & (n - 1); /* fftshift */
    float p = acc[src] * scale;
    out_dbfs[k] = p > 1e-20f ? 10.0f * log10f(p) : SDR_DBFS_EMPTY;
  }
  fill_dc(out_dbfs, n, cfg->dc_bins);
  return segments;
}
