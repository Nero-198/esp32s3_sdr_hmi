#include "sdr_dsp.h"

#include <math.h>

/* ---- Sweep stitching --------------------------------------------------- */

void sdr_trace_clear(sdr_trace_t *t) {
  for (size_t i = 0; i < t->n_points; i++) {
    t->dbfs[i] = SDR_DBFS_EMPTY;
    t->filled[i] = false;
  }
}

static double column_hz(const sdr_trace_t *t) {
  return t->n_points > 1 ? (t->f_stop_hz - t->f_start_hz) / (double)(t->n_points - 1)
                         : 0.0;
}

double sdr_trace_freq(const sdr_trace_t *t, size_t idx) {
  return t->f_start_hz + column_hz(t) * (double)idx;
}

bool sdr_trace_index(const sdr_trace_t *t, double f_hz, size_t *idx) {
  double col = column_hz(t);
  if (col <= 0.0 || f_hz < t->f_start_hz - col / 2 || f_hz > t->f_stop_hz + col / 2)
    return false;
  double x = floor((f_hz - t->f_start_hz) / col + 0.5);
  if (x < 0) x = 0;
  if (x > (double)(t->n_points - 1)) x = (double)(t->n_points - 1);
  *idx = (size_t)x;
  return true;
}

void sdr_trace_merge(sdr_trace_t *t, const float *spec_dbfs, size_t fft_n,
                     double centre_hz, double sample_rate_hz,
                     double usable_fraction) {
  if (fft_n < 2 || sample_rate_hz <= 0.0 || usable_fraction <= 0.0) return;
  if (usable_fraction > 1.0) usable_fraction = 1.0;
  double bin_hz = sample_rate_hz / (double)fft_n;
  double half_usable = sample_rate_hz * usable_fraction / 2.0;
  double width_bins = column_hz(t) / bin_hz;

  for (size_t i = 0; i < t->n_points; i++) {
    double f = sdr_trace_freq(t, i);
    if (fabs(f - centre_hz) > half_usable) continue;
    double b = (f - centre_hz) / bin_hz + (double)(fft_n / 2);
    float v;
    if (width_bins > 1.0) {
      long lo = (long)ceil(b - width_bins / 2);
      long hi = (long)floor(b + width_bins / 2);
      if (lo < 0) lo = 0;
      if (hi > (long)fft_n - 1) hi = (long)fft_n - 1;
      v = SDR_DBFS_EMPTY;
      for (long k = lo; k <= hi; k++)
        if (spec_dbfs[k] > v) v = spec_dbfs[k];
    } else {
      long k0 = (long)floor(b);
      if (k0 < 0) k0 = 0;
      if (k0 > (long)fft_n - 2) k0 = (long)fft_n - 2;
      float frac = (float)(b - (double)k0);
      if (frac < 0.0f) frac = 0.0f;
      if (frac > 1.0f) frac = 1.0f;
      v = spec_dbfs[k0] + (spec_dbfs[k0 + 1] - spec_dbfs[k0]) * frac;
    }
    if (!t->filled[i] || v > t->dbfs[i]) t->dbfs[i] = v;
    t->filled[i] = true;
  }
}

size_t sdr_sweep_plan(double f_start_hz, double f_stop_hz,
                      double usable_bw_hz, double *centres_hz,
                      size_t max_steps) {
  double span = f_stop_hz - f_start_hz;
  if (span < 0.0 || usable_bw_hz <= 0.0) return 0;
  size_t n = (size_t)ceil(span / usable_bw_hz);
  if (n == 0) n = 1;
  double step = span / (double)n;
  for (size_t i = 0; i < n && i < max_steps; i++)
    centres_hz[i] = f_start_hz + step * ((double)i + 0.5);
  return n;
}

/* ---- Trace post-processing -------------------------------------------- */

void sdr_post_reset(sdr_post_t *p) {
  p->avg_count = 0;
  p->peak_valid = false;
}

void sdr_post_peak_reset(sdr_post_t *p) { p->peak_valid = false; }

void sdr_post_update(sdr_post_t *p, const float *dbfs, float dt_s) {
  unsigned len = p->avg_len ? p->avg_len : 1;
  if (p->avg_count < len) p->avg_count++;
  float k = 1.0f / (float)p->avg_count;
  float decay = p->peak_decay_db_s > 0.0f && dt_s > 0.0f ? p->peak_decay_db_s * dt_s : 0.0f;
  for (size_t i = 0; i < p->n_points; i++) {
    if (p->avg_count == 1)
      p->avg[i] = dbfs[i];
    else
      p->avg[i] += (dbfs[i] - p->avg[i]) * k;
    float held = p->peak_valid ? p->peak[i] - decay : dbfs[i];
    p->peak[i] = dbfs[i] > held ? dbfs[i] : held;
  }
  p->peak_valid = true;
}

/* ---- Vertical range ---------------------------------------------------- */

#define HIST_MIN (-160.0f)
#define HIST_STEP 0.5f
#define HIST_BINS 340
#define FLOOR_PERCENTILE 0.2f
#define RANGE_MARGIN_DB 10.0f
#define RANGE_MIN_SPAN_DB 40.0f
#define RANGE_MAX_TOP_DB 10.0f

void sdr_range_update(sdr_range_t *r, const float *dbfs, size_t n,
                      float alpha) {
  if (!r->automatic || n == 0) return;
  unsigned hist[HIST_BINS] = {0};
  size_t counted = 0;
  float peak = SDR_DBFS_EMPTY;
  for (size_t i = 0; i < n; i++) {
    if (dbfs[i] <= SDR_DBFS_EMPTY) continue;
    int b = (int)((dbfs[i] - HIST_MIN) / HIST_STEP);
    if (b < 0) b = 0;
    if (b >= HIST_BINS) b = HIST_BINS - 1;
    hist[b]++;
    counted++;
    if (dbfs[i] > peak) peak = dbfs[i];
  }
  if (counted == 0) return;

  size_t want = (size_t)(FLOOR_PERCENTILE * (float)counted), seen = 0;
  int fb = 0;
  for (; fb < HIST_BINS - 1; fb++) {
    seen += hist[fb];
    if (seen > want) break;
  }
  float floor_db = HIST_MIN + HIST_STEP * (float)fb;

  float bottom = floor_db - RANGE_MARGIN_DB;
  float top = peak + RANGE_MARGIN_DB;
  if (top > RANGE_MAX_TOP_DB) top = RANGE_MAX_TOP_DB;
  if (top - bottom < RANGE_MIN_SPAN_DB) top = bottom + RANGE_MIN_SPAN_DB;

  if (alpha <= 0.0f || alpha > 1.0f || r->top_dbfs <= r->bottom_dbfs) alpha = 1.0f;
  r->top_dbfs += (top - r->top_dbfs) * alpha;
  r->bottom_dbfs += (bottom - r->bottom_dbfs) * alpha;
}

/* ---- Axis helpers ------------------------------------------------------ */

double sdr_wifi_channel_hz(int ch) {
  if (ch >= 1 && ch <= 13) return (2407.0 + 5.0 * ch) * 1e6;
  if (ch == 14) return 2484.0e6;
  return 0.0;
}

int sdr_wifi_channel_at(double f_hz) {
  int best = 0;
  double best_d = 2.5e6;
  for (int ch = 1; ch <= 14; ch++) {
    double d = fabs(f_hz - sdr_wifi_channel_hz(ch));
    if (d <= best_d) {
      best_d = d;
      best = ch;
    }
  }
  return best;
}

double sdr_tick_step_hz(double span_hz, unsigned max_ticks) {
  if (span_hz <= 0.0 || max_ticks == 0) return 0.0;
  double raw = span_hz / (double)max_ticks;
  double mag = pow(10.0, floor(log10(raw)));
  static const double steps[] = {1.0, 2.0, 5.0, 10.0};
  for (size_t i = 0; i < 4; i++)
    if (steps[i] * mag >= raw) return steps[i] * mag;
  return 10.0 * mag;
}

/* ---- Marker ------------------------------------------------------------ */

bool sdr_peak_index(const float *dbfs, const bool *filled, size_t n,
                    size_t *idx) {
  bool found = false;
  for (size_t i = 0; i < n; i++) {
    if (filled && !filled[i]) continue;
    if (!found || dbfs[i] > dbfs[*idx]) {
      *idx = i;
      found = true;
    }
  }
  return found;
}
