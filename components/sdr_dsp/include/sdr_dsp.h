/* Hardware-independent DSP for the ESP32-S3 HMI spectrum analyzer.
 *
 * Everything here is plain C99 and builds both inside ESP-IDF and on the
 * host (tests/host). Levels are relative dBFS: a full-scale complex tone
 * (|I + jQ| = 127 on signed 8-bit samples) reads 0 dBFS.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SDR_FFT_MAX_N 2048
/* Value of trace columns that have no data. */
#define SDR_DBFS_EMPTY (-200.0f)

/* ---- FFT / spectrum ---------------------------------------------------- */

/* In-place complex radix-2 FFT. Not reentrant (shared twiddle/window tables);
 * call from a single task. n must be a power of two <= SDR_FFT_MAX_N.
 * re/im are interleaved: buf[2k] = re, buf[2k+1] = im. */
bool sdr_fft(float *buf, size_t n);

typedef struct {
  size_t fft_n;      /* power of two, <= SDR_FFT_MAX_N */
  size_t dc_bins;    /* bins on each side of DC replaced by interpolation */
} sdr_spectrum_cfg_t;

/* Welch-averaged power spectrum of signed 8-bit interleaved I/Q.
 * iq holds n_samples complex samples (2 * n_samples bytes). Uses as many
 * non-overlapping Hann-windowed segments of fft_n as fit. Output is fft_n
 * dBFS values, DC-centred (index fft_n/2 is the LO frequency).
 * work must hold 2 * fft_n floats; acc must hold fft_n floats.
 * Returns the number of segments averaged (0 on bad arguments). */
size_t sdr_spectrum_dbfs(const int8_t *iq, size_t n_samples,
                         const sdr_spectrum_cfg_t *cfg,
                         float *work, float *acc, float *out_dbfs);

/* ---- Sweep stitching --------------------------------------------------- */

/* A display trace covering [f_start_hz, f_stop_hz] with n_points columns. */
typedef struct {
  double f_start_hz;
  double f_stop_hz;
  size_t n_points;
  float *dbfs;       /* n_points values */
  bool *filled;      /* n_points flags, which columns got data */
} sdr_trace_t;

void sdr_trace_clear(sdr_trace_t *t);

/* Column index nearest to f_hz; returns false if outside the trace. */
bool sdr_trace_index(const sdr_trace_t *t, double f_hz, size_t *idx);
double sdr_trace_freq(const sdr_trace_t *t, size_t idx);

/* Merge one DC-centred spectrum (fft_n bins, LO at centre_hz, spanning
 * sample_rate_hz) into the trace. Only the central usable_fraction of the
 * spectrum (0 < f <= 1) is used, to drop the anti-alias filter skirts.
 * Each trace column takes the maximum of the bins that fall into it; columns
 * narrower than a bin are interpolated linearly. Where segments overlap the
 * larger value wins. */
void sdr_trace_merge(sdr_trace_t *t, const float *spec_dbfs, size_t fft_n,
                     double centre_hz, double sample_rate_hz,
                     double usable_fraction);

/* LO centre frequencies needed to cover the trace with the given usable
 * bandwidth. Writes up to max_steps entries; returns the count needed. */
size_t sdr_sweep_plan(double f_start_hz, double f_stop_hz,
                      double usable_bw_hz, double *centres_hz,
                      size_t max_steps);

/* ---- Trace post-processing -------------------------------------------- */

/* Averaging is done on dBFS values (log-power "video" averaging), which reads
 * noise about 2.5 dB lower than power averaging but needs no extra buffer.
 * The max-hold trace follows the raw input, not the average, so short bursts
 * (Wi-Fi frames, Bluetooth hops) stay visible. It falls by peak_decay_db_s per
 * second towards newer data; 0 holds forever. */
typedef struct {
  size_t n_points;
  float *avg;        /* running average, dBFS */
  float *peak;       /* max hold, dBFS */
  unsigned avg_count;   /* frames averaged so far, saturates at avg_len */
  unsigned avg_len;     /* 1, 4, 16 ... */
  float peak_decay_db_s;
  bool peak_valid;
} sdr_post_t;

void sdr_post_reset(sdr_post_t *p);
void sdr_post_peak_reset(sdr_post_t *p);
/* Feed one new trace taken dt_s seconds after the previous one; afterwards
 * p->avg and p->peak hold the results. */
void sdr_post_update(sdr_post_t *p, const float *dbfs, float dt_s);

/* ---- Vertical range ---------------------------------------------------- */

typedef struct {
  bool automatic;
  float top_dbfs;
  float bottom_dbfs;
} sdr_range_t;

/* Track noise floor and peak of a trace. In automatic mode the range moves
 * towards [floor - margin, peak + margin] with smoothing alpha (0..1] and a
 * minimum span; in fixed mode it is left unchanged. */
void sdr_range_update(sdr_range_t *r, const float *dbfs, size_t n,
                      float alpha);

/* ---- Axis helpers ------------------------------------------------------ */

/* 2.4 GHz Wi-Fi channel centre (1..14) in Hz, 0 for invalid channels. */
double sdr_wifi_channel_hz(int ch);
/* Nearest Wi-Fi channel to f_hz, or 0 if no channel centre is within
 * 2.5 MHz (channel 14 is 12 MHz from channel 13, so gaps exist). */
int sdr_wifi_channel_at(double f_hz);

/* "Nice" tick step (1/2/5 x 10^k Hz) giving at most max_ticks over span. */
double sdr_tick_step_hz(double span_hz, unsigned max_ticks);

/* ---- Marker ------------------------------------------------------------ */

/* Index of the strongest filled column, or false if none is filled. */
bool sdr_peak_index(const float *dbfs, const bool *filled, size_t n,
                    size_t *idx);

#ifdef __cplusplus
}
#endif
