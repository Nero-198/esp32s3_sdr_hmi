/* Spectrum analyzer screen (800x480).
 *
 *   +----------------------------------------------+--------+
 *   | spectrum 680x210 (avg yellow, hold cyan,     | FULL   |
 *   | marker magenta, dBFS grid labels)            | ZOOM   |
 *   +-- axis strip 680x32 (tap: ch <-> MHz) -------+ PEAK   |
 *   | waterfall 680x204 (raw traces)               | HOLD   |
 *   |                                              | AVG    |
 *   +----------------------------------------------+ AUTO   |
 *   | status line                                           |
 *   +-------------------------------------------------------+
 *
 * Touch: tap spectrum/waterfall = marker, tap axis = ch <-> MHz,
 * HOLD short = infinite hold on/off, HOLD long = clear hold.
 *
 * Spectrum and waterfall are not LVGL widgets: ui_render() paints them straight
 * into the frame buffer LVGL is about to render (LV_EVENT_RENDER_START), and
 * LVGL then draws only the small invalidated widgets on top. This avoids a
 * PSRAM canvas, its blit and the double-buffer sync copy (~1.4 MB per frame).
 * Labels that sit on the plots are saved and restored around the painting so
 * they survive renders in which LVGL does not redraw them.
 */
#include "ui.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "lvgl.h"
#include "sdr_dsp.h"
#include "sdr_task.h"

#define W SDR_COLS
#define SPEC_H 210
#define AXIS_Y SPEC_H
#define AXIS_H 32
#define WF_Y (AXIS_Y + AXIS_H)
#define WF_H 204
#define STATUS_Y (WF_Y + WF_H + 2)
#define BTN_X (W + 8)
#define BTN_W 104
#define BTN_H 66
#define BTN_GAP 8

#define MAX_AXIS_LABELS 16
#define AXIS_LABEL_W 72
#define MAX_DB_LABELS 10
#define HOLD_DECAY_DB_S 10.0f
#define OVERLAY_SAVE_PX (24 * 1024)
#define BAND_H 21

static const unsigned AVG_STEPS[] = {1, 4, 16};

static uint16_t *wf_ring;   /* WF_H rows of W px, newest at wf_head */
static int wf_head;
static uint16_t *overlay_save;
static int fb_stride;
static uint16_t wf_lut[256];
static lv_obj_t *spec_area, *wf_area, *axis_strip, *marker_label, *status_label;
static lv_obj_t *axis_labels[MAX_AXIS_LABELS], *db_labels[MAX_DB_LABELS];
static lv_obj_t *btn_full, *btn_zoom, *btn_hold, *btn_avg, *btn_auto;
static int grid_x[MAX_AXIS_LABELS], n_grid;

static sdr_frame_t frame;
static float avg_db[W], peak_db[W];
static sdr_post_t post = {.n_points = W, .avg = avg_db, .peak = peak_db, .avg_len = 4,
                          .peak_decay_db_s = HOLD_DECAY_DB_S};
static sdr_range_t range = {.automatic = true};
static sdr_settings_t settings = {.mode = SDR_MODE_SWEEP};

static bool have_frame, axis_mhz, hold_infinite;
static unsigned avg_idx = 1;
static double marker_hz; /* 0 = no marker */
static sdr_mode_t shown_mode;
static double shown_start, shown_stop;
static int64_t last_frame_us;
static float fps;

static inline uint16_t rgb(uint32_t hex) {
  return (uint16_t)(((hex >> 8) & 0xf800) | ((hex >> 5) & 0x07e0) | ((hex >> 3) & 0x001f));
}

static void build_lut(void) {
  /* black -> blue -> cyan -> yellow -> red */
  static const uint32_t stops[] = {0x000000, 0x0000b4, 0x00c8ff, 0xffdc00, 0xff2020};
  for (int i = 0; i < 256; i++) {
    float p = i / 255.0f * 4.0f;
    int s = p >= 4.0f ? 3 : (int)p;
    float f = p - s;
    uint32_t a = stops[s], b = stops[s + 1];
    uint32_t r = (uint32_t)(((a >> 16) & 255) * (1 - f) + ((b >> 16) & 255) * f);
    uint32_t g = (uint32_t)(((a >> 8) & 255) * (1 - f) + ((b >> 8) & 255) * f);
    uint32_t bl = (uint32_t)((a & 255) * (1 - f) + (b & 255) * f);
    wf_lut[i] = rgb((r << 16) | (g << 8) | bl);
  }
}

/* ---- Coordinates -------------------------------------------------------- */

static int freq_to_x(double f) {
  return (int)lround((f - shown_start) / (shown_stop - shown_start) * (W - 1));
}

static double x_to_freq(int x) { return shown_start + (shown_stop - shown_start) * x / (W - 1); }

static int db_to_y(float v) {
  int y = (int)((range.top_dbfs - v) / (range.top_dbfs - range.bottom_dbfs) * (SPEC_H - 1));
  return y < 0 ? 0 : y >= SPEC_H ? SPEC_H - 1 : y;
}

/* ---- Rendering (into the frame buffer, see ui_render) --------------------- */

/* The spectrum is composed in internal-RAM bands of BAND_H rows and copied
 * to the PSRAM frame buffer row by row: drawing trace columns straight into
 * PSRAM touches a new cache line per pixel and was ~4x slower. */
static uint16_t *band;
static int16_t y_peak[W], y_avg[W];

static void band_vline(int x, int lo, int hi, int y0, int h, uint16_t c, int dash) {
  if (x < 0 || x >= W) return;
  if (lo < y0) lo = y0;
  if (hi > y0 + h - 1) hi = y0 + h - 1;
  for (int y = lo; y <= hi; y++)
    if (!dash || (y / dash) % 2 == 0) band[(y - y0) * W + x] = c;
}

static void band_trace(const int16_t *ys, int y0, int h, uint16_t c) {
  int prev = ys[0];
  for (int x = 0; x < W; x++) {
    int y = ys[x];
    int lo = prev < y ? prev : y, hi = prev < y ? y : prev;
    if (hi >= y0 && lo < y0 + h) band_vline(x, lo, hi, y0, h, c, 0);
    prev = y;
  }
}

static void draw_spectrum(uint16_t *fb) {
  const uint16_t grid = rgb(0x303030);
  int grid_rows[16], n_rows = 0;
  if (have_frame) {
    for (int db = (int)ceilf(range.bottom_dbfs / 10.0f) * 10; db <= range.top_dbfs && n_rows < 16;
         db += 10)
      grid_rows[n_rows++] = db_to_y((float)db);
    for (int x = 0; x < W; x++) {
      y_peak[x] = (int16_t)db_to_y(peak_db[x]);
      y_avg[x] = (int16_t)db_to_y(avg_db[x]);
    }
  }
  int mx = marker_hz > 0 && have_frame ? freq_to_x(marker_hz) : -1;
  for (int y0 = 0; y0 < SPEC_H; y0 += BAND_H) {
    int h = SPEC_H - y0 < BAND_H ? SPEC_H - y0 : BAND_H;
    memset(band, 0, (size_t)W * h * sizeof(uint16_t));
    if (have_frame) {
      for (int i = 0; i < n_rows; i++)
        if (grid_rows[i] >= y0 && grid_rows[i] < y0 + h)
          for (int x = 0; x < W; x++) band[(grid_rows[i] - y0) * W + x] = grid;
      for (int i = 0; i < n_grid; i++) band_vline(grid_x[i], y0, y0 + h - 1, y0, h, grid, 0);
      band_trace(y_peak, y0, h, rgb(0x00c8ff));
      band_trace(y_avg, y0, h, rgb(0xffd000));
      band_vline(mx, y0, y0 + h - 1, y0, h, rgb(0xff40ff), 4);
    }
    for (int r = 0; r < h; r++)
      memcpy(fb + (y0 + r) * fb_stride, band + r * W, W * sizeof(uint16_t));
  }
}

static void draw_waterfall(uint16_t *fb) {
  for (int y = 0; y < WF_H; y++)
    memcpy(fb + (WF_Y + y) * fb_stride, wf_ring + ((wf_head + y) % WF_H) * W,
           W * sizeof(uint16_t));
}

/* Copy the on-plot label rectangles between the frame buffer and
 * overlay_save (save = true) or back (save = false). */
static void overlay_copy(uint16_t *fb, bool save) {
  lv_obj_t *objs[MAX_DB_LABELS + 1];
  int n = 0;
  for (int i = 0; i < MAX_DB_LABELS; i++) objs[n++] = db_labels[i];
  objs[n++] = marker_label;
  size_t used = 0;
  for (int i = 0; i < n; i++) {
    if (lv_obj_has_flag(objs[i], LV_OBJ_FLAG_HIDDEN)) continue;
    lv_area_t a;
    lv_obj_get_coords(objs[i], &a);
    if (a.x1 < 0) a.x1 = 0;
    if (a.y1 < 0) a.y1 = 0;
    if (a.x2 >= W) a.x2 = W - 1;
    if (a.y2 >= SPEC_H) a.y2 = SPEC_H - 1;
    int w = a.x2 - a.x1 + 1, h = a.y2 - a.y1 + 1;
    if (w <= 0 || h <= 0 || used + (size_t)w * h > OVERLAY_SAVE_PX) continue;
    for (int y = a.y1; y <= a.y2; y++, used += w) {
      uint16_t *px = fb + y * fb_stride + a.x1;
      if (save)
        memcpy(overlay_save + used, px, w * sizeof(uint16_t));
      else
        memcpy(px, overlay_save + used, w * sizeof(uint16_t));
    }
  }
}

void ui_render(uint16_t *fb, int stride) {
  fb_stride = stride;
  overlay_copy(fb, true);
  draw_spectrum(fb);
  draw_waterfall(fb);
  overlay_copy(fb, false);
}

static void push_waterfall_row(const float *v) {
  wf_head = (wf_head + WF_H - 1) % WF_H;
  uint16_t *row = wf_ring + wf_head * W;
  float span = range.top_dbfs - range.bottom_dbfs;
  for (int x = 0; x < W; x++) {
    int i = (int)((v[x] - range.bottom_dbfs) / span * 255.0f);
    row[x] = wf_lut[i < 0 ? 0 : i > 255 ? 255 : i];
  }
}

static void update_db_labels(void) {
  int n = 0;
  for (int db = (int)ceilf(range.bottom_dbfs / 10.0f) * 10; db <= range.top_dbfs && n < MAX_DB_LABELS;
       db += 10, n++) {
    int y = db_to_y((float)db);
    lv_label_set_text_fmt(db_labels[n], "%d", db);
    lv_obj_set_pos(db_labels[n], 2, y < 14 ? y + 1 : y - 15);
    lv_obj_remove_flag(db_labels[n], LV_OBJ_FLAG_HIDDEN);
    lv_obj_invalidate(db_labels[n]);
  }
  for (; n < MAX_DB_LABELS; n++) lv_obj_add_flag(db_labels[n], LV_OBJ_FLAG_HIDDEN);
}

/* ---- Labels ------------------------------------------------------------- */

static void update_axis(void) {
  n_grid = 0;
  if (axis_mhz) {
    double step = sdr_tick_step_hz(shown_stop - shown_start, 8);
    for (double f = ceil(shown_start / step) * step; f <= shown_stop + 1 && n_grid < MAX_AXIS_LABELS;
         f += step) {
      lv_label_set_text_fmt(axis_labels[n_grid], "%.0f MHz", f / 1e6);
      grid_x[n_grid++] = freq_to_x(f);
    }
  } else {
    for (int ch = 1; ch <= 14 && n_grid < MAX_AXIS_LABELS; ch++) {
      double f = sdr_wifi_channel_hz(ch);
      if (f < shown_start || f > shown_stop) continue;
      lv_label_set_text_fmt(axis_labels[n_grid], "ch%d", ch);
      grid_x[n_grid++] = freq_to_x(f);
    }
  }
  for (int i = 0; i < MAX_AXIS_LABELS; i++) {
    if (i < n_grid) {
      lv_obj_set_x(axis_labels[i], grid_x[i] - AXIS_LABEL_W / 2);
      lv_obj_remove_flag(axis_labels[i], LV_OBJ_FLAG_HIDDEN);
    } else {
      lv_obj_add_flag(axis_labels[i], LV_OBJ_FLAG_HIDDEN);
    }
  }
}

static void update_marker_label(void) {
  if (marker_hz <= 0 || !have_frame) {
    lv_obj_add_flag(marker_label, LV_OBJ_FLAG_HIDDEN);
    return;
  }
  int x = freq_to_x(marker_hz);
  if (x < 0 || x >= W) {
    lv_obj_add_flag(marker_label, LV_OBJ_FLAG_HIDDEN);
    return;
  }
  int ch = sdr_wifi_channel_at(marker_hz);
  char chs[8] = "";
  if (ch) snprintf(chs, sizeof(chs), "ch%d", ch);
  lv_label_set_text_fmt(marker_label, "MKR %.2f MHz %s  avg %.1f  hold %.1f dBFS", marker_hz / 1e6,
                        chs, avg_db[x], peak_db[x]);
  lv_obj_remove_flag(marker_label, LV_OBJ_FLAG_HIDDEN);
  lv_obj_invalidate(marker_label);
}

static void set_btn_text(lv_obj_t *btn, const char *text) {
  lv_label_set_text(lv_obj_get_child(btn, 0), text);
}

static void update_buttons(void) {
  lv_obj_set_state(btn_full, LV_STATE_CHECKED, settings.mode == SDR_MODE_SWEEP);
  lv_obj_set_state(btn_zoom, LV_STATE_CHECKED, settings.mode == SDR_MODE_ZOOM);
  lv_obj_set_state(btn_hold, LV_STATE_CHECKED, hold_infinite);
  set_btn_text(btn_hold, hold_infinite ? "HOLD\ninfinite" : "HOLD\ndecay");
  char t[16];
  snprintf(t, sizeof(t), "AVG\n%u", AVG_STEPS[avg_idx]);
  set_btn_text(btn_avg, t);
  lv_obj_set_state(btn_auto, LV_STATE_CHECKED, range.automatic);
  set_btn_text(btn_auto, range.automatic ? "RANGE\nauto" : "RANGE\nfixed");
}

static void update_status(void) {
  lv_label_set_text_fmt(status_label,
                        "%s %.0f-%.0f MHz   %.1f fps   avg %u   hold %s   range %s   axis %s   "
                        "fail %u",
                        shown_mode == SDR_MODE_ZOOM ? "ZOOM" : "SWEEP", shown_start / 1e6,
                        shown_stop / 1e6, fps, AVG_STEPS[avg_idx], hold_infinite ? "inf" : "decay",
                        range.automatic ? "auto" : "fixed", axis_mhz ? "MHz" : "ch",
                        (unsigned)frame.failures);
}

/* ---- Frame handling ----------------------------------------------------- */

static void poll_sdr(lv_timer_t *timer) {
  if (!sdr_task_get(frame.seq, &frame)) return;
  int64_t now = esp_timer_get_time();
  if (!have_frame || frame.mode != shown_mode || frame.f_start_hz != shown_start) {
    shown_mode = frame.mode;
    shown_start = frame.f_start_hz;
    shown_stop = frame.f_stop_hz;
    sdr_post_reset(&post);
    memset(wf_ring, 0, (size_t)W * WF_H * sizeof(uint16_t));
    update_axis();
    have_frame = true;
    last_frame_us = now;
  }
  float dt = (now - last_frame_us) / 1e6f;
  if (dt > 0) fps = fps > 0 ? fps * 0.9f + 0.1f / dt : 1.0f / dt;
  last_frame_us = now;

  sdr_post_update(&post, frame.dbfs, dt);
  sdr_range_update(&range, post.peak, W, 0.1f);
  push_waterfall_row(frame.dbfs);
  update_db_labels();
  update_marker_label();
  update_status();
}

/* ---- Events ------------------------------------------------------------- */

static void on_plot_touch(lv_event_t *e) {
  if (!have_frame) return;
  lv_point_t p;
  lv_indev_get_point(lv_indev_active(), &p);
  lv_area_t a;
  lv_obj_get_coords(lv_event_get_target_obj(e), &a);
  int x = p.x - a.x1;
  if (x < 0) x = 0;
  if (x >= W) x = W - 1;
  marker_hz = x_to_freq(x);
  update_marker_label();
}

static void on_axis(lv_event_t *e) {
  axis_mhz = !axis_mhz;
  if (have_frame) update_axis();
  update_status();
}

static void on_full(lv_event_t *e) {
  settings.mode = SDR_MODE_SWEEP;
  sdr_task_set(&settings);
  update_buttons();
}

static void on_zoom(lv_event_t *e) {
  if (!have_frame) return;
  double centre = marker_hz;
  if (centre <= 0) {
    size_t i;
    centre = sdr_peak_index(peak_db, NULL, W, &i) ? x_to_freq((int)i) : 2437e6;
  }
  if (!marker_hz) marker_hz = centre;
  settings.mode = SDR_MODE_ZOOM;
  settings.zoom_centre_hz = round(centre / 1e5) * 1e5;
  sdr_task_set(&settings);
  update_buttons();
}

static void on_peak(lv_event_t *e) {
  size_t i;
  if (have_frame && sdr_peak_index(peak_db, NULL, W, &i)) {
    marker_hz = x_to_freq((int)i);
    update_marker_label();
  }
}

static void on_hold(lv_event_t *e) {
  if (lv_event_get_code(e) == LV_EVENT_LONG_PRESSED) {
    sdr_post_peak_reset(&post);
    return;
  }
  hold_infinite = !hold_infinite;
  post.peak_decay_db_s = hold_infinite ? 0.0f : HOLD_DECAY_DB_S;
  update_buttons();
  update_status();
}

static void on_avg(lv_event_t *e) {
  avg_idx = (avg_idx + 1) % (sizeof(AVG_STEPS) / sizeof(AVG_STEPS[0]));
  post.avg_len = AVG_STEPS[avg_idx];
  post.avg_count = 0;
  update_buttons();
  update_status();
}

static void on_auto(lv_event_t *e) {
  range.automatic = !range.automatic;
  update_buttons();
  update_status();
}

/* ---- Construction ------------------------------------------------------- */

static lv_obj_t *make_button(lv_obj_t *parent, int idx, const char *text, lv_event_cb_t cb,
                             lv_event_code_t code) {
  lv_obj_t *b = lv_button_create(parent);
  lv_obj_set_pos(b, BTN_X, 4 + idx * (BTN_H + BTN_GAP));
  lv_obj_set_size(b, BTN_W, BTN_H);
  lv_obj_t *l = lv_label_create(b);
  lv_label_set_text(l, text);
  lv_obj_set_style_text_align(l, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_center(l);
  lv_obj_add_event_cb(b, cb, code, NULL);
  return b;
}

/* Transparent touch target over a directly painted plot. */
static lv_obj_t *make_plot_area(lv_obj_t *parent, int y, int h) {
  lv_obj_t *c = lv_obj_create(parent);
  lv_obj_remove_style_all(c);
  lv_obj_set_pos(c, 0, y);
  lv_obj_set_size(c, W, h);
  lv_obj_add_flag(c, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(c, on_plot_touch, LV_EVENT_PRESSED, NULL);
  lv_obj_add_event_cb(c, on_plot_touch, LV_EVENT_PRESSING, NULL);
  return c;
}

static lv_obj_t *make_label(lv_obj_t *parent, uint32_t color) {
  lv_obj_t *l = lv_label_create(parent);
  lv_obj_set_style_text_color(l, lv_color_hex(color), 0);
  return l;
}

void ui_build(void) {
  lv_display_t *disp = lv_display_get_default();
  lv_display_set_theme(disp, lv_theme_default_init(disp, lv_palette_main(LV_PALETTE_BLUE),
                                                   lv_palette_main(LV_PALETTE_CYAN), true,
                                                   &lv_font_montserrat_14));
  lv_obj_t *scr = lv_screen_active();
  lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
  lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
  build_lut();

  wf_ring = heap_caps_calloc((size_t)W * WF_H, sizeof(uint16_t), MALLOC_CAP_SPIRAM);
  overlay_save = heap_caps_malloc(OVERLAY_SAVE_PX * sizeof(uint16_t), MALLOC_CAP_SPIRAM);
  band = heap_caps_malloc((size_t)W * BAND_H * sizeof(uint16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  spec_area = make_plot_area(scr, 0, SPEC_H);
  wf_area = make_plot_area(scr, WF_Y, WF_H);

  for (int i = 0; i < MAX_DB_LABELS; i++) {
    db_labels[i] = make_label(spec_area, 0x909090);
    lv_obj_set_style_bg_color(db_labels[i], lv_color_black(), 0);
    lv_obj_set_style_bg_opa(db_labels[i], LV_OPA_COVER, 0);
    lv_obj_add_flag(db_labels[i], LV_OBJ_FLAG_HIDDEN);
  }
  marker_label = make_label(spec_area, 0xff80ff);
  lv_obj_set_style_bg_color(marker_label, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(marker_label, LV_OPA_70, 0);
  lv_obj_set_style_pad_all(marker_label, 3, 0);
  lv_obj_align(marker_label, LV_ALIGN_TOP_RIGHT, -4, 4);
  lv_obj_add_flag(marker_label, LV_OBJ_FLAG_HIDDEN);

  axis_strip = lv_obj_create(scr);
  lv_obj_remove_style_all(axis_strip);
  lv_obj_set_pos(axis_strip, 0, AXIS_Y);
  lv_obj_set_size(axis_strip, W, AXIS_H);
  /* Styled like a button so it reads as tappable; brightens while pressed. */
  lv_obj_set_style_bg_color(axis_strip, lv_color_hex(0x0c1a2e), 0);
  lv_obj_set_style_bg_color(axis_strip, lv_color_hex(0x24508c), LV_STATE_PRESSED);
  lv_obj_set_style_bg_opa(axis_strip, LV_OPA_COVER, 0);
  lv_obj_set_style_border_color(axis_strip, lv_palette_main(LV_PALETTE_BLUE), 0);
  lv_obj_set_style_border_width(axis_strip, 2, 0);
  lv_obj_set_style_radius(axis_strip, 6, 0);
  lv_obj_add_flag(axis_strip, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(axis_strip, on_axis, LV_EVENT_CLICKED, NULL);
  for (int i = 0; i < MAX_AXIS_LABELS; i++) {
    axis_labels[i] = make_label(axis_strip, 0xc0c0c0);
    lv_obj_set_width(axis_labels[i], AXIS_LABEL_W);
    lv_obj_set_style_text_align(axis_labels[i], LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_y(axis_labels[i], (AXIS_H - 16) / 2);
    lv_obj_add_flag(axis_labels[i], LV_OBJ_FLAG_HIDDEN);
  }

  status_label = make_label(scr, 0xb0b0b0);
  lv_obj_set_pos(status_label, 4, STATUS_Y + 8);
  lv_label_set_text(status_label, "starting receiver...");

  btn_full = make_button(scr, 0, "FULL", on_full, LV_EVENT_CLICKED);
  btn_zoom = make_button(scr, 1, "ZOOM", on_zoom, LV_EVENT_CLICKED);
  make_button(scr, 2, "PEAK", on_peak, LV_EVENT_CLICKED);
  btn_hold = make_button(scr, 3, "", on_hold, LV_EVENT_SHORT_CLICKED);
  lv_obj_add_event_cb(btn_hold, on_hold, LV_EVENT_LONG_PRESSED, NULL);
  btn_avg = make_button(scr, 4, "", on_avg, LV_EVENT_CLICKED);
  btn_auto = make_button(scr, 5, "", on_auto, LV_EVENT_CLICKED);
  update_buttons();

  lv_timer_create(poll_sdr, 10, NULL);
}
